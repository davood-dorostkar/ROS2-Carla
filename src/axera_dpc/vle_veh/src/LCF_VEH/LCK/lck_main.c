/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/**
@ingroup LCK
@{ */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "lck.h"

#include "lck_str_ang_ctrl.h"
#include "lck_par.h"
#include "tue_common_libs.h"

/*****************************************************************************
  SYMBOLIC CONSTANTS
*****************************************************************************/

/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/
#define EP30_LCK_VERSION
#define SCALE_COEFFICIENT 2.0f

/*****************************************************************************
  CONSTS
*****************************************************************************/

/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE2_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/* last controller mode for linear fading  of reference steering angle */
static eLCKMode_t eLCKLinFadingModeLast = LCK_MODE_OFF;

/* last fading offset */
static float32 fLCKLinFadingOffsetLast = LCK_INIT_F_ZERO;

/* last fading input */
static float32 fLCKLinFadingInLast = LCK_INIT_F_ZERO;

/* Last current steering angle */
static float32 fLCKStrAngLast = LCK_INIT_F_ZERO;

/* Last Reference steering angle output */
/* QAC suppression of Msg(3:3218) File scope static, 'fLCKRefStrAngLast', is
 * only accessed in one function. */
/* PRQA S 3218 2 */
static float32 fLCKRefStrAngLast = LCK_INIT_F_ZERO;
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  PROTOTYPES
*****************************************************************************/

static void LCKLinFading(const eLCKMode_t eMode,
                         const float32 fFadingCoeff,
                         const float32 fStrAng,
                         float32 *fRefStrAng);

static eGDBError_t LCKLimiter(const eLCKMode_t eMode,
                              float32 *fMag,
                              float32 *fGrad,
                              float32 const fAbsMagLimit,
                              float32 const fAbsGradLimit,
                              float32 const fDeltaT);

/* ***************************************************************************
 @fn            LCKRun                                                  */ /*!
           @brief         runs one LCK loop for all modules

           @description

           @param         *pInputData           all internal input data for LCK
          module

           @return        error

           @pre           pInputData must be set, pModuleList must be set

           @post          [none]

          ****************************************************************************
          */
eGDBError_t LCKRun(sLCKInput_t const *pInputData,
                   sLCKParam_t const *pParam,
                   sLCKOutput_t *pLCKOutput,
                   float32 const fCycleTime) {
    //-- declaration and initialization ---------------------------------------
    eGDBError_t eGDBError = GDB_ERROR_NONE;

    //-- check for valid input data (NULL pointer) ----------------------------
    if ((pInputData == NULL) || (pParam == NULL) || (pLCKOutput == NULL)) {
        eGDBError = GDB_ERROR_POINTER_NULL;
    } else {
        //-- select controller -------------------------------------------------
        switch (pInputData->eLCKMode) {
            case LCK_MODE_OFF:
            case LCK_MODE_OFF_NOW:
                //-- switch off lateral control
                //--------------------------------------
                //   reference steering angle is set to current steering angle
                //   and
                //   reference steering angle request flag is set to zero
                LCKOff(pInputData, pLCKOutput);
                break;

            case LCK_MODE_ON_SP:

                //-- calculate reference steering angle by state space
                // controller -----
                eGDBError = LCKStateSpace(pInputData, pParam, pLCKOutput);

#ifdef EP30_LCK_VERSION
                // Temp to scale right request angle
                if (pLCKOutput->fRefStrAng > 0.0) {
                    pLCKOutput->fRefStrAng =
                        pLCKOutput->fRefStrAng *
                        SCALE_COEFFICIENT;  // scale right side angle *1.2
                }
#endif
                break;

            case LCK_MODE_ON_CAS:

                //-- calculate reference steering angle by cascading controller
                //-------
                LCKCascade(pInputData, pParam, pLCKOutput);
                break;

            default:
                //-- ERROR: Invalid LCKMode
                //-------------------------------------------
                //   switch off lateral control
                LCKOff(pInputData, pLCKOutput);
                eGDBError = GDB_ERROR_VALUE_RANGE;
                break;

        }  // end switch LCKMode

        /* linear fading */
        pLCKGlobalDebugData->fLinFadingIn = pLCKOutput->fRefStrAng;

        LCKLinFading(
            pInputData->eLCKMode, /* controller mode */
            DEG2RAD(LCK_PAR_LIN_FADING_COEF) /
                pParam->fSteeringRatio, /* fading coefficient */
            pInputData->fSteerAng,      /* current steering angle [rad] */
            &(pLCKOutput->fRefStrAng)); /* reference steering angle from state
                                           space controller output [rad] */

        pLCKGlobalDebugData->fLinFadingOut = pLCKOutput->fRefStrAng;

        pLCKOutput->fRefStrVel =
            (pLCKOutput->fRefStrAng - fLCKRefStrAngLast) / fCycleTime;

        /* LCK saturation and rate limiter */
        eGDBError = LCKLimiter(
            pInputData->eLCKMode, &(pLCKOutput->fRefStrAng),
            &(pLCKOutput->fRefStrVel),
            DEG2RAD(LCK_PAR_MAX_REF_STR_ANG) / pParam->fSteeringRatio,
            DEG2RAD(LCK_PAR_MAX_REF_STR_VEL) / pParam->fSteeringRatio,
            fCycleTime);

        fLCKRefStrAngLast = pLCKOutput->fRefStrAng;
        pLCKGlobalDebugData->fRefStrVel =
            pLCKOutput->fRefStrVel * (pParam->fSteeringRatio);

    }  // end null pointer check

    return eGDBError;
}

/* ***********************************************************************
  @fn            LCKLinFading                                           */ /*!
    @brief         LCK linear fading.
    @description   LCK linear fading.
                   As soon as the LDP algorithm starts with the interaction
                   a jerk in the steering wheel is occurring. This is due to
                   the fact, that the control difference as seen by the
                   dynamic controller (LCD) is unequal to zero and that the
                   dynamic controller possesses a direct feed through path
                   resulting in an immediate EPS torque request.
                   A possible remedy is the integration of a signal fading
                   function, that provides a control error of zero at the
                   beginning of the intervention and which is decaying with
                   defined dynamic. Here, a linear fading function is
                   implemented.
                   High change rate in the EPS torque jerk signal after
                   fading is completed, should be evaluated in the vehicle.
    @param[in]     eMode : controller mode
    @param[in]     fFadingCoeff : fading coefficient
    @param[in]     fStrAng : current steering angle [rad]
    @param[in,out] fRefStrAng : reference steering angle [rad]
    @return        void
  ****************************************************************************
  */
static void LCKLinFading(const eLCKMode_t eMode,
                         const float32 fFadingCoeff,
                         const float32 fStrAng,
                         float32 *fRefStrAng) {
    /* -- declaration and initialization of local variables ------------------
     */
    float32 fOffset = LCK_INIT_F_ZERO;       /* fading offset        */
    float32 fLinFadingOut = LCK_INIT_F_ZERO; /* linear fading output */
    float32 fLinFadingIn = *fRefStrAng;      /* linear fading input  */

    /* -- linear fading ------------------------------------------------------
     */
    if ((eMode > LCK_MODE_OFF) && (eLCKLinFadingModeLast > LCK_MODE_OFF)) {
        /*************************************************************************
         * fading *
         *************************************************************************/
        if (fLCKLinFadingOffsetLast > fFadingCoeff) {
            /* positive fading */
            fOffset = fLCKLinFadingOffsetLast - fFadingCoeff;

            /* compensate for opposite movement of reference steering angle */
            if (fLCKLinFadingInLast > fLinFadingIn) {
                fOffset = fOffset + (fLinFadingIn - fLCKLinFadingInLast);
            }

            /* compensate for movement of steering angle in same direction */
            if (fLCKStrAngLast < fStrAng) {
                fOffset = fOffset - (fStrAng - fLCKStrAngLast);
            }

            /* if offset flipped the sign due to compensation, set it to zero */
            if (fOffset < LCK_INIT_F_ZERO) {
                fOffset = LCK_INIT_F_ZERO;
            }

        } else if (fLCKLinFadingOffsetLast < -fFadingCoeff) {
            /* negative fading */
            fOffset = fLCKLinFadingOffsetLast + fFadingCoeff;

            /* compensate for opposite movement of reference steering angle */
            if (fLCKLinFadingInLast < fLinFadingIn) {
                fOffset = fOffset + (fLinFadingIn - fLCKLinFadingInLast);
            }

            /* compensate for movement of steering angle in same direction */
            if (fLCKStrAngLast > fStrAng) {
                fOffset = fOffset + (fLCKStrAngLast - fStrAng);
            }

            /* if offset flipped the sign due to compensation, set it to zero */
            if (fOffset > LCK_INIT_F_ZERO) {
                fOffset = LCK_INIT_F_ZERO;
            }

        } else {
            /* fading finished */
            fOffset = LCK_INIT_F_ZERO;
        }

    } else {
        /*************************************************************************
         * no fading *
         *************************************************************************/
        /* controller switched off OR *
         * controller in first cycle in which it is switched on *
         * ---------------------------------------------------------------------
         **
         * At start of a steering intervention the output of the linear fading *
         * equals always to the current steering angle, i.e. there is no control
         **
         * error at the beginning. *
         *************************************************************************/
        fOffset = fLinFadingIn - fStrAng;
    }

#if (FCT_CFG_LKS_SYSID_TEST_SIGNAL_GEN)
    /* in system identification mode,
       set linear fading coefficient always to zero */
    if (eMode == LCK_MODE_SYSID) {
        fOffset = LCK_INIT_F_ZERO;
    }
#endif /* FCT_CFG_LKS_SYSID_TEST_SIGNAL_GEN */

    /* faded reference steering angle */
    fLinFadingOut = fLinFadingIn - fOffset;

    /* update states for next cycle */
    fLCKLinFadingInLast = fLinFadingIn;
    fLCKLinFadingOffsetLast = fOffset;
    eLCKLinFadingModeLast = eMode;
    fLCKStrAngLast = fStrAng;

    /* output faded reference steering angle */
    (*fRefStrAng) = fLinFadingOut;
}

/* ***********************************************************************
  @fn            LCKLimiter                                              */ /*!
  @brief         Output limiter
  @description   Output limiter, compare to simulink saturation block. The
                 function limits an input variable in amplitude and gradient.
                 The input variables to this function is the steering angle
                 and it's gradient. The limits/ thresholds to which the angle
                 signal is to be limited is also passed as an input arguement
                 to the function. Since the function is discrete and called
                 approximately every 20ms. The sample time is also passed as
                 an input arguement to the function which is used for
                 calculating the gradient. The input and output to this
                 function is the same variable as the input is modified
                 and passed as an output.

                 The current steering angle sample is obtained at the input
                 and is compared with the last sample and the angle gradient
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
  Value Range: [-inf, inf]rad;
  @param[in,out] *fGrad         : Gradient of the Input sample to be limited
  Value Range: [-inf, inf]rad/s;
  @param[in]     fAbsMagLimit   : Absolute value of magnitude limit
  Value Range: [0  0.1 ] rad
  @param[in]     fAbsGradLimit  : Absolute value of gradient limit
  Value Range: [0  0.1 ] rad/s
  @param[in]     fDeltaT        : Sample time [s]
  @return        eGDBError_t    : error code
**************************************************************************** */
static eGDBError_t LCKLimiter(eLCKMode_t const eMode,
                              float32 *fMag,
                              float32 *fGrad,
                              float32 const fAbsMagLimit,
                              float32 const fAbsGradLimit,
                              float32 const fDeltaT) {
    eGDBError_t eRetValue = GDB_ERROR_NONE;
    float32 fMagLast = LCK_INIT_F_ZERO;

    if ((eMode == LCK_MODE_OFF) || (eMode == LCK_MODE_OFF_NOW) ||
        (eMode == LCK_MODE_SYSID)) {
        /*DO nothing: DO not limit the outputs as they shall be as
        dynamic as possible ot the current steering angle*/
    } else {
        if (fDeltaT > LCK_INIT_F_ZERO) /* check valid input data*/
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

            /* This output torque gradient is calculated irrespective of changes
             * in torque and gradient*/
            *fGrad = (*fMag - fMagLast) / fDeltaT;
        } else {
            /* invalid sample time */
            eRetValue = GDB_ERROR_VALUE_RANGE;

            (*fMag) = LCK_INIT_F_ZERO;
            (*fGrad) = LCK_INIT_F_ZERO;
        }
    }

    return eRetValue;
}

/** @} end ingroup */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
