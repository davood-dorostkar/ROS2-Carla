/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/**
@ingroup lck
@{ */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "lck.h"
#include "TM_Global_Types.h"
#include "tue_common_libs.h"

#include "lck_par.h"
#include "lck_str_ang_ctrl.h"

/*****************************************************************************
  MODULE QAC messages suppression
*****************************************************************************/
/* QAC suppression of Msg(2:3121) Hard-coded 'magic' floating constant '_._f'.
 */
/* PRQA S 3121 ++ */

/*****************************************************************************
  SYMBOLIC CONSTANTS
*****************************************************************************/

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

/*****************************************************************************
  PROTOTYPES
*****************************************************************************/
static float32 LCKStrAngFeedForward(float32 const fVhclSpeed,
                                    float32 const fCurvature,
                                    float32 const fWheelBase);

static float32 LCKYawRateFeedForward(float32 const fVhclSpeed,
                                     float32 const fCurvature);

static eGDBError_t LCKFeedForward(float32 const fCurvature,
                                  float32 const fVhclSpeed,
                                  sLCKParam_t const *psLCKParam,
                                  float32 *pfSteerAngFF);

static eGDBError_t LCKStateFeedback(float32 const fVhclSpeed,
                                    float32 const fSteerAng,
                                    float32 const fSteerAngVel,
                                    float32 const fYawRate,
                                    float32 const fHeading,
                                    float32 const fLatDev,
                                    float32 *pfSteerAngSF);

static float32 LCKLookUp2D(float32 const fArrayX[],
                           float32 const fArrayY[],
                           uint32 const uiLength,
                           float32 const fValue);

/*****************************************************************************
  FUNCTIONS
*****************************************************************************/

/* ***********************************************************************
  @fn            LCKStateSpace */ /*!
                                                                                      @brief         Reference steering
                                                                                    angle based on state feedback control
                                                                                      @description   Calculation of
                                                                                    steering angle at front tires to be
                                                                                    set in
                                                                                                     order to guide
                                                                                    vehicle on a certain trajectory. This
                                                                                                     function is based on
                                                                                    a state feedback controller, as
                                                                                                     states the current
                                                                                    steering angle, the yaw rate, the
                                                                                                     orientation of the
                                                                                    vehicle within the lane (heading) and
                                                                                                     the lateral deviation
                                                                                    to the trajectory are used. The
                                                                                                     feedback gains are
                                                                                    calculated by linear-quadratic
                                                                                    regulator
                                                                                                     (LQR) for different
                                                                                    vehicle speeds.
                                                                                      @param[in]     sLCKInput : Inputs
                                                                                    for kinematic lateral controller
                                                                                      @param[in]     sLCKParam : Parameter
                                                                                    for kinematic lateral controller
                                                                                      @param[out]    sLCKOutput : Output
                                                                                    of kinematic lateral controller
                                                                                      @return        void
                                                                                    ****************************************************************************
                                                                                    */
eGDBError_t LCKStateSpace(sLCKInput_t const *sLCKInput,
                          sLCKParam_t const *sLCKParam,
                          sLCKOutput_t *sLCKOutput) {
    /* -- declaration and initialization -------------------------------------
     */
    eGDBError_t eRetValue; /* error code */

    float32 fStrAngFeedForward =
        LCK_INIT_F_ZERO;         /* steering angle feedforward    */
    float32 fYawRateFeedForward; /* yaw rate feedforward */
    // float32 fYawRateFeedForward = LCK_INIT_F_ZERO; /* yaw rate feedforward */
    float32 fStrAngFF = LCK_INIT_F_ZERO; /* feedforward steering angle    */
    float32 fStrAngSF = LCK_INIT_F_ZERO; /* state feedback steering angle */

    float32 fWheelBase =
        sLCKParam->fDistToFrontAxle + sLCKParam->fDistToRearAxle;

    /* no input parameter check required: null pointer check is done in
       lck_main,
       sLCKParam->fCycleTime > 0.0f is check in lck wrapper */

    /* -- feed forward based on expected steering angle ----------------------
     */
    fStrAngFeedForward =
        LCK_PAR_FF_STR_ANG_MASTER_GAIN *
        LCKStrAngFeedForward(sLCKInput->fVhclSpeed, sLCKInput->fCurvature,
                             fWheelBase);

    /* -- feed forward based on expected yaw rate ----------------------------
     */
    fYawRateFeedForward =
        LCK_PAR_FF_YAW_RATE_MASTER_GAIN *
        LCKYawRateFeedForward(sLCKInput->fVhclSpeed, sLCKInput->fCurvature);

    /* -- feed forward based on curvature, vehicle speed and self-steering
     * gradient */
    eRetValue = LCKFeedForward(sLCKInput->fCurvature, /* lane curvature */
                               sLCKInput->fVhclSpeed, /* vehicle speed */
                               sLCKParam,   /* vehicle parameter        */
                               &fStrAngFF); /* reference steering angle */

    fStrAngFF = LCK_PAR_FEEDFORWARD_MASTER_GAIN * fStrAngFF;

    /* -- state feedback steering angle --------------------------------------
     */
    if (eRetValue == GDB_ERROR_NONE) {
        eRetValue = LCKStateFeedback(
            sLCKInput->fVhclSpeed, /* vehicle speed [m/s] */
            sLCKInput->fSteerAng,  /* current steering angle [rad] */
            sLCKInput
                ->fSteerAngVel,  /* current steering angle velocity [rad/s] */
            sLCKInput->fYawRate, /* yaw rate [rad/s] */
            sLCKInput
                ->fHeading, /* vehicle's orientation w.r.t. the lane [rad] */
            sLCKInput->fLatDev, /* lateral deviation to target trajectory [m] */
            &fStrAngSF); /* state feedback steering angle [rad]         */
    }

    /* -- reference steering angle -------------------------------------------
     */
    if (eRetValue == GDB_ERROR_NONE) {
        sLCKOutput->bRefStrAnglReq = TRUE;

        /* QAC suppression of Msg(2:3393) Extra parentheses recommended. An
         * arithmetic operation (* / + -) is the operand of a different operator
         * with the same precedence. */
        /* PRQA S 3393 5 */
        sLCKOutput->fRefStrAng =
            fStrAngFeedForward + fYawRateFeedForward + fStrAngFF - fStrAngSF;
    } else {
        sLCKOutput->bRefStrAnglReq = FALSE;
        sLCKOutput->fRefStrAng = sLCKInput->fSteerAng;
    }

    /* debug output */
    // pLCKGlobalDebugData->fFFStrAng      = fStrAngFeedForward;
    // pLCKGlobalDebugData->fFFYawRate     = fYawRateFeedForward;
    // pLCKGlobalDebugData->fFeedForward   = fStrAngFF;
    // pLCKGlobalDebugData->fStateFeedback = fStrAngSF;

    return eRetValue;

}  // end function

/* ***********************************************************************
  @fn            LCKCascade                                              */ /*!
  @brief         Reference steering angle based on cascading controller
  @description   Reference steering angle based on cascading controller,
                 FFM kinematic controller.
  @param[in]     sLCKInput : Inputs for kinematic lateral controller
  @param[in]     sLCKParam : Parameter for kinematic lateral controller
  @param[out]    sLCKOutput : Output of kinematic lateral controller
  @return        void
**************************************************************************** */
void LCKCascade(sLCKInput_t const *sLCKInput,
                sLCKParam_t const *sLCKParam,
                sLCKOutput_t *sLCKOutput) {
    //-- Null pointer check ---------------------------------------------------
    if ((sLCKInput == NULL) || (sLCKParam == NULL) || (sLCKOutput == NULL)) {
        /* NULL Pointer ERROR!!! */
    } else {
        //-- calculate output reference steering angle
        //--------------------------
        sLCKOutput->bRefStrAnglReq = FALSE;
        sLCKOutput->fRefStrAng = LCK_INIT_F_ZERO;

    }  // end null pointer check

}  // end function

/* ***********************************************************************
  @fn            LCKOff                                                  */ /*!
  @brief         Switch off lateral control
  @description   Switch off lateral control.
  @param[in]     sLCKInput : Inputs for kinematic lateral controller
  @param[in]     sLCKParam : Parameter for kinematic lateral controller
  @param[out]    sLCKOutput : Output of kinematic lateral controller
  @return        void
**************************************************************************** */
void LCKOff(sLCKInput_t const *sLCKInput, sLCKOutput_t *sLCKOutput) {
    /* Null pointer check not necessary,
      because input parameter already checked in lcd_main */

    //-- reference steering angle -------------------------------------------
    sLCKOutput->bRefStrAnglReq = FALSE;
    sLCKOutput->fRefStrAng = LCK_INIT_F_ZERO;

}  // end function

/* ***********************************************************************
  @fn            LCKStrAngFeedForward                                    */ /*!
  @brief         Feed forward steering angle due to steering angle.
  @description   Feed forward steering angle due to steering angle.
                 Starting from the steady-state cornering, the steering
                 angle can be described by the following equation:

                  delta = kappa * l

                 where delta is the steering angle, kappa the curvature,
                 and l the wheel base of the vehicle. When cornering,
                 feedforward removes that portion of the control output,
                 which is expected due to the known curvature. Hence,
                 the steering angle calculated by the curvature is
                 multiplied by the state feedback steering angle gains,
                 so that it can be removed from control output.

  @param[in]     fVhclSpeed : Vehicle speed [m/s]
  @param[in]     fCurvature : lane curvature [1/m]
  @param[in]     fWheelBase : wheel base [m]
  @return        steering angle feedforward
**************************************************************************** */
static float32 LCKStrAngFeedForward(float32 const fVhclSpeed,
                                    float32 const fCurvature,
                                    float32 const fWheelBase) {
    float32 fStrAngFeedForward = LCK_INIT_F_ZERO;

    /* expected steering angle based on lane curvature and wheel base */
    fStrAngFeedForward = fCurvature * fWheelBase;

    /* multiply with yaw rate state feedback gain */
    fStrAngFeedForward =
        (LCK_PAR_STR_ANG_MASTER_GAIN * fStrAngFeedForward) *
        LCKLookUp2D(fLCKParVelArray_c,         /* velocity array */
                    fLCKParStrAngGainsArray_c, /* yaw rate feedback gains */
                    LCK_PAR_NO_SUPPORT_POINTS, /* number of support points */
                    fVhclSpeed); /* current vehicle speed                  */

    return fStrAngFeedForward;
}

/* ***********************************************************************
  @fn            LCKYawRateFeedForward                                   */ /*!
  @brief         Feed forward steering angle due to yaw rate.
  @description   Feed forward steering angle due to yaw rate. Starting
                 from the steady-state cornering, the yaw rate can be
                 described by the following equation:

                  dpsi = kappa * v

                 where dpsi is the yaw rate, kappa the curvature, and v
                 the longitudinal vehicle speed. When cornering,
                 feedforward removes that portion of the control output,
                 which is expected due to the known curvature. Hence,
                 the yaw rate calculated by the curvature is multiplied
                 by the state feedback yaw rate gains, so that it can be
                 removed from control output.

  @param[in]     fVhclSpeed : Vehicle speed [m/s]
  @param[in]     fCurvature : lane curvature [1/m]
  @return        yaw rate feedforward
**************************************************************************** */
static float32 LCKYawRateFeedForward(float32 const fVhclSpeed,
                                     float32 const fCurvature) {
    float32 fYawRateFeedForward = LCK_INIT_F_ZERO;

    /* expected yaw rate based on lane curvature and vehicle speed */
    fYawRateFeedForward = fCurvature * fVhclSpeed;

    /* multiply with yaw rate state feedback gain */
    fYawRateFeedForward =
        (fYawRateFeedForward * LCK_PAR_YAW_RATE_MASTER_GAIN) *
        LCKLookUp2D(fLCKParVelArray_c,          /* velocity array */
                    fLCKParYawRateGainsArray_c, /* yaw rate feedback gains */
                    LCK_PAR_NO_SUPPORT_POINTS,  /* number of support points */
                    fVhclSpeed); /* current vehicle speed                  */

    return fYawRateFeedForward;
}

/* ***********************************************************************
  @fn            LCKFeedForward                                    */ /*!
              @brief         Calculation of feed forward steering angle
              @description   Calculation of feed forward steering angle. The
            feed
                             forward steering angle corresponds to the steering
            angle
                             to be set so that the vehicle follows a specific
            lane
                             curvature. It depends on the vehicle speed, the
            self
                             steering gradient and vehicle dynamics parameter
            like
                             vehicle weight, corneringstiffness of front/rear
            tires
                             and distance between vehicle's center of gravity
            (CoG)
                             and front/rear axle.
              @param[in]     pfCurvature : Lane curvature [1/m]
              @param[in]     pfVhclSpeed : Vehicle speed [m/s]
              @param[in]     psLCKParam : Vehicle parameter [-]
              @param[out]    pfSteerAngFF : Feed forward steering angle [rad]
              @return        eGDBError_t : error code
            ****************************************************************************
            */
static eGDBError_t LCKFeedForward(float32 const fCurvature,
                                  float32 const fVhclSpeed,
                                  sLCKParam_t const *psLCKParam,
                                  float32 *pfSteerAngFF) {
    //-- declaration and initialization --------------------------------------
    eGDBError_t eRetValue = GDB_ERROR_NONE;  // error code

    //-- Null pointer check ---------------------------------------------------
    if (pfSteerAngFF == NULL) /* psLCKParam already checked */
    {
        /* NULL Pointer ERROR!!! */
        eRetValue = GDB_ERROR_POINTER_NULL;
    } else if ((psLCKParam->fDistToRearAxle > 0.0f) &&
               (psLCKParam->fDistToFrontAxle > 0.0f) &&
               (psLCKParam->fCrnStiffFr > 0.0f) &&
               (psLCKParam->fCrnStiffRe > 0.0f)) {
        //-- calculate self steering gradient ---------------------------------
        //
        //           EG = m*(cr*lr - cf*lf)/(cr*cf*(lr+lf))
        //

        //-- calculate feed forward steering angle ----------------------------
        //   pfStrAngFF = fCurvature*(lf + lr + EG*fVhclSpeed^2);
        *pfSteerAngFF =
            fCurvature *
            (psLCKParam->fDistToFrontAxle + psLCKParam->fDistToRearAxle +
             (psLCKParam->fSelfStrGrad * (fVhclSpeed * fVhclSpeed)));

    } else {
        /* parameter out of range ERROR !!! */
        eRetValue = GDB_ERROR_VALUE_RANGE;
    }

    return eRetValue;

}  // end function

/* ***********************************************************************
  @fn            LCKStateFeedback                                  */ /*!
              @brief         Calculation of state feedback steering angle
              @description   Calculation of state feedback steering angle.
              @param[in]     pfVhclSpeed : Vehicle speed [m/s]
              @param[in]     pfSteerAng : Current steering angle [rad]
              @param[in]     pfSteerAng : Current steering angle velocity
            [rad/s]
              @param[in]     pfYawRate : Yaw rate [rad/s]
              @param[in]     pfHeading : Vehicle's orientation within the lane
            [rad]
              @param[in]     pfLatDev : Lateral deviation to target trajectory
            [m]
              @param[out]    pfSteerAngSF : State feedback steering angle [rad]
              @return        void
            ****************************************************************************
            */
static eGDBError_t LCKStateFeedback(float32 const fVhclSpeed,
                                    float32 const fSteerAng,
                                    float32 const fSteerAngVel,
                                    float32 const fYawRate,
                                    float32 const fHeading,
                                    float32 const fLatDev,
                                    float32 *pfSteerAngSF) {
    //-- declaration and initialization --------------------------------------
    eGDBError_t eRetValue = GDB_ERROR_NONE;  // error code
    float32 fStrAngGain =
        LCK_INIT_F_ZERO;  // steering angle state feedback gain
    float32 fStrAngVelGain =
        LCK_INIT_F_ZERO;  // steering angle velocity state feedback gain
    float32 fYawRateGain = LCK_INIT_F_ZERO;  // yaw rate state feedback gain
    float32 fHeadingGain = LCK_INIT_F_ZERO;  // heading state feedback gain
    float32 fLatDevGain =
        LCK_INIT_F_ZERO;  // lateral deviation state feedback gain

    //-- Null pointer check ---------------------------------------------------
    if (pfSteerAngSF == NULL) /* psLCKParam already checked */
    {
        /* NULL Pointer ERROR!!! */
        eRetValue = GDB_ERROR_POINTER_NULL;
    } else {
        //-- interpolate feedback gains according to current vehicle speed ----
        fStrAngGain =
            LCK_PAR_STR_ANG_MASTER_GAIN *
            LCKLookUp2D(
                fLCKParVelArray_c, /* velocity array                         */
                fLCKParStrAngGainsArray_c, /* steering angle feedback gains */
                LCK_PAR_NO_SUPPORT_POINTS, /* number of support points */
                fVhclSpeed); /* current vehicle speed                  */

        fStrAngVelGain =
            LCK_PAR_STR_VEL_MASTER_GAIN *
            LCKLookUp2D(
                fLCKParVelArray_c, /* velocity array                         */
                fLCKParStrVelGainsArray_c, /* steering angle velocity feedback
                                              gains */
                LCK_PAR_NO_SUPPORT_POINTS, /* number of support points */
                fVhclSpeed); /* current vehicle speed                  */

        fYawRateGain =
            LCK_PAR_YAW_RATE_MASTER_GAIN *
            LCKLookUp2D(
                fLCKParVelArray_c, /* velocity array                         */
                fLCKParYawRateGainsArray_c, /* yaw rate feedback gains */
                LCK_PAR_NO_SUPPORT_POINTS,  /* number of support points */
                fVhclSpeed); /* current vehicle speed                  */

        fHeadingGain =
            LCK_PAR_HEADING_MASTER_GAIN *
            LCKLookUp2D(
                fLCKParVelArray_c, /* velocity array                         */
                fLCKParHeadingGainsArray_c, /* heading feedback gains */
                LCK_PAR_NO_SUPPORT_POINTS,  /* number of support points */
                fVhclSpeed); /* current vehicle speed                  */

        fLatDevGain =
            LCK_PAR_LAT_DEV_MASTER_GAIN *
            LCKLookUp2D(
                fLCKParVelArray_c, /* velocity array                         */
                fLCKParLatDevGainsArray_c, /* lateral deviation feedback gains
                                            */
                LCK_PAR_NO_SUPPORT_POINTS, /* number of support points */
                fVhclSpeed); /* current vehicle speed                  */

        //-- calculate state feedback steering angle --------------------------
        *pfSteerAngSF = (fStrAngGain * fSteerAng) +
                        (fStrAngVelGain * fSteerAngVel) +
                        (fYawRateGain * fYawRate) + (fHeadingGain * fHeading) +
                        (fLatDevGain * fLatDev);

        /* debug output */
        // pLCKGlobalDebugData->fStrAngGain        = fStrAngGain;
        // pLCKGlobalDebugData->fStrAngFeedback    = fStrAngGain*fSteerAng;
        // pLCKGlobalDebugData->fStrAngVelGain     = fStrAngVelGain;
        // pLCKGlobalDebugData->fStrAngVelFeedback =
        // fStrAngVelGain*fSteerAngVel;
        // pLCKGlobalDebugData->fYawRateGain       = fYawRateGain;
        // pLCKGlobalDebugData->fYawRateFeedback   = fYawRateGain*fYawRate;
        // pLCKGlobalDebugData->fHeadingGain       = fHeadingGain;
        // pLCKGlobalDebugData->fHeadingFeedback   = fHeadingGain*fHeading;
        // pLCKGlobalDebugData->fLatDevGain        = fLatDevGain;
        // pLCKGlobalDebugData->fLatDevFeedback    = fLatDevGain*fLatDev;

    }  // end null pointer check

    return eRetValue;

}  // end function

/* ***********************************************************************
  @fn            LCKLookUp2D                                             */ /*!
  @brief         Piecewise linear two-dimensional interpolation
  @description   Piecewise linear two-dimensional interpolation. See
                 http://blogs.mathworks.com/loren/2008/08/25/piecewise-linear-interpolation/
  @param[in]     pfArrayX : Array of horizontal support points
  @param[in]     pfArrayY : Array of vertical support points
  @param[in]     uiLength : Number of support points
  @param[in]     fValue : horizontal position for interpolation
  @return        float32 : interpolated value
**************************************************************************** */
static float32 LCKLookUp2D(float32 const fArrayX[],
                           float32 const fArrayY[],
                           uint32 const uiLength,
                           float32 const fValue) {
    //-- declaration and initialization
    //-----------------------------------------
    float32 fRetVal = LCK_INIT_F_ZERO; /* interpolated value as return value */
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
            fRetVal = ((1.0f - fRetVal) * fArrayY[uiSeg]) +
                      (fRetVal * fArrayY[uiSeg + 1]);

        }  // end if-elsif-else

    }  // end null pointer check

    return fRetVal;

}  // end function

/** @} end ingroup */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
