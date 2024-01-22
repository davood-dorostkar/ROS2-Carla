/*
* Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
* wen mingshu <wenmingshu@senseauto.com>
*/
/** \defgroup tuePrvLkfCoordinatedTurn TUE Fusion Track Management
 * \{
 * \file       tue_prv_lkf_coordinatedTurn.c
 *
 */
/* PRQA S 0292 ++ */ /* MKS */
/*
 */
/* PRQA S 0292 -- */
/*
 *
 *          (C) Copyright Tuerme Inc. All rights reserved.
 *
 */
/*==================[inclusions]============================================*/

#include "tue_prv_lkf_coordinatedTurn.h"
#include "tue_prv_lkf_coordinatedTurn_int.h"
#include "tue_prv_fusion_math.h"
#include "TueObjFusn_ConfigAlgorithm.h"
#include "TueObjFusn_ErrorCodes.h"
#include "TueObjFusn_AAU_Codes.h"
#include "tue_prv_error_management.h"
#include "TueObjFusn_TrackableConstants.h"
#include "TueObjFusn_TrackableProps.h"

/*****************************************************************************
FUNCTIONS
*****************************************************************************/
/** @name tue_prv_lkf_coordinatedTurn.c functions */

/**
 * \fn bool_t LKF_CoordinatedTurn_DoPredict(TueObjFusn_TrackableListType * const
 * pTrkbl, f32_t const f32Q, f32_t const f32Dt)
 * \brief  predict the heading and variance for a specified time dT depending on
 * the process noise (Q)
 *
 * \param  [in, out] pTrkbl   TueObjFusn_TrackableType * const, trackable which
 * shall be predicted
 * \param  f32Q               f32_t const, process noise Q
 * \param  f32Dt              f32_t const, time interval
 *
 * \return  TRUE (ok) or FALSE (error occured)
 */
#define ObjFusn_START_SEC_CODE

/* PRQA S 1532 2 */ /* Function called from external AAU */
uint32 LKF_CoordinatedTurn_DoPredict(CONSTP2VAR(TueObjFusn_TrackableType,
                                                AUTOMATIC,
                                                ObjFusn_VAR_NOINIT) pTrkbl,
                                     const float32 f32PredictionDt) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    /* PRQA S 3204 1 */ // variable may be set to other values depending on
                        //    run-time check activation
    float32 f32Q_1_1;
    float32 f32Q_1_2;
    float32 f32Q_2_2;
    float32 f32HeadingYawRateCovar;
    float32 f32Tmp;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if (NULL_PTR == pTrkbl) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_COORDINATED_TURN,
            TUEOBJFUSN_AAU_LKF_COORDINATED_TURN_DO_PREDICT);
    } else
#endif
        if (pTrkbl->vecX.nRows > TUEOBJFUSN_MATRIX_SIZE) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_COORDINATED_TURN,
            TUEOBJFUSN_AAU_LKF_COORDINATED_TURN_DO_PREDICT);
    } else if (pTrkbl->vecX.nRows != (pTrkbl->matP).u16Size) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_SIZE_NOT_EQUAL;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_COORDINATED_TURN,
            TUEOBJFUSN_AAU_LKF_COORDINATED_TURN_DO_PREDICT);
    } else if (pTrkbl->vecX.nRows <
               TUE_PRV_LKF_COORDINATED_TURN_MIN_STATE_SIZE) {
        u32Success = TUEOBJFUSN_ERROR_INPUT_NOT_ENOUGH_STATES;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_COORDINATED_TURN,
            TUEOBJFUSN_AAU_LKF_COORDINATED_TURN_DO_PREDICT);
    } else if (f32PredictionDt < FLT_ZERO) {
        u32Success = TUEOBJFUSN_ERROR_INVALID_INPUT;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_COORDINATED_TURN,
            TUEOBJFUSN_AAU_LKF_COORDINATED_TURN_DO_PREDICT);
    } else
#endif
    {
        f32Q_2_2 = TUE_PRV_COORDINATED_TURN_VARIANCE_IN_HEADING_FOR_Q *
                   f32PredictionDt;
        f32Q_1_2 = (f32PredictionDt * f32Q_2_2);
        f32Q_1_1 = FLT_ONE_THIRD * (f32PredictionDt * f32Q_1_2);
        f32Q_1_2 *= FLT_ONE_HALF;
        f32HeadingYawRateCovar = pTrkbl->f32CovarHeadingYawRate;

        f32Tmp = ((pTrkbl->vecX).data[TRACKABLE_VELX] *
                  (pTrkbl->vecX).data[TRACKABLE_VELX]) +
                 ((pTrkbl->vecX).data[TRACKABLE_VELY] *
                  (pTrkbl->vecX).data[TRACKABLE_VELY]);

        /* No prediction for stationary targets */
        if (f32Tmp > TUE_PRV_COORDINATED_TURN_MIN_TARGET_SPEED) {
            f32Tmp = f32PredictionDt * pTrkbl->f32YawRateVar;

            /* Predict using F-Matrix  (1 Delta T)    */
            /*                         (0    1   )    */
            pTrkbl->f32Heading += (f32PredictionDt * pTrkbl->f32YawRate);
            pTrkbl->f32Heading = tue_prv_fusion_norm_angle(
                pTrkbl->f32Heading);  // normalizied between [-pi pi]

            /* Predict variances and covariance */
            pTrkbl->f32HeadingVar +=
                f32PredictionDt *
                f32Tmp;  // add propagation of yaw rate variance
            pTrkbl->f32HeadingVar +=
                (FLT_TWO *
                 (f32PredictionDt *
                  f32HeadingYawRateCovar));     // add propagation of covariance
            pTrkbl->f32HeadingVar += f32Q_1_1;  // add process noise
            pTrkbl->f32CovarHeadingYawRate +=
                f32Tmp;  // add propagation of yaw rate variance
            pTrkbl->f32CovarHeadingYawRate += f32Q_1_2;  // add process noise
            pTrkbl->f32YawRateVar += f32Q_2_2;           // add process noise
        } else {
            pTrkbl->f32YawRate = FLT_ZERO;
        }
        pTrkbl->f32HeadingVar = tue_prv_fusion_min_max_F32(
            pTrkbl->f32HeadingVar, TUEOBJFUSN_TRACKABLE_F32HEADINGVAR_MIN,
            TUEOBJFUSN_TRACKABLE_F32HEADINGVAR_MAX);
        pTrkbl->f32YawRateVar = tue_prv_fusion_min_max_F32(
            pTrkbl->f32YawRateVar, TUEOBJFUSN_TRACKABLE_F32YAWRATEVAR_MIN,
            TUEOBJFUSN_TRACKABLE_F32YAWRATEVAR_MAX);
    }

    return u32Success;
}
#define ObjFusn_STOP_SEC_CODE

/**
 * \fn bool_t LKF_CoordinatedTurn_DoCorrect(TueObjFusn_TrackableListType * const
 * pTrkbl)
 * \brief computes the (separate) Kalman filter update for heading and yaw rate
 * and variance.
 *
 * The heading, yaw rate as well as the variances and covariance of pTrkbl are
 * updated by the sensor measurement data z and R.
 *
 * \param  [in,out] pTrkbl   TueObjFusn_TrackableType * const, LKF track which
 * shall be adapted
 *
 * \return  TRUE (ok) or FALSE (error occured)
 */
#define ObjFusn_START_SEC_CODE

/* PRQA S 1532 2 */ /* Function called from external AAU */
uint32 LKF_CoordinatedTurn_DoCorrect(CONSTP2VAR(TueObjFusn_TrackableType,
                                                AUTOMATIC,
                                                ObjFusn_VAR_NOINIT) pTrkbl) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    /* PRQA S 3204 1 */ // variable may be set to other values depending on
                        //    run-time check activation
    float32 f32HeadingMeas;
    float32 f32HeadingMeasVar;
    float32 f32CovarHeadingYaw;
    float32 f32XVel;
    float32 f32YVel;
    float32 f32Tmp;
    float32 f32Res;
    float32 f32S;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if (NULL_PTR == pTrkbl) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_COORDINATED_TURN,
            TUEOBJFUSN_AAU_LKF_COORDINATED_TURN_DO_CORRECT);
    } else
#endif
        if (pTrkbl->vecX.nRows > TUEOBJFUSN_MATRIX_SIZE) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_COORDINATED_TURN,
            TUEOBJFUSN_AAU_LKF_COORDINATED_TURN_DO_CORRECT);
    } else if (pTrkbl->vecX.nRows != pTrkbl->matP.u16Size) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_SIZE_NOT_EQUAL;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_COORDINATED_TURN,
            TUEOBJFUSN_AAU_LKF_COORDINATED_TURN_DO_CORRECT);
    } else if (pTrkbl->vecX.nRows <
               TUE_PRV_LKF_COORDINATED_TURN_MIN_STATE_SIZE) {
        u32Success = TUEOBJFUSN_ERROR_INPUT_NOT_ENOUGH_STATES;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_COORDINATED_TURN,
            TUEOBJFUSN_AAU_LKF_COORDINATED_TURN_DO_CORRECT);
    } else
#endif
    {
        f32XVel = pTrkbl->vecX.data[TRACKABLE_VELX] *
                  pTrkbl->vecX.data[TRACKABLE_VELX];
        f32YVel = pTrkbl->vecX.data[TRACKABLE_VELY] *
                  pTrkbl->vecX.data[TRACKABLE_VELY];

        f32Tmp = (f32YVel * pTrkbl->matP.data[TRACKABLE_INDEX_VARIANCE_VELX]) +
                 (f32XVel * pTrkbl->matP.data[TRACKABLE_INDEX_VARIANCE_VELY]);
        f32HeadingMeas =
            tue_prv_fusion_atan2(pTrkbl->vecX.data[TRACKABLE_VELY],
                                 pTrkbl->vecX.data[TRACKABLE_VELX]);
        f32HeadingMeasVar = tue_prv_fusion_sqrt(f32Tmp);
        f32Tmp = f32XVel + f32YVel;

        /* No correction step if stationary target */
        if (f32Tmp > TUE_PRV_COORDINATED_TURN_MIN_TARGET_SPEED) {
            /** Use covariance from cycle t-1 */
            f32CovarHeadingYaw = pTrkbl->f32CovarHeadingYawRate;

            /** Variance of measurement not defined for speed == 0 */
            f32HeadingMeasVar /= f32Tmp;

            /* Calculate measurement residual */
            f32Res = f32HeadingMeas - (pTrkbl->f32Heading);
            f32Res = tue_prv_fusion_norm_angle(
                f32Res);  // normalizied between [-pi pi]
            /* Calculate S = H*P*H^T + R, simplifies to measured var + track var
             * with H = [1 0] */
            f32S = f32HeadingMeasVar + (pTrkbl->f32HeadingVar);
            /* Update heading and yaw rate with Kalman gain K = P * H * S^-1 =
             * 1/S * (Var_Heading Covar)^T */
            f32Tmp = f32Res / f32S;
            pTrkbl->f32Heading += pTrkbl->f32HeadingVar * f32Tmp;
            pTrkbl->f32Heading = tue_prv_fusion_norm_angle(
                pTrkbl->f32Heading);  // normalizied between [-pi pi]
            pTrkbl->f32YawRate += pTrkbl->f32CovarHeadingYawRate * f32Tmp;
            /* Update P Matrix P = (I - K*H)*P */
            f32Tmp = FLT_ONE - (pTrkbl->f32HeadingVar / f32S);
            pTrkbl->f32HeadingVar *= f32Tmp;
            pTrkbl->f32CovarHeadingYawRate *= f32Tmp;
            f32Tmp = (f32CovarHeadingYaw * f32CovarHeadingYaw);
            pTrkbl->f32YawRateVar -= (f32Tmp / f32S);
        } else {
            pTrkbl->f32YawRate =
                FLT_ZERO; // set yaw rate to zero  - a stationary vehicle can
                            //  not change its yaw rate
        }
        pTrkbl->f32HeadingVar = tue_prv_fusion_min_max_F32(
            pTrkbl->f32HeadingVar, TUEOBJFUSN_TRACKABLE_F32HEADINGVAR_MIN,
            TUEOBJFUSN_TRACKABLE_F32HEADINGVAR_MAX);
        pTrkbl->f32YawRateVar = tue_prv_fusion_min_max_F32(
            pTrkbl->f32YawRateVar, TUEOBJFUSN_TRACKABLE_F32YAWRATEVAR_MIN,
            TUEOBJFUSN_TRACKABLE_F32YAWRATEVAR_MAX);
    }

    return u32Success;
}
#define ObjFusn_STOP_SEC_CODE

/**
 * @}
 */
