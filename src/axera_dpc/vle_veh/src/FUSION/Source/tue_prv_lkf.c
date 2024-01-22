/*
* Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
* wen mingshu <wenmingshu@senseauto.com>
*/
/** \defgroup tuePrvlkf TUE Fusion Linear Kalman Filter
 * \{
 * This AAU consists of a four-, five- and six-state Linear Kalman Filter (LKF).
 * The LKF consists of a prediction and update step.
 *
 * During the prediction step the state x is predicted by dt on basis of a
 * motion model, the covariance matrix P is increased:
 * - x = A * x    -> state is predicted
 * - P = A * P * A^T + Q   -> covariance is predicted and an model uncertainty Q
 * is added
 *
 * The update step integrates the sensor measurement data z to the LKF state and
 * reduces the covariance matrix P dependent on the sensor measurement noise R
 * and transformation matrix H, which describes how x can be tranformed to z
 * - z_hat = z - H * x     ->innovation
 * - S = H * P * H^T + R   -> covariance matrix of innovation
 * - K = P * H^T * S^-1    -> Kalman gain
 * - x = x + K * z_hat     -> update state vector
 * - P = P - K * S * K^T   -> update covariance matrix (general form)
 *
 * To enhance stability of the covariance matrix update due to matrix inversion,
 * the Joseph form is used:
 * - P = (I - K * H) * P * (I - K * H)^T + K * R * K^T (I = identity matrix)
 *
 * the state vector x of a single object is:
 * x(0): x-position of object
 * x(1): y-position of object
 * x(2): x-velocity of object
 * x(3): y-velocity of object
 * x(4): ax-position of object (optional)
 * x(5): ax-position of object (optional)
 * accordingly to x the P matrix represents the corresponding variances and
 * covariances of the states
 *
 *
 * Motion Models (is calculated seperatedly for x and y):
 * - constant velocity: A = [1, dt; 0, 1 ]; Q = q * [dt^3/3, dt^2/2; dt^2/2,
 * dt];
 * - constant acceleration: A = [1, dt, dt^2/2; 0, 1, dt; 0, 0, 1];
 *    Q = q * [dt^5/20, dt^4/8, dt^3/6; dt^4/8, dt^3/3, dt^2/2; dt^3/6, dt^3/3,
 * dt]
 *
 * The measurement model is based on the transformation matrix H and the noise
 * matrix R. The H matrix is the identity matrix however with zeros at the
 * diagonal values if the sensor does not use this state. The R matrix is the
 * covariance matrix of the sensor.
 *
 * \file       tue_prv_lkf.c
 * \brief  Main LKF AAU file
 *
 */
/* PRQA S 0292 ++ */ /* MKS */
                     /*
                      */
/* PRQA S 0292 -- */
/*
 *
 *
 *          (C) Copyright Tuerme Inc. All rights reserved.
 *
 */

/*==================[inclusions]============================================*/
#include "tue_prv_lkf.h"
#include "tue_prv_lkf_int.h"
#include "TueObjFusn_ParameterInterface.h"
#include "TueObjFusn_ConfigAlgorithm.h"
#include "TueObjFusn_TrackableConstants.h"
#include "TueObjFusn_Eps.h"
#include "tue_prv_fusion_math.h"
#include "TueObjFusn_ErrorCodes.h"
#include "TueObjFusn_AAU_Codes.h"
#include "tue_prv_error_management.h"
#include "TueObjFusn_TrackableProps.h"

/**
* \fn bool_t LKF_AddNewTrkbl(TueObjFusn_TrackableType * const pTrkbl,
TueObjFusn_TrackableType const * const pMeas)
* \brief  Initializes the state vector x and the covariance matrix P of a new
track of the Kalman filter.

Only the states are filled which are used for the motion model in pTrkble (e.g.
constant velocity, constant acceleration) and the measurement model of pMeas
(e.g. camera sensor does not provide acceleration states). Which model is used,
is set in the parameter interface.

The sizes of x and P are dependent on the motion model:
- Constant velocity -> x = [4x1], P = [4x4]
- Constant acceleration in x, constant velocity in y -> x = [5x1], P = [5,5]
- Constant acceleration in y (in x it is independent) -> x = [6x1], P = [6x6]

How the P matrix is initialized, is set in the configuration. Possible methods:
- FILTER_P_INIT_EMPTY: full-zero matrix
- FILTER_P_INIT_IDENTITY: identity matrix
- FILTER_P_INIT_VARIANCES: diagonal values are set with the variances of the
sensor track, if they are usable by the measurement model
- FILTER_P_INIT_COVARIANCES: all values except the diagonal ones are filled with
the corresponding covariances of the sensor track, if they are usable by the
measurement model
- FILTER_P_INIT_OW_X_X: set P matrix for value at XX with a constant value
*
* \param  [out] pTrkbl    TueObjFusn_TrackableType * const, initialized Kalman
track
* \param  pMeas           TueObjFusn_TrackableType const * const, track from
sensor
*
* \return  TRUE (ok) or FALSE (error occured)
*/
#define ObjFusn_START_SEC_CODE

/* PRQA S 1532 2 */ /* Function called by external AAU */
uint32 LKF_AddNewTrkbl(
    CONSTP2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pTrkbl,
    CONSTP2CONST(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pMeas) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;

    float32 f32VelX = FLT_ZERO;
    float32 f32VelY = FLT_ZERO;
    float32 f32Tmp = FLT_ZERO;
    float32 f32VarHeading = FLT_ZERO;

#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    /** - check input validity */
    if ((NULL_PTR == pTrkbl) || (NULL_PTR == pMeas)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_LKF, TUEOBJFUSN_AAU_LKF_ADD_NEW_TRKBL);
    } else
#endif
    {
        u32Success |= f32SymMatZeros(&pTrkbl->matP, TUEOBJFUSN_MATRIX_SIZE);

        /** - initialize state vector x according to configuration setting and
         * measurement model of sensor*/
        pTrkbl->vecX.nRows = TUEOBJFUSN_MATRIX_SIZE;
        pTrkbl->vecX.data[TRACKABLE_POSX] = pMeas->vecX.data[TRACKABLE_POSX];
        pTrkbl->vecX.data[TRACKABLE_POSY] = pMeas->vecX.data[TRACKABLE_POSY];
        pTrkbl->vecX.data[TRACKABLE_VELX] = pMeas->vecX.data[TRACKABLE_VELX];
        pTrkbl->vecX.data[TRACKABLE_VELY] = pMeas->vecX.data[TRACKABLE_VELY];
        if (TRACKABLE_ACCX < pMeas->vecX.nRows) {
            /* AccX available and used */
            pTrkbl->vecX.data[TRACKABLE_ACCX] =
                pMeas->vecX.data[TRACKABLE_ACCX];
            if (TRACKABLE_ACCY < pMeas->vecX.nRows) {
                /* AccY available and used*/
                pTrkbl->vecX.data[TRACKABLE_ACCY] =
                    pMeas->vecX.data[TRACKABLE_ACCY];
            } else {
                pTrkbl->vecX.data[TRACKABLE_ACCY] = FLT_ZERO;
            }
        } else {
            pTrkbl->vecX.data[TRACKABLE_ACCX] = FLT_ZERO;
            pTrkbl->vecX.data[TRACKABLE_ACCY] = FLT_ZERO;
        }

        if ((TUE_PRV_FUSION_POS_EPS_ABS <
             pMeas->matP.data[TRACKABLE_INDEX_VARIANCE_POSX]) &&
            (TUE_PRV_FUSION_POS_EPS_ABS <
             pMeas->matP.data[TRACKABLE_INDEX_VARIANCE_POSY]) &&
            (TUE_PRV_FUSION_POS_EPS_ABS <
             pMeas->matP.data[TRACKABLE_INDEX_VARIANCE_VELX]) &&
            (TUE_PRV_FUSION_POS_EPS_ABS <
             pMeas->matP.data[TRACKABLE_INDEX_VARIANCE_VELY])) {
            pTrkbl->matP.data[TRACKABLE_INDEX_VARIANCE_POSX] =
                pMeas->matP.data[TRACKABLE_INDEX_VARIANCE_POSX];
            pTrkbl->matP.data[TRACKABLE_INDEX_VARIANCE_POSY] =
                pMeas->matP.data[TRACKABLE_INDEX_VARIANCE_POSY];
            pTrkbl->matP.data[TRACKABLE_INDEX_VARIANCE_VELX] =
                pMeas->matP.data[TRACKABLE_INDEX_VARIANCE_VELX];
            pTrkbl->matP.data[TRACKABLE_INDEX_VARIANCE_VELY] =
                pMeas->matP.data[TRACKABLE_INDEX_VARIANCE_VELY];

            /** Add covariances */
            pTrkbl->matP.data[TRACKABLE_INDEX_COVARIANCE_POSX_POSY] =
                pMeas->matP.data[TRACKABLE_INDEX_COVARIANCE_POSX_POSY];
            pTrkbl->matP.data[TRACKABLE_INDEX_COVARIANCE_POSX_VELX] =
                pMeas->matP.data[TRACKABLE_INDEX_COVARIANCE_POSX_VELX];
            pTrkbl->matP.data[TRACKABLE_INDEX_COVARIANCE_POSX_VELY] =
                pMeas->matP.data[TRACKABLE_INDEX_COVARIANCE_POSX_VELY];
            pTrkbl->matP.data[TRACKABLE_INDEX_COVARIANCE_POSY_VELX] =
                pMeas->matP.data[TRACKABLE_INDEX_COVARIANCE_POSY_VELX];
            pTrkbl->matP.data[TRACKABLE_INDEX_COVARIANCE_POSY_VELY] =
                pMeas->matP.data[TRACKABLE_INDEX_COVARIANCE_POSY_VELY];
            pTrkbl->matP.data[TRACKABLE_INDEX_COVARIANCE_VELX_VELY] =
                pMeas->matP.data[TRACKABLE_INDEX_COVARIANCE_VELX_VELY];

            /* use initially variance or default value if parameter is not used
             * in measurement system */
            if ((TUE_PRV_FUSION_POS_EPS_ABS <
                 pMeas->matP.data[TRACKABLE_INDEX_VARIANCE_ACCX]) &&
                (TRACKABLE_ACCX < pMeas->matP.u16Size)) {
                pTrkbl->matP.data[TRACKABLE_INDEX_VARIANCE_ACCX] =
                    pMeas->matP.data[TRACKABLE_INDEX_VARIANCE_ACCX];
                pTrkbl->matP.data[TRACKABLE_INDEX_COVARIANCE_POSX_ACCX] =
                    pMeas->matP.data[TRACKABLE_INDEX_COVARIANCE_POSX_ACCX];
                pTrkbl->matP.data[TRACKABLE_INDEX_COVARIANCE_POSY_ACCX] =
                    pMeas->matP.data[TRACKABLE_INDEX_COVARIANCE_POSY_ACCX];
                pTrkbl->matP.data[TRACKABLE_INDEX_COVARIANCE_VELX_ACCX] =
                    pMeas->matP.data[TRACKABLE_INDEX_COVARIANCE_VELX_ACCX];
                pTrkbl->matP.data[TRACKABLE_INDEX_COVARIANCE_VELY_ACCX] =
                    pMeas->matP.data[TRACKABLE_INDEX_COVARIANCE_VELY_ACCX];
            } else {
                pTrkbl->matP.data[TRACKABLE_INDEX_VARIANCE_ACCX] =
                    TUE_PRV_LKF_FILTER_P_DEFAULT_DIAG;
                pTrkbl->matP.data[TRACKABLE_INDEX_COVARIANCE_POSX_ACCX] =
                    FLT_ZERO;
                pTrkbl->matP.data[TRACKABLE_INDEX_COVARIANCE_POSY_ACCX] =
                    FLT_ZERO;
                pTrkbl->matP.data[TRACKABLE_INDEX_COVARIANCE_VELX_ACCX] =
                    FLT_ZERO;
                pTrkbl->matP.data[TRACKABLE_INDEX_COVARIANCE_VELY_ACCX] =
                    FLT_ZERO;
            }

            if ((TUE_PRV_FUSION_POS_EPS_ABS <
                 pMeas->matP.data[TRACKABLE_INDEX_VARIANCE_ACCY]) &&
                (TRACKABLE_ACCY < pMeas->matP.u16Size)) {
                pTrkbl->matP.data[TRACKABLE_INDEX_VARIANCE_ACCY] =
                    pMeas->matP.data[TRACKABLE_INDEX_VARIANCE_ACCY];
                pTrkbl->matP.data[TRACKABLE_INDEX_COVARIANCE_POSX_ACCY] =
                    pMeas->matP.data[TRACKABLE_INDEX_COVARIANCE_POSX_ACCY];
                pTrkbl->matP.data[TRACKABLE_INDEX_COVARIANCE_POSY_ACCY] =
                    pMeas->matP.data[TRACKABLE_INDEX_COVARIANCE_POSY_ACCY];
                pTrkbl->matP.data[TRACKABLE_INDEX_COVARIANCE_VELX_ACCY] =
                    pMeas->matP.data[TRACKABLE_INDEX_COVARIANCE_VELX_ACCY];
                pTrkbl->matP.data[TRACKABLE_INDEX_COVARIANCE_VELY_ACCY] =
                    pMeas->matP.data[TRACKABLE_INDEX_COVARIANCE_VELY_ACCY];
                pTrkbl->matP.data[TRACKABLE_INDEX_COVARIANCE_ACCX_ACCY] =
                    pMeas->matP.data[TRACKABLE_INDEX_COVARIANCE_ACCX_ACCY];
            } else {
                pTrkbl->matP.data[TRACKABLE_INDEX_VARIANCE_ACCY] =
                    TUE_PRV_LKF_FILTER_P_DEFAULT_DIAG;
                pTrkbl->matP.data[TRACKABLE_INDEX_COVARIANCE_POSX_ACCY] =
                    FLT_ZERO;
                pTrkbl->matP.data[TRACKABLE_INDEX_COVARIANCE_POSY_ACCY] =
                    FLT_ZERO;
                pTrkbl->matP.data[TRACKABLE_INDEX_COVARIANCE_VELX_ACCY] =
                    FLT_ZERO;
                pTrkbl->matP.data[TRACKABLE_INDEX_COVARIANCE_VELY_ACCY] =
                    FLT_ZERO;
                pTrkbl->matP.data[TRACKABLE_INDEX_COVARIANCE_ACCX_ACCY] =
                    FLT_ZERO;
            }

            /* Init Heading */
            if (TRACKABLE_VELY <
                pMeas->vecX.nRows) {
                /* VelX and VelY has to be available */
                f32VelX = (pTrkbl->vecX).data[TRACKABLE_VELX];
                f32VelY = (pTrkbl->vecX).data[TRACKABLE_VELY];
                f32Tmp = (f32VelX * f32VelX) *
                         ((pTrkbl->matP).data[TRACKABLE_INDEX_VARIANCE_VELY]);
                f32Tmp += (f32VelY * f32VelY) *
                          ((pTrkbl->matP).data[TRACKABLE_INDEX_VARIANCE_VELX]);
                f32VarHeading = tue_prv_fusion_sqrt(f32Tmp);
                f32Tmp = (f32VelX * f32VelX) + (f32VelY * f32VelY);

                if (f32Tmp > TUE_PRV_COORDINATED_TURN_MIN_TARGET_SPEED) {
                    pTrkbl->f32HeadingVar = tue_prv_fusion_min_max_F32(
                        f32VarHeading / f32Tmp,
                        TUEOBJFUSN_TRACKABLE_F32HEADINGVAR_MIN,
                        TUEOBJFUSN_TRACKABLE_F32HEADINGVAR_MAX);
                    pTrkbl->f32Heading = tue_prv_fusion_atan2(f32VelY, f32VelX);
                } else {
                    /* MISRA */
                }
            } else {
                /* Already initialized with default values */
            }
        } else {
            pTrkbl->matP.data[TRACKABLE_INDEX_VARIANCE_POSX] =
                TUE_PRV_LKF_FILTER_P_DEFAULT_DIAG;
            pTrkbl->matP.data[TRACKABLE_INDEX_VARIANCE_POSY] =
                TUE_PRV_LKF_FILTER_P_DEFAULT_DIAG;
            pTrkbl->matP.data[TRACKABLE_INDEX_VARIANCE_VELX] =
                TUE_PRV_LKF_FILTER_P_DEFAULT_DIAG;
            pTrkbl->matP.data[TRACKABLE_INDEX_VARIANCE_VELY] =
                TUE_PRV_LKF_FILTER_P_DEFAULT_DIAG;
            pTrkbl->matP.data[TRACKABLE_INDEX_VARIANCE_ACCX] =
                TUE_PRV_LKF_FILTER_P_DEFAULT_DIAG;
            pTrkbl->matP.data[TRACKABLE_INDEX_VARIANCE_ACCY] =
                TUE_PRV_LKF_FILTER_P_DEFAULT_DIAG;
        }
    }

    return u32Success;
}

/**
 * \fn bool_t LKF_DoPredict(TueObjFusn_TrackableType * const pTrkbl,
 * stf32Matrix_t const * const pMatA, stf32Matrix_t const * const pMatQ) \brief
 * predict the state vector x for a specified time using the motion mode (A) and
 * increases the uncertainty (covariance matrix p) depending on the motion model
 * (A) and additional process noise (Q)
 *
 *
 * \param  [out] pTrkbl    TueObjFusn_TrackableType * const, trackable which
 * shall be predicted \param  pMatA           stf32Matrix_t * const, dynamic
 * model matrix A \param  pMatQ           stf32Matrix_t * const, process noise
 * matrix Q
 *
 * \return  TRUE (ok) or FALSE (error occured)
 */

/* PRQA S 1532 2 */ /* Function called by external AAU */
uint32 LKF_DoPredict(CONSTP2VAR(TueObjFusn_TrackableType,
                                AUTOMATIC,
                                ObjFusn_VAR_NOINIT) pTrkbl,
                     const float32 f32dT) {
    /** function process: */
    VAR(stf32SymMatrix_t, ObjFusn_VAR_NOINIT) matQ;
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    const float32 f32dTSquared = FLT_ONE_HALF * f32dT * f32dT;
    const float32 f32dTpow2 = f32dT * f32dT;
    const float32 f32dTpow3 = f32dTpow2 * f32dT;
    const float32 f32dTpow4 = f32dTpow3 * f32dT;
    const float32 f32dTpow5 = f32dTpow4 * f32dT;
    float32 f32PrefactorNoiseX; /* = q for x direction in equation on top */
    float32 f32PrefactorNoiseY; /* = q for y direction in equation on top */

    matQ.u16Size = pTrkbl->vecX.nRows;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
    if (pTrkbl->vecX.nRows < TRACKABLE_VELY) {
        u32Success = TUEOBJFUSN_ERROR_INPUT_NOT_ENOUGH_STATES;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_INPUT_NOT_ENOUGH_STATES, TUEOBJFUSN_AAU_LKF,
            TUEOBJFUSN_AAU_LKF_COORDINATED_TURN_DO_PREDICT);
    } else
#endif
    {
        /* Get prefactors depending on class type */
        /**
         *  This if statements follows the idea that vehicles and motorbikes
         * typically move on a longitudinal trajectory whereas pedestrians and
         * bicycles may also move on lateral trajectories Especially when
         * focussing on the possible camera classifiers, bicycles are typically
         * detected when moving on lateral trajectories
         */
        if (((pTrkbl->u16Class &
              TUEOBJFUSN_TRACKABLE_U16CLASS_VEHICLE_UNKNOWN) > 0u) ||
            (pTrkbl->u16Class ==
             TUEOBJFUSN_TRACKABLE_U16CLASS_2WHEELER_MOTORCYCLE)) {
            f32PrefactorNoiseX = Fusion_get_f32VehicleVarianceInXForQ();
            f32PrefactorNoiseY = Fusion_get_f32VehicleVarianceInYForQ();
        } else {
            f32PrefactorNoiseX = Fusion_get_f32PedestrianVarianceInXForQ();
            f32PrefactorNoiseY = Fusion_get_f32PedestrianVarianceInYForQ();
        }

        if (TRACKABLE_ACCY < pTrkbl->vecX.nRows) {
            pTrkbl->vecX.data[TRACKABLE_POSX] =
                (pTrkbl->vecX.data[TRACKABLE_POSX] +
                 (f32dT * pTrkbl->vecX.data[TRACKABLE_VELX])) +
                (f32dTSquared * pTrkbl->vecX.data[TRACKABLE_ACCX]);
            pTrkbl->vecX.data[TRACKABLE_POSY] =
                (pTrkbl->vecX.data[TRACKABLE_POSY] +
                 (f32dT * pTrkbl->vecX.data[TRACKABLE_VELY])) +
                (f32dTSquared * pTrkbl->vecX.data[TRACKABLE_ACCY]);
            pTrkbl->vecX.data[TRACKABLE_VELX] =
                pTrkbl->vecX.data[TRACKABLE_VELX] +
                (f32dT * pTrkbl->vecX.data[TRACKABLE_ACCX]);
            pTrkbl->vecX.data[TRACKABLE_VELY] =
                pTrkbl->vecX.data[TRACKABLE_VELY] +
                (f32dT * pTrkbl->vecX.data[TRACKABLE_ACCY]);

            /* Init Q Matrix for vx, vy, ax ay */
            /* PRQA S 3121 1 */ /* Mathematical formula */
            matQ.data[TRACKABLE_INDEX_VARIANCE_POSX] =
                0.05f * f32PrefactorNoiseX * f32dTpow5;
            /* PRQA S 3120 */   /* Mathematical formula */
            /* PRQA S 3121 1 */ /* Mathematical formula */
            matQ.data[TRACKABLE_INDEX_VARIANCE_POSY] =
                0.05f * f32PrefactorNoiseY * f32dTpow5;
            /* PRQA S 3120 */ /* Mathematical formula */
            matQ.data[TRACKABLE_INDEX_COVARIANCE_POSX_VELX] =
                (f32PrefactorNoiseX * f32dTpow4) / FLT_EIGHT;
            matQ.data[TRACKABLE_INDEX_VARIANCE_VELX] =
                (f32PrefactorNoiseX * f32dTpow3) / FLT_THREE;
            matQ.data[TRACKABLE_INDEX_COVARIANCE_POSY_VELY] =
                (f32PrefactorNoiseY * f32dTpow4) / FLT_EIGHT;
            matQ.data[TRACKABLE_INDEX_VARIANCE_VELY] =
                (f32PrefactorNoiseY * f32dTpow3) / FLT_THREE;
            matQ.data[TRACKABLE_INDEX_COVARIANCE_POSX_ACCX] =
                (f32PrefactorNoiseX * f32dTpow3) / FLT_SIX;
            matQ.data[TRACKABLE_INDEX_COVARIANCE_VELX_ACCX] =
                FLT_ONE_HALF * f32PrefactorNoiseX * f32dTpow2;
            matQ.data[TRACKABLE_INDEX_VARIANCE_ACCX] =
                f32PrefactorNoiseX * f32dT;
            matQ.data[TRACKABLE_INDEX_COVARIANCE_POSY_ACCY] =
                (f32PrefactorNoiseY * f32dTpow3) / FLT_SIX;
            matQ.data[TRACKABLE_INDEX_COVARIANCE_VELY_ACCY] =
                FLT_ONE_HALF * f32PrefactorNoiseY * f32dTpow2;
            matQ.data[TRACKABLE_INDEX_VARIANCE_ACCY] =
                f32PrefactorNoiseY * f32dT;
        } else if (TRACKABLE_ACCX < pTrkbl->vecX.nRows) {
            pTrkbl->vecX.data[TRACKABLE_POSX] =
                (pTrkbl->vecX.data[TRACKABLE_POSX] +
                 (f32dT * pTrkbl->vecX.data[TRACKABLE_VELX])) +
                (f32dTSquared * pTrkbl->vecX.data[TRACKABLE_ACCX]);
            pTrkbl->vecX.data[TRACKABLE_POSY] =
                pTrkbl->vecX.data[TRACKABLE_POSY] +
                (f32dT * pTrkbl->vecX.data[TRACKABLE_VELY]);
            pTrkbl->vecX.data[TRACKABLE_VELX] =
                pTrkbl->vecX.data[TRACKABLE_VELX] +
                (f32dT * pTrkbl->vecX.data[TRACKABLE_ACCX]);

            /* Init Q matrix for vx, vy, ax */
            /* PRQA S 3121 1 */ /* Mathematical formula */
            matQ.data[TRACKABLE_INDEX_VARIANCE_POSX] =
                0.05f * f32PrefactorNoiseX * f32dTpow5;
            /* PRQA S 3120 */ /* Mathematical formula */
            matQ.data[TRACKABLE_INDEX_VARIANCE_POSY] =
                (f32PrefactorNoiseY * f32dTpow3) / FLT_THREE;
            matQ.data[TRACKABLE_INDEX_COVARIANCE_POSX_VELX] =
                (f32PrefactorNoiseX * f32dTpow4) / FLT_EIGHT;
            matQ.data[TRACKABLE_INDEX_VARIANCE_VELX] =
                (f32PrefactorNoiseX * f32dTpow3) / FLT_THREE;
            matQ.data[TRACKABLE_INDEX_COVARIANCE_POSY_VELY] =
                FLT_ONE_HALF * f32PrefactorNoiseY * f32dTpow2;
            matQ.data[TRACKABLE_INDEX_VARIANCE_VELY] =
                f32PrefactorNoiseY * f32dT;
            matQ.data[TRACKABLE_INDEX_COVARIANCE_POSX_ACCX] =
                (f32PrefactorNoiseX * f32dTpow3) / FLT_SIX;
            matQ.data[TRACKABLE_INDEX_COVARIANCE_VELX_ACCX] =
                FLT_ONE_HALF * f32PrefactorNoiseX * f32dTpow2;
            matQ.data[TRACKABLE_INDEX_VARIANCE_ACCX] =
                f32PrefactorNoiseX * f32dT;
            matQ.data[TRACKABLE_INDEX_COVARIANCE_POSY_ACCY] = FLT_ZERO;
            matQ.data[TRACKABLE_INDEX_COVARIANCE_VELY_ACCY] = FLT_ZERO;
            matQ.data[TRACKABLE_INDEX_VARIANCE_ACCY] = FLT_ZERO;
        } else {
            pTrkbl->vecX.data[TRACKABLE_POSX] =
                pTrkbl->vecX.data[TRACKABLE_POSX] +
                (f32dT * pTrkbl->vecX.data[TRACKABLE_VELX]);
            pTrkbl->vecX.data[TRACKABLE_POSY] =
                pTrkbl->vecX.data[TRACKABLE_POSY] +
                (f32dT * pTrkbl->vecX.data[TRACKABLE_VELY]);

            /* Init Q Matrix for vx, vy */
            matQ.data[TRACKABLE_INDEX_VARIANCE_POSX] =
                (f32PrefactorNoiseX * f32dTpow3) / FLT_THREE;
            matQ.data[TRACKABLE_INDEX_VARIANCE_POSY] =
                (f32PrefactorNoiseY * f32dTpow3) / FLT_THREE;
            matQ.data[TRACKABLE_INDEX_COVARIANCE_POSX_VELX] =
                FLT_ONE_HALF * f32PrefactorNoiseX * f32dTpow2;
            matQ.data[TRACKABLE_INDEX_VARIANCE_VELX] =
                f32PrefactorNoiseX * f32dT;
            matQ.data[TRACKABLE_INDEX_COVARIANCE_POSY_VELY] =
                FLT_ONE_HALF * f32PrefactorNoiseY * f32dTpow2;
            matQ.data[TRACKABLE_INDEX_VARIANCE_VELY] =
                f32PrefactorNoiseY * f32dT;

            matQ.data[TRACKABLE_INDEX_COVARIANCE_POSX_ACCX] = FLT_ZERO;
            matQ.data[TRACKABLE_INDEX_COVARIANCE_VELX_ACCX] = FLT_ZERO;
            matQ.data[TRACKABLE_INDEX_VARIANCE_ACCX] = FLT_ZERO;
            matQ.data[TRACKABLE_INDEX_COVARIANCE_POSY_ACCY] = FLT_ZERO;
            matQ.data[TRACKABLE_INDEX_COVARIANCE_VELY_ACCY] = FLT_ZERO;
            matQ.data[TRACKABLE_INDEX_VARIANCE_ACCY] = FLT_ZERO;
        }

        matQ.data[TRACKABLE_INDEX_COVARIANCE_POSX_POSY] = FLT_ZERO;
        matQ.data[TRACKABLE_INDEX_COVARIANCE_POSY_VELX] = FLT_ZERO;
        matQ.data[TRACKABLE_INDEX_COVARIANCE_POSX_VELY] = FLT_ZERO;
        matQ.data[TRACKABLE_INDEX_COVARIANCE_VELX_VELY] = FLT_ZERO;
        matQ.data[TRACKABLE_INDEX_COVARIANCE_POSY_ACCX] = FLT_ZERO;
        matQ.data[TRACKABLE_INDEX_COVARIANCE_VELY_ACCX] = FLT_ZERO;
        matQ.data[TRACKABLE_INDEX_COVARIANCE_POSX_ACCY] = FLT_ZERO;
        matQ.data[TRACKABLE_INDEX_COVARIANCE_VELX_ACCY] = FLT_ZERO;
        matQ.data[TRACKABLE_INDEX_COVARIANCE_ACCX_ACCY] = FLT_ZERO;

        /** - Calculate new covariance matrix P according to LKF equations */
        /*   P = A * P * A' + Q; */
        u32Success = LKF_PredictSymMat(&pTrkbl->matP, f32dT);
        u32Success |= f32SymMatAddSymMat(&matQ, &pTrkbl->matP, &pTrkbl->matP);
    }

    return u32Success;
}

/**
 * \fn bool_t LKF_DoCorrect(TueObjFusn_TrackableType * const pTrkbl,
 * TueObjFusn_TrackableType const * const pMeas) \brief computes the Kalman
 * filter update according to LKF equations
 *
 * The state vector x and the covariance matrix P  of pTrkble are updated by the
 * sensor measurement data z and R
 * (= x and P of pMeas) according to LKF equations.
 *
 * The necessary measurement noise matrix R is set by using only the values from
 * the covariance matrix P of pMeas which can be used according to the dynamic
 * model and the sensor measurement model.
 *
 *
 * \param  [out] pTrkbl   TueObjFusn_TrackableType * const, LKF track which
 * shall be adapted \param  pMeas          TueObjFusn_TrackableType const *
 * const, sensor track whose measured data shall update the LKF track
 *
 * \return  TRUE (ok) or FALSE (error occured)
 */

/* PRQA S 1532 2 */ /* Function called by external AAU */
uint32 LKF_DoCorrect(
    CONSTP2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pTrkbl,
    CONSTP2CONST(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pMeas) {
    /** function process: */
    uint16 u16nRows;
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;

    /* buffer matrices for a better overview during calculations */
    VAR(stf32SymMatrix_t, ObjFusn_VAR_NOINIT) _matPSym;
    VAR(stf32Matrix_t, ObjFusn_VAR_NOINIT) _matP;

    VAR(stf32Matrix_t, ObjFusn_VAR_NOINIT) matS;
    VAR(stf32Vec_t, ObjFusn_VAR_NOINIT) vecKZ_hat;
    VAR(stf32Vec_t, ObjFusn_VAR_NOINIT) vecZ_hat;
    VAR(stf32Matrix_t, ObjFusn_VAR_NOINIT) matEye;
    VAR(stf32Matrix_t, ObjFusn_VAR_NOINIT) matEyeK;
    VAR(stf32Matrix_t, ObjFusn_VAR_NOINIT) matX_buf;
    VAR(stf32Matrix_t, ObjFusn_VAR_NOINIT) matEyeKT;
    VAR(stf32Matrix_t, ObjFusn_VAR_NOINIT) matK;
    VAR(stf32Matrix_t, ObjFusn_VAR_NOINIT) matKR;
    VAR(stf32Matrix_t, ObjFusn_VAR_NOINIT) matKT;
    VAR(stf32Matrix_t, ObjFusn_VAR_NOINIT) matKRKT;
    VAR(stf32Matrix_t, ObjFusn_VAR_NOINIT) matEyePEyeT;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    /** - check input validity*/
    if ((NULL_PTR == pTrkbl) || (NULL_PTR == pMeas)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(u32Success, TUEOBJFUSN_AAU_LKF,
                                                TUEOBJFUSN_AAU_LKF_DO_CORRECT);
    } else if (pMeas == pTrkbl) {
        u32Success = TUEOBJFUSN_ERROR_INVALID_POINTER;
        (void)tue_prv_error_management_addError(u32Success, TUEOBJFUSN_AAU_LKF,
                                                TUEOBJFUSN_AAU_LKF_DO_CORRECT);
    } else
#endif
        /** - check consistency of coordinate system: coordinate system of the
           LKF track and sensor track shall be the same*/
        if (pTrkbl->u8CoordSystem != pMeas->u8CoordSystem) {
        u32Success = TUEOBJFUSN_ERROR_COORD_SYSTEM_MISMATCH;
        (void)tue_prv_error_management_addError(u32Success, TUEOBJFUSN_AAU_LKF,
                                                TUEOBJFUSN_AAU_LKF_DO_CORRECT);
    } else
#endif
    {
        u16nRows = pTrkbl->vecX.nRows;

        /* initializes all matrices due to calculation errors */
        // comment unsed initial process only for CPU saving by guotao 20190910
        // f32MatInit(&_matP);
        (void)f32MatEye(&matEye, u16nRows, u16nRows);
        // f32MatInit(&matS);
        // f32VecInit(&vecZ_hat);
        // f32VecInit(&vecKZ_hat);
        // f32MatInit(&matEyeK);
        // f32MatInit(&matX_buf);
        // f32MatInit(&matEyeKT);
        // u32Success = f32MatZeros(&matK,
        // pTrkbl->matP.u16Size,pTrkbl->matP.u16Size); // has to be initialized
        // with zeros f32MatInit(&matKR); f32MatInit(&matKT);
        // f32MatInit(&matKRKT);
        // f32MatInit(&matEyePEyeT);

        /** - calculate innovation z_hat */
        /*   z_hat = z - H * x; */
        /*  in Kalman case H only takes subset of x -> so in this case from
         * vector z only available elements in x are taken */
        u32Success |= f32VecSubPart(&pMeas->vecX, &pTrkbl->vecX, &vecZ_hat);

        /** - calculate covariance matrix of innovation S */
        /*   S = H * P * H' + R; */
        /*   size of R is used and equivalent rows of P */
        u32Success |= f32SymMatAddSymMatPart(&pMeas->matP, &pTrkbl->matP,
                                             &_matPSym); /* S = HPHT + R */
        u32Success |= f32SymMatToMat(&_matPSym, &matS);

        u32Success |= f32MatInv(&(matS)); /* S = S^-1 */

        /** - calculate Kalman gain vector K */
        /*   K = P * H' * inv(S); */
        u32Success |= f32SymMatToMat(&pTrkbl->matP, &_matP);
        if (pTrkbl->matP.u16Size > pMeas->matP.u16Size) {
            _matP.nCols =
                (pMeas->matP)
                    .u16Size; // This is equivalent to P * H' (exclude elements
                                //  of P which are not in R resp S
            u32Success |= f32MatMul(&_matP, &matS, &matK);
            _matP.nCols = (pTrkbl->matP).u16Size; // reset back sicne _matP is
                                                    //  required further done
        } else {
            u32Success |= f32MatMul(&_matP, &matS, &matK);
        }

        /** - calculate new state vector x */
        /*   X = X + K * z_hat; */
        u32Success |=
            f32MatMulVec(&matK, &vecZ_hat, &vecKZ_hat); /* KZ_hat = K * Z_hat */
        u32Success |= f32VecAdd(&pTrkbl->vecX, &vecKZ_hat,
                                &pTrkbl->vecX); /* X = X + KZ_hat */

        /** - calculate new covariance matrix P */
        matK.nCols = pTrkbl->matP.u16Size;
        u32Success |= f32MatSub(
            &matEye, &matK, &matEyeK); /* I - K * H -> modifaction to matK */
        u32Success |=
            f32MatMul(&matEyeK, &_matP, &matX_buf); /* (I - K  * H) * P */
        u32Success |= f32MatToSymMat(&matX_buf, &(pTrkbl->matP));
        // changes this only for cpu saving by guotao 20190910
        // u32Success |= f32MatTranspose(&matEyeK, &matEyeKT); /* (I - K * H )^T
        // */ u32Success |= f32MatMul(&matX_buf, &matEyeKT, &matEyePEyeT); /* (I
        // - K * H ) * P * (I - K * H)^T */ matK.nCols = pMeas->matP.u16Size; /*
        // convert back to measurment size */ u32Success |=
        // LKF_Calculate_KR(&(pMeas->matP), &matK, &matKR);
        ////u32Success |= f32MatMul(&matK, &matR, &matKR);  /* K * R */
        // u32Success |= f32MatTranspose(&(matK), &matKT); /* K^T */
        // u32Success |= f32MatMul(&matKR, &matKT, &matKRKT); /* K * R * K^T */
        // u32Success |= f32MatAdd(&matEyePEyeT, &matKRKT, &_matP); /* (I - K *
        // H) * P * (I - K* H)^T + K * R * K^T */ u32Success |=
        // f32MatToSymMat(&_matP, &(pTrkbl->matP));
    }

    return u32Success;
}

/* PRQA S 1505 2*/ /* Library Function */
LOCAL uint32 LKF_Calculate_KR(
    CONSTP2CONST(stf32SymMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) matP,
    CONSTP2CONST(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) matK,
    CONSTP2VAR(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) matKR) {
    /* At the moment only variances are used from measurement noise - change
     * this? */
    uint16 u16i;
    uint16 u16j;
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    /* PRQA S 3204 1 */ // variable may be set to other values depending on
                        //    run-time check activation
    float32 af32Variances[TUEOBJFUSN_MATRIX_SIZE];

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
    if (matK->nCols != matP->u16Size) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_MATRIX_SIZE_NOT_EQUAL, TUEOBJFUSN_AAU_LKF,
            TUEOBJFUSN_AAU_LKF_CALCULATE_KR);
    } else if ((matK->nCols > TUEOBJFUSN_MATRIX_SIZE) ||
               (matK->nRows > TUEOBJFUSN_MATRIX_SIZE)) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE, TUEOBJFUSN_AAU_LKF,
            TUEOBJFUSN_AAU_LKF_CALCULATE_KR);
    } else if (matP->u16Size < TRACKABLE_VELY) {
        u32Success = TUEOBJFUSN_ERROR_INPUT_NOT_ENOUGH_STATES;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE, TUEOBJFUSN_AAU_LKF,
            TUEOBJFUSN_AAU_LKF_CALCULATE_KR);
    } else
#endif
    {
        af32Variances[TRACKABLE_POSX] =
            matP->data[TRACKABLE_INDEX_VARIANCE_POSX];
        af32Variances[TRACKABLE_POSY] =
            matP->data[TRACKABLE_INDEX_VARIANCE_POSY];
        af32Variances[TRACKABLE_VELX] =
            matP->data[TRACKABLE_INDEX_VARIANCE_VELX];
        af32Variances[TRACKABLE_VELY] =
            matP->data[TRACKABLE_INDEX_VARIANCE_VELY];
        af32Variances[TRACKABLE_ACCX] =
            matP->data[TRACKABLE_INDEX_VARIANCE_ACCX];
        af32Variances[TRACKABLE_ACCY] =
            matP->data[TRACKABLE_INDEX_VARIANCE_ACCY];

        for (u16i = 0u; u16i < matK->nRows; u16i++) {
            float32* tempKR;

            const float32* tempK;
            tempKR = &matKR->data[u16i][0];
            tempK = &matK->data[u16i][0];
            for (u16j = 0u; u16j < matP->u16Size; u16j++) {
                tempKR[u16j] = tempK[u16j] * af32Variances[u16j];
            }
        }

        matKR->nCols = matP->u16Size;
        matKR->nRows = matK->nRows;
    }

    return u32Success;
}

uint32 LKF_PredictSymMat(CONSTP2VAR(stf32SymMatrix_t,
                                    AUTOMATIC,
                                    ObjFusn_VAR_NOINIT) A,
                         CONST(float32, ObjFusn_VAR_NOINIT) f32dt) {
    uint32 u32Success;

    float32 f32Sum05;
    float32 f32Sum04;
    float32 f32Sum03;
    float32 f32Sum02;
    float32 f32Sum15;
    float32 f32Sum14;
    float32 f32Sum13;
    float32 f32Sum12;

    const float32 f32dtsqhalf = FLT_ONE_HALF * (f32dt * f32dt);

    const float32 f32Value00 = A->data[TRACKABLE_INDEX_VARIANCE_POSX];
    const float32 f32Value01 = A->data[TRACKABLE_INDEX_COVARIANCE_POSX_POSY];
    const float32 f32Value11 = A->data[TRACKABLE_INDEX_VARIANCE_POSY];
    const float32 f32Value02 = A->data[TRACKABLE_INDEX_COVARIANCE_POSX_VELX];
    const float32 f32Value12 = A->data[TRACKABLE_INDEX_COVARIANCE_POSY_VELX];
    const float32 f32Value22 = A->data[TRACKABLE_INDEX_VARIANCE_VELX];
    const float32 f32Value03 = A->data[TRACKABLE_INDEX_COVARIANCE_POSX_VELY];
    const float32 f32Value13 = A->data[TRACKABLE_INDEX_COVARIANCE_POSY_VELY];
    const float32 f32Value32 = A->data[TRACKABLE_INDEX_COVARIANCE_VELX_VELY];
    const float32 f32Value33 = A->data[TRACKABLE_INDEX_VARIANCE_VELY];
    const float32 f32Value04 = A->data[TRACKABLE_INDEX_COVARIANCE_POSX_ACCX];
    const float32 f32Value14 = A->data[TRACKABLE_INDEX_COVARIANCE_POSY_ACCX];
    const float32 f32Value24 = A->data[TRACKABLE_INDEX_COVARIANCE_VELX_ACCX];
    const float32 f32Value34 = A->data[TRACKABLE_INDEX_COVARIANCE_VELY_ACCX];
    const float32 f32Value44 = A->data[TRACKABLE_INDEX_VARIANCE_ACCX];
    const float32 f32Value05 = A->data[TRACKABLE_INDEX_COVARIANCE_POSX_ACCY];
    const float32 f32Value15 = A->data[TRACKABLE_INDEX_COVARIANCE_POSY_ACCY];
    const float32 f32Value25 = A->data[TRACKABLE_INDEX_COVARIANCE_VELX_ACCY];
    const float32 f32Value35 = A->data[TRACKABLE_INDEX_COVARIANCE_VELY_ACCY];
    const float32 f32Value54 = A->data[TRACKABLE_INDEX_COVARIANCE_ACCX_ACCY];
    const float32 f32Value55 = A->data[TRACKABLE_INDEX_VARIANCE_ACCY];

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
    if (A->u16Size > TUEOBJFUSN_MATRIX_SIZE) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_PREDICT_SYMMAT);
    } else
#endif
    {
        if (A->u16Size > TRACKABLE_ACCY) {
            f32Sum05 =
                (f32Value54 * f32dtsqhalf) + (f32Value25 * f32dt) + f32Value05;
            f32Sum15 =
                (f32Value55 * f32dtsqhalf) + (f32Value35 * f32dt) + f32Value15;
            f32Sum12 = f32dtsqhalf * f32Value25;
            f32Sum13 = f32dtsqhalf * f32Value35;
            f32Sum14 = f32Value54 * f32dtsqhalf;
            A->data[TRACKABLE_INDEX_VARIANCE_POSY] = f32dtsqhalf * f32Value15;
            A->data[TRACKABLE_INDEX_VARIANCE_VELY] = f32Value35 * f32dt;
            A->data[TRACKABLE_INDEX_COVARIANCE_VELY_ACCX] = f32Value54 * f32dt;
            A->data[TRACKABLE_INDEX_COVARIANCE_POSX_ACCY] = f32Sum05;
            A->data[TRACKABLE_INDEX_COVARIANCE_POSY_ACCY] = f32Sum15;
            A->data[TRACKABLE_INDEX_COVARIANCE_VELX_ACCY] =
                (f32Value54 * f32dt) + f32Value25;
            A->data[TRACKABLE_INDEX_COVARIANCE_VELY_ACCY] =
                (f32Value55 * f32dt) + f32Value35;
            A->data[TRACKABLE_INDEX_COVARIANCE_ACCX_ACCY] = f32Value54;
            A->data[TRACKABLE_INDEX_VARIANCE_ACCY] = f32Value55;
        } else {
            f32Sum05 = FLT_ZERO;
            f32Sum12 = FLT_ZERO;
            f32Sum13 = FLT_ZERO;
            f32Sum14 = FLT_ZERO;
            f32Sum15 = FLT_ZERO;

            A->data[TRACKABLE_INDEX_VARIANCE_POSY] = FLT_ZERO;
            A->data[TRACKABLE_INDEX_VARIANCE_VELY] = FLT_ZERO;
            A->data[TRACKABLE_INDEX_COVARIANCE_VELY_ACCX] = FLT_ZERO;
            A->data[TRACKABLE_INDEX_COVARIANCE_POSX_ACCY] = FLT_ZERO;
            A->data[TRACKABLE_INDEX_COVARIANCE_POSY_ACCY] = FLT_ZERO;
            A->data[TRACKABLE_INDEX_COVARIANCE_VELX_ACCY] = FLT_ZERO;
            A->data[TRACKABLE_INDEX_COVARIANCE_VELY_ACCY] = FLT_ZERO;
            A->data[TRACKABLE_INDEX_COVARIANCE_ACCX_ACCY] = FLT_ZERO;
            A->data[TRACKABLE_INDEX_VARIANCE_ACCY] = FLT_ZERO;
        }

        if (A->u16Size > TRACKABLE_ACCX) {
            f32Sum04 =
                (f32Value44 * f32dtsqhalf) + (f32Value24 * f32dt) + f32Value04;
            f32Sum14 += (f32Value34 * f32dt) + f32Value14;
            f32Sum02 = f32dtsqhalf * f32Value24;
            f32Sum03 = f32dtsqhalf * f32Value34;
            A->data[TRACKABLE_INDEX_VARIANCE_POSX] = f32dtsqhalf * f32Value04;
            A->data[TRACKABLE_INDEX_COVARIANCE_POSX_POSY] =
                f32dtsqhalf * f32Value14;
            A->data[TRACKABLE_INDEX_VARIANCE_VELX] = f32Value24 * f32dt;
            A->data[TRACKABLE_INDEX_COVARIANCE_VELX_VELY] = f32Value34 * f32dt;
            A->data[TRACKABLE_INDEX_COVARIANCE_POSX_ACCX] = f32Sum04;
            A->data[TRACKABLE_INDEX_COVARIANCE_POSY_ACCX] = f32Sum14;
            A->data[TRACKABLE_INDEX_COVARIANCE_VELX_ACCX] =
                (f32Value44 * f32dt) + f32Value24;
            A->data[TRACKABLE_INDEX_COVARIANCE_VELY_ACCX] += f32Value34;
            A->data[TRACKABLE_INDEX_VARIANCE_ACCX] = f32Value44;
        } else {
            f32Sum02 = FLT_ZERO;
            f32Sum03 = FLT_ZERO;
            f32Sum04 = FLT_ZERO;

            A->data[TRACKABLE_INDEX_VARIANCE_POSX] = FLT_ZERO;
            A->data[TRACKABLE_INDEX_COVARIANCE_POSX_POSY] = FLT_ZERO;
            A->data[TRACKABLE_INDEX_VARIANCE_VELX] = FLT_ZERO;
            A->data[TRACKABLE_INDEX_COVARIANCE_VELX_VELY] = FLT_ZERO;
            A->data[TRACKABLE_INDEX_COVARIANCE_POSX_ACCX] = FLT_ZERO;
            A->data[TRACKABLE_INDEX_COVARIANCE_POSY_ACCX] = FLT_ZERO;
            A->data[TRACKABLE_INDEX_COVARIANCE_VELX_ACCX] = FLT_ZERO;
            A->data[TRACKABLE_INDEX_COVARIANCE_VELY_ACCX] = FLT_ZERO;
            A->data[TRACKABLE_INDEX_VARIANCE_ACCX] = FLT_ZERO;
        }

        f32Sum02 += (f32Value22 * f32dt) + f32Value02;
        f32Sum03 += (f32Value32 * f32dt) + f32Value03;
        f32Sum12 += (f32Value32 * f32dt) + f32Value12;
        f32Sum13 += (f32Value33 * f32dt) + f32Value13;

        A->data[TRACKABLE_INDEX_VARIANCE_POSX] +=
            (f32Sum04 * f32dtsqhalf) + (f32Sum02 * f32dt) +
            (f32dt * f32Value02) + f32Value00;
        A->data[TRACKABLE_INDEX_COVARIANCE_POSX_POSY] +=
            (f32Sum05 * f32dtsqhalf) + (f32Sum03 * f32dt) +
            (f32dt * f32Value12) + f32Value01;
        A->data[TRACKABLE_INDEX_VARIANCE_POSY] +=
            (f32Sum15 * f32dtsqhalf) + (f32Sum13 * f32dt) +
            (f32dt * f32Value13) + f32Value11;
        A->data[TRACKABLE_INDEX_COVARIANCE_POSX_VELX] =
            (f32Sum04 * f32dt) + f32Sum02;
        A->data[TRACKABLE_INDEX_COVARIANCE_POSY_VELX] =
            (f32Sum14 * f32dt) + f32Sum12;
        A->data[TRACKABLE_INDEX_VARIANCE_VELX] +=
            (A->data[TRACKABLE_INDEX_COVARIANCE_VELX_ACCX] * f32dt) +
            f32Value22;
        A->data[TRACKABLE_INDEX_COVARIANCE_POSX_VELY] =
            (f32Sum05 * f32dt) + f32Sum03;
        A->data[TRACKABLE_INDEX_COVARIANCE_POSY_VELY] =
            (f32Sum15 * f32dt) + f32Sum13;
        A->data[TRACKABLE_INDEX_COVARIANCE_VELX_VELY] +=
            (A->data[TRACKABLE_INDEX_COVARIANCE_VELX_ACCY] * f32dt) +
            f32Value32;
        A->data[TRACKABLE_INDEX_VARIANCE_VELY] +=
            (A->data[TRACKABLE_INDEX_COVARIANCE_VELY_ACCY] * f32dt) +
            f32Value33;

        u32Success = TUEOBJFUSN_ERROR_NOERROR;
    }

    return u32Success;
}

#define ObjFusn_STOP_SEC_CODE

/** \} */
