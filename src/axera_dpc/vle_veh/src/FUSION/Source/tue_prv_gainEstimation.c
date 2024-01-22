/*
* Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
* wen mingshu <wenmingshu@senseauto.com>
*/
/** \defgroup tuePrvGainEstimation TUE Gain Estimation AAU
 * \brief  calculates gain an corresponding variables
 *
 * \addtogroup tuePrvgainEstimation
 * \{
 * \file    tue_prv_gainEstimation.c
 * \brief   source code of ego motion module
 */
/* PRQA S 0292 ++ */ /* MKS */
/* */
/* PRQA S 0292 -- */
/*          (C) Copyright Tuerme Inc. All rights reserved.
 */
/*==================[inclusions]============================================*/

#include "tue_prv_gainEstimation.h"
#include "tue_prv_gainEstimation_int.h"
#include "TueObjFusn_ParameterInterface.h"
#include "TueObjFusn_TrackableConstants.h"
#include "TueObjFusn_TrackableListUtils.h"
#include "TueObjFusn_ErrorCodes.h"
#include "tue_prv_fusion_math.h"
#include "tue_prv_error_management.h"
#include "TueObjFusn_AAU_Codes.h"
#include "TueObjFusn_TrackableProps.h"

#if TUE_PRV_GAIN_ESTIMATION_ENABLE_GAIN_CALCULATION == STD_ON

#define ObjFusn_START_SEC_CODE

/* PRQA S 1532 2 */ /* Function called from external AAU */
uint32 gain_update(
    CONSTP2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pTrkbl,
    CONSTP2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pMeas,
    CONSTP2CONST(TueObjFusn_EgoMotionType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pEgoMotion) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    uint16 u16SensorPos = 0u;
    uint16 u16PreviousCam = 0u;
    uint16 u16PreviousRad = 0u;
    float32 f32NewGain = FLT_ZERO;
    float32 f32NewGainVar = FLT_ZERO;
    float32 f32GainVarSum = FLT_ZERO;

#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if ((NULL_PTR == pMeas) || (NULL_PTR == pTrkbl)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION,
            TUEOBJFUSN_AAU_GAINESTIMATION,
            TUEOBJFUSN_AAU_GAIN_ESTIMATION_GAIN_UPDATE);
    } else if (pMeas == pTrkbl) {
        u32Success = TUEOBJFUSN_ERROR_INVALID_POINTER;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_INVALID_POINTER, TUEOBJFUSN_AAU_GAINESTIMATION,
            TUEOBJFUSN_AAU_GAIN_ESTIMATION_GAIN_UPDATE);
    } else
#endif
        if ((TUE_PRV_GAIN_ESTIMATION_MINIMAL_XPOS >
             pMeas->vecX.data[TRACKABLE_POSX]) ||
            (TUE_PRV_GAIN_ESTIMATION_MINIMAL_XPOS >
             pTrkbl->vecX.data[TRACKABLE_POSX])) {
        /* no update for small x values since 1/0 -> infinity */
    } else {
        u16SensorPos = Trackable_getSensPos(pMeas->u32SensorsCurr);
#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
        /* initialize or update gain, if both sensors see object */
        if (TUEOBJFUSN_SENS_POS_INVALID == u16SensorPos) {
            u32Success = TUEOBJFUSN_ERROR_INTERNAL_ERROR;
            (void)tue_prv_error_management_addError(
                TUEOBJFUSN_ERROR_INTERNAL_ERROR, TUEOBJFUSN_AAU_GAINESTIMATION,
                TUEOBJFUSN_AAU_GAIN_ESTIMATION_GAIN_UPDATE);
        } else
#endif
        {
            if (0u != ((pMeas->u32SensorsCurr) &
                       TUEOBJFUSN_TRACKABLE_U32SENSOR_VISION)) { 
                /* camera measurement (1) */
                /* identify if same camera objects updated this track */
                u16PreviousCam = pTrkbl->au16SensorIDLast[u16SensorPos];

                /* update from same camera measurement -> gain is updated if
                   radar was available the last
                   TUE_PRV_GAIN_RADAR_CYCLES_ESTIMATION cycles
                 */
                if ((pMeas->u16ID == u16PreviousCam) &&
                    (pTrkbl->u8CyclesNoRadar <
                     TUE_PRV_GAIN_RADAR_CYCLES_ESTIMATION)) {
                    /* (1a) radar is available shortly before -> update gain */
                    /* Gain = xPosMeas / xPosTrk; */
                    u32Success |= gain_estimation(pTrkbl, pMeas, &f32NewGain,
                                                  &f32NewGainVar);
                    /* initialisation of gain if not calculated before */
                    if (FLT_ZERO > pTrkbl->f32GainVar) {
                        pTrkbl->f32Gain = f32NewGain;
                        pTrkbl->f32GainVar = f32NewGainVar;
                    } else {
                        /* update gain */
                        f32GainVarSum = (f32NewGainVar + (pTrkbl->f32GainVar));
                        pTrkbl->f32Gain = ((f32NewGainVar / f32GainVarSum) *
                                           (pTrkbl->f32Gain));
                        pTrkbl->f32Gain +=
                            (((pTrkbl->f32GainVar) / f32GainVarSum) *
                             f32NewGain);
                        pTrkbl->f32GainVar =
                            ((f32NewGainVar * (pTrkbl->f32GainVar)) /
                             f32GainVarSum);

                        /* limit gain based on camera measurement */
                        u32Success |=
                            limitGain(pMeas->u16Class, &pTrkbl->f32Gain,
                                      &pTrkbl->f32GainVar);
                    }
                } else if ((pMeas->u16ID != u16PreviousCam) &&
                           (pTrkbl->u8CyclesNoRadar <
                            TUE_PRV_GAIN_RADAR_CYCLES_ESTIMATION)) {
                    /* (1b) track is associated to other camera / was not
                     * associated before -> calculate new gain */
                    u32Success |= gain_estimation(
                        pTrkbl, pMeas, &pTrkbl->f32Gain, &pTrkbl->f32GainVar);
                } else {
                    /* no radar information available inside track */
                }

                /* compensate camera measurement, if no gain is available P
                 * matrix is rotated */
                u32Success |=
                    gain_compensation(pMeas, pMeas, pTrkbl->f32Gain,
                                      pTrkbl->f32GainVar, FALSE, pEgoMotion);
            } else if ((0u != (pMeas->u32SensorsCurr &
                               TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR)) &&
                       (0u != (pTrkbl->u32SensorsCurr &
                               TUEOBJFUSN_TRACKABLE_U32SENSOR_VISION))) {
                /* measurment is radar and track is based on camera*/
                u16PreviousRad = pTrkbl->au16SensorIDLast[u16SensorPos];

                if (FLT_ZERO > pTrkbl->f32GainVar) {
                    /* track is camera only (otherwise gain would have been
                     * estimated) and no gain available or
                     * corresponding radar changed / radar information too old
                     * -> estimate gain and compensate track
                     */
                    u32Success |= gain_estimation(
                        pMeas, pTrkbl, &pTrkbl->f32Gain, &pTrkbl->f32GainVar);

                    /* Track has to be compensated */
                    u32Success |= gain_compensation(
                        pTrkbl, pTrkbl, pTrkbl->f32Gain, pTrkbl->f32GainVar,
                        FALSE, pEgoMotion);

                    /* reset xAcceleration - which does not fit to radar values
                     */
                    pTrkbl->matP.data[TRACKABLE_INDEX_VARIANCE_ACCX] =
                        TUE_PRV_GAIN_ESTIMATION_RESET_VAR_ACC;

                } else if ((TUE_PRV_GAIN_RADAR_CYCLES_RESET <
                            pTrkbl->u8CyclesNoRadar) ||
                           (u16PreviousRad != pMeas->u16ID)) {
                    /*  P of track has to be increased, since it was degenerated
                       by camera measurements or by other radar,
                        further the previous gain and gain variance have to be
                       resetted */
                    /*  Gain initialized at this point, no further checks needed
                     */
                    u32Success |=
                        gain_PIncrease(pTrkbl, pTrkbl->f32GainVar, pEgoMotion);
                    pTrkbl->f32Gain = TUEOBJFUSN_TRACKABLE_F32GAIN_DEFAULT;
                    pTrkbl->f32GainVar =
                        TUEOBJFUSN_TRACKABLE_F32GAINVAR_DEFAULT;
                } else {
                    /* MISRA */
                }
            } else {
                /* MISAR - radar measurement - radar with no camera update */
            }
        }
    }
    return u32Success;
}

/* PRQA S 1532 2 */ /* Function called from external AAU */
uint32 gain_prediction(CONSTP2VAR(TueObjFusn_TrackableType,
                                  AUTOMATIC,
                                  ObjFusn_VAR_NOINIT) pTrkbl,
                       const float32 f32dt) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    /* PRQA S 3204 1 */ // variable may be set to other values depending on
                        //    run-time check activation

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if (pTrkbl == NULL_PTR) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION,
            TUEOBJFUSN_AAU_GAINESTIMATION,
            TUEOBJFUSN_AAU_GAIN_ESTIMATION_GAIN_PREDICTION);
    } else
#endif
        if (FLT_ZERO > f32dt) {
        u32Success = TUEOBJFUSN_ERROR_INVALID_INPUT;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_INVALID_INPUT, TUEOBJFUSN_AAU_GAINESTIMATION,
            TUEOBJFUSN_AAU_GAIN_ESTIMATION_GAIN_PREDICTION);
    } else
#endif
        if (FLT_ZERO <= pTrkbl->f32GainVar) {
        pTrkbl->f32GainVar += ((TUE_PRV_GAIN_PREDICTION_Q * f32dt) * f32dt);
        pTrkbl->f32GainVar = tue_prv_fusion_min_max_F32(
            (pTrkbl->f32GainVar), TUEOBJFUSN_TRACKABLE_F32GAINVAR_MIN,
            TUEOBJFUSN_TRACKABLE_F32GAINVAR_MAX);
    } else {
        /* MISRA */
    }

    return u32Success;
}

uint32 gain_estimation(
    CONSTP2CONST(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pTrkbl,
    CONSTP2CONST(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pMeas,
    CONSTP2VAR(float32, AUTOMATIC, ObjFusn_VAR_NOINIT) pf32Gain,
    CONSTP2VAR(float32, AUTOMATIC, ObjFusn_VAR_NOINIT) pf32GainVar) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    float32 f32TrackRangeSq = FLT_ZERO;
    float32 f32MeasRangeSq = FLT_ZERO;
    float32 f32PosXSquaredTrack = FLT_ZERO;
    float32 f32PosYSquaredTrack = FLT_ZERO;
    float32 f32PosXSquaredMeas = FLT_ZERO;
    float32 f32PosYSquaredMeas = FLT_ZERO;
    float32 f32GainVarTmp = FLT_ZERO;
    float32 f32RadMeas = FLT_ZERO;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if ((NULL_PTR == pTrkbl) || (NULL_PTR == pMeas) || (NULL_PTR == pf32Gain) ||
        (NULL_PTR == pf32GainVar)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION,
            TUEOBJFUSN_AAU_GAINESTIMATION,
            TUEOBJFUSN_AAU_GAIN_ESTIMATION_GAIN_ESTIMATION);
    } else if (pMeas == pTrkbl) {
        u32Success = TUEOBJFUSN_ERROR_INVALID_POINTER;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_INVALID_POINTER, TUEOBJFUSN_AAU_GAINESTIMATION,
            TUEOBJFUSN_AAU_GAIN_ESTIMATION_GAIN_UPDATE);
    } else
#endif
        if ((TUE_PRV_GAIN_PREDICTION_MIN_STATE_SIZE > pTrkbl->matP.u16Size) ||
            (TUE_PRV_GAIN_PREDICTION_MIN_STATE_SIZE > pTrkbl->vecX.nRows) ||
            (TUE_PRV_GAIN_PREDICTION_MIN_STATE_SIZE > pMeas->matP.u16Size) ||
            (TUE_PRV_GAIN_PREDICTION_MIN_STATE_SIZE > pMeas->vecX.nRows)) {
        u32Success = TUEOBJFUSN_ERROR_INVALID_INPUT;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_INVALID_INPUT, TUEOBJFUSN_AAU_GAINESTIMATION,
            TUEOBJFUSN_AAU_GAIN_ESTIMATION_GAIN_ESTIMATION);
    } else
#endif
    {
        /* measurment is trackable which has scaling error (e.g. camera
        measurement, or camera-only track), pTrkbl is free of scaling error
        e.g. radar measurement or fused / radar-only track) */

        f32PosXSquaredTrack =
            tue_prv_fusion_pow2((pTrkbl->vecX.data[TRACKABLE_POSX] -
                                 TUE_PRV_VEHICLE_DISTANCE_REAR_AXLE_TO_CAMERA));
        f32PosYSquaredTrack =
            tue_prv_fusion_pow2(pTrkbl->vecX.data[TRACKABLE_POSY]);
        f32PosXSquaredMeas =
            tue_prv_fusion_pow2((pMeas->vecX.data[TRACKABLE_POSX] -
                                 TUE_PRV_VEHICLE_DISTANCE_REAR_AXLE_TO_CAMERA));
        f32PosYSquaredMeas =
            tue_prv_fusion_pow2(pMeas->vecX.data[TRACKABLE_POSY]);

        /* determine radial distance of both rad = sqrt(x^2+y^2) with
         * compensated x position*/
        f32TrackRangeSq = f32PosXSquaredTrack + f32PosYSquaredTrack;
        f32MeasRangeSq = f32PosXSquaredMeas + f32PosYSquaredMeas;

        f32RadMeas = tue_prv_fusion_sqrt(f32MeasRangeSq);
        if ((FLT_EPSILON < f32TrackRangeSq) ||
            (FLT_EPSILON < f32MeasRangeSq)) {
            // only possible if TrackRange and
            // MeasRange != 0, this should not happen
            *pf32Gain = f32RadMeas / tue_prv_fusion_sqrt(f32TrackRangeSq);

            /* Error propagation of range for gain rMeas / rRange*/
            f32GainVarTmp =
                (((f32PosXSquaredMeas *
                   pMeas->matP.data[TRACKABLE_INDEX_VARIANCE_POSX]) +
                  (f32PosYSquaredMeas *
                   pMeas->matP.data[TRACKABLE_INDEX_VARIANCE_POSY])) /
                 f32TrackRangeSq) /
                f32MeasRangeSq;
            f32GainVarTmp +=
                (((f32PosXSquaredTrack *
                   pTrkbl->matP.data[TRACKABLE_INDEX_VARIANCE_POSX]) +
                  (f32PosYSquaredTrack *
                   pTrkbl->matP.data[TRACKABLE_INDEX_VARIANCE_POSY])) *
                 f32MeasRangeSq) /
                ((f32TrackRangeSq * f32TrackRangeSq) * f32TrackRangeSq);

            *pf32GainVar = f32GainVarTmp;
            u32Success = limitGain(pMeas->u16Class, pf32Gain, pf32GainVar);
        } else {
#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
            /* since internal states (rear axle coordinate system) are
             * investigated, tracks can never be 0 / 0 */
            u32Success = TUEOBJFUSN_ERROR_INTERNAL_ERROR;
            (void)tue_prv_error_management_addError(
                TUEOBJFUSN_ERROR_INTERNAL_ERROR, TUEOBJFUSN_AAU_GAINESTIMATION,
                TUEOBJFUSN_AAU_GAIN_ESTIMATION_GAIN_ESTIMATION);
#endif
        }
    }

    return u32Success;
}

uint32 gain_compensation(
    CONSTP2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pOutTrkbl,
    CONSTP2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pTrkbl,
    const float32 f32Gain,
    const float32 f32GainVar,
    const boolean bIncreaseP,
    CONSTP2CONST(TueObjFusn_EgoMotionType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pEgoMotion) {
    VAR(stRelativeStates, ObjFusn_VAR_NOINIT) sRelative;
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if (NULL_PTR == pTrkbl) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION,
            TUEOBJFUSN_AAU_GAINESTIMATION,
            TUEOBJFUSN_AAU_GAIN_ESTIMATION_GAIN_COMPENSATION);
    } else
#endif
        if ((TUE_PRV_GAIN_PREDICTION_MIN_STATE_SIZE > pTrkbl->matP.u16Size) ||
            (TUE_PRV_GAIN_PREDICTION_MIN_STATE_SIZE > pTrkbl->vecX.nRows)) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_SIZE_NOT_EQUAL;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_MATRIX_SIZE_NOT_EQUAL,
            TUEOBJFUSN_AAU_GAINESTIMATION,
            TUEOBJFUSN_AAU_GAIN_ESTIMATION_GAIN_COMPENSATION);
    } else
#endif
    {
        /* calculate relative states and ego */
        u32Success = getRelativeStates(pTrkbl, &sRelative, pEgoMotion);

        if ((TRUE == bIncreaseP) &&
            (FLT_ZERO < f32GainVar)) { // increase P matrix if track is shrunk -
                                        //   only possible if gain is available
            u32Success |= increaseP(pTrkbl, &sRelative, f32GainVar);
        } else {
            /* MISRA */
        }

        if (FLT_ZERO < f32Gain) {
            /* if gain is available -> compensate state */
            /* for PosX take camera offset in x-direction into account */
            pOutTrkbl->vecX.data[TRACKABLE_POSX] =
                (sRelative.f32PosXRel / f32Gain) +
                TUE_PRV_VEHICLE_DISTANCE_REAR_AXLE_TO_CAMERA;
            pOutTrkbl->vecX.data[TRACKABLE_POSY] /= f32Gain;
            /* for VelX take relative velocity into account */
            pOutTrkbl->vecX.data[TRACKABLE_VELX] =
                (sRelative.f32VelXRel / f32Gain) + sRelative.f32EgoVelX;
            /* for VelX take relative velocity into account */
            pOutTrkbl->vecX.data[TRACKABLE_VELY] =
                (sRelative.f32VelYRel / f32Gain) + sRelative.f32EgoVelY;

            /* reset xAcceleration - which does not fit to radar values ->
             * camera only has no acceleration */
            if (pTrkbl->matP.u16Size >=
                TUE_PRV_GAIN_PREDICTION_MIN_STATE_SIZE) {
                pOutTrkbl->matP.data[TRACKABLE_INDEX_VARIANCE_ACCX] =
                    TUE_PRV_GAIN_ESTIMATION_RESET_VAR_ACC;
                pOutTrkbl->matP.data[TRACKABLE_INDEX_VARIANCE_ACCY] =
                    TUE_PRV_GAIN_ESTIMATION_RESET_VAR_ACC;
            } else {
                /* MISRA */
            }
        } else {
            /* MISRA */
        }
    }

    return u32Success;
}

uint32 gain_PIncrease(
    CONSTP2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pTrkbl,
    const float32 f32GainVar,
    CONSTP2CONST(TueObjFusn_EgoMotionType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pEgoMotion) {
    VAR(stRelativeStates, ObjFusn_VAR_NOINIT) sRelative;
    uint32 u32Success;

#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if (NULL_PTR == pTrkbl) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION,
            TUEOBJFUSN_AAU_GAINESTIMATION,
            TUEOBJFUSN_AAU_GAIN_ESTIMATION_GAIN_PINCREASE);
    } else
#endif
    {
        u32Success = getRelativeStates(pTrkbl, &sRelative, pEgoMotion);
        u32Success |= increaseP(pTrkbl, &sRelative, f32GainVar);
    }

    return u32Success;
}

/* PRQA S 1505 2 */ // Function is used in distance score AAU and gainEstimation
                    //    depending on configuration
uint32 limitGain(const uint16 u16Class,
                 CONSTP2VAR(float32, AUTOMATIC, ObjFusn_VAR_NOINIT) pf32Gain,
                 CONSTP2VAR(float32, AUTOMATIC, ObjFusn_VAR_NOINIT)
                     pf32GainVar) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    /* PRQA S 3204 1 */ // variable may be set to other values depending on
                        //    run-time pointer check activation

#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if ((NULL_PTR == pf32Gain) || (NULL_PTR == pf32GainVar)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION,
            TUEOBJFUSN_AAU_GAINESTIMATION,
            TUEOBJFUSN_AAU_GAIN_ESTIMATION_GAIN_LIMITGAIN);
    } else
#endif
    {
        /* limit depending on class */
        if (0u !=
            (u16Class &
             TUEOBJFUSN_TRACKABLE_U16CLASS_PEDESTRIAN_UNKNOWN)) {
            /* ToDo: Redefine Macros such that multiplication is no longer
             * needed */
            *pf32Gain = tue_prv_fusion_min_max_F32(
                *pf32Gain, TUE_PRV_GAIN_PREDICTION_CAM_GAIN_PED_MIN,
                TUE_PRV_GAIN_PREDICTION_CAM_GAIN_PED_MAX);
        } else if (u16Class ==
                   TUEOBJFUSN_TRACKABLE_U16CLASS_VEHICLE_CAR) {
            *pf32Gain = tue_prv_fusion_min_max_F32(
                *pf32Gain, TUE_PRV_GAIN_PREDICTION_CAM_GAIN_CAR_MIN,
                TUE_PRV_GAIN_PREDICTION_CAM_GAIN_CAR_MAX);
        } else if (u16Class == TUEOBJFUSN_TRACKABLE_U16CLASS_VEHICLE_TRUCK) {
            *pf32Gain = tue_prv_fusion_min_max_F32(
                *pf32Gain, TUE_PRV_GAIN_PREDICTION_CAM_GAIN_TRUCK_MIN,
                TUE_PRV_GAIN_PREDICTION_CAM_GAIN_TRUCK_MAX);
        } else {
            *pf32Gain = tue_prv_fusion_min_max_F32(
                *pf32Gain, TUE_PRV_GAIN_PREDICTION_CAM_GAIN_DEFAULT_MIN,
                TUE_PRV_GAIN_PREDICTION_CAM_GAIN_DEFAULT_MAX);
        }

        *pf32GainVar = tue_prv_fusion_min_max_F32(
            *pf32GainVar, TUEOBJFUSN_TRACKABLE_F32GAINVAR_MIN,
            TUEOBJFUSN_TRACKABLE_F32GAINVAR_MAX);
    }

    return u32Success;
}

/* PRQA S 3206 ++ */ // argument may be set to other values depending on
                        // run-time check activation
LOCAL uint32 increaseP(
    CONSTP2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pTrkbl,
    CONSTP2CONST(stRelativeStates, AUTOMATIC, ObjFusn_VAR_NOINIT) sRelative,
    const float32 f32GainVar) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    /* PRQA S 3204 1 */ // variable may be set to other values depending on
                        //    run-time check activation

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if (NULL_PTR == pTrkbl) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION,
            TUEOBJFUSN_AAU_GAINESTIMATION,
            TUEOBJFUSN_AAU_GAIN_ESTIMATION_GAIN_INCREASEP);
    } else
#endif
        if (FLT_ZERO > f32GainVar) {
        u32Success = TUEOBJFUSN_ERROR_INVALID_INPUT;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_INVALID_INPUT, TUEOBJFUSN_AAU_GAINESTIMATION,
            TUEOBJFUSN_AAU_GAIN_ESTIMATION_GAIN_INCREASEP);
    } else
#endif
    { // simplicfication don't take gain into account, since it could be wrong,
        //  since radar changed
        pTrkbl->matP.data[TRACKABLE_INDEX_VARIANCE_POSX] +=
            ((sRelative->f32PosXRel) * (sRelative->f32PosXRel)) * f32GainVar;
        pTrkbl->matP.data[TRACKABLE_INDEX_VARIANCE_POSY] +=
            ((pTrkbl->vecX.data[TRACKABLE_POSY]) *
             (pTrkbl->vecX.data[TRACKABLE_POSY])) *
            f32GainVar;
        pTrkbl->matP.data[TRACKABLE_INDEX_VARIANCE_VELX] +=
            ((sRelative->f32VelXRel) * (sRelative->f32VelXRel)) * f32GainVar;
        pTrkbl->matP.data[TRACKABLE_INDEX_VARIANCE_VELY] +=
            ((sRelative->f32VelYRel) * (sRelative->f32VelYRel)) * f32GainVar;
    }

    return u32Success;
}
/* PRQA S 3206 -- */

LOCAL uint32 getRelativeStates(
    CONSTP2CONST(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pTrkbl,
    CONSTP2VAR(stRelativeStates, AUTOMATIC, ObjFusn_VAR_NOINIT) psRelative,
    CONSTP2CONST(TueObjFusn_EgoMotionType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pEgoMotionItem) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    /* PRQA S 3204 1 */ // variable may be set to other values depending on
                        //    run-time pointer check activation

#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if ((NULL_PTR == pTrkbl) || (NULL_PTR == psRelative)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION,
            TUEOBJFUSN_AAU_GAINESTIMATION,
            TUEOBJFUSN_AAU_GAIN_ESTIMATION_GAIN_RELATIVESTATES);
    } else
#endif
    {
        /* calculate ego velocity */
        psRelative->f32EgoVelX = ((pEgoMotionItem->f32Speed) -
                                  ((pEgoMotionItem->f32YawRate) *
                                   ((pTrkbl->vecX).data[TRACKABLE_POSY])));
        psRelative->f32EgoVelY = ((pEgoMotionItem->f32YawRate) *
                                  ((pTrkbl->vecX).data[TRACKABLE_POSX]));

        psRelative->f32PosXRel = pTrkbl->vecX.data[TRACKABLE_POSX] -
                                 TUE_PRV_VEHICLE_DISTANCE_REAR_AXLE_TO_CAMERA;
        psRelative->f32VelXRel =
            pTrkbl->vecX.data[TRACKABLE_VELX] - psRelative->f32EgoVelX;
        psRelative->f32VelYRel =
            pTrkbl->vecX.data[TRACKABLE_VELY] - psRelative->f32EgoVelY;
    }
    return u32Success;
}

#define ObjFusn_STOP_SEC_CODE

#endif /* TUE_PRV_GAIN_ESTIMATION_ENABLE_GAIN_CALCULATION == STD_ON */

/**
 * \}
 */
