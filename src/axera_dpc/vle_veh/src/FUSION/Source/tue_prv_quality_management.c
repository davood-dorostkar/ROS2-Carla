/** \defgroup tuePrvQualityManagement TUE Quality Management
 * \{
 * \file       tue_prv_quality_management.c
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

#include "tue_prv_quality_management.h"
#include "tue_prv_fusion_math.h"
#include "tue_prv_quality_management_int.h"
#include "TueObjFusn_TrackableConstants.h"
#include "TueObjFusn_ErrorCodes.h"
#include "TueObjFusn_AAU_Codes.h"
#include "tue_prv_error_management.h"
#include "tue_prv_fusion_tools.h"
#include "TueObjFusn_TrackableProps.h"

/*****************************************************************************
VARIABLES
*****************************************************************************/

/*****************************************************************************
FUNCTIONS
*****************************************************************************/

#define ObjFusn_START_SEC_CODE

LOCAL uint32 getProbability_TruePositive(
    CONSTP2CONST(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pTrkbl,
    CONSTP2VAR(float32, AUTOMATIC, ObjFusn_VAR_NOINIT) pProbabilityTruePositive,
    const uint32 u32SensorUpdatePattern) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    float32 f32PosX = FLT_ZERO;
    float32 f32DistanceThreshold_1 = FLT_ZERO;
    float32 f32DistanceTrehshold_2 = FLT_ZERO;
    float32 f32ConstantTruePosProbabilityHigh = FLT_ZERO;
    float32 f32ConstantTruePosProbabilityLow = FLT_ZERO;
    float32 f32Slope = FLT_ZERO;
    float32 f32Offset = FLT_ZERO;
    float32 f32TruePositiveProbability = FLT_ZERO;
    const uint16 u16CurrClass = pTrkbl->u16Class;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
    if ((((u32SensorUpdatePattern & TUEOBJFUSN_TRACKABLE_U32SENSOR_CAMERA) ==
          0u) &&
         ((u32SensorUpdatePattern & TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR) ==
          0u)) ||
        /* Check for input pattern FUSED, should not happen */
        (((u32SensorUpdatePattern & TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR) >
          0u) &&
         ((u32SensorUpdatePattern & TUEOBJFUSN_TRACKABLE_U32SENSOR_CAMERA) >
          0u))) {
        u32Success = TUEOBJFUSN_ERROR_INVALID_SENSOR_PATTERN;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_QUALITY_MANAGEMENT,
            TUEOBJFUSN_AAU_QUALITY_MANAGEMENT_GET_PROBABILITY_TRUE_POSITIVE);
    } else if (pTrkbl->vecX.nRows > TUEOBJFUSN_MATRIX_SIZE) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_QUALITY_MANAGEMENT,
            TUEOBJFUSN_AAU_QUALITY_MANAGEMENT_GET_PROBABILITY_TRUE_POSITIVE);
    } else if (pTrkbl->vecX.nRows < TUE_PRV_QUALITY_MANAGEMENT_MIN_STATE_SIZE) {
        u32Success = TUEOBJFUSN_ERROR_INPUT_NOT_ENOUGH_STATES;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_QUALITY_MANAGEMENT,
            TUEOBJFUSN_AAU_QUALITY_MANAGEMENT_GET_PROBABILITY_TRUE_POSITIVE);
    } else
#endif
    {
        if (((u32SensorUpdatePattern)&TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR) >
            0u) {
            f32DistanceThreshold_1 =
                TUE_PRV_QUALITY_MANAGEMENT_FLR_PROBABILITY_TRUE_POSITIVE_DISTANCE_THRESHOLD_1;
            f32DistanceTrehshold_2 =
                TUE_PRV_QUALITY_MANAGEMENT_FLR_PROBABILITY_TRUE_POSITIVE_DISTANCE_THRESHOLD_2;
            f32ConstantTruePosProbabilityHigh =
                TUE_PRV_QUALITY_MANAGEMENT_FLR_PROBABILITY_TRUE_POSITIVE_CONSTANT_1;
            f32ConstantTruePosProbabilityLow =
                TUE_PRV_QUALITY_MANAGEMENT_FLR_PROBABILITY_TRUE_POSITIVE_CONSTANT_2;

        } else {
            /* If not RADAR must be CAMERA */
            if ((u16CurrClass & TUEOBJFUSN_TRACKABLE_U16CLASS_VEHICLE_UNKNOWN) >
                0u) {
                f32DistanceThreshold_1 =
                    TUE_PRV_QUALITY_MANAGEMENT_MVS_VEHICLE_PROBABILITY_TRUE_POSITIVE_DISTANCE_THRESHOLD_1;
                f32DistanceTrehshold_2 =
                    TUE_PRV_QUALITY_MANAGEMENT_MVS_VEHICLE_PROBABILITY_TRUE_POSITIVE_DISTANCE_THRESHOLD_2;
                f32ConstantTruePosProbabilityHigh =
                    TUE_PRV_QUALITY_MANAGEMENT_MVS_VEHICLE_PROBABILITY_TRUE_POSITIVE_CONSTANT_1;
                f32ConstantTruePosProbabilityLow =
                    TUE_PRV_QUALITY_MANAGEMENT_MVS_VEHICLE_PROBABILITY_TRUE_POSITIVE_CONSTANT_2;
            } else if ((u16CurrClass &
                        TUEOBJFUSN_TRACKABLE_U16CLASS_PEDESTRIAN_UNKNOWN) >
                       0u) {
                f32DistanceThreshold_1 =
                    TUE_PRV_QUALITY_MANAGEMENT_MVS_PEDESTRIAN_PROBABILITY_TRUE_POSITIVE_DISTANCE_THRESHOLD_1;
                f32DistanceTrehshold_2 =
                    TUE_PRV_QUALITY_MANAGEMENT_MVS_PEDESTRIAN_PROBABILITY_TRUE_POSITIVE_DISTANCE_THRESHOLD_2;
                f32ConstantTruePosProbabilityHigh =
                    TUE_PRV_QUALITY_MANAGEMENT_MVS_PEDESTRIAN_PROBABILITY_TRUE_POSITIVE_CONSTANT_1;
                f32ConstantTruePosProbabilityLow =
                    TUE_PRV_QUALITY_MANAGEMENT_MVS_PEDESTRIAN_PROBABILITY_TRUE_POSITIVE_CONSTANT_2;
            } else if (u16CurrClass ==
                       TUEOBJFUSN_TRACKABLE_U16CLASS_2WHEELER_BICYCLE) {
                f32DistanceThreshold_1 =
                    TUE_PRV_QUALITY_MANAGEMENT_MVS_BICYCLE_PROBABILITY_TRUE_POSITIVE_DISTANCE_THRESHOLD_1;
                f32DistanceTrehshold_2 =
                    TUE_PRV_QUALITY_MANAGEMENT_MVS_BICYCLE_PROBABILITY_TRUE_POSITIVE_DISTANCE_THRESHOLD_2;
                f32ConstantTruePosProbabilityHigh =
                    TUE_PRV_QUALITY_MANAGEMENT_MVS_BICYCLE_PROBABILITY_TRUE_POSITIVE_CONSTANT_1;
                f32ConstantTruePosProbabilityLow =
                    TUE_PRV_QUALITY_MANAGEMENT_MVS_BICYCLE_PROBABILITY_TRUE_POSITIVE_CONSTANT_2;
            } else if ((u16CurrClass &
                        TUEOBJFUSN_TRACKABLE_U16CLASS_2WHEELER_UNKNOWN) > 0u) {
                f32DistanceThreshold_1 =
                    TUE_PRV_QUALITY_MANAGEMENT_MVS_MOTORBIKE_PROBABILITY_TRUE_POSITIVE_DISTANCE_THRESHOLD_1;
                f32DistanceTrehshold_2 =
                    TUE_PRV_QUALITY_MANAGEMENT_MVS_MOTORBIKE_PROBABILITY_TRUE_POSITIVE_DISTANCE_THRESHOLD_2;
                f32ConstantTruePosProbabilityHigh =
                    TUE_PRV_QUALITY_MANAGEMENT_MVS_MOTORBIKE_PROBABILITY_TRUE_POSITIVE_CONSTANT_1;
                f32ConstantTruePosProbabilityLow =
                    TUE_PRV_QUALITY_MANAGEMENT_MVS_MOTORBIKE_PROBABILITY_TRUE_POSITIVE_CONSTANT_2;
            } else {
                f32DistanceThreshold_1 =
                    TUE_PRV_QUALITY_MANAGEMENT_MVS_UNKNOWN_PROBABILITY_TRUE_POSITIVE_DISTANCE_THRESHOLD_1;
                f32DistanceTrehshold_2 =
                    TUE_PRV_QUALITY_MANAGEMENT_MVS_UNKNOWN_PROBABILITY_TRUE_POSITIVE_DISTANCE_THRESHOLD_2;
                f32ConstantTruePosProbabilityHigh =
                    TUE_PRV_QUALITY_MANAGEMENT_MVS_UNKNOWN_PROBABILITY_TRUE_POSITIVE_CONSTANT_1;
                f32ConstantTruePosProbabilityLow =
                    TUE_PRV_QUALITY_MANAGEMENT_MVS_UNKNOWN_PROBABILITY_TRUE_POSITIVE_CONSTANT_2;
            }
        }

        f32PosX = (pTrkbl->vecX).data[TRACKABLE_POSX] -
                  TUE_PRV_VEHICLE_DISTANCE_REAR_AXLE_TO_FRONT_BUMBER;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
        if ((f32DistanceThreshold_1 >= f32DistanceTrehshold_2) ||
            (f32ConstantTruePosProbabilityLow >=
             (f32ConstantTruePosProbabilityHigh + FLT_EPSILON))) {
            u32Success = TUEOBJFUSN_ERROR_INTERNAL_ERROR;
            (void)tue_prv_error_management_addError(
                u32Success, TUEOBJFUSN_AAU_QUALITY_MANAGEMENT,
                TUEOBJFUSN_AAU_QUALITY_MANAGEMENT_GET_PROBABILITY_TRUE_POSITIVE);
        } else
#endif
        {
            if (f32PosX < f32DistanceThreshold_1) {
                *pProbabilityTruePositive = f32ConstantTruePosProbabilityHigh;
            } else if (f32PosX > f32DistanceTrehshold_2) {
                *pProbabilityTruePositive = f32ConstantTruePosProbabilityLow;
            } else {
                f32Slope = (f32ConstantTruePosProbabilityHigh -
                            f32ConstantTruePosProbabilityLow) /
                           (f32DistanceThreshold_1 - f32DistanceTrehshold_2);
                f32Offset = (((-FLT_ONE) * f32Slope) * f32DistanceTrehshold_2) +
                            f32ConstantTruePosProbabilityLow;

                f32TruePositiveProbability = (f32Slope * f32PosX) + f32Offset;
#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
                if (f32TruePositiveProbability < FLT_ZERO) {
                    *pProbabilityTruePositive = FLT_ZERO;

                    /* Should never happen */
                    u32Success = TUEOBJFUSN_ERROR_INTERNAL_ERROR;
                    (void)tue_prv_error_management_addError(
                        u32Success, TUEOBJFUSN_AAU_QUALITY_MANAGEMENT,
                        TUEOBJFUSN_AAU_QUALITY_MANAGEMENT_GET_PROBABILITY_TRUE_POSITIVE);
                } else {
                    *pProbabilityTruePositive = f32TruePositiveProbability;
                }
#else
                *pProbabilityTruePositive = tue_prv_fusion_max_F32(
                    f32TruePositiveProbability, FLT_ZERO);
#endif
            }
        }
    }

    return u32Success;
}
#define ObjFusn_STOP_SEC_CODE

#define ObjFusn_START_SEC_CODE

LOCAL uint32 getProbability_FalsePositive(
    CONSTP2CONST(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pTrkbl,
    CONSTP2VAR(float32, AUTOMATIC, ObjFusn_VAR_NOINIT) pProbalityFalsePositive,
    const uint32 u32SensorUpdatePattern) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    /* PRQA S 3204 1 */ /* variable may be set to other values depending on
                           run-time check activation */
    uint16 u16ClassType = 0u;
    float32 f32PosX = FLT_ZERO;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if ((NULL_PTR == pTrkbl) || (NULL_PTR == pProbalityFalsePositive)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_QUALITY_MANAGEMENT,
            TUEOBJFUSN_AAU_QUALITY_MANAGEMENT_GET_PROBABILITY_FALSE_POSITIVE);
    } else
#endif
        if ((((u32SensorUpdatePattern)&TUEOBJFUSN_TRACKABLE_U32SENSOR_CAMERA) ==
             0u) &&
            (((u32SensorUpdatePattern)&TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR) ==
             0u)) {
        u32Success = TUEOBJFUSN_ERROR_INVALID_SENSOR_PATTERN;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_QUALITY_MANAGEMENT,
            TUEOBJFUSN_AAU_QUALITY_MANAGEMENT_GET_PROBABILITY_FALSE_POSITIVE);
    } else
#endif
    {
        u16ClassType = (pTrkbl->u16Class);
        f32PosX = (pTrkbl->vecX.data[TRACKABLE_POSX]);

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
        if (((u32SensorUpdatePattern & TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR) >
             0u) &&
            ((u32SensorUpdatePattern & TUEOBJFUSN_TRACKABLE_U32SENSOR_CAMERA) >
             0u)) {
            /* Never call this function with a fused trackable */
            u32Success = TUEOBJFUSN_ERROR_INVALID_SENSOR_PATTERN;
            (void)tue_prv_error_management_addError(
                u32Success, TUEOBJFUSN_AAU_QUALITY_MANAGEMENT,
                TUEOBJFUSN_AAU_QUALITY_MANAGEMENT_GET_PROBABILITY_FALSE_POSITIVE);
        } else
#endif
            if (((u32SensorUpdatePattern)&TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR) >
                0u) {
            *pProbalityFalsePositive =
                TUE_PRV_QUALITY_MANAGEMENT_FLR_PROBABILITY_FALSE_POSITIVE;
        } else if (
            ((u32SensorUpdatePattern)&TUEOBJFUSN_TRACKABLE_U32SENSOR_CAMERA) >
            0u) {
            if ((u16ClassType & TUEOBJFUSN_TRACKABLE_U16CLASS_VEHICLE_UNKNOWN) >
                0u) {
                if (f32PosX >
                    TUE_PRV_QUALITY_MANAGEMENT_MVS_VEHICLE_PROBABILITY_FALSE_POSITIVE_MAX_DIST) {
                    f32PosX =
                        TUE_PRV_QUALITY_MANAGEMENT_MVS_VEHICLE_PROBABILITY_FALSE_POSITIVE_MAX_DIST;
                } else {
                    /* MISRA */
                }

                *pProbalityFalsePositive =
                    TUE_PRV_QUALITY_MANAGEMENT_MVS_VEHICLE_PROBABILITY_FALSE_POSITIVE_OFFSET +
                    (f32PosX *
                     TUE_PRV_QUALITY_MANAGEMENT_MVS_VEHICLE_PROBABILITY_FALSE_POSITIVE_SLOPE);
            } else {
                /** Use pedetrian model for all remaining tracks (Pedestrian / 2
                 * Wheeler) */
                if (f32PosX >
                    TUE_PRV_QUALITY_MANAGEMENT_MVS_PEDESTRIAN_PROBABILITY_FALSE_POSITIVE_MAX_DIST) {
                    f32PosX =
                        TUE_PRV_QUALITY_MANAGEMENT_MVS_PEDESTRIAN_PROBABILITY_FALSE_POSITIVE_MAX_DIST;
                } else {
                    /* MISRA */
                }

                *pProbalityFalsePositive =
                    TUE_PRV_QUALITY_MANAGEMENT_MVS_PEDESTRIAN_PROBABILITY_FALSE_POSITIVE_OFFSET +
                    (f32PosX *
                     TUE_PRV_QUALITY_MANAGEMENT_MVS_PEDESTRIAN_PROBABILITY_FALSE_POSITIVE_SLOPE);
            }
        } else {
#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
            u32Success = TUEOBJFUSN_ERROR_INVALID_SENSOR_PATTERN;
            (void)tue_prv_error_management_addError(
                u32Success, TUEOBJFUSN_AAU_QUALITY_MANAGEMENT,
                TUEOBJFUSN_AAU_QUALITY_MANAGEMENT_GET_PROBABILITY_FALSE_POSITIVE);
#endif
            *pProbalityFalsePositive = FLT_ZERO;
        }
    }

    return u32Success;
}
#define ObjFusn_STOP_SEC_CODE

#define ObjFusn_START_SEC_CODE

/* PRQA S 1532 2 */ /* Function called from external AAU */
uint32 tue_prv_update_existence_probability(
    CONSTP2VAR(TueObjFusn_TrackableListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pTrkblList,
    const uint32 u32SensorUpdatePattern) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    uint16 u16i;
    P2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pCurrTrkbl;

    for (u16i = 0u; u16i < pTrkblList->u16ValidTrackables; u16i++) {
        pCurrTrkbl =
            &(pTrkblList->aTrackable[pTrkblList->as16TrackableMap[u16i]]);

        if (((pCurrTrkbl->u32SensorsHist) & u32SensorUpdatePattern) != 0u) {
            u32Success |=
                updateExistenceProbability(pCurrTrkbl, u32SensorUpdatePattern);
        } else {
            /* MISRA */
        }
    }

    return u32Success;
}
#define ObjFusn_STOP_SEC_CODE

#define ObjFusn_START_SEC_CODE

LOCAL uint32 updateExistenceProbability(CONSTP2VAR(TueObjFusn_TrackableType,
                                                   AUTOMATIC,
                                                   ObjFusn_VAR_NOINIT) pTrkbl,
                                        const uint32 u32SensorUpdatePattern) {
    uint32 u32Success;
    float32 f32ProbabilityTruePositive;
    float32 f32ProbabilityFalsePositive;
    float32 f32ExistenceProbability;
    float32 f32ProbRatio;
    boolean bDetected = FALSE;
    float32 f32FacAva = FLT_ONE;
    float32 f32FacMiss = FLT_ONE;

#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if (NULL_PTR == pTrkbl) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_QUALITY_MANAGEMENT,
            TUEOBJFUSN_AAU_QUALITY_MANAGEMENT_UPDATE_EXISTENCE_PROBABILITY);
    } else
#endif
    {
        f32ExistenceProbability = (pTrkbl->f32ExistenceQuality);
        if (0u != ((pTrkbl->u32SensorsCurr) & u32SensorUpdatePattern)) {
            bDetected = TRUE;
        } else {
            bDetected = FALSE;
        }

        u32Success = getProbability_TruePositive(
            pTrkbl, &f32ProbabilityTruePositive, u32SensorUpdatePattern);
        u32Success |= getProbability_FalsePositive(
            pTrkbl, &f32ProbabilityFalsePositive, u32SensorUpdatePattern);

        /* get existence probabilites */
        if (TRUE == bDetected) {
            f32ProbRatio = tue_prv_fusion_log(f32ProbabilityTruePositive /
                                              f32ProbabilityFalsePositive);
        } else {
            f32ProbRatio =
                tue_prv_fusion_log((FLT_ONE - f32ProbabilityTruePositive) /
                                   (FLT_ONE - f32ProbabilityFalsePositive));
        }

        /* get factors of current zone */
        if (TUE_PRV_QUALITY_MANAGEMENT_EXISTENCE_LEVEL_3 <
            pTrkbl->f32ExistenceQuality) {
            f32FacMiss = TUE_PRV_QUALITY_MANAGEMENT_EXISTENCE_LEVEL_3_FAC_MISS;
            f32FacAva = TUE_PRV_QUALITY_MANAGEMENT_EXISTENCE_LEVEL_3_FAC_AVA;
        } else if (TUE_PRV_QUALITY_MANAGEMENT_EXISTENCE_LEVEL_2 <
                   pTrkbl->f32ExistenceQuality) {
            f32FacMiss = TUE_PRV_QUALITY_MANAGEMENT_EXISTENCE_LEVEL_2_FAC_MISS;
        } else if (TUE_PRV_QUALITY_MANAGEMENT_EXISTENCE_LEVEL_1 <
                   pTrkbl->f32ExistenceQuality) {
            f32FacAva = TUE_PRV_QUALITY_MANAGEMENT_EXISTENCE_LEVEL_1_FAC_AVA;
        } else {
            f32FacAva = TUE_PRV_QUALITY_MANAGEMENT_EXISTENCE_LEVEL_1_FAC_AVA;
            f32FacMiss = TUE_PRV_QUALITY_MANAGEMENT_EXISTENCE_LEVEL_1_FAC_MISS;
        }

        /* get existence probabilites */
        if (TRUE == bDetected) {
            f32ExistenceProbability += f32FacAva * f32ProbRatio;
        } else {
            f32ExistenceProbability += f32FacMiss * f32ProbRatio;
        }

        f32ExistenceProbability = tue_prv_fusion_min_max_F32(
            f32ExistenceProbability,
            TUEOBJFUSN_TRACKABLE_F32EXISTENCEQUALITY_MIN,
            TUEOBJFUSN_TRACKABLE_F32EXISTENCEQUALITY_MAX);
        pTrkbl->f32ExistenceQuality = f32ExistenceProbability;

        /** Debug only */
        // pTrkbl->f32TrackQuality    += f32ProbRatio;
    }

    return u32Success;
}
#define ObjFusn_STOP_SEC_CODE

/** \} */
