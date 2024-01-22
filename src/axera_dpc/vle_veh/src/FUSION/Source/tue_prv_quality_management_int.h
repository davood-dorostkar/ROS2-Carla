/** \addtogroup tuePrvQualityManagement
 * @{
 * \file        tue_prv_lkf_quality_management_int.h
 *
 *
 *
 *
 *          (C) Copyright Tuerme Inc. All rights reserved.
 *
 */

#ifndef TUE_PRV_QUALITY_MANAGEMENT_INT_H
#define TUE_PRV_QUALITY_MANAGEMENT_INT_H

#ifdef __cplusplus
extern "C" {
#endif

/*==================[inclusions]============================================*/

#include "tue_prv_common_types.h"         /*uint16t, float32*/
#include "TueObjFusn_TrackableListType.h" /*TueObjFusn_TrackableType */

/*==================[macros]================================================*/

/**
 * Parametrization can be done as follows:
 *
 *
 *    P_D,k
 *    ^
 *    |
 *    |
 * c1 |  |------------------\
 *    |  |                   \
 *    |  |                    \
 * c2 |  |                     \_______
 *    |--|
 *    ---------------------x----x-----> Pos_x
 *       t_0              t_1  t_2
 *
 *
 *    The define
 * TUE_PRV_QUALITY_MANAGEMENT_(SENSOR)_(CLASS)_PROBABILITY_TRUE_POSITIVE_CONSTANT
 * refers to c
 *    The define
 * TUE_PRV_QUALITY_MANAGEMENT_(SENSOR)_(CLASS)_PROBABILITY_TRUE_POSITIVE_DISTANCE_START
 * refers to threshold t_0
 *    The define
 * TUE_PRV_QUALITY_MANAGEMENT_(SENSOR)_(CLASS)_PROBABILITY_TRUE_POSITIVE_DISTANCE_THRESHOLD_1
 * refers to threshold t_1
 *    The define
 * TUE_PRV_QUALITY_MANAGEMENT_(SENSOR)_(CLASS)_PROBABILITY_TRUE_POSITIVE_DISTANCE_THRESHOLD_2
 * refers to threshold t_2
 *
 */

/** False-Positive Thresholds */
/** FP Vehicle */
#define TUE_PRV_QUALITY_MANAGEMENT_MVS_VEHICLE_PROBABILITY_FALSE_POSITIVE_OFFSET \
    (-6.636e-05f)
#define TUE_PRV_QUALITY_MANAGEMENT_MVS_VEHICLE_PROBABILITY_FALSE_POSITIVE_SLOPE \
    (0.00710880f)
#define TUE_PRV_QUALITY_MANAGEMENT_MVS_VEHICLE_PROBABILITY_FALSE_POSITIVE_MAX_DIST \
    (80.f)

/** FP Pedestrian */
#define TUE_PRV_QUALITY_MANAGEMENT_MVS_PEDESTRIAN_PROBABILITY_FALSE_POSITIVE_OFFSET \
    (0.007697048f)
#define TUE_PRV_QUALITY_MANAGEMENT_MVS_PEDESTRIAN_PROBABILITY_FALSE_POSITIVE_SLOPE \
    (-1.4292e-04f)
#define TUE_PRV_QUALITY_MANAGEMENT_MVS_PEDESTRIAN_PROBABILITY_FALSE_POSITIVE_MAX_DIST \
    (30.f)

#define TUE_PRV_QUALITY_MANAGEMENT_FLR_PROBABILITY_FALSE_POSITIVE (0.20f)

/** Thresholds for MVS Sensor */
#define TUE_PRV_QUALITY_MANAGEMENT_MVS_VEHICLE_PROBABILITY_TRUE_POSITIVE_CONSTANT_1 \
    (0.95f)
#define TUE_PRV_QUALITY_MANAGEMENT_MVS_VEHICLE_PROBABILITY_TRUE_POSITIVE_CONSTANT_2 \
    (0.45f)
#define TUE_PRV_QUALITY_MANAGEMENT_MVS_VEHICLE_PROBABILITY_TRUE_POSITIVE_DISTANCE_THRESHOLD_1 \
    (80.f)
#define TUE_PRV_QUALITY_MANAGEMENT_MVS_VEHICLE_PROBABILITY_TRUE_POSITIVE_DISTANCE_THRESHOLD_2 \
    (100.f)

#define TUE_PRV_QUALITY_MANAGEMENT_MVS_MOTORBIKE_PROBABILITY_TRUE_POSITIVE_CONSTANT_1 \
    (0.70f)
#define TUE_PRV_QUALITY_MANAGEMENT_MVS_MOTORBIKE_PROBABILITY_TRUE_POSITIVE_CONSTANT_2 \
    (0.25f)
#define TUE_PRV_QUALITY_MANAGEMENT_MVS_MOTORBIKE_PROBABILITY_TRUE_POSITIVE_DISTANCE_THRESHOLD_1 \
    (50.f)
#define TUE_PRV_QUALITY_MANAGEMENT_MVS_MOTORBIKE_PROBABILITY_TRUE_POSITIVE_DISTANCE_THRESHOLD_2 \
    (80.f)

#define TUE_PRV_QUALITY_MANAGEMENT_MVS_BICYCLE_PROBABILITY_TRUE_POSITIVE_CONSTANT_1 \
    (0.70f)
#define TUE_PRV_QUALITY_MANAGEMENT_MVS_BICYCLE_PROBABILITY_TRUE_POSITIVE_CONSTANT_2 \
    (0.20f)
#define TUE_PRV_QUALITY_MANAGEMENT_MVS_BICYCLE_PROBABILITY_TRUE_POSITIVE_DISTANCE_THRESHOLD_1 \
    (30.f)
#define TUE_PRV_QUALITY_MANAGEMENT_MVS_BICYCLE_PROBABILITY_TRUE_POSITIVE_DISTANCE_THRESHOLD_2 \
    (60.f)

#define TUE_PRV_QUALITY_MANAGEMENT_MVS_PEDESTRIAN_PROBABILITY_TRUE_POSITIVE_CONSTANT_1 \
    (0.80f)
#define TUE_PRV_QUALITY_MANAGEMENT_MVS_PEDESTRIAN_PROBABILITY_TRUE_POSITIVE_CONSTANT_2 \
    (0.20f)
#define TUE_PRV_QUALITY_MANAGEMENT_MVS_PEDESTRIAN_PROBABILITY_TRUE_POSITIVE_DISTANCE_THRESHOLD_1 \
    (30.f)
#define TUE_PRV_QUALITY_MANAGEMENT_MVS_PEDESTRIAN_PROBABILITY_TRUE_POSITIVE_DISTANCE_THRESHOLD_2 \
    (50.f)

#define TUE_PRV_QUALITY_MANAGEMENT_MVS_UNKNOWN_PROBABILITY_TRUE_POSITIVE_CONSTANT_1 \
    (0.50f)
#define TUE_PRV_QUALITY_MANAGEMENT_MVS_UNKNOWN_PROBABILITY_TRUE_POSITIVE_CONSTANT_2 \
    (0.20f)
#define TUE_PRV_QUALITY_MANAGEMENT_MVS_UNKNOWN_PROBABILITY_TRUE_POSITIVE_DISTANCE_THRESHOLD_1 \
    (300.f)
#define TUE_PRV_QUALITY_MANAGEMENT_MVS_UNKNOWN_PROBABILITY_TRUE_POSITIVE_DISTANCE_THRESHOLD_2 \
    (350.f)

/** Thresholds for FLR Sensor */
#define TUE_PRV_QUALITY_MANAGEMENT_FLR_PROBABILITY_TRUE_POSITIVE_CONSTANT_1 \
    (0.70f)
#define TUE_PRV_QUALITY_MANAGEMENT_FLR_PROBABILITY_TRUE_POSITIVE_CONSTANT_2 \
    TUE_PRV_QUALITY_MANAGEMENT_FLR_PROBABILITY_FALSE_POSITIVE
#define TUE_PRV_QUALITY_MANAGEMENT_FLR_PROBABILITY_TRUE_POSITIVE_DISTANCE_THRESHOLD_1 \
    (60.f)
#define TUE_PRV_QUALITY_MANAGEMENT_FLR_PROBABILITY_TRUE_POSITIVE_DISTANCE_THRESHOLD_2 \
    (120.f)

#define TUE_PRV_QUALITY_MANAGEMENT_MIN_STATE_SIZE (2u)

#define TUE_PRV_QUALITY_MANAGEMENT_EXISTENCE_LEVEL_3 (75.0f)
#define TUE_PRV_QUALITY_MANAGEMENT_EXISTENCE_LEVEL_2 (50.0f)
#define TUE_PRV_QUALITY_MANAGEMENT_EXISTENCE_LEVEL_1 (25.0f)

/** Parameters used to tune the speed the existence quality adapts to changes */
#define TUE_PRV_QUALITY_MANAGEMENT_EXISTENCE_LEVEL_3_FAC_MISS (FLT_THREE)
#define TUE_PRV_QUALITY_MANAGEMENT_EXISTENCE_LEVEL_3_FAC_AVA (FLT_ONE_HALF)
#define TUE_PRV_QUALITY_MANAGEMENT_EXISTENCE_LEVEL_2_FAC_MISS (FLT_TWO)
#define TUE_PRV_QUALITY_MANAGEMENT_EXISTENCE_LEVEL_2_FAC_AVA (FLT_TWO)
#define TUE_PRV_QUALITY_MANAGEMENT_EXISTENCE_LEVEL_1_FAC_MISS (FLT_ONE_HALF)
#define TUE_PRV_QUALITY_MANAGEMENT_EXISTENCE_LEVEL_1_FAC_AVA (FLT_THREE)

/** Parameters used to tune the effect of the track quality on the existence
 * probability */
#define TUE_PRV_QUALITY_MANAGEMENT_PREFACTOR_TRACK_QUALITY_LEVEL_0_MISS \
    (FLT_ONE)
#define TUE_PRV_QUALITY_MANAGEMENT_PREFACTOR_TRACK_QUALITY_LEVEL_0_HIT (FLT_TWO)
#define TUE_PRV_QUALITY_MANAGEMENT_PREFACTOR_TRACK_QUALITY_LEVEL_1_MISS \
    (FLT_ONE)
#define TUE_PRV_QUALITY_MANAGEMENT_PREFACTOR_TRACK_QUALITY_LEVEL_1_HIT (FLT_TWO)
#define TUE_PRV_QUALITY_MANAGEMENT_PREFACTOR_TRACK_QUALITY_LEVEL_2_MISS \
    (FLT_TWO)
#define TUE_PRV_QUALITY_MANAGEMENT_PREFACTOR_TRACK_QUALITY_LEVEL_2_HIT (FLT_ONE)
#define TUE_PRV_QUALITY_MANAGEMENT_PREFACTOR_TRACK_QUALITY_LEVEL_3_MISS \
    (FLT_THREE)
#define TUE_PRV_QUALITY_MANAGEMENT_PREFACTOR_TRACK_QUALITY_LEVEL_3_HIT (FLT_ONE)

#define TUE_PRV_QUALITY_MANAGEMENT_EXISTENCE_SENSOR_RESET_LEVEL3 (5u)
#define TUE_PRV_QUALITY_MANAGEMENT_EXISTENCE_SENSOR_RESET_LEVEL2 (5u)
#define TUE_PRV_QUALITY_MANAGEMENT_EXISTENCE_ID_SWITCH (10.0f)

#define TUE_PRV_QUALITY_MANAGEMENT_TRACK_PEDESTRIAN_YMAX (1.3f)
#define TUE_PRV_QUALITY_MANAGEMENT_TRACK_PEDESTRIAN_PROB (0.3f)
#define TUE_PRV_QUALITY_MANAGEMENT_TRACK_VEHICLE_YMAX (1.5f)
#define TUE_PRV_QUALITY_MANAGEMENT_TRACK_VEHICLE_PROB (0.2f)
#define TUE_PRV_QUALITY_MANAGEMENT_TRACK_UNKNOWN_YMAX (1.0f)
#define TUE_PRV_QUALITY_MANAGEMENT_TRACK_UNKNOWN_PROB (0.2f)

/*==================[type definitions]======================================*/

/*****************************************************************************
   FORWARD DECLARATIONS
*****************************************************************************/

/*****************************************************************************
   TYPEDEFS
*****************************************************************************/

/*****************************************************************************
   SYMBOLIC CONSTANTS
*****************************************************************************/

/*****************************************************************************
   RETURN CODES
*****************************************************************************/

/*****************************************************************************
   FUNCTIONS
*****************************************************************************/

/**
 * @fn   LOCAL bool_t getProbability_TruePositive(const TueObjFusn_TrackableType
 * * const pTrkbl, f32_t * const pProbabilityTruePositive)
 *
 * @brief   Computes the probability that the measurement is a true positive
 * event
 *
 * @param   pTrkbl                            const TueObjFusn_TrackableType *
 * const, the measurement
 * @param   [out]pProbabilityTruePositive    f32_t * const, the probability that
 * the inputted measurement is a true positive event
 *
 * @return  TUEOBJFUSN_ERROR_NOERROR if no error occured and error code
 * otherwise.
 */
#define ObjFusn_START_SEC_CODE

LOCAL uint32 getProbability_TruePositive(
    CONSTP2CONST(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pTrkbl,
    CONSTP2VAR(float32, AUTOMATIC, ObjFusn_VAR_NOINIT) pProbabilityTruePositive,
    const uint32 u32SensorUpdatePattern);
#define ObjFusn_STOP_SEC_CODE

/**
 * @fn   LOCAL bool_t getProbability_FalsePositive(const
 * TueObjFusn_TrackableType * const pTrkbl, f32_t * const
 * pProbabilityTruePositive)
 *
 * @brief   Computes the probability that the measurement is a true positive
 * event
 *
 * @param   pMeas                           const TueObjFusn_TrackableType *
 * const, the measurement
 * @param   [out]pProbalityFalsePositive    f32_t * const, the probability that
 * the inputted measurement is a false positive event
 *
 * @return  TUEOBJFUSN_ERROR_NOERROR if no error occured and error code
 * otherwise.
 */
#define ObjFusn_START_SEC_CODE

LOCAL uint32 getProbability_FalsePositive(
    CONSTP2CONST(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pTrkbl,
    CONSTP2VAR(float32, AUTOMATIC, ObjFusn_VAR_NOINIT) pProbalityFalsePositive,
    const uint32 u32SensorUpdatePattern);
#define ObjFusn_STOP_SEC_CODE

/**
 * @fn   LOCAL bool_t updateExistenceProbability(TueObjFusn_TrackableType *
 * const pTrkbl, uint32 u32SensorUpdatePattern)
 *
 * @brief   Computes the tracks existence probability
 *
 * @param   [in,out]pTrkbl                  TueObjFusn_TrackableType * const, a
 * trackable
 * @param   [out]pProbalityFalsePositive    u32_t, sensor update pattern
 *
 * @return  TUEOBJFUSN_ERROR_NOERROR if no error occured and error code
 * otherwise.
 */
#define ObjFusn_START_SEC_CODE

LOCAL uint32 updateExistenceProbability(CONSTP2VAR(TueObjFusn_TrackableType,
                                                   AUTOMATIC,
                                                   ObjFusn_VAR_NOINIT) pTrkbl,
                                        const uint32 u32SensorUpdatePattern);
#define ObjFusn_STOP_SEC_CODE

/*==================[external function declarations]========================*/
/*==================[external constants]====================================*/
/*==================[external data]=========================================*/

#ifdef __cplusplus
}
#endif

#endif /* TUE_PRV_LKF_QUALITY_MANAGEMENT_INT_H */
/*==================[end of file]===========================================*/
/** @} */
