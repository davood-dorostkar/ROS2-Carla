/** \addtogroup tuePrvDistance
 * @{
 * \file        tue_prv_distance_score_int.h
 * \brief       private header for distance score module

 *
 *
 *          (C) Copyright Tuerme Inc. All rights reserved.
 *
 */

#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_SOURCE_TUE_PRV_DISTANCE_SCORE_INT_H_
#define TUE_PRV_SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_SOURCE_TUE_PRV_DISTANCE_SCORE_INT_H_DISTANCE_SCORE_INT_H

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
   INCLUDES
*****************************************************************************/
#include "tue_prv_common_types.h"
#include "tue_prv_common_matrix.h"
#include "TueObjFusn_DistMatrix.h"
#include "TueObjFusn_TrackableListType.h"
#include "TueObjFusn_ConfigAlgorithm.h"
#include "TueObjFusn_EgoMotionType.h"

/*****************************************************************************
   SYMBOLIC CONSTANTS
*****************************************************************************/

/*****************************************************************************
   MACROS
*****************************************************************************/

/*****************************************************************************
   CONSTANTS
*****************************************************************************/

/** Index of extra costs for class unknown */
#define DIST_SCORE_CLASS_INDEX_UNKNOWN (0x0u)

/** Index of extra costs for class vehicle */
#define DIST_SCORE_CLASS_INDEX_VEHICLE (0x1u)

/** Index of extra costs for class pedestrian */
#define DIST_SCORE_CLASS_INDEX_PEDESTRIAN (0x2u)

/** Default index for extra costs */
#define DIST_SCORE_CLASS_INDEX_DEFAULT (0x3u)

/** Dimension of the cost map of classes */
#define DIST_SCORE_DIM_COST_MAP (4u)

#define DIST_SCORE_MAX_CLASS_VALUE \
    (TUEOBJFUSN_TRACKABLE_U16CLASS_PEDESTRIAN_UNKNOWN)

/**
 * Cost map matrix for classification.
 * The rows represent the classes of the fusion center track: (vehicle ,
 * pedestian, generic)
 * The columns represent the classes of the sensor track: (unknown, vehicle ,
 * pedestian, generic)
 */
#define DIST_SCORE_COST_MAP_VALUES                                         \
    {                                                                      \
        {FLT_ZERO, FLT_ZERO, FLT_ZERO, FLT_ZERO},                          \
            {FLT_ZERO,                                                     \
             -(FLT_ONE_HALF * TUE_PRV_DISTANCE_SCORE_MAX_CLASS_PENALTY),   \
             TUE_PRV_DISTANCE_SCORE_MAX_CLASS_PENALTY,                     \
             TUE_PRV_DISTANCE_SCORE_MAX_CLASS_PENALTY},                    \
            {FLT_ZERO, TUE_PRV_DISTANCE_SCORE_MAX_CLASS_PENALTY,           \
             -(FLT_ONE_HALF * TUE_PRV_DISTANCE_SCORE_MAX_CLASS_PENALTY),   \
             TUE_PRV_DISTANCE_SCORE_MAX_CLASS_PENALTY},                    \
        {                                                                  \
            FLT_ZERO, TUE_PRV_DISTANCE_SCORE_MAX_CLASS_PENALTY,            \
                TUE_PRV_DISTANCE_SCORE_MAX_CLASS_PENALTY,                  \
                -(FLT_ONE_HALF * TUE_PRV_DISTANCE_SCORE_MAX_CLASS_PENALTY) \
        }                                                                  \
    }

/**
 * Return this default value in case an error is detected during distance
 * calculation
 */
#define DIST_SCORE_DEFAULT_ON_ERROR (1000.f)

#define DIST_SCORE_MAHAL_MIN_MATRIX_SIZE (2u)

/*****************************************************************************
   FORWARD DECLARATIONS
*****************************************************************************/

/*****************************************************************************
   TYPEDEFS
*****************************************************************************/

typedef struct stMeasGainCompType_t {
    boolean bUsed;
    float32 f32tempMT;
    float32 f32RadMeas;
    float32 f32MeasRangeSq;
} stMeasGainCompType;

/*****************************************************************************
   VARIABLES
*****************************************************************************/

/*****************************************************************************
   FUNCTIONS
*****************************************************************************/
#define ObjFusn_START_SEC_CODE

/**
 * @fn      bool_t calculateDistance(  const stf32Matrix_t* const pStateA, const
 * stf32Matrix_t* const pCovA,
 *                                     const stf32Matrix_t* const pStateB, const
 * stf32Matrix_t* const pCovB, f32_t* const pf32Distance)
 *
 * @brief   Computes distance depending on the selected distance mode.
 *
 * @param   pStateA              const stf32Matrix_t* const, State estimation of
 * the fusion center
 * @param   pCovA                const stf32Matrix_t* const, Covariance matrix
 * of the fusion center
 * @param   pStateB              const stf32Matrix_t* const, State estimation of
 * the sensor
 * @param   pCovB                const stf32Matrix_t* const, Covariance of the
 * sensor
 * @param   [out]pf32Distance    f32_t* const, calculated distance
 *
 * @return  TRUE (ok) or FALSE (error occured)
 */

LOCAL uint32 calculateDistance(
    CONSTP2CONST(stf32Vec_t, AUTOMATIC, ObjFusn_VAR_NOINIT) pVecA,
    CONSTP2CONST(stf32SymMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) pCovA,
    CONSTP2CONST(stf32Vec_t, AUTOMATIC, ObjFusn_VAR_NOINIT) pVecB,
    CONSTP2CONST(stf32SymMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) pCovB,
    CONSTP2VAR(float32, AUTOMATIC, ObjFusn_VAR_NOINIT) pf32SquareDistance);

#if TUE_PRV_GAIN_ESTIMATION_ENABLE_GAIN_CALCULATION == STD_ON
LOCAL uint32 distance_score_calculateGain(
    CONSTP2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pTrack,
    CONSTP2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pMeas,
    CONSTP2VAR(stMeasGainCompType, AUTOMATIC, ObjFusn_VAR_NOINIT) pMeasComp,
    CONSTP2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pTrkblTmp,
    CONSTP2CONST(TueObjFusn_EgoMotionType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pEgoMotion,
    const float32 f32TrackRangeSq,
    const float32 f32RadTrack,
    const float32 f32RadTrackVar,
    CONSTP2VAR(boolean, AUTOMATIC, ObjFusn_VAR_NOINIT) pbMeasModified,
    CONSTP2VAR(boolean, AUTOMATIC, ObjFusn_VAR_NOINIT) pbTrackModified);
#endif
/**
 * @fn      u16_t getCostMapIndex(u16_t const u16Class)
 *
 * @brief   Reduces the number of track types for class cost computation.
 *
 * @param   u16Class          u16_t const, Class identifier of the trackable
 * list
 *
 * @return  index for the class cost map
 */
#if TUE_PRV_DISTANCE_SCORE_USE_CLASS_INFORMATION == STD_ON
LOCAL uint16 getCostMapIndex(const uint16 u16Class);
#endif

#define ObjFusn_STOP_SEC_CODE

/*==================[external function declarations]========================*/
/*==================[external constants]====================================*/
/*==================[external data]=========================================*/

#ifdef __cplusplus
}
#endif

#endif // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_SOURCE_TUE_PRV_DISTANCE_SCORE_INT_H_
       /**@}==================[end of
        * file]===========================================*/
