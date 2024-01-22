/** \addtogroup tuePrvDistance
 * @{
 * \file        tue_prv_distance_score.h
 * \brief       public header for distance score module
 *
 *
 *
 *          (C) Copyright Tuerme Inc. All rights reserved.
 *
 */

#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_INCLUDE_TUE_PRV_DISTANCE_SCORE_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_INCLUDE_TUE_PRV_DISTANCE_SCORE_H_

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
   INCLUDES
*****************************************************************************/
#include "tue_prv_common_types.h"
#include "TueObjFusn_TrackableListType.h"
#include "TueObjFusn_ObjectListType.h"
#include "TueObjFusn_DistMatrix.h"
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

/*****************************************************************************
   FORWARD DECLARATIONS
*****************************************************************************/

/*****************************************************************************
   TYPEDEFS
*****************************************************************************/

/*****************************************************************************
   VARIABLES
*****************************************************************************/

/*****************************************************************************
   FUNCTIONS
*****************************************************************************/

#define ObjFusn_START_SEC_CODE

/**
 * @fn   boolean distance_score_track2track( const TueObjFusn_TrackableListType*
 const pTrackListRow,
                                  const TueObjFusn_TrackableListType* const
 pTrackListCol,
                                  float32 const f32MatchGate,
                                  stDistMatrix_t* const pDist_mat )
 *
 * @brief   Computes distance matrix of two lists of trackables.
 *
 * @param   pTrackListInternal			const
 TueObjFusn_TrackableListType * const, first list of trackables
 * @param   pTrackListSensor			const
 TueObjFusn_TrackableListType * const, second list of trackables
 * @param   _f32MatchGate				const float32 , max
 match gate used in association step
 * @param   [out] pDist_mat				stDistMatrix_t*,
 calculated distance matrix
 * @param	au16IndicesMeasurement		const uint16[], trackable
 map for measurements trackable list
 * @param	au16IndicesSensors			const uint16[],
 trackable map for sensor trackable list
 * @param	pEgoMotion					const
 TueObjFusn_EgoMotionType * const, pointer to ego motion item, used for gain
 estimation
 *
 * @return  TUEOBJFUSN_ERROR_NOERROR or corresponding error code
 */
uint32 distance_score_track2track(
    CONSTP2VAR(TueObjFusn_TrackableListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pTrackListInternal,
    CONSTP2VAR(TueObjFusn_ObjectListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pTrackListSensor,
    const float32 _f32MatchGate,
    CONSTP2VAR(stDistMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) pDist_mat,
    uint16 au16IndicesMeasurements[],
    uint16 au16IndicesInternal[],
    CONSTP2CONST(TueObjFusn_EgoMotionType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pEgoMotion);

/**
 * @fn      void initDistMatrix(stDistMatrix_t* const pMat, uint16 const
 * u16Rows, uint16 const u16Cols, float32 const f32InitVal)
 *
 * @brief   Inits the distance matrix to the inputted default value
 *
 * @param   [in,out] pMat     stDistMatrix_t* const, distance matrix
 * @param   u16Rows           uint16 const, number of rows
 * @param   u16Cols           uint16 const, number of cols
 * @param   f32InitVal        float32 const, default value
 *
 * @return  void
 */
void initDistMatrix(CONSTP2VAR(stDistMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT)
                        spMat,
                    const uint16 u16Rows,
                    const uint16 u16Cols,
                    const float32 f32InitVal);

/**
 * @fn      bool_t computeExtendedDist(const TueObjFusn_TrackableType* const
 * pTrackA, const TueObjFusn_TrackableType* const pTrackB,
 *                                     f32_t const f32MatchGate, f32_t const
 * f32MaxExtendedDist, f32_t* const pf32distance)
 *
 * @brief   Computes the association score version 1.
 *          The association score of version 1 is defined as
 *          l = l_meas + l_ID + l_class
 *          l_meas:          Measurement costs
 *          l_class:         Class cost
 *          l_ID:            ID costs
 *
 * @param   pTrackA                 const TueObjFusn_TrackableType* const, Track
 * of the fusion center
 * @param   pTrackB                 const TueObjFusn_TrackableType* const,
 * Sensor track
 * @param   f32MaxExtendedDist      f32_t const, If there is no association, the
 * distance will be set to that value
 * @param   [out]pf32Distance       f32_t* const, calculated distance
 *
 * @return  TRUE (ok) or FALSE (error occured)
 */
uint32 distance_score_computeExtendedDist(
    CONSTP2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pTrack,
    CONSTP2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pMeas,
    const float32 _f32MatchGate,
    CONSTP2VAR(float32, AUTOMATIC, ObjFusn_VAR_NOINIT) pf32Distance);

#define ObjFusn_STOP_SEC_CODE

/*==================[external function declarations]========================*/
/*==================[external constants]====================================*/
/*==================[external data]=========================================*/

#ifdef __cplusplus
}
#endif

#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_INCLUDE_TUE_PRV_DISTANCE_SCORE_H_
       /**@}==================[end of
        * file]===========================================*/
