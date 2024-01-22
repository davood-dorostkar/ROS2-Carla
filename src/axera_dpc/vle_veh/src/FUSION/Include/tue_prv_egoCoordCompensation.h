/** \defgroup tueEgoCoordComp TUE Fusion Ego Motion Compensation
 * @{
 * The ego motion compensation module predicts the covered path of
 * the ego vehicle during a certain time and adapts the coordinate system
 * of the target objects to the new ego vehicle position
 *
 * \file        tue_prv_egoCoordCompensation.h
 *
 * \brief header of the ego motion compensation AAU
 *
 *
 *
 *          (C) Copyright Tuerme Inc. All rights reserved.
 *
 */

#ifndef TUE_PRV_EGOCOORDCOMPENSATION_H
#define TUE_PRV_EGOCOORDCOMPENSATION_H

#ifdef __cplusplus
extern "C" {
#endif

/*==================[inclusions]============================================*/
#include "tue_prv_common_types.h"
#include "TueObjFusn_TrackableListType.h"

/*==================[macros]================================================*/
/*==================[type definitions]======================================*/
/*==================[forward declarations]==================================*/
/*==================[symbolic constants]====================================*/
/*==================[return codes]==========================================*/
/*==================[functions]=============================================*/

/**
 * @fn   uint32 EgoCoordCompensation_compensateTrackbleList(
 * TueObjFusn_TrackableListType * const pTrkableList, const float32
 * f32AgePredStart, const float32 f32AgePredEnd, const boolean bRotVel, const
 * boolean bRotAccX, const boolean bRotAccY, const boolean rotCovMat);
 *
 * @remark   EgoCoordCompensation_compensateTrackbleList is always called before
 * a prediction
 * @brief   EgoCoordCompensation_compensateTrackbleList compensates the ego
 * motion which has been occured during a prediction.
 *
 * @param [in,out] pTrkableList     TueObjFusn_TrackableListType * const,
 * trackable list to be compensated
 * @param   f32AgePredStart   const float32, start time of compensation
 * @param   f32AgePredEnd     const float32, end time of
 * @param   bRotAccX          const boolean, rotate x acceleration
 * @param   bRotAccY          const boolean, rotate y acceleration
 * @param   rotCovMat         const boolean, rotate covariance matrix
 *
 * @return  TUEOBJFUSN_ERROR_NOERROR if no error occured and corresponding error
 * code otherwise
 */

/** @todo remove all four boolean parameters from this method
 * bRotVel, bRotAccX, bRotAccY shall be decided by u8MotionModel or measurement
 * matrix
 * rotCovMat shall be removed to be always true after issue 433241
 */
#define ObjFusn_START_SEC_CODE

uint32 EgoCoordCompensation_compensateTrackbleList(
    VAR(TueObjFusn_TrackableType, ObjFusn_VAR_NOINIT) aTrackableList[],
    const uint16 u16NumTrackables,
    const sint16 as16TrackableMap[],
    const boolean bUseTrackableMap,
    const float32 f32AgePredStart,
    const float32 f32AgePredEnd,
    const boolean rotCovMat);
#define ObjFusn_STOP_SEC_CODE

/*==================[external function declarations]========================*/
/*==================[external constants]====================================*/
/*==================[external data]=========================================*/

#ifdef __cplusplus
}
#endif

#endif  // TUE_PRV_EGOCOORDCOMPENSATION_H
/*==================[end of file]===========================================*/

/* @} */
