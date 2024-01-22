/** \addtogroup tuePrvmodes
 *  @{
 * \file       tue_prv_modes_filtering_int.h
 * \brief internal header file for tue_prv_modes_filtering.c
 *
 * for unit tests define LOCAL prior to including this file!
 *
 *
 *          (C) Copyright Tuerme Inc. All rights reserved.
 *
 */

#ifndef TUE_PRV_FILTERING_INT_H
#define TUE_PRV_FILTERING_INT_H

#ifdef __cplusplus
extern "C" {
#endif

/*==================[inclusions]============================================*/
#include "tue_prv_common_types.h"
#include "TueObjFusn_ObjectListType.h"
#include "TueObjFusn_TrackableListType.h"
#include "TueObjFusn_MatchIndex.h"
#include "TueObjFusn_EgoMotionType.h"
/*==================[macros]================================================*/
/*==================[type definitions]======================================*/

#define TUE_PRV_TRK_MGMT_INIT_BUFFER_AGE \
    (TUEOBJFUSN_OBJLISTINPUT_F32MEASUREMENTLATENCY_MAX * 2.0f)
/*==================[forward declarations]==================================*/
/*==================[symbolic constants]====================================*/
/*==================[return codes]==========================================*/
/*==================[functions]=============================================*/

/**
 * Sets the list update counter in the provided object list and all objects in
 * the list.
 * Increases the local counter variable afterwards. */
#define ObjFusn_START_SEC_CODE

LOCAL void modes_updateCounter(CONSTP2VAR(TueObjFusn_ObjectListType,
                                          AUTOMATIC,
                                          ObjFusn_VAR_NOINIT) pObjList);

/**
 * performs in-place postprocessing on the given trackable list in all fusion
 * modes.
 * - multi track removal (if requested via parameter)
 * - absolute to relative velocities & accelerations (if requested via
 * parameter)
 * - update updateCounter
 */
LOCAL uint32 modes_postprocessing(CONSTP2VAR(TueObjFusn_ObjectListType,
                                             AUTOMATIC,
                                             ObjFusn_VAR_NOINIT) outList);

/**
 * uses the single input list provided (and the corresponding sensor bit
 * pattern) for object-to-track fusion within the kalman filter.
 * Following steps are performed:
 * - lkfTrackManagement_deleteSensorBits
 * - distance matrix evaluation
 * - association
 * - lkfTrackManagement_updateTracks
 * TRUE is returned if all steps succeed.
 *
 * Actual implementation is located in runObjectToTrackUpdate in \ref
 * tue_prv_modes_int.h
 *
 * \todo: move this function to tue_prv_modes_filtering.h?*/
LOCAL uint32 modes_measToLkfUpdate(CONSTP2VAR(TueObjFusn_ObjectListType,
                                              AUTOMATIC,
                                              ObjFusn_VAR_NOINIT) pObjectList);

LOCAL uint32 modes_runAssociation(
    CONSTP2VAR(TueObjFusn_TrackableListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pTrackListInternal,
    CONSTP2VAR(TueObjFusn_ObjectListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pTrackListSensor,
    CONSTP2VAR(stMatchIndexArrayType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pMatchesArray,
    CONSTP2CONST(TueObjFusn_EgoMotionType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pEgoMotion);

LOCAL uint32 modes_predictObjectList(CONSTP2VAR(TueObjFusn_ObjectListType,
                                                AUTOMATIC,
                                                ObjFusn_VAR_NOINIT) pObjectList,
                                     const float32 f32_dTforward,
                                     const boolean bPredictHeading);

#define ObjFusn_STOP_SEC_CODE

/*==================[external function declarations]========================*/
/*==================[external constants]====================================*/
/*==================[external data]=========================================*/

#ifdef __cplusplus
}
#endif

#endif  // TUE_PRV_FILTERING_INT_H
        /**@}==================[end of
         * file]===========================================*/
