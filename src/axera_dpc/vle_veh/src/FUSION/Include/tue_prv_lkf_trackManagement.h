/** \addtogroup tuePrvlkfTrkMgt
 * @{
 * \file        tue_prv_lkf_trackManagement.h
 *
 *
 *
 *
 *          (C) Copyright Tuerme Inc. All rights reserved.
 *
 */

#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_INCLUDE_TUE_PRV_LKF_TRACKMANAGEMENT_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_INCLUDE_TUE_PRV_LKF_TRACKMANAGEMENT_H_

#ifdef __cplusplus
extern "C" {
#endif

/*==================[inclusions]============================================*/
#include "tue_prv_common_types.h"
#include "TueObjFusn_TrackableListType.h"
#include "TueObjFusn_ObjectListType.h"
#include "TueObjFusn_MatchIndex.h"
#include "TueObjFusn_EgoMotionType.h"

/*==================[macros]================================================*/
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

#define ObjFusn_START_SEC_SLOW_CODE

/** initializes the kalman filter track list (call once during startup) */
void lkfTrackManagement_init(void);

/** prepares Kalman filter for current cycle. Call at the beginning of each
 * cycle
 * before calling LKF_filterStep. Includes decreasing current life span of all
 * active tracks (u16Update field)
 * Increases the age of the tracklist by f32CycleDt
 */
void lkfTrackManagement_startCycle(const float32 f32CycleDt);

/** cleanup step that has to be performed once after each cycle. Reduces the
 * lifespan of objects and cleans up dead objects (to prevent reassociation). */
uint32 lkfTrackManagement_finalizeCycle(const float32 f32CycleDt);

/**
 * Deletes all sensor bits provided in u32SensorUpdatePattern in the
 * u32SensorsCurr
 * field of all valid tracks. If the track gets an update, the bit will be set
 * again, otherwise the track can die as soon as the u32SensorsCurr equals zero.
 */
uint32 lkfTrackManagement_deleteSensorInformation(
    const uint32 u32SensorUpdatePattern);

/**
 * Predict all internal tracks with given prediction delta
 */
uint32 lkfTrackManagement_predictInternalTrkbl(const float32 f32PredictionDt);

/**
 * Compensate all internal tracks with ego motion to measurement instant of time
 */
extern uint32 lkfTrackManagement_compensateInternalTrkbleList(
    const float32 f32MeasurementLatency);

/**
 * Update tracks with sensor input based on provided data association result
 * Create new tracks for sensor objects which have not been associated
 */
uint32 lkfTrackManagement_updateTrkList(
    CONSTP2VAR(TueObjFusn_ObjectListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pMeasList,
    CONSTP2CONST(stMatchIndexArrayType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pMatchIndexArray,
    VAR(boolean, ObjFusn_VAR_NOINIT) abObjectAssignedFlags[],
    CONSTP2CONST(TueObjFusn_EgoMotionType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pEgoMotion);

/**
 * Setup new tracks for all objects in object list that have not been assigned
 */
uint32 lkfTrackManagement_setupNewLkfTrkble(
    CONSTP2CONST(TueObjFusn_ObjectListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pMeasList,
    CONST(boolean, ObjFusn_VAR_NOINIT) abObjectAssignedFlags[]);

/**
 * Copy track data to output list
 */
uint32 lkfTrackManagement_copyLkfTrkbl2Outputlist(
    CONSTP2VAR(TueObjFusn_ObjectListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pOutputList,
    const boolean abObjectsToBeDropped[]);

/**
 * get age of the track list
 * i.e. the data was valid (updated/predicted to)
 * lkfTrackManagement_GetTrkListAge() sec ago
 */
extern float32 lkfTrackManagement_getTrkListAge(void);

/** returns the internal track list data structure of the tracker for reading
 * (must not be modified) */
extern P2VAR(TueObjFusn_TrackableListType, AUTOMATIC, ObjFusn_CODE)
    lkfTrackManagement_getTrkbleList(void);

/** updates the existence probability for the internal track list */
uint32 lkfTrackManagement_updateExistenceProbability(
    const uint32 u32SensorUpdatePattern);

/** selects VIO trackables from internal trackable list if coasting is active */
uint32 lkfTrackManagement_selectVioTrackables(const uint16 u16NumMeasurements,
                                              const uint16 u16nBestMatches);

/** Calls the track merge algorithm */
uint32 lkfTrackManagement_runTrackMerge(CONSTP2CONST(
    TueObjFusn_EgoMotionType, AUTOMATIC, ObjFusn_VAR_NOINIT) pEgoMotion);

/** increase P on diagonal */
void lkfTrackManagement_increaseDiagonalP(CONSTP2VAR(stf32SymMatrix_t,
                                                     AUTOMATIC,
                                                     ObjFusn_VAR_NOINIT) matP);

uint32 lkfTrackManagement_removeObsoleteTracks(const uint32 u32SensorPattern);
#define ObjFusn_STOP_SEC_SLOW_CODE

/*==================[external function declarations]========================*/
/*==================[external constants]====================================*/
/*==================[external data]=========================================*/

#ifdef __cplusplus
}
#endif
#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_INCLUDE_TUE_PRV_LKF_TRACKMANAGEMENT_H_
/*==================[end of file]===========================================*/
/** @} */
