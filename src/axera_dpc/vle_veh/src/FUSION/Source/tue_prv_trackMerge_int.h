/** \addtogroup tuePrvTrackMerge
 * @{
 * \file        tue_prv_trackMerge_int.h
 * \brief       internal header for track merge
 *
 *
 *
 *
 *          (C) Copyright Tuerme Inc. All rights reserved.
 *
 */

#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_SOURCE_TUE_PRV_TRACKMERGE_INT_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_SOURCE_TUE_PRV_TRACKMERGE_INT_H_

#ifdef __cplusplus
extern "C" {
#endif

/*==================[inclusions]============================================*/
#include "tue_prv_common_types.h" /*uint16, float32*/
#include "TueObjFusn_DistMatrix.h"
#include "TueObjFusn_EgoMotionType.h"
/*==================[macros]================================================*/

/*==================[type definitions]======================================*/
#define TUE_PRV_TRACKMERGE_FACTOR_GATE (1.2f)

/*==================[forward declarations]==================================*/
/*==================[symbolic constants]====================================*/
/*==================[return codes]==========================================*/
/*==================[functions]=============================================*/

/**
 * @fn   trackMerge_mergeTrkble(TueObjFusn_TrackableType * const pTrackA,
 * TueObjFusn_TrackableType * const pTrackB)
 *
 * @brief   takes two trackables and merge them and output it in pTrackA, the ID
 * of pTrackB will be released
 *
 * @param   [in,out]  pTrackA   TueObjFusn_TrackableType * const, trackable,
 * which is kept
 * @param   [in]      pTrackB   TueObjFusn_TrackableType * const, trackable,
 * which will be deleted, can not be const since gain correction is applied
 *
 * @return  TRUE (ok) or FALSE (error occured)
 */
#define ObjFusn_START_SEC_CODE

LOCAL uint32 trackMerge_mergeTrkble(
    CONSTP2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pTrackA,
    CONSTP2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pTrackB,
    CONSTP2CONST(TueObjFusn_EgoMotionType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pEgoMotion);
#define ObjFusn_STOP_SEC_CODE

/**
 * @fn   trackMerge_distanceCam2Radar(const TueObjFusn_TrackableListType * const
 * pTrackList, f32_t * const pf32MaxExtendedDist,stDistMatrix_t* const
 * pDist_mat);
 *
 * @brief   calculates distance between all camera-only and radar-only tracks,
 * taking gain uncertainty of camera into accout,
 *          if gain value of camera only track is available and
 * Fusion_get_bCalculateCameraGain() == TRUE, since gain is used to adapt camera
 * measurement
 *          gain value of radar-only track is never used
 *          distance is stored in pDist_mat, currently it is Trackable_Size *
 * Trackable_Size
 *          todo: make it sparse, so only for camera_only * radar_only
 *
 * @param   [in] pTrackList             TueObjFusn_TrackableListType * const,
 * trackable list (usually TRK_LIST)
 *          [in] pf32MaxExtendedDist    f32_t, if distance is larger than this
 * value -> this value is stored in pDist_Mat
 *          [in,out] pDist_mat          stDistMatrix_t*, distance matrix between
 * all camera-only and radar-only tracks
 * @return  TRUE (ok) or FALSE (error occured)
 */
#define ObjFusn_START_SEC_CODE

LOCAL uint32 trackMerge_distanceCam2Radar(
    CONSTP2CONST(TueObjFusn_TrackableListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pTrackList,
    const float32 f32MatchGate,
    CONSTP2VAR(stDistMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) pDist_mat,
    const uint16 au16IdxRadarOnlyTracks[],
    const uint16 u16NumRadarOnlyTracks,
    const uint16 au16IdxVisionOnlyTracks[],
    const uint16 u16NumVisionOnlyTracks,
    CONSTP2CONST(TueObjFusn_EgoMotionType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pEgoMotion);
#define ObjFusn_STOP_SEC_CODE

/**
 * @fn   trackMerge_findNonFusedTracks(const TueObjFusn_TrackableListType *
 * const pTrackList, s16_t * const au8IdxRadarToDistMatrix, u16_t const
 * u16nElementsRadarToDistMatrix, s16_t * const au8IdxVisionToDIstMatrix, u16_t
 * const u16nElementsVisionToDistMatrix )
 *
 * @brief    Identifies all non fused tracks by looping through the trackable
 * list and writes the indices of non fused tracks to
 *           the corresponding index array
 *
 * @param    [in]        pTrackList
 * TueObjFusn_TrackableListType * const, trackable list (usually TRK_LIST)
 *           [in,out]    as16IdxRadarToDistMatrix         s16_t * const, pointer
 * to index array for radar tracks
 *           [in,out]    as16IdxVisionToDistMatrix        s16_t * const, pointer
 * to index array for vision tracks
 *           [out]       u16NumRadarOnlyTracks            number of radar only
 * tracks in tracklist
 *           [out]       u16NumVisionOnlyTracks           number of vision only
 * tracks in tracklist
 *           [in]        u16nElementsRadarToDistMatrix    length of index array
 * for radar tracks
 *           [in]        u16nElementsVisionToDistMatrix   length of index array
 * for vision tracks
 *
 * @return  TRUE (ok) or FALSE (error occured)
 */
#define ObjFusn_START_SEC_CODE

LOCAL uint32 trackMerge_findNonFusedTracks(
    CONSTP2CONST(TueObjFusn_TrackableListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pTrackList,
    uint16 au16IdxRadarOnlyTracks[],
    const uint16 u16nElementsRadarToDistMatrix,
    CONSTP2VAR(uint16, AUTOMATIC, ObjFusn_VAR_NOINIT) pu16NumRadarOnlyTracks,
    uint16 au16IdxVisionOnlyTracks[],
    const uint16 u16nElementsVisionToDistMatrix,
    CONSTP2VAR(uint16, AUTOMATIC, ObjFusn_VAR_NOINIT) pu16NumVisionOnlyTracks);
#define ObjFusn_STOP_SEC_CODE

/*==================[external function declarations]========================*/
/*==================[external constants]====================================*/
/*==================[external data]=========================================*/

#ifdef __cplusplus
}
#endif

#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_SOURCE_TUE_PRV_TRACKMERGE_INT_H_
       /**
        * @}
        */
/*==================[end of file]===========================================*/
