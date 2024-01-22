/*
* Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
* wen mingshu <wenmingshu@senseauto.com>
*/
/** \addtogroup tuePrvlkfTrkMgt
 * @{
 * \file        tue_prv_lkf_trackManagement_int.h
 *
 *
 *
 * internal header file for tue_prv_lkf_trackManagement.c
 *
 * for unit tests define LOCAL prior to including this file!
 *
 *
 *          (C) Copyright Tuerme Inc. All rights reserved.
 *tue_prv_motionType.c
 */

#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_SOURCE_TUE_PRV_LKF_TRACKMANAGEMENT_INT_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_SOURCE_TUE_PRV_LKF_TRACKMANAGEMENT_INT_H_

#include "tue_prv_lkf.h"
#include "TueObjFusn_EgoMotionType.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TUE_PRV_LKF_MILISEC_PER_SEC (1000.f)

#define ObjFusn_START_SEC_CODE

LOCAL uint32 lkfTrackManagement_updateTrkbleInfos(
    CONSTP2CONST(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pMeas,
    CONSTP2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pLkfTrkble,
    const uint16 u16SensPos);

LOCAL uint32 updateAge(CONSTP2VAR(TueObjFusn_TrackableType,
                                  AUTOMATIC,
                                  ObjFusn_VAR_NOINIT) pLkfTrkble,
                       const float32 f32CycleDt);

LOCAL uint32
lkfTrackManagement_dropObjects(const boolean abMarkedForDeletion[]);

LOCAL uint32 lkfTrackManagement_addTrkbl(CONSTP2CONST(TueObjFusn_TrackableType,
                                                      AUTOMATIC,
                                                      ObjFusn_VAR_NOINIT) pMeas,
                                         const uint16 u16SensPos);

LOCAL uint32 lkfTrackManagement_clearVisionInformation(CONSTP2VAR(
    TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pTueTrkbl);

LOCAL uint32 lkfTrackManagement_clearRadarInformation(CONSTP2VAR(
    TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pTueTrkbl);

LOCAL uint32
lkfTrackManagement_clearVisionInformationFromList(const uint16 u16SensorID,
                                                  const uint16 u16SensPos,
                                                  const uint32 u32SensPattern);

#define ObjFusn_STOP_SEC_CODE

/* begin: getter/setter for unit testing */
#ifdef UNITTEST
LOCAL TueObjFusn_TrackableListType* accessTrkMgmtTrackList(void);
#endif /* UNITTEST */
/* end: getter/setter for unit testing */

#ifdef __cplusplus
}
#endif

#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_SOURCE_TUE_PRV_LKF_TRACKMANAGEMENT_INT_H_

/** @} */
