/** \addtogroup tuePrvmodes
 *  @{
 * \file        TueObjFusn_TrackableListUtils.h
 * \brief header file for TueObjFusn_TrackableListUtils.c
 *
 *
 *
 *          (C) Copyright Tuerme Inc. All rights reserved.
 *
 */

#ifndef TUEOBJFUSN_TRACKABLE_LIST_UTILS_H
#define TUEOBJFUSN_TRACKABLE_LIST_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

/*==================[inclusions]============================================*/
#include "tue_prv_common_types.h"
#include "TueObjFusn_TrackableListType.h"
#include "TueObjFusn_ObjectListType.h"

/*==================[macros]================================================*/

#define TUEOBJFUSN_SENS_POS_INVALID (61234u)

/*==================[type definitions]======================================*/
/*==================[forward declarations]==================================*/
/*==================[symbolic constants]====================================*/
/*==================[return codes]==========================================*/
/*==================[functions]=============================================*/

#define ObjFusn_START_SEC_SLOW_CODE

void Trackable_initSensorPatternBuffer(void);
#define ObjFusn_STOP_SEC_SLOW_CODE

#define ObjFusn_START_SEC_CODE

uint32 Trackable_setSensorInfos(const uint32 au32SensPatList[],
                                const uint16 u16NumLists);

uint16 Trackable_getSensPos(const uint32 u32SensorCurr);

uint32 Trackable_getSensPattern(const uint16 u16Idx);

uint32 Trackable_initObjectList(CONSTP2VAR(TueObjFusn_ObjectListType,
                                           AUTOMATIC,
                                           ObjFusn_VAR_NOINIT) pObjList);

uint32 Trackable_listInit(CONSTP2VAR(TueObjFusn_TrackableListType,
                                     AUTOMATIC,
                                     ObjFusn_VAR_NOINIT) pTueTrackableList);

uint32 Trackable_init(CONSTP2VAR(TueObjFusn_TrackableType,
                                 AUTOMATIC,
                                 ObjFusn_VAR_NOINIT) pTrkble);

uint32 Trackable_copyTrackable(
    CONSTP2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pDest,
    CONSTP2CONST(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pSrc);

extern void Trackable_getSensorInfos(uint32 au32SensorInfos[]);
#define ObjFusn_STOP_SEC_CODE

/*==================[external function declarations]========================*/
/*==================[external constants]====================================*/
/*==================[external data]=========================================*/

#ifdef __cplusplus
}
#endif

#endif  // TUEOBJFUSN_TRACKABLE_LIST_UTILS_H
        /**@}==================[end of
         * file]===========================================*/
