/** \addtogroup tuePrvLkfCoordinatedTurn
 * @{
 * \file        tue_prv_lkf_coordinatedTurn.h
 *
 *
 *
 *
 *          (C) Copyright Tuerme Inc. All rights reserved.
 *
 */

#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_INCLUDE_TUE_PRV_VALIDATION_MANAGEMENT_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_INCLUDE_TUE_PRV_VALIDATION_MANAGEMENT_H_

#ifdef __cplusplus
extern "C" {
#endif

/*==================[inclusions]============================================*/
#include "tue_prv_common_types.h"
#include "TueObjFusn_TrackableListType.h"
#include "TueObjFusn_ObjectListType.h"

/*==================[macros]================================================*/

#define TUEOBJFUSN_TRACKABLE_LIST_TYPE_INPUT (0x0001u)
#define TUEOBJFUSN_TRACKABLE_LIST_TYPE_OUTPUT (0x0002u)
#define TUEOBJFUSN_TRACKABLE_LIST_TYPE_INTERNAL (0x0003u)

/*==================[type definitions]======================================*/
/*==================[forward declarations]==================================*/
/*==================[symbolic constants]====================================*/
/*==================[return codes]==========================================*/
/*==================[functions]=============================================*/

#define ObjFusn_START_SEC_CODE

uint32 tue_prv_validate_trackable_list(
    CONSTP2CONST(TueObjFusn_TrackableListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pTueTrackableList);

uint32 tue_prv_validate_trackable_map(CONSTP2CONST(TueObjFusn_TrackableListType,
                                                   AUTOMATIC,
                                                   ObjFusn_VAR_NOINIT)
                                          pTueTrackableList);

#define ObjFusn_STOP_SEC_CODE

#ifdef __cplusplus
}
#endif
#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_INCLUDE_TUE_PRV_VALIDATION_MANAGEMENT_H_
