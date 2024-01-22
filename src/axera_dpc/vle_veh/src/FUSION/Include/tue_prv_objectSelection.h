/** \addtogroup target
 *  @{
 * \file        tue_prv_objectSelection.h
 * \brief       object selection module for TueObjFusn_ObjInputType and
 * TueObjFusn_ObjListInputType
 *
 *
 *
 * <br>=====================================================<br>
 * <b>Copyright 2014 by Tuerme.</b>
 *
 *  All rights reserved. Property of Tuerme.<br>
 *  Restricted rights to use, duplicate or disclose of this code<br>
 *  are granted through contract.
 * <br>=====================================================<br>
 */

#ifndef TUE_PRV_OBJECTSELECTION_H
#define TUE_PRV_OBJECTSELECTION_H

#include "tue_prv_common_types.h"
#include "TueObjFusn_TrackableListType.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ObjFusn_START_SEC_CODE

uint32 tue_prv_object_selection_select_objects(
    CONSTP2VAR(TueObjFusn_TrackableListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pTrkblList,
    VAR(sint16, ObjFusn_VAR_NOINIT) s16ObjectsToDelete,
    VAR(boolean, ObjFusn_VAR_NOINIT) abObjectsToBeDropped[]);
#define ObjFusn_STOP_SEC_CODE

#ifdef __cplusplus
}
#endif

#endif /**@} TUE_PRV_OBJECTSELECTION_H */