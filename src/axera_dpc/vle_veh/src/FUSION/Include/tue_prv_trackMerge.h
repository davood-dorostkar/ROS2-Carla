/** \addtogroup tuePrvTrackMerge
 * @{
 * \file        tue_prv_trackMerge.h
 * \brief       public header for gain estimation
 *
 *
 *
 *
 *          (C) Copyright Tuerme Inc. All rights reserved.
 *
 */

#ifndef TUE_PRV_TRACKMERGE_H
#define TUE_PRV_TRACKMERGE_H

#ifdef __cplusplus
extern "C" {
#endif

/*==================[inclusions]============================================*/
#include "TueObjFusn_TrackableListType.h"
#include "tue_prv_common_types.h" /*uint16, float32*/
#include "TueObjFusn_EgoMotionType.h"
/*==================[macros]================================================*/
/*==================[type definitions]======================================*/
/*==================[forward declarations]==================================*/
/*==================[symbolic constants]====================================*/
/*==================[return codes]==========================================*/
/*==================[functions]=============================================*/

/**
 * @fn   trackMerge_mergeTracks(TueObjFusn_TrackableType * const pTrackList)
 *
 * @brief   postprocessing step to merge camera-only with radar-only
 *          loops through TRK_List and fuses close camera-only and radar-only
 * tracks based
 *          on a modified distance, scaled by a factor (taking systematic camera
 * error into account)
 *
 * @param   [in,out] pTrackList   TueObjFusn_TrackableListType * const,
 * trackable list (usually TRK_LIST)
 *
 * @return  TRUE (ok) or FALSE (error occured)
 */
#define ObjFusn_START_SEC_CODE

uint32 trackMerge_mergeTracks(
    CONSTP2VAR(TueObjFusn_TrackableListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pTrackList,
    VAR(boolean, ObjFusn_VAR_NOINIT) abMarkedToDrop[],
    CONSTP2CONST(TueObjFusn_EgoMotionType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pEgoMotion);
#define ObjFusn_STOP_SEC_CODE

/*==================[external function declarations]========================*/
/*==================[external constants]====================================*/
/*==================[external data]=========================================*/

#ifdef __cplusplus
}
#endif

#endif /* TUE_PRV_TRACKMERGE_H */
       /**
        * @}
        */
/*==================[end of file]===========================================*/
