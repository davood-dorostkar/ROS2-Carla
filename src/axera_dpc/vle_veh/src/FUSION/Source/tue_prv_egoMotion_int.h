/** \addtogroup tuePrvEgoMotion
 * @{
 * \file        tue_prv_egoMotion_int.h
 * \brief       internal header for ego motion module
 *
 *
 *
 *
 *          (C) Copyright Tuerme Inc. All rights reserved.
 *
 */

#ifndef TUE_PRV_EGOMOTION_INT_H
#define TUE_PRV_EGOMOTION_INT_H

#ifdef __cplusplus
extern "C" {
#endif

/*==================[inclusions]============================================*/
#include "tue_prv_common_types.h"
#include "TueObjFusn_EgoMotionType.h"
#include "TueObjFusn_ConfigVehicle.h"
/*==================[macros]================================================*/
/* defaults for egomotionhistory */
#define TUEOBJFUSN_EGOMOTIONHISTORY_U16NEXTWRITEIDX_DEFAULT \
    (0u) /**< default for next write index in ring buffer */
#define TUEOBJFUSN_EGOMOTIONHISTORY_U16NUMOFITEMS_DEFAULT \
    (0u) /**< default for number of items in ring buffer */
#define TUEOBJFUSN_EGOMOTIONHISTORY_F32SPACER_DEFAULT \
    (0.0f) /**< default for spacer (used for padding) */

/*==================[type definitions]======================================*/
/**
 * \brief Type to store ego motion in ring buffer
 *
 * Data are stored "oldest first", i.e. elements with lower index are older,
 * except for ring-buffer wrap-around.
 */
typedef struct TueObjFusn_EgoMotionHistoryTag {
    TueObjFusn_EgoMotionType
        aEgoMotionItem[TUEOBJFUSN_EGOMOTIONHISTORY_SIZE]; /**< ringbuffer items
                                                             for an amount of
                                                             time*/
    uint16 u16NextWriteIdx; /**< index of next free item */
    uint16 u16NumOfItems;   /**< amount of items saved */
    float32 f32Spacer;      /**< padding field to ensure 64bit alignment; shall
                               always be set to 0 */
} TueObjFusn_EgoMotionHistory;

/*==================[forward declarations]==================================*/
/*==================[symbolic constants]====================================*/
/*==================[return codes]==========================================*/
/*==================[functions]=============================================*/
#define ObjFusn_START_SEC_CODE

LOCAL void EgoMotion_itemInit(CONSTP2VAR(TueObjFusn_EgoMotionType,
                                         AUTOMATIC,
                                         ObjFusn_VAR_NOINIT) pEgoMotionItem);

LOCAL uint32 EgoMotion_getEgoMotionHistoryIdx(
    CONSTP2VAR(uint16, AUTOMATIC, ObjFusn_VAR_NOINIT) pu16Idx,
    const float32 f32Age);

LOCAL uint32 EgoMotion_copyEgoMotionItem(
    CONSTP2VAR(TueObjFusn_EgoMotionType, AUTOMATIC, ObjFusn_VAR_NOINIT) pDest,
    CONSTP2CONST(TueObjFusn_EgoMotionType, AUTOMATIC, ObjFusn_VAR_NOINIT) pSrc);

LOCAL uint32 EgoMotion_copyEgoMotionHistory(
    CONSTP2VAR(TueObjFusn_EgoMotionHistory, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pDest,
    CONSTP2CONST(TueObjFusn_EgoMotionHistory, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pSrc);
#define ObjFusn_STOP_SEC_CODE

/* begin: getter/setter for unit testing */
#ifdef UNITTEST
TueObjFusn_EgoMotionHistory* EgoMotion_accessEgoMotionHistory(void);
#endif /* UNITTEST */

/*==================[external function declarations]========================*/
/*==================[external constants]====================================*/
/*==================[external data]=========================================*/

#ifdef __cplusplus
}
#endif

#endif /* TUE_PRV_EGOMOTION_INT_H */
       /**
        * @}
        */
/*==================[end of file]===========================================*/
