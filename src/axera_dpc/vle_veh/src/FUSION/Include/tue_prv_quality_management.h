/** \addtogroup tuePrvQualityManagement
 * @{
 * \file        tue_prv_lkf_quality_management.h
 *
 *
 *
 *
 *          (C) Copyright Tuerme Inc. All rights reserved.
 *
 */

#ifndef TUE_PRV_QUALITY_MANAGEMENT_H
#define TUE_PRV_QUALITY_MANAGEMENT_H

#ifdef __cplusplus
extern "C" {
#endif

/*==================[inclusions]============================================*/

#include "tue_prv_common_types.h"         /*uint16, float32*/
#include "TueObjFusn_TrackableListType.h" /*TueObjFusn_TrackableType */

/*==================[macros]================================================*/
/*==================[type definitions]======================================*/
/*==================[forward declarations]==================================*/
/*==================[symbolic constants]====================================*/
/*==================[return codes]==========================================*/
/*==================[functions]=============================================*/

#define ObjFusn_START_SEC_CODE

uint32 tue_prv_update_existence_probability(
    CONSTP2VAR(TueObjFusn_TrackableListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pTrkblList,
    const uint32 u32SensorUpdatePattern);

#define ObjFusn_STOP_SEC_CODE

/*==================[external function declarations]========================*/
/*==================[external constants]====================================*/
/*==================[external data]=========================================*/

#ifdef __cplusplus
}
#endif

#endif /* TUE_PRV_LKF_QUALITY_MANAGEMENT_H */
/*==================[end of file]===========================================*/
/** @} */
