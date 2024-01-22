/** \addtogroup tuePrvValidationManagement
 *  @{
 * \file       tue_prv_validation_management_int.h
 * \brief internal header file for tue_prv_validation_management.c
 *
 * for unit tests define LOCAL prior to including this file!
 *
 *
 *
 *          (C) Copyright Tuerme Inc. All rights reserved.
 *
 */

#ifndef TUEOBJFUSN_VALIDATION_MANAGEMENT_INT_H
#define TUEOBJFUSN_VALIDATION_MANAGEMENT_INT_H

#ifdef __cplusplus
extern "C" {
#endif

/*==================[inclusions]============================================*/
#include "tue_prv_common_types.h"
#include "TueObjFusn_TrackableListType.h"

/*==================[type definitions]======================================*/
/*==================[forward declarations]==================================*/
/*==================[symbolic constants]====================================*/
/*==================[return codes]==========================================*/
/*==================[functions]=============================================*/
#define ObjFusn_START_SEC_CODE

LOCAL uint32 tue_prv_validate_trackable(CONSTP2CONST(
    TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pTrackable);
#define ObjFusn_STOP_SEC_CODE

/*==================[external function declarations]========================*/
/*==================[external constants]====================================*/
/*==================[external data]=========================================*/

#ifdef __cplusplus
}
#endif

#endif  // TUEOBJFUSN_VALIDATION_MANAGEMENT_INT_H
        /**@}==================[end of
         * file]===========================================*/
