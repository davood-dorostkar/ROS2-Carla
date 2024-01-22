/** \addtogroup tuePrvEgoMotion
 * @{
 * \file        tue_prv_egoMotion.h
 * \brief       public header for ego motion module
 *
 *
 *
 *
 *          (C) Copyright Tuerme Inc. All rights reserved.
 *
 */

#ifndef TUE_PRV_EGOMOTION_H
#define TUE_PRV_EGOMOTION_H

#ifdef __cplusplus
extern "C" {
#endif

/*==================[inclusions]============================================*/
#include "TueObjFusn_EgoMotionType.h"
#include "tue_prv_common_types.h"
/*==================[macros]================================================*/
/*==================[type definitions]======================================*/
/*==================[forward declarations]==================================*/
/*==================[symbolic constants]====================================*/
/*==================[return codes]==========================================*/
/*==================[functions]=============================================*/
/*==================[external function declarations]========================*/

#define ObjFusn_START_SEC_CODE

void EgoMotion_historyInit(void);
#define ObjFusn_STOP_SEC_CODE

#define ObjFusn_START_SEC_CODE

uint32 EgoMotion_addItemToHistory(CONSTP2CONST(TueObjFusn_EgoMotionType,
                                               AUTOMATIC,
                                               ObjFusn_VAR_NOINIT)
                                      pNewEgoMotion,
                                  const float32 f32Dt);
#define ObjFusn_STOP_SEC_CODE

#define ObjFusn_START_SEC_CODE

uint32 EgoMotion_getEgoMotionHistory(CONSTP2VAR(TueObjFusn_EgoMotionType *,
                                                AUTOMATIC,
                                                ObjFusn_VAR_NOINIT) pEgoMotion,
                                     const float32 f32Age);
#define ObjFusn_STOP_SEC_CODE

#define ObjFusn_START_SEC_CODE

uint32 EgoMotion_getDeltaPath(
    CONSTP2VAR(float32, AUTOMATIC, ObjFusn_VAR_NOINIT) pf32Dx,
    CONSTP2VAR(float32, AUTOMATIC, ObjFusn_VAR_NOINIT) pf32Dy,
    CONSTP2VAR(float32, AUTOMATIC, ObjFusn_VAR_NOINIT) pf32Dphi,
    CONSTP2VAR(float32, AUTOMATIC, ObjFusn_VAR_NOINIT) pf32Sin,
    CONSTP2VAR(float32, AUTOMATIC, ObjFusn_VAR_NOINIT) pf32Cos,
    float32 f32AgeStart,
    const float32 f32AgeEnd);
#define ObjFusn_STOP_SEC_CODE

/*==================[external constants]====================================*/
/*==================[external data]=========================================*/

#ifdef __cplusplus
}
#endif

#endif /* TUE_PRV_EGOMOTION_H */
       /**
        * @}
        */
/*==================[end of file]===========================================*/
