/** \addtogroup tuePrvAssociation
 *  @{
 * \file        tue_prv_association.h
 * \brief       header file of tue_prv_association.c
 *
 *
 *
 *          (C) Copyright Tuerme Inc. All rights reserved.
 *
 */

#ifndef TUE_PRV_ASSOCIATION_H
#define TUE_PRV_ASSOCIATION_H

#ifdef __cplusplus
extern "C" {
#endif

/*==================[inclusions]============================================*/
#include "tue_prv_common_types.h"  /* standard types (uint16, float32, etc.) */
#include "TueObjFusn_DistMatrix.h" /* distance score matrix */
#include "TueObjFusn_MatchIndex.h" /* association matrix */

/*==================[macros]================================================*/
/*==================[type definitions]======================================*/

/*****************************************************************************
   FORWARD DECLARATIONS
*****************************************************************************/

/*****************************************************************************
   TYPEDEFS
*****************************************************************************/

/*****************************************************************************
   VARIABLES
*****************************************************************************/

/*****************************************************************************
   FUNCTIONS
*****************************************************************************/

/*==================[external function declarations]========================*/
#define ObjFusn_START_SEC_CODE

uint32 tue_prv_association_runAssociation(
    CONSTP2CONST(stDistMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) pSdist_mat,
    const float32 f32MatchGate,
    CONSTP2VAR(stMatchIndexArrayType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pMatchIndexArray);
#define ObjFusn_STOP_SEC_CODE

/*==================[external constants]====================================*/
/*==================[external data]=========================================*/

#ifdef __cplusplus
}
#endif

#endif /* TUE_PRV_ASSOCIATION_H */
       /**@}==================[end of
        * file]===========================================*/
