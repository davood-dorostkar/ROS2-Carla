/** \addtogroup matrix
 *  @{
 * \file        tue_prv_common_matrix_int.h
 *
 *
 *
 * <br>=====================================================<br>
 * <b>Copyright 2013 by Tuerme.</b>
 *
 *  All rights reserved. Property of Tuerme.<br>
 *  Restricted rights to use, duplicate or disclose of this code<br>
 *  are granted through contract.
 * <br>=====================================================<br>
 */

#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_SOURCE_TUE_PRV_COMMON_MATRIX_INT_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_SOURCE_TUE_PRV_COMMON_MATRIX_INT_H_

#ifdef __cplusplus
extern "C" {
#endif

/*==================[inclusions]============================================*/
#include "tue_prv_common_types.h"
#include "tue_prv_common_matrix.h"
#include "TueObjFusn_ConfigVehicle.h"

/*==================[macros]================================================*/

#define TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_00 (0u)
#define TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_01 (1u)
#define TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_02 (3u)
#define TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_03 (6u)
#define TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_04 (10u)
#define TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_05 (15u)
#define TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_11 (2u)
#define TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_12 (4u)
#define TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_13 (7u)
#define TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_14 (11u)
#define TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_15 (16u)
#define TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_22 (5u)
#define TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_23 (8u)
#define TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_24 (12u)
#define TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_25 (17u)
#define TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_33 (9u)
#define TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_34 (13u)
#define TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_35 (18u)
#define TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_44 (14u)
#define TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_45 (19u)
#define TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_55 (20u)

/*==================[type definitions]======================================*/

/**
 * threshold for consistency check in matrix inversion
 */
#define TUE_PRV_COMMON_MATRIX_INV_EPS (1e-6f)

#define TUE_PRV_COMMON_MATRIX_MAT_INV_3X3_SIZE (3u)

/*==================[functions]============================================*/

#define ObjFusn_START_SEC_CODE

LOCAL uint32
    f32CopyMat(CONSTP2VAR(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) pDest,
               CONSTP2CONST(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) pSrc);

#define ObjFusn_STOP_SEC_CODE

/*==================[external function declarations]========================*/
/*==================[external constants]====================================*/
/*==================[external data]=========================================*/

#ifdef __cplusplus
}
#endif

#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_SOURCE_TUE_PRV_COMMON_MATRIX_INT_H_
       /**@}==================[end of
        * file]===========================================*/
