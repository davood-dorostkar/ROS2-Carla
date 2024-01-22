/** \addtogroup tuePrvlkf
 * @{
 * \file        tue_prv_lkf_int.h
 * \brief internal header file of tue_prv_lkf.c
 *
 *
 *
 *
 *          (C) Copyright Tuerme Inc. All rights reserved.
 *
 */

#ifndef TUE_PRV_LKF_INT_H
#define TUE_PRV_LKF_INT_H

#ifdef __cplusplus
extern "C" {
#endif

/*==================[inclusions]============================================*/
#include "tue_prv_common_types.h"  /*u16_t, f32_t*/
#include "tue_prv_common_matrix.h" /*stf32Matrix_t*/

/*==================[macros]================================================*/
/*==================[type definitions]======================================*/
/*==================[forward declarations]==================================*/
/*==================[symbolic constants]====================================*/
/*==================[return codes]==========================================*/
/*==================[functions]=============================================*/

#define ObjFusn_START_SEC_CODE

LOCAL uint32 LKF_Calculate_KR(
    CONSTP2CONST(stf32SymMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) matP,
    CONSTP2CONST(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) matK,
    CONSTP2VAR(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) matKR);

uint32 LKF_PredictSymMat(CONSTP2VAR(stf32SymMatrix_t,
                                    AUTOMATIC,
                                    ObjFusn_VAR_NOINIT) A,
                         CONST(float32, ObjFusn_VAR_NOINIT) f32dt);
#define ObjFusn_STOP_SEC_CODE

/*==================[external function declarations]========================*/
/*==================[external constants]====================================*/
/*==================[external data]=========================================*/

#ifdef __cplusplus
}
#endif

#endif  // TUE_PRV_LKF_INT_H

/**
 * @}
 */
/*==================[end of file]===========================================*/
