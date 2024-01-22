/** \addtogroup tuePrvlkf
 * @{
 * \file        tue_prv_lkf.h
 * \brief header file of tue_prv_lkf.c
 *
 *
 *
 *
 *          (C) Copyright Tuerme Inc. All rights reserved.
 *
 */
#ifndef TUE_PRV_LKF_H
#define TUE_PRV_LKF_H

#ifdef __cplusplus
extern "C" {
#endif

/*==================[inclusions]============================================*/
#include "tue_prv_common_types.h"         /*uint16, float32*/
#include "TueObjFusn_TrackableListType.h" /*TueObjFusn_TrackableType */
#include "tue_prv_common_matrix.h"        /*stf32Matrix_t*/

/*==================[macros]================================================*/

/*==================[type definitions]======================================*/

/**
 * @struct stInformationTrk_t
 * @brief   data structure for a information filter (inverse covariance filter)
 */
typedef struct {
    stf32Matrix_t mat_i; /**< information vector */
    stf32Matrix_t mat_I; /**< information matrix */
} stInformationTrk_t;

/*****************************************************************************
   FUNCTIONS
*****************************************************************************/
#define ObjFusn_START_SEC_CODE

uint32 LKF_AddNewTrkbl(
    CONSTP2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pTrkbl,
    CONSTP2CONST(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pMeas);
#define ObjFusn_STOP_SEC_CODE

#define ObjFusn_START_SEC_CODE

uint32 LKF_DoPredict(CONSTP2VAR(TueObjFusn_TrackableType,
                                AUTOMATIC,
                                ObjFusn_VAR_NOINIT) pTrkbl,
                     const float32 f32dT);
#define ObjFusn_STOP_SEC_CODE

#define ObjFusn_START_SEC_CODE

uint32 LKF_DoCorrect(
    CONSTP2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pTrkbl,
    CONSTP2CONST(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pMeas);
#define ObjFusn_STOP_SEC_CODE

/*==================[external function declarations]========================*/
/*==================[external constants]====================================*/
/*==================[external data]=========================================*/

#ifdef __cplusplus
}
#endif

#endif /* TUE_PRV_LKF_H */

/**
 * @}
 */
/*==================[end of file]===========================================*/
