/** \addtogroup tuePrvLkfCoordinatedTurn
 * @{
 * \file        tue_prv_lkf_coordinatedTurn.h
 *
 *
 *
 *
 *          (C) Copyright Tuerme Inc. All rights reserved.
 *
 */

#ifndef TUE_PRV_LKF_COORDINATED_TURN_H
#define TUE_PRV_LKF_COORDINATED_TURN_H

#ifdef __cplusplus
extern "C" {
#endif

/*==================[inclusions]============================================*/
#include "tue_prv_common_types.h"
#include "TueObjFusn_TrackableType.h"

/*==================[macros]================================================*/
/*==================[type definitions]======================================*/

/*****************************************************************************
   FORWARD DECLARATIONS
*****************************************************************************/

/*****************************************************************************
   TYPEDEFS
*****************************************************************************/

/*****************************************************************************
   FUNCTIONS
*****************************************************************************/
#define ObjFusn_START_SEC_CODE

uint32 LKF_CoordinatedTurn_DoPredict(CONSTP2VAR(TueObjFusn_TrackableType,
                                                AUTOMATIC,
                                                ObjFusn_VAR_NOINIT) pTrkbl,
                                     const float32 f32PredictionDt);
#define ObjFusn_STOP_SEC_CODE

#define ObjFusn_START_SEC_CODE

uint32 LKF_CoordinatedTurn_DoCorrect(CONSTP2VAR(TueObjFusn_TrackableType,
                                                AUTOMATIC,
                                                ObjFusn_VAR_NOINIT) pTrkbl);
#define ObjFusn_STOP_SEC_CODE

#ifdef __cplusplus
}
#endif
#endif /** TUE_PRV_LKF_COORDINATED_TURN_MODEL_H */
