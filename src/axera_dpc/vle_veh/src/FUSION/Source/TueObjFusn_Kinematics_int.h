/*
 * \file       TueObjFusn_Kinematics_int.h
 *
 *
 *
 *
 *          (C) Copyright Tuerme Inc. All rights reserved.
 *
 */

#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_SOURCE_TUEOBJFUSN_KINEMATICS_INT_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_SOURCE_TUEOBJFUSN_KINEMATICS_INT_H_

#ifdef __cplusplus
extern "C" {
#endif

/*==================[inclusions]============================================*/
#include "TueObjFusn_Kinematics.h"

/*==================[macros]================================================*/
/*==================[type definitions]======================================*/
/*==================[forward declarations]==================================*/
/*==================[symbolic constants]====================================*/
/*==================[return codes]==========================================*/
/*==================[functions]=============================================*/

#define ObjFusn_START_SEC_CODE

/**
 * @brief Implements ego motion compensation with yaw rate f32w for covariance
 * matrix P_overGround = A * P_relative * A^T
 * in a sparse way (A is highly sparse)
 *
 * number of rows and columns must not exceed @ref TUEOBJFUSN_MATRIX_SIZE
 */
LOCAL uint32 EgoCompensationRel2OverGroundP(
    CONSTP2VAR(stf32SymMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) _matP,
    CONSTP2CONST(TueObjFusn_EgoMotionType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pEgoMotion);
#define ObjFusn_STOP_SEC_CODE

/**
 * @brief transforms a single trackable from a trackable list
 * this method is the same as the method "Kinematics_OverGround2Relative" from
 * TueObjFusn_Kinematics.h - the only difference is the fact that this method
 * works on trackables instead of object lists.
 *
 * params see Kinematics_Relative2OverGround
 *
 * @return see Kinematics_Relative2OverGround
 */
#define ObjFusn_START_SEC_CODE

LOCAL uint32 OverGround2Relative(
    CONSTP2CONST(TueObjFusn_EgoMotionType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        stEgoMotion,
    CONSTP2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) stTrkbl,
    const float32 f32deltaX);
#define ObjFusn_STOP_SEC_CODE

/*==================[external function declarations]========================*/
/*==================[external constants]====================================*/
/*==================[external data]=========================================*/

#ifdef __cplusplus
}
#endif

#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_SOURCE_TUEOBJFUSN_KINEMATICS_INT_H_
/*==================[end of file]===========================================*/
