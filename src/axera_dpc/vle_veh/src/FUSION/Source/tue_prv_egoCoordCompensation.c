/*
* Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
* wen mingshu <wenmingshu@senseauto.com>
*/
/** \addtogroup tueEgoCoordComp
 * \{
 * \file    tue_prv_egoCoordCompensation.c
 *
 * \brief  This AAU helps to compensate the ego motion when predicting
 * a target
 *
 */
/* PRQA S 0292 ++ */ /* MKS */
/*
 */
/* PRQA S 0292 -- */ /* MKS */
                     /*
                      *
                      * <br>=====================================================<br>
                      * <b>Copyright 2014 by Tuerme.</b>
                      * <br>
                      * All rights reserved. Property of Tuerme.<br>
                      * Restricted rights to use, duplicate or disclose of this code<br>
                      * are granted through contract.
                      * <br>=====================================================<br>
                      */

/*==================[inclusions]============================================*/

#include "tue_prv_egoCoordCompensation.h"
#include "tue_prv_egoCoordCompensation_int.h"
#include "tue_prv_egoMotion.h"
#include "tue_prv_fusion_math.h"
#include "TueObjFusn_TrackableListUtils.h"
#include "TueObjFusn_TrackableConstants.h"
#include "TueObjFusn_ErrorCodes.h"
#include "TueObjFusn_AAU_Codes.h"
#include "tue_prv_error_management.h"

#define ObjFusn_START_SEC_CODE

/*==================[method definitions]====================================*/

uint32 EgoCoordCompensation_compensateTrackbleList(
    VAR(TueObjFusn_TrackableType, ObjFusn_VAR_NOINIT) aTrackableList[],
    const uint16 u16NumTrackables,
    const sint16 as16TrackableMap[],
    const boolean bUseTrackableMap,
    const float32 f32AgePredStart,
    const float32 f32AgePredEnd,
    const boolean rotCovMat) {
    /** - check if the trackable list is valid including NULL pointer, mapping
     * matrix and lifespan check */
    uint32 u32Success;
    uint16 u16idx;
    float32 f32DX;
    float32 f32DY;
    float32 f32DPhi;
    float32 f32Sin;
    float32 f32Cos;
    P2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pTrkble;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
    uint16 u16MaxTrackables = 0u;

    if (FALSE == bUseTrackableMap) {
        u16MaxTrackables = TUE_PRV_FUSION_OBJECT_LIST_SIZE;
    } else {
        u16MaxTrackables = TUEOBJFUSN_TRACKABLELIST_U16VALIDTRACKABLES_MAX;
    }

#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if (NULL_PTR == aTrackableList) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_EGOCOORDCOMPENSATION,
            TUEOBJFUSN_AAU_EGO_COORDINATE_COMPENSATION_COMPENSATE_TRACKABLE_LIST);
    } else if ((TRUE == bUseTrackableMap) && (NULL_PTR == as16TrackableMap)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_EGOCOORDCOMPENSATION,
            TUEOBJFUSN_AAU_EGO_COORDINATE_COMPENSATION_COMPENSATE_TRACKABLE_LIST);
    } else
#endif
        if (u16NumTrackables > u16MaxTrackables) {
        u32Success = TUEOBJFUSN_ERROR_NUMBER_OF_OBJECTS_EXCEEDS_MAXIMUM_SIZE;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_EGOCOORDCOMPENSATION,
            TUEOBJFUSN_AAU_EGO_COORDINATE_COMPENSATION_COMPENSATE_TRACKABLE_LIST);
    } else
#endif
    {
        /** - get covered path of ego vehicle during a certain time */
        u32Success =
            EgoMotion_getDeltaPath(&f32DX, &f32DY, &f32DPhi, &f32Sin, &f32Cos,
                                   f32AgePredStart, f32AgePredEnd);
        f32Sin = -f32Sin;

        /** - compensate all trackables */
        for (u16idx = 0u; (TUEOBJFUSN_ERROR_NOERROR == u32Success) &&
                          (u16idx < u16NumTrackables);
             u16idx++) {
            if (TRUE == bUseTrackableMap) {
                pTrkble = &aTrackableList[as16TrackableMap[u16idx]];
            } else {
                pTrkble = &aTrackableList[u16idx];
            }

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
            /**   -- only perform compensation if in rear-axle over-ground
             * coordinates.
             * \todo other implementations pending (altered dPhi, dX and dY!).
             */
            if (TUEOBJFUSN_U8COORDSYSTEM_REARAXLE_OVERGROUND !=
                pTrkble->u8CoordSystem) {
                u32Success |= TUEOBJFUSN_ERROR_COORD_SYSTEM_MISMATCH;
                (void)tue_prv_error_management_addError(
                    u32Success, TUEOBJFUSN_AAU_EGOCOORDCOMPENSATION,
                    TUEOBJFUSN_AAU_EGO_COORDINATE_COMPENSATION_COMPENSATE_TRACKABLE_LIST);
            } else if ((TUEOBJFUSN_MATRIX_SIZE < pTrkble->matP.u16Size) ||
                       (TUEOBJFUSN_MATRIX_SIZE < pTrkble->vecX.nRows)) {
                u32Success |= TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE;
                (void)tue_prv_error_management_addError(
                    u32Success, TUEOBJFUSN_AAU_EGOCOORDCOMPENSATION,
                    TUEOBJFUSN_AAU_EGO_COORDINATE_COMPENSATION_COMPENSATE_TRACKABLE_LIST);
            } else
#endif
            {
                /**   -- rotate position */
                EgoCoordCompensation_CoordRotation(
                    &pTrkble->vecX.data[TRACKABLE_POSX],
                    &pTrkble->vecX.data[TRACKABLE_POSY], f32Sin, f32Cos);
                /**   -- shift position */
                EgoCoordCompensation_CoordTranslation(
                    &pTrkble->vecX.data[TRACKABLE_POSX],
                    &pTrkble->vecX.data[TRACKABLE_POSY], -f32DX, -f32DY);

                /**   -- if desired rotate velocity vector */
                EgoCoordCompensation_CoordRotation(
                    &pTrkble->vecX.data[TRACKABLE_VELX],
                    &pTrkble->vecX.data[TRACKABLE_VELY], f32Sin, f32Cos);

                /**   -- if desired rotate acceleration vector */
                if (pTrkble->vecX.nRows > TRACKABLE_ACCY) {
                    EgoCoordCompensation_CoordRotation(
                        &pTrkble->vecX.data[TRACKABLE_ACCX],
                        &pTrkble->vecX.data[TRACKABLE_ACCY], f32Sin, f32Cos);
                } else {
                    /* MISRA */
                }

                /**   -- if desired rotate covariance matrix */
                if (TRUE == rotCovMat) {
                    u32Success |=
                        f32RotateSymMat(&pTrkble->matP, &pTrkble->matP, f32Sin,
                                        f32Cos); /* embedded optimization */
                } else {
                    /* MISRA */
                }

                /**   -- rotate heading */
                pTrkble->f32Heading =
                    tue_prv_fusion_norm_angle(pTrkble->f32Heading - f32DPhi);
            } /* end if correct coordinate system */
        }     /* end for all trackables */
    }         /* end if valid input pointer */

    return u32Success;
}

LOCAL void EgoCoordCompensation_CoordRotation(
    CONSTP2VAR(float32, AUTOMATIC, ObjFusn_VAR_NOINIT) pf32X,
    CONSTP2VAR(float32, AUTOMATIC, ObjFusn_VAR_NOINIT) pf32Y,
    const float32 f32Sin,
    const float32 f32Cos) {
    /** - temp x position because this might be overwritten */
    const float32 f32XBuf = *pf32X;
    const float32 f32YBuf = *pf32Y;

    *pf32X = (f32XBuf * f32Cos) - (f32YBuf * f32Sin); /** - xcos - ysin */
    *pf32Y = (f32XBuf * f32Sin) + (f32YBuf * f32Cos); /** - xsin + ycos */
}

LOCAL void EgoCoordCompensation_CoordTranslation(
    CONSTP2VAR(float32, AUTOMATIC, ObjFusn_VAR_NOINIT) pf32X,
    CONSTP2VAR(float32, AUTOMATIC, ObjFusn_VAR_NOINIT) pf32Y,
    const float32 f32DX,
    const float32 f32DY) {
    *pf32X += f32DX; /** - x + dx */
    *pf32Y += f32DY; /** - y + dy */
}
#define ObjFusn_STOP_SEC_CODE

/**
 * \}
 */
