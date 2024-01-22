/**
 * \{
 * \file        TueObjFusn_Kinematics.c
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
#include "TueObjFusn_Kinematics.h"
#include "TueObjFusn_Kinematics_int.h"
#include "TueObjFusn_AAU_Codes.h"
#include "TueObjFusn_ErrorCodes.h"
#include "tue_prv_error_management.h"
#include "TueObjFusn_TrackableConstants.h"
#include "TueObjFusn_ConfigVehicle.h"
#include "TueObjFusn_ConfigAlgorithm.h"
#include "tue_prv_fusion_math.h"

#define ObjFusn_START_SEC_CODE

LOCAL uint32 OverGround2Relative(
    CONSTP2CONST(TueObjFusn_EgoMotionType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        stEgoMotion,
    CONSTP2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) stTrkbl,
    const float32 f32deltaX) {
    uint32 u32Success;

    VAR(stf32Matrix_t, ObjFusn_VAR_NOINIT) _matP;
    VAR(stf32Matrix_t, ObjFusn_VAR_NOINIT) matAinv;
    VAR(stf32Vec_t, ObjFusn_VAR_NOINIT) vecC;
    /* temporary matrices */
    VAR(stf32Matrix_t, ObjFusn_VAR_NOINIT) matAinvP;
    VAR(stf32Matrix_t, ObjFusn_VAR_NOINIT) matAinvT;
    VAR(stf32Vec_t, ObjFusn_VAR_NOINIT) vecXtmp;
    uint16 u16nRows;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if ((NULL_PTR == stEgoMotion) || (NULL_PTR == stTrkbl)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION, TUEOBJFUSN_AAU_KINEMATICS,
            TUEOBJFUSN_AAU_KINEMATICS_OVER_GROUND_2_RELATIVE);
    } else
#endif
        /* input coordinate system has to be over ground */
        if (TUEOBJFUSN_U8COORDSYSTEM_REARAXLE_OVERGROUND !=
            stTrkbl->u8CoordSystem) {
        u32Success = TUEOBJFUSN_ERROR_COORD_SYSTEM_MISMATCH;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_COORD_SYSTEM_MISMATCH, TUEOBJFUSN_AAU_KINEMATICS,
            TUEOBJFUSN_AAU_KINEMATICS_OVER_GROUND_2_RELATIVE);
    } else
#endif
    {
        u16nRows = stTrkbl->vecX.nRows;

        /* set output coordinate system to relativ */
        stTrkbl->u8CoordSystem =
            TUEOBJFUSN_U8COORDSYSTEM_FRONTBUMP_RELATIVE;  // \todo: better
                                                          // handling of
                                                          // different
                                                          // coordinate systems
                                                          // required (flexible
                                                          // deltaX)

        /* setup matrices Ainv and c such that
         * x_relative = Ainv * (x_overGround - c) */
        u32Success = f32VecZeros(&vecC, u16nRows);
        u32Success |= f32VecZeros(&vecXtmp, u16nRows);
        u32Success |= f32MatEye(&matAinv, u16nRows, u16nRows);

        vecC.data[TRACKABLE_POSX] = f32deltaX;
        vecC.data[TRACKABLE_POSY] = FLT_ZERO;
        vecC.data[TRACKABLE_VELX] = stEgoMotion->f32Speed;
        vecC.data[TRACKABLE_VELY] = stEgoMotion->f32YawRate * f32deltaX;
        matAinv.data[TRACKABLE_VELX][TRACKABLE_POSY] = stEgoMotion->f32YawRate;
        matAinv.data[TRACKABLE_VELY][TRACKABLE_POSX] = -stEgoMotion->f32YawRate;

        /* more than 4 rows => AccX available */
        if (TRACKABLE_ACCX < u16nRows) {
            vecC.data[TRACKABLE_ACCX] =
                stEgoMotion->f32Acc -
                (stEgoMotion->f32YawRate * stEgoMotion->f32YawRate * f32deltaX);
            matAinv.data[TRACKABLE_ACCX][TRACKABLE_POSX] =
                -stEgoMotion->f32YawRate * stEgoMotion->f32YawRate;
            matAinv.data[TRACKABLE_ACCX][TRACKABLE_VELY] =
                FLT_TWO * stEgoMotion->f32YawRate;
        } else {
            /* MISRA */
        }

        /* more than 5 rows => AccY available */
        if (TRACKABLE_ACCY < u16nRows) {
            vecC.data[TRACKABLE_ACCY] =
                (stEgoMotion->f32YawRate * stEgoMotion->f32Speed);
            matAinv.data[TRACKABLE_ACCY][TRACKABLE_POSY] =
                -stEgoMotion->f32YawRate * stEgoMotion->f32YawRate;
            matAinv.data[TRACKABLE_ACCY][TRACKABLE_VELX] =
                -FLT_TWO * stEgoMotion->f32YawRate;
        } else {
            /* MISRA */
        }

        /* x_relative = Ainv * (x_overGround - c) */
        u32Success |= f32VecSub(&(stTrkbl->vecX), &vecC, &vecXtmp);
        // u32Success |= f32VecZeros(&(stTrkbl->vecX), TUEOBJFUSN_MATRIX_SIZE);
        u32Success |= f32MatMulVec(&matAinv, &vecXtmp, &(stTrkbl->vecX));

        /* P_overGround = Ainv * P_relative * Ainv^T */
        u32Success |= f32SymMatToMat(&(stTrkbl->matP), &_matP);
        u32Success |= f32MatMul(&matAinv, &_matP, &matAinvP);
        u32Success |= f32MatTranspose(&matAinv, &matAinvT);
        u32Success |= f32MatMul(&matAinvP, &matAinvT, &_matP);
        u32Success |= f32MatToSymMat(&_matP, &(stTrkbl->matP));
    }

    return u32Success;
}

/* PRQA S 1532 2 */ /* Function called from external AAU */
uint32 Kinematics_Relative2OverGround(
    CONSTP2CONST(TueObjFusn_EgoMotionType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        stEgoMotion,
    CONSTP2VAR(TueObjFusn_ObjectListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pObjList,
    const float32 f32deltaX) {
    /* check if EgoMotion and Trackable List is available and Trackable does not
     * exceed maximal size */
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    uint16 u16i;
    uint16 u16nRows;
    float32 f32PosX;
    float32 f32PosY;
    float32 f32VelX;
    float32 f32VelY;
    float32 f32AccX;
    float32 f32AccY;
    float32 f32YawRateSq;
    float32 f32YawRateTwo;
    P2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pCurrTrkbl;

#if TUE_PRV_KINEMATICS_USE_EGO_MOTION_FOR_VARIANCE == STD_ON
    float32 f32AbsEgoAcc;
    float32 f32AbsYawPosX;
#endif

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if ((NULL_PTR == stEgoMotion) || (NULL_PTR == pObjList)) {
        u32Success =
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION; /* check if EgoMotion and
                                                        Trackable List is
                                                        available */
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION, TUEOBJFUSN_AAU_KINEMATICS,
            TUEOBJFUSN_AAU_KINEMATICS_KINEMATICS_RELATIVE_2_OVER_GROUND);
    } else
#endif
        if (pObjList->u16NumObjects > TUE_PRV_FUSION_MAX_INPUT_OBJECTS) {
        u32Success =
            TUEOBJFUSN_ERROR_NUMBER_OF_OBJECTS_EXCEEDS_MAXIMUM_SIZE; /* check
                                                                        that
                                                                        number
                                                                        of
                                                                        trackables
                                                                        do not
                                                                        exceed
                                                                        maximal
                                                                        size */
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NUMBER_OF_OBJECTS_EXCEEDS_MAXIMUM_SIZE,
            TUEOBJFUSN_AAU_KINEMATICS,
            TUEOBJFUSN_AAU_KINEMATICS_KINEMATICS_RELATIVE_2_OVER_GROUND);
    } else
#endif
    {
        f32YawRateSq = stEgoMotion->f32YawRate * stEgoMotion->f32YawRate;
        f32YawRateTwo = FLT_TWO * stEgoMotion->f32YawRate;

        for (u16i = 0u; u16i < pObjList->u16NumObjects; u16i++) {
            pCurrTrkbl = &pObjList->aTrackable[u16i];

            if ((TUEOBJFUSN_U8COORDSYSTEM_REARAXLE_RELATIVE ==
                 pCurrTrkbl->u8CoordSystem) ||
                (TUEOBJFUSN_U8COORDSYSTEM_FRONTBUMP_RELATIVE ==
                 pCurrTrkbl->u8CoordSystem)) {
                u16nRows = pCurrTrkbl->vecX.nRows;

                /* Save Local copies of unmodified values */
                f32PosX = pCurrTrkbl->vecX.data[TRACKABLE_POSX];
                f32PosY = pCurrTrkbl->vecX.data[TRACKABLE_POSY];
                f32VelX = pCurrTrkbl->vecX.data[TRACKABLE_VELX];
                f32VelY = pCurrTrkbl->vecX.data[TRACKABLE_VELY];

                pCurrTrkbl->vecX.data[TRACKABLE_VELX] =
                    ((-stEgoMotion->f32YawRate * f32PosY) + f32VelX) +
                    (stEgoMotion->f32Speed);
                pCurrTrkbl->vecX.data[TRACKABLE_VELY] =
                    (((stEgoMotion->f32YawRate) * f32PosX) + f32VelY);

                if (TRACKABLE_ACCX < u16nRows) {
                    f32AccX = pCurrTrkbl->vecX.data[TRACKABLE_ACCX];
                    pCurrTrkbl->vecX.data[TRACKABLE_ACCX] =
                        (((((-f32YawRateSq) * f32PosX) -
                           (f32YawRateTwo * f32VelY)) +
                          f32AccX) +
                         (stEgoMotion->f32Acc));
                } else {
                    /* MISRA */
                }

                /** ToDo: Consider removing this code as this section is never
                 * executed with the current sensor setup */
                if (TRACKABLE_ACCY < u16nRows) {
                    f32AccY = pCurrTrkbl->vecX.data[TRACKABLE_ACCY];
                    pCurrTrkbl->vecX.data[TRACKABLE_ACCY] =
                        ((((-f32YawRateSq) * f32PosY) +
                          (f32YawRateTwo * f32VelX)) +
                         f32AccY) +
                        ((stEgoMotion->f32YawRate) * (stEgoMotion->f32Speed));
                } else {
                    /* MISRA */
                }

                if (TUEOBJFUSN_U8COORDSYSTEM_FRONTBUMP_RELATIVE ==
                    pCurrTrkbl->u8CoordSystem) {
                    /* PosY is not modified */
                    pCurrTrkbl->vecX.data[TRACKABLE_POSX] += f32deltaX;
                    pCurrTrkbl->vecX.data[TRACKABLE_VELY] +=
                        ((stEgoMotion->f32YawRate) * f32deltaX);

                    if (TRACKABLE_ACCX < u16nRows) {
                        pCurrTrkbl->vecX.data[TRACKABLE_ACCX] -=
                            (f32YawRateSq * f32deltaX);
                    } else {
                        /* MISRA */
                    }
                } else {
                    /* MISRA */
                }

                /* P_overGround = A * P_relative * A^T */
                u32Success |= EgoCompensationRel2OverGroundP(
                    &(pCurrTrkbl->matP), stEgoMotion);

                /* set output coordinate system to over ground */
                pCurrTrkbl->u8CoordSystem =
                    TUEOBJFUSN_U8COORDSYSTEM_REARAXLE_OVERGROUND;
            } else {
                /* MISRA */
            }

#if TUE_PRV_KINEMATICS_USE_EGO_MOTION_FOR_VARIANCE == STD_ON
            /* Add uncertainity caused by ego motion */
            f32AbsEgoAcc = tue_prv_fusion_abs(stEgoMotion->f32Acc);
            f32AbsYawPosX =
                tue_prv_fusion_abs(stEgoMotion->f32YawRate *
                                   (pCurrTrkbl->vecX.data[TRACKABLE_POSX]));

            pCurrTrkbl->matP.data[TRACKABLE_INDEX_VARIANCE_VELX] +=
                f32AbsEgoAcc * TUEOBJFUSN_EGOMOTION_VARIANCE_ACC_SLOPE;
            pCurrTrkbl->matP.data[TRACKABLE_INDEX_VARIANCE_VELY] +=
                f32AbsYawPosX * TUEOBJFUSN_EGOMOTION_VARIANCE_YAW_SLOPE;

            if (TRACKABLE_ACCX < pCurrTrkbl->matP.u16Size) {
                pCurrTrkbl->matP.data[TRACKABLE_INDEX_VARIANCE_ACCX] +=
                    f32AbsEgoAcc * TUEOBJFUSN_EGOMOTION_VARIANCE_ACC_SLOPE;
            } else {
                /* MISRA */
            }
#endif
        }
    }

    return u32Success;
}

/* PRQA S 1532 2 */ /* Function called from external AAU */
uint32 Kinematics_OverGround2Relative(
    CONSTP2CONST(TueObjFusn_EgoMotionType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pEgoMotion,
    CONSTP2VAR(TueObjFusn_ObjectListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pObjList,
    const float32 f32deltaX) {
    /* check if EgoMotion and Trackable List is available and Trackable does not
     * exceed maximal size */
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    uint16 u16i;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if ((NULL_PTR == pEgoMotion) || (NULL_PTR == pObjList)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;  // check if
                                                               // EgoMotion and
                                                               // Trackable List
                                                               // is available
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION, TUEOBJFUSN_AAU_KINEMATICS,
            TUEOBJFUSN_AAU_KINEMATICS_CHECK_INPUT_VALIDITY);
    } else
#endif
        if (pObjList->u16NumObjects > TUE_PRV_FUSION_OBJECT_LIST_SIZE) {
        u32Success =
            TUEOBJFUSN_ERROR_NUMBER_OF_OBJECTS_EXCEEDS_MAXIMUM_SIZE; /* check
                                                                        that
                                                                        number
                                                                        of
                                                                        trackables
                                                                        do not
                                                                        exceed
                                                                        maximal
                                                                        size */
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NUMBER_OF_OBJECTS_EXCEEDS_MAXIMUM_SIZE,
            TUEOBJFUSN_AAU_KINEMATICS,
            TUEOBJFUSN_AAU_KINEMATICS_CHECK_INPUT_VALIDITY);
    } else
#endif
    {
        for (u16i = 0u; u16i < pObjList->u16NumObjects; u16i++) {
            u32Success |= OverGround2Relative(
                pEgoMotion, &pObjList->aTrackable[u16i], f32deltaX);
        }
    }

    return u32Success;
}

LOCAL uint32 EgoCompensationRel2OverGroundP(
    CONSTP2VAR(stf32SymMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) _matP,
    CONSTP2CONST(TueObjFusn_EgoMotionType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pEgoMotion) {
    uint32 u32Success;
    const float32 f32w = pEgoMotion->f32YawRate;
    const float32 f32TWOw = FLT_TWO * f32w; /* two times yaw rate */
    const float32 f32wsq = f32w * f32w;     /* yaw rate squared */

#if TUE_PRV_KINEMATICS_USE_SENSOR_COVARIANCES == STD_ON
    float32 f32C32Part; /* temporary expressions inside calculation */
    float32 f32C33Part; /* temporary expressions inside calculation */
    float32 f32C22Part; /* temporary expressions inside calculation */
#endif

    const float32 f32Value00 = _matP->data[TRACKABLE_INDEX_VARIANCE_POSX];
#if TUE_PRV_KINEMATICS_USE_SENSOR_COVARIANCES == STD_ON
    const float32 f32Value01 =
        _matP->data[TRACKABLE_INDEX_COVARIANCE_POSX_POSY];
#endif
    const float32 f32Value11 = _matP->data[TRACKABLE_INDEX_VARIANCE_POSY];
#if TUE_PRV_KINEMATICS_USE_SENSOR_COVARIANCES == STD_ON
    const float32 f32Value02 =
        _matP->data[TRACKABLE_INDEX_COVARIANCE_POSX_VELX];
    const float32 f32Value12 =
        _matP->data[TRACKABLE_INDEX_COVARIANCE_POSY_VELX];
#endif
    const float32 f32Value22 = _matP->data[TRACKABLE_INDEX_VARIANCE_VELX];
#if TUE_PRV_KINEMATICS_USE_SENSOR_COVARIANCES == STD_ON
    const float32 f32Value03 =
        _matP->data[TRACKABLE_INDEX_COVARIANCE_POSX_VELY];
    const float32 f32Value13 =
        _matP->data[TRACKABLE_INDEX_COVARIANCE_POSY_VELY];
    const float32 f32Value32 =
        _matP->data[TRACKABLE_INDEX_COVARIANCE_VELX_VELY];
#endif
    const float32 f32Value33 = _matP->data[TRACKABLE_INDEX_VARIANCE_VELY];
#if TUE_PRV_KINEMATICS_USE_SENSOR_COVARIANCES == STD_ON
    const float32 f32Value04 =
        _matP->data[TRACKABLE_INDEX_COVARIANCE_POSX_ACCX];
    const float32 f32Value14 =
        _matP->data[TRACKABLE_INDEX_COVARIANCE_POSY_ACCX];
    const float32 f32Value24 =
        _matP->data[TRACKABLE_INDEX_COVARIANCE_VELX_ACCX];
    const float32 f32Value34 =
        _matP->data[TRACKABLE_INDEX_COVARIANCE_VELY_ACCX];
#endif
    const float32 f32Value44 = _matP->data[TRACKABLE_INDEX_VARIANCE_ACCX];
#if TUE_PRV_KINEMATICS_USE_SENSOR_COVARIANCES == STD_ON
    const float32 f32Value05 =
        _matP->data[TRACKABLE_INDEX_COVARIANCE_POSX_ACCY];
    const float32 f32Value15 =
        _matP->data[TRACKABLE_INDEX_COVARIANCE_POSY_ACCY];
    const float32 f32Value25 =
        _matP->data[TRACKABLE_INDEX_COVARIANCE_VELX_ACCY];
    const float32 f32Value35 =
        _matP->data[TRACKABLE_INDEX_COVARIANCE_VELY_ACCY];
    const float32 f32Value54 =
        _matP->data[TRACKABLE_INDEX_COVARIANCE_ACCX_ACCY];
#endif
    const float32 f32Value55 = _matP->data[TRACKABLE_INDEX_VARIANCE_ACCY];

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if (NULL_PTR == _matP) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_EGOCOMP_SYMMAT);
    } else
#endif
        if (TUEOBJFUSN_MATRIX_SIZE < _matP->u16Size) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_EGOCOMP_SYMMAT);
    } else
#endif
    {
#if TUE_PRV_KINEMATICS_USE_SENSOR_COVARIANCES == STD_ON
        // Dimension x and y are decoupled
        f32C22Part = (-f32Value12 * f32w) + f32Value22;
        f32C32Part = f32Value32 - (f32Value13 * f32w);
        f32C33Part = (f32Value03 * f32w) + f32Value33;

        _matP->data[TRACKABLE_INDEX_COVARIANCE_POSX_VELX] =
            f32Value02 - (f32Value01 * f32w);
        _matP->data[TRACKABLE_INDEX_COVARIANCE_POSY_VELX] =
            f32Value12 - (f32Value11 * f32w);
        _matP->data[TRACKABLE_INDEX_VARIANCE_VELX] =
            (-_matP->data[TRACKABLE_INDEX_COVARIANCE_POSY_VELX] * f32w) +
            f32C22Part;

        _matP->data[TRACKABLE_INDEX_COVARIANCE_POSX_VELY] =
            f32Value03 + (f32Value00 * f32w);
        _matP->data[TRACKABLE_INDEX_COVARIANCE_POSY_VELY] =
            f32Value13 + (f32Value01 * f32w);
#else
        _matP->data[TRACKABLE_INDEX_COVARIANCE_POSY_VELX] =
            -(f32Value11 * f32w);
        _matP->data[TRACKABLE_INDEX_VARIANCE_VELX] =
            (-_matP->data[TRACKABLE_INDEX_COVARIANCE_POSY_VELX] * f32w) +
            f32Value22;
        _matP->data[TRACKABLE_INDEX_COVARIANCE_POSX_VELY] = (f32Value00 * f32w);
#endif

#if TUE_PRV_KINEMATICS_USE_SENSOR_COVARIANCES == STD_ON
        _matP->data[TRACKABLE_INDEX_COVARIANCE_VELX_VELY] =
            (_matP->data[TRACKABLE_INDEX_COVARIANCE_POSX_VELX] * f32w) +
            f32C32Part;
        _matP->data[TRACKABLE_INDEX_VARIANCE_VELY] =
            (_matP->data[TRACKABLE_INDEX_COVARIANCE_POSX_VELY] * f32w) +
            f32C33Part;
#else
        _matP->data[TRACKABLE_INDEX_VARIANCE_VELY] =
            (_matP->data[TRACKABLE_INDEX_COVARIANCE_POSX_VELY] * f32w) +
            f32Value33;
#endif

        if (TRACKABLE_ACCX < _matP->u16Size) {
#if TUE_PRV_KINEMATICS_USE_SENSOR_COVARIANCES == STD_ON
            _matP->data[TRACKABLE_INDEX_COVARIANCE_POSX_ACCX] =
                ((-f32Value00 * f32wsq) - (f32Value03 * f32TWOw)) + f32Value04;

            _matP->data[TRACKABLE_INDEX_COVARIANCE_POSY_ACCX] =
                (-(f32Value01 * f32wsq) - (f32Value13 * f32TWOw)) + f32Value14;

            _matP->data[TRACKABLE_INDEX_COVARIANCE_VELX_ACCX] =
                (-(_matP->data[TRACKABLE_INDEX_COVARIANCE_POSX_VELX] * f32wsq) -
                 (f32C32Part * f32TWOw) - (f32Value14 * f32w)) +
                f32Value24;

            _matP->data[TRACKABLE_INDEX_COVARIANCE_VELY_ACCX] =
                (((-((_matP->data[TRACKABLE_INDEX_COVARIANCE_POSX_VELY]) *
                     f32wsq)) -
                  (f32C33Part * f32TWOw)) +
                 (f32Value04 * f32w)) +
                (f32Value34);

            _matP->data[TRACKABLE_INDEX_VARIANCE_ACCX] =
                (((-f32TWOw) *
                  ((((-f32Value03) * f32wsq) - (f32Value33 * f32TWOw)) +
                   (FLT_TWO * f32Value34))) -
                 (f32wsq *
                  ((_matP->data[TRACKABLE_INDEX_COVARIANCE_POSX_ACCX]) +
                   f32Value04))) +
                f32Value44;
#else
            _matP->data[TRACKABLE_INDEX_COVARIANCE_POSX_ACCX] =
                (-f32Value00 * f32wsq);

            _matP->data[TRACKABLE_INDEX_COVARIANCE_POSY_ACCX] = FLT_ZERO;

            _matP->data[TRACKABLE_INDEX_COVARIANCE_VELX_ACCX] =
                -(_matP->data[TRACKABLE_INDEX_COVARIANCE_POSX_VELX] * f32wsq);

            _matP->data[TRACKABLE_INDEX_COVARIANCE_VELY_ACCX] =
                ((-((_matP->data[TRACKABLE_INDEX_COVARIANCE_POSX_VELY]) *
                    f32wsq)) -
                 (f32Value33 * f32TWOw));

            _matP->data[TRACKABLE_INDEX_VARIANCE_ACCX] =
                (((-f32TWOw) * (-(f32Value33 * f32TWOw))) -
                 (f32wsq *
                  (_matP->data[TRACKABLE_INDEX_COVARIANCE_POSX_ACCX]))) +
                f32Value44;

#endif
        } else {
            /* MISRA */
        }

        if (TRACKABLE_ACCY < _matP->u16Size) {
#if TUE_PRV_KINEMATICS_USE_SENSOR_COVARIANCES == STD_ON
            _matP->data[TRACKABLE_INDEX_COVARIANCE_POSX_ACCY] =
                (-f32Value01 * f32wsq) + (f32Value02 * f32TWOw) + f32Value05;

            _matP->data[TRACKABLE_INDEX_COVARIANCE_POSY_ACCY] =
                (-f32Value11 * f32wsq) + (f32Value12 * f32TWOw) + f32Value15;

            _matP->data[TRACKABLE_INDEX_COVARIANCE_VELX_ACCY] =
                (((-((_matP->data[TRACKABLE_INDEX_COVARIANCE_POSY_VELX]) *
                     f32wsq)) +
                  (f32C22Part * f32TWOw)) -
                 (f32Value15 * f32w)) +
                f32Value25;

            _matP->data[TRACKABLE_INDEX_COVARIANCE_VELY_ACCY] =
                -(_matP->data[TRACKABLE_INDEX_COVARIANCE_POSY_VELY] * f32wsq) +
                (f32TWOw * ((f32Value02 * f32w) + f32Value32)) +
                (f32Value05 * f32w) + f32Value35;

            _matP->data[TRACKABLE_INDEX_COVARIANCE_ACCX_ACCY] =
                ((f32TWOw *
                  (((((-f32Value02) * f32wsq) - (f32Value32 * f32TWOw)) +
                    f32Value24) -
                   f32Value35)) -
                 (f32wsq *
                  ((_matP->data[TRACKABLE_INDEX_COVARIANCE_POSY_ACCX]) +
                   f32Value05))) +
                f32Value54;

            _matP->data[TRACKABLE_INDEX_VARIANCE_ACCY] =
                ((f32TWOw *
                  ((((-f32Value12) * f32wsq) + (f32Value22 * f32TWOw)) +
                   (FLT_TWO * f32Value25))) -
                 (f32wsq *
                  ((_matP->data[TRACKABLE_INDEX_COVARIANCE_POSY_ACCY]) +
                   f32Value15))) +
                f32Value55;
#else
            _matP->data[TRACKABLE_INDEX_COVARIANCE_POSX_ACCY] = FLT_ZERO;

            _matP->data[TRACKABLE_INDEX_COVARIANCE_POSY_ACCY] =
                (-f32Value11 * f32wsq);

            _matP->data[TRACKABLE_INDEX_COVARIANCE_VELX_ACCY] =
                ((-((_matP->data[TRACKABLE_INDEX_COVARIANCE_POSY_VELX]) *
                    f32wsq)) +
                 (f32Value22 * f32TWOw));

            _matP->data[TRACKABLE_INDEX_COVARIANCE_VELY_ACCY] =
                -(_matP->data[TRACKABLE_INDEX_COVARIANCE_POSY_VELY] * f32wsq);

            _matP->data[TRACKABLE_INDEX_COVARIANCE_ACCX_ACCY] = FLT_ZERO;

            _matP->data[TRACKABLE_INDEX_VARIANCE_ACCY] =
                ((f32TWOw * (f32Value22 * f32TWOw)) -
                 (f32wsq *
                  (_matP->data[TRACKABLE_INDEX_COVARIANCE_POSY_ACCY]))) +
                f32Value55;

#endif
        } else {
            /* MISRA */
        }

        u32Success = TUEOBJFUSN_ERROR_NOERROR;
    }

    return u32Success;
}
#define ObjFusn_STOP_SEC_CODE

/** /} */
