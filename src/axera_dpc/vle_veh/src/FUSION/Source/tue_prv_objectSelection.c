/** \defgroup target Object Selection
 * \brief  object selection module for TueObjFusn_ObjInputType and
 * TueObjFusn_ObjListInputType
 * \addtogroup target
 * \{
 * \file        tue_prv_objectSelection.c
 * \brief       prediction module for TueObjFusn_ObjInputType and
 * TueObjFusn_ObjListInputType
 */
/* PRQA S 0292 ++ */ /* MKS */
                     /*
                      */
/* PRQA S 0292 -- */ /* MKS */
/* <br>=====================================================<br>
 * <b>Copyright 2014 by Tuerme.</b>
 *
 *  All rights reserved. Property of Tuerme.<br>
 *  Restricted rights to use, duplicate or disclose of this code<br>
 *  are granted through contract.
 * <br>=====================================================<br>
 */

/*==================[inclusions]============================================*/
#include "tue_prv_objectSelection.h"
#include "tue_prv_objectSelection_int.h"
#include "tue_prv_common_array_utils.h"
#include "tue_prv_error_management.h"
#include "tue_prv_idProvider.h"
#include "TueObjFusn_TrackableListUtils.h"
#include "TueObjFusn_TrackableProps.h"
#include "TueObjFusn_ErrorCodes.h"
#include "TueObjFusn_AAU_Codes.h"
#include "tue_prv_fusion_math.h"

/*==================[function definitions]=========================*/

/* stores number of trackables in this bin */
//#define ObjFusn_START_SEC_VAR8
#define ASW_QM_CORE5_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
LOCAL VAR(sint16, ObjFusn_VAR_ZERO_INIT)
    as16TrkblBinCounter[TUE_PRV_OBJECTSELECTION_MAX_BINS] = {0};
#define ASW_QM_CORE5_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
//#define ObjFusn_STOP_SEC_VAR8

#define ObjFusn_START_SEC_CODE

/*!****************************************************************************
 * @details
 *     implements MIO selection based on the following criteria
 *     1. Drop all coasted objects outside of MPF feature observation zone
 *     2. Drop coasted stationary objects
 *
 * @param[in]
 *     TueObjFusn_TrackableListType input pointer list
 *     u16MaxTrackables number of trackables bigger than
 *TUEOBJFUSN_TRACKABLELIST_U16VALIDTRACKABLES_MAX
 *     DK: number of total trackables - not difference
 * @return
 *     bSuccess    boolean success flag
 ******************************************************************************/
uint32 tue_prv_object_selection_select_objects(
    CONSTP2VAR(TueObjFusn_TrackableListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pTrkblList,
    VAR(sint16, ObjFusn_VAR_NOINIT) s16ObjectsToDelete,
    VAR(boolean, ObjFusn_VAR_NOINIT) abObjectsToBeDropped[]) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    uint16 u16i;
    uint8 u8Bin;
    uint8 u8LastBinToDelete;
    sint16 s16Idx;

    P2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pCurTrkbl;
    tue_prv_common_array_utils_defaultInit_abool(
        abObjectsToBeDropped, TUEOBJFUSN_TRACKABLELIST_U16VALIDTRACKABLES_MAX,
        FALSE);

#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if (NULL_PTR == pTrkblList) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_OBJECT_SELECTION,
            TUEOBJFUSN_AAU_OBJECT_SELECTION_VIO_TRACKABLE_SELECTION);
    } else
#endif
        /* Mark all objects as 'selected' */
        if (s16ObjectsToDelete > 0) {
        /* reset update counter */
        initTrackableBinCounter();

        /* set for each trackable to which bin it belongs and increase update
         * counter */
        for (u16i = 0u; u16i < pTrkblList->u16ValidTrackables; u16i++) {
            s16Idx = pTrkblList->as16TrackableMap[u16i];
            pCurTrkbl = &(pTrkblList->aTrackable[s16Idx]);
            u32Success |= setTrackableBinPosition(pCurTrkbl);
        }

        /* determine which bins should be deleted */
        u32Success |=
            setDroppedTrackableBins(&u8LastBinToDelete, s16ObjectsToDelete);

        /* loop through bins and delete trackables */
        for (u8Bin = 0u; u8Bin <= u8LastBinToDelete; u8Bin++) {
            /* loop through all trackables and delete which are in this bin */
            for (u16i = 0u; (u16i < pTrkblList->u16ValidTrackables) &&
                            (0 < s16ObjectsToDelete);
                 u16i++) {
                pCurTrkbl = &(
                    pTrkblList->aTrackable[pTrkblList->as16TrackableMap[u16i]]);

                /* Remove coasted stationary objects */
                if (u8Bin == pCurTrkbl->u8TrkblBinPosition) {
                    s16ObjectsToDelete--;
                    abObjectsToBeDropped[u16i] = TRUE;
                } else {
                    /** MISRA */
                }
            }
        }

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
        if (0 < s16ObjectsToDelete) {
            u32Success |= TUEOBJFUSN_ERROR_INTERNAL_ERROR;
            (void)tue_prv_error_management_addError(
                TUEOBJFUSN_ERROR_INTERNAL_ERROR,
                TUEOBJFUSN_AAU_OBJECT_SELECTION,
                TUEOBJFUSN_AAU_OBJECT_SELECTION_VIO_TRACKABLE_SELECTION);
        } else {
            /* MISRA */
        }
#endif
    } else {
        /* MISRA */
    } /** End else */

    return u32Success;
}

LOCAL uint32 calculateObjectRange(
    CONSTP2CONST(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pCurTrkbl,
    CONSTP2VAR(float32, AUTOMATIC, ObjFusn_VAR_NOINIT) f32Range) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    /* PRQA S 3204 1 */ /* variable may be set to other values depending on
                           run-time pointer check activation */
    float32 f32PosX;
    float32 f32PosY;
    float32 f32RangeTmp;

#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if ((NULL_PTR == pCurTrkbl) || (NULL_PTR == f32Range)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_OBJECT_SELECTION,
            TUEOBJFUSN_AAU_OBJECT_SELECTION_CALCULATE_OBJECT_RANGE);
    } else
#endif
    {
        f32PosX = ((pCurTrkbl->vecX).data)[TRACKABLE_POSX];
        f32PosY = ((pCurTrkbl->vecX).data)[TRACKABLE_POSY];
        f32RangeTmp = ((f32PosX * f32PosX) + (f32PosY * f32PosY));
        *f32Range = tue_prv_fusion_sqrt(f32RangeTmp);
    }

    return u32Success;
}

LOCAL uint32 calculateObjectAngle(
    CONSTP2CONST(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pCurTrkbl,
    CONSTP2VAR(float32, AUTOMATIC, ObjFusn_VAR_NOINIT) f32Angle) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    /* PRQA S 3204 1 */ /* variable may be set to other values depending on
                           run-time pointer check activation */
    float32 f32PosX;
    float32 f32PosY;
    float32 f32AngleTmp;

#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if ((NULL_PTR == pCurTrkbl) || (NULL_PTR == f32Angle)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_OBJECT_SELECTION,
            TUEOBJFUSN_AAU_OBJECT_SELECTION_CALCULATE_OBJECT_ANGLE);
    } else
#endif
    {
        f32PosX = pCurTrkbl->vecX.data[TRACKABLE_POSX];
        f32PosY = pCurTrkbl->vecX.data[TRACKABLE_POSY];

        f32AngleTmp = tue_prv_fusion_atan2(f32PosY, f32PosX);
        f32AngleTmp = tue_prv_fusion_rtod(f32AngleTmp);
        *f32Angle = tue_prv_fusion_abs(f32AngleTmp);
    }

    return u32Success;
}
#define ObjFusn_STOP_SEC_CODE

#define ObjFusn_START_SEC_SLOW_CODE

LOCAL void initTrackableBinCounter(void) {
    uint8 u8Bin;

    for (u8Bin = 0u; u8Bin < TUE_PRV_OBJECTSELECTION_MAX_BINS; u8Bin++) {
        as16TrkblBinCounter[u8Bin] = 0;
    }
}
#define ObjFusn_STOP_SEC_SLOW_CODE

#define ObjFusn_START_SEC_CODE

LOCAL uint32 setDroppedTrackableBins(
    CONSTP2VAR(uint8, AUTOMATIC, ObjFusn_VAR_NOINIT) u8LastBinToDelete,
    VAR(sint16, ObjFusn_VAR_NOINIT) s16NumberTrackablesToDelete) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    /* PRQA S 3204 1 */ /* variable may be set to other values depending on
                           run-time check activation */
    uint8 u8Bin;
    uint8 u8LastBin;

    u8LastBin = TUE_PRV_OBJECTSELECTION_MAX_BINS;
    for (u8Bin = 0u; (u8Bin < TUE_PRV_OBJECTSELECTION_MAX_BINS) &&
                     (0 < s16NumberTrackablesToDelete);
         u8Bin++) {
        if (as16TrkblBinCounter[u8Bin] < s16NumberTrackablesToDelete) {
            /* bin has to be dropped completly -> sets bin to be dropped
             * (TUE_PRV_OBJECTSELECTION_DROP_BIN)*/
            s16NumberTrackablesToDelete -= as16TrkblBinCounter[u8Bin];
        } else {
            /* bin has to be dropped party -> write number which has to be
             * dropped */
            s16NumberTrackablesToDelete = 0;
            u8LastBin = u8Bin; /* this is the last bin which has to be (partly)
                                  deleted */
        }
    }
    *u8LastBinToDelete = u8LastBin;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
    if (0 < s16NumberTrackablesToDelete) {
        /* sufficient number of objects could be deleted */
        u32Success = TUEOBJFUSN_ERROR_INTERNAL_ERROR;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_OBJECT_SELECTION,
            TUEOBJFUSN_AAU_OBJECT_SELECTION_SET_DROPPED_TRACKABLES_BINS);
    } else {
        /* MISRA */
    }
#endif

    return u32Success;
}

LOCAL uint32 setTrackableBinPosition(CONSTP2VAR(TueObjFusn_TrackableType,
                                                AUTOMATIC,
                                                ObjFusn_VAR_NOINIT) pCurTrkbl) {
    float32 f32ObjectRange;
    float32 f32ObjectAngle;
    uint32 u32Success;
    uint32 u32SensorCurr;
    uint32 u32SensorHist;
    uint16 _u16MotionType;

#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if (NULL_PTR == pCurTrkbl) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_OBJECT_SELECTION,
            TUEOBJFUSN_AAU_OBJECT_SELECTION_SET_TRACKABLE_BIN_POSITION);
    } else
#endif
    {
        u32Success = calculateObjectRange(pCurTrkbl, &f32ObjectRange);
        u32Success |= calculateObjectAngle(pCurTrkbl, &f32ObjectAngle);

        u32SensorCurr = (pCurTrkbl->u32SensorsCurr);
        u32SensorHist = (pCurTrkbl->u32SensorsHist);
        _u16MotionType = (pCurTrkbl->u16MotionType);

        if (((TUE_PRV_OBJECTSELECTION_LONG_DISTANCE_ROI1_THRESHOLD_OBJ <
              f32ObjectRange) &&
             (TUE_PRV_OBJECTSELECTION_ANGLE_ROI2_THRESHOLD_OBJ <
              f32ObjectAngle)) ||
            ((TUE_PRV_OBJECTSELECTION_LONG_DISTANCE_ROI1_THRESHOLD_OBJ >
              f32ObjectRange) &&
             (TUE_PRV_OBJECTSELECTION_ANGLE_ROI1_THRESHOLD_OBJ <
              f32ObjectAngle)) ||
            (TUE_PRV_OBJECTSELECTION_LONG_DISTANCE_ROI0_THRESHOLD_OBJ <
             f32ObjectRange)) {
            pCurTrkbl->u8TrkblBinPosition =
                TUE_PRV_OBJECTSELECTION_GLOBAL_OBSZONE_BIN;
            as16TrkblBinCounter[TUE_PRV_OBJECTSELECTION_GLOBAL_OBSZONE_BIN]++;
        } /* Remove all radar only stationary objects */
        /* either radar only or coasted radar only and stationary */
        else if ((u32SensorHist ==
                  TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR_FRONT_CENTER) &&
                 (_u16MotionType ==
                  TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_STATIONARY)) {
            pCurTrkbl->u8TrkblBinPosition =
                TUE_PRV_OBJECTSELECTION_RADARONLY_BIN;
            as16TrkblBinCounter[TUE_PRV_OBJECTSELECTION_RADARONLY_BIN]++;
        } else if (TUE_PRV_OBJECTSELECTION_RANGE_THRESH_OBSZONE4 <
                   f32ObjectRange) {
            if (u32SensorCurr == TUEOBJFUSN_TRACKABLE_U32SENSOR_COASTED) {
                if (u32SensorHist ==
                    TUE_PRV_OBJECT_SELECTION_SENSOR_PATTERN_FUSED) {
                    pCurTrkbl->u8TrkblBinPosition =
                        TUE_PRV_OBJECTSELECTION_FUSED_OBSZONE1_BIN;
                    as16TrkblBinCounter
                        [TUE_PRV_OBJECTSELECTION_FUSED_OBSZONE1_BIN]++;
                } else {
                    pCurTrkbl->u8TrkblBinPosition =
                        TUE_PRV_OBJECTSELECTION_OBSZONE1_BIN;
                    as16TrkblBinCounter[TUE_PRV_OBJECTSELECTION_OBSZONE1_BIN]++;
                }
            } else {
                pCurTrkbl->u8TrkblBinPosition =
                    TUE_PRV_OBJECTSELECTION_NOTCOASTED_OBSZONE1_BIN;
                as16TrkblBinCounter
                    [TUE_PRV_OBJECTSELECTION_NOTCOASTED_OBSZONE1_BIN]++;
            }
        } else if ((TUE_PRV_OBJECTSELECTION_RANGE_THRESH_OBSZONE4 >
                    f32ObjectRange) &&
                   (TUE_PRV_OBJECTSELECTION_RANGE_THRESH_OBSZONE5 <
                    f32ObjectRange)) {
            if (u32SensorCurr == TUEOBJFUSN_TRACKABLE_U32SENSOR_COASTED) {
                if (u32SensorHist ==
                    TUE_PRV_OBJECT_SELECTION_SENSOR_PATTERN_FUSED) {
                    pCurTrkbl->u8TrkblBinPosition =
                        TUE_PRV_OBJECTSELECTION_FUSED_OBSZONE2_BIN;
                    as16TrkblBinCounter
                        [TUE_PRV_OBJECTSELECTION_FUSED_OBSZONE2_BIN]++;
                } else {
                    pCurTrkbl->u8TrkblBinPosition =
                        TUE_PRV_OBJECTSELECTION_OBSZONE2_BIN;
                    as16TrkblBinCounter[TUE_PRV_OBJECTSELECTION_OBSZONE2_BIN]++;
                }
            } else {
                pCurTrkbl->u8TrkblBinPosition =
                    TUE_PRV_OBJECTSELECTION_NOTCOASTED_OBSZONE2_BIN;
                as16TrkblBinCounter
                    [TUE_PRV_OBJECTSELECTION_NOTCOASTED_OBSZONE2_BIN]++;
            }
        } else if ((TUE_PRV_OBJECTSELECTION_RANGE_THRESH_OBSZONE5 >
                    f32ObjectRange) &&
                   (TUE_PRV_OBJECTSELECTION_RANGE_THRESH_OBSZONE6 <
                    f32ObjectRange)) {
            if (u32SensorCurr == TUEOBJFUSN_TRACKABLE_U32SENSOR_COASTED) {
                if (u32SensorHist ==
                    TUE_PRV_OBJECT_SELECTION_SENSOR_PATTERN_FUSED) {
                    pCurTrkbl->u8TrkblBinPosition =
                        TUE_PRV_OBJECTSELECTION_FUSED_OBSZONE3_BIN;
                    as16TrkblBinCounter
                        [TUE_PRV_OBJECTSELECTION_FUSED_OBSZONE3_BIN]++;
                } else {
                    pCurTrkbl->u8TrkblBinPosition =
                        TUE_PRV_OBJECTSELECTION_OBSZONE3_BIN;
                    as16TrkblBinCounter[TUE_PRV_OBJECTSELECTION_OBSZONE3_BIN]++;
                }
            } else {
                pCurTrkbl->u8TrkblBinPosition =
                    TUE_PRV_OBJECTSELECTION_NOTCOASTED_OBSZONE3_BIN;
                as16TrkblBinCounter
                    [TUE_PRV_OBJECTSELECTION_NOTCOASTED_OBSZONE3_BIN]++;
            }
        } else if (TUE_PRV_OBJECTSELECTION_RANGE_THRESH_OBSZONE6 >
                   f32ObjectRange) {
            if (u32SensorCurr == TUEOBJFUSN_TRACKABLE_U32SENSOR_COASTED) {
                if (u32SensorHist ==
                    TUE_PRV_OBJECT_SELECTION_SENSOR_PATTERN_FUSED) {
                    pCurTrkbl->u8TrkblBinPosition =
                        TUE_PRV_OBJECTSELECTION_FUSED_OBSZONE4_BIN;
                    as16TrkblBinCounter
                        [TUE_PRV_OBJECTSELECTION_FUSED_OBSZONE4_BIN]++;
                } else {
                    pCurTrkbl->u8TrkblBinPosition =
                        TUE_PRV_OBJECTSELECTION_OBSZONE4_BIN;
                    as16TrkblBinCounter[TUE_PRV_OBJECTSELECTION_OBSZONE4_BIN]++;
                }
            } else {
                pCurTrkbl->u8TrkblBinPosition =
                    TUE_PRV_OBJECTSELECTION_NOTCOASTED_OBSZONE4_BIN;
                as16TrkblBinCounter
                    [TUE_PRV_OBJECTSELECTION_NOTCOASTED_OBSZONE4_BIN]++;
            }
        } else {
            /** MISRA */
        }
    }
    return u32Success;
}
#define ObjFusn_STOP_SEC_CODE

/** /} */
