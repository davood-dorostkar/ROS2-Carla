/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * wen mingshu <wenmingshu@senseauto.com>
 */
/** \defgroup tuePrvlkfTrkMgt TUE Fusion Track Management
 * \{
 * \file       tue_prv_lkf_trackManagement.c
 *
 */
/* PRQA S 0292 ++ */ /* MKS */
/*
 */
/* PRQA S 0292 -- */
/*
 *
 *          (C) Copyright Tuerme Inc. All rights reserved.
 *
 */
/*==================[inclusions]============================================*/
#include "tue_prv_lkf_trackManagement.h"
#include "tue_prv_lkf_trackManagement_int.h"

#include "TueObjFusn_TrackableListUtils.h"
#include "tue_prv_idProvider.h"
#include "tue_prv_common_array_utils.h"
#include "tue_prv_common_matrix.h"
#include "tue_prv_motionType.h"
#include "tue_prv_gainEstimation.h"
#include "tue_prv_egoCoordCompensation.h"
#include "tue_prv_fusion_math.h"
#include "TueObjFusn_Eps.h"
#include "TueObjFusn_ParameterInterface.h"
#include "TueObjFusn_ErrorCodes.h"
#include "TueObjFusn_AAU_Codes.h"

#include "tue_prv_quality_management.h"
#include "tue_prv_objectSelection.h"
#include "tue_prv_trackMerge.h"
#include "tue_prv_lkf_coordinatedTurn.h"
#include "tue_prv_validation_management.h"
#include "tue_prv_error_management.h"

#include "TueObjFusn_TrackableProps.h"
/*****************************************************************************
VARIABLES
*****************************************************************************/
/** @name tue_prv_lkf_trackManagement.c global variables */
/**
 * LKF track list
 */
//#define ObjFusn_START_SEC_VAR_UNSPECIFIED
#define ASW_QM_CORE5_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
LOCAL VAR(TueObjFusn_TrackableListType, ObjFusn_VAR_ZERO_INIT) TRK_LIST = {0};
/* PRQA S 0612*/ /* Size of trackable list exceeds 32767 bytes */
//#define ObjFusn_STOP_SEC_VAR_UNSPECIFIED
#define ASW_QM_CORE5_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
FUNCTIONS
*****************************************************************************/
#define ObjFusn_START_SEC_SLOW_CODE

/** @name tue_prv_lkf4d_trackManagement.c functions */
/* PRQA S 1532 2 */ /* Init function called from external AAU */
void lkfTrackManagement_init(void) {
    (void)Trackable_listInit(&TRK_LIST);

    TRK_LIST.u16ValidTrackables = 0u;
    TRK_LIST.f32MeasurementLatency = FLT_ZERO;
    /* Init list update counter to zero to ensure list validation does not fail
     */
    TRK_LIST.u16ListUpdateCounter = 0u;
}

/** This is a general pre-cycle method for the Kalman filter. It reduces the
 * lifespan of active tracks resulting in a cleanup if they are not updated
 * for FILTER_TRACK_LIFESPAN many cycles */
/* PRQA S 1532 2 */ /* Interface function called from external AAU */
void lkfTrackManagement_startCycle(const float32 f32CycleDt) {
    uint16 u16i;
    P2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pCurTrkbl;
    /* Ensure trackable map is valid, this is especially important for the first
     * cycle */

    /*
     * Trackable map is checked within the finalize cycle method and shall not
     * be modified during the transition of cycles
     */
    for (u16i = 0u; u16i < TRK_LIST.u16ValidTrackables; u16i++) {
        pCurTrkbl = &TRK_LIST.aTrackable[TRK_LIST.as16TrackableMap[u16i]];
        /* this track was active in the last cycle.
         * reduce its lifespan */
        if (TUEOBJFUSN_TRACKABLE_U16LIFESPAN_DEAD < pCurTrkbl->u16Lifespan) {
            // decrease update counter
            pCurTrkbl->u16Lifespan--;
            pCurTrkbl->bUpdated = FALSE;
            // pCurTrkbl->f32TrackQuality = FLT_ZERO;
        } else {
            /* MISRA */
        }
    }

    /* TRK_LIST is now older by one cycle time */
    TRK_LIST.f32MeasurementLatency += f32CycleDt;
}

/** This is a general post-cycle method for the Kalman filter. It cleans up all
 * data that need to be discaded for the next cycle.*/

/* PRQA S 1532 2 */ /* Interface function called from external AAU */
uint32 lkfTrackManagement_finalizeCycle(const float32 f32CycleDt) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    uint16 u16i;
    P2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pCurTrkbl;
    VAR(boolean, ObjFusn_VAR_NOINIT)
    baMarkedToDrop
        [TUEOBJFUSN_TRACKABLELIST_U16VALIDTRACKABLES_MAX];  // todo change to
                                                            // add all list of
                                                            // removing trkbles

    const boolean bUseCoastingTmp = Fusion_get_bUseCoasting();

    tue_prv_common_array_utils_defaultInit_abool(
        baMarkedToDrop, TUEOBJFUSN_TRACKABLELIST_U16VALIDTRACKABLES_MAX, FALSE);

    for (u16i = 0u; u16i < (TRK_LIST.u16ValidTrackables); u16i++) {
        pCurTrkbl = &TRK_LIST.aTrackable[TRK_LIST.as16TrackableMap[u16i]];

        // measurement is available for 'coasted sensor' OR
        // check if the track was 'fused' in history (lifespan > new) - if it
        // was updated (TRUE == bUpdate) -> set to coasted
        if ((TRUE == pCurTrkbl->bUpdated) && (TRUE == bUseCoastingTmp) &&
            (/*(pCurTrkbl->u16Lifespan > TUEOBJFUSN_TRACKABLE_U16LIFESPAN_NEW)
                || */
             ((pCurTrkbl->u32SensorsCurr & TUE_PRV_COASTING_SENSOR_PATTERN) ==
              TUE_PRV_COASTING_SENSOR_PATTERN))) {
            pCurTrkbl->u16Lifespan = TUEOBJFUSN_TRACKABLE_U16LIFESPAN_COASTED;
            // update age of track
            u32Success |= updateAge(pCurTrkbl, f32CycleDt);
        }
        // ensure tracks without updates don't die if they are not supposed to
        // (expecting an update from at least one sensor in the next cycles,
        // i.e. u32SensorsCurr is not zero)
        else if ((TUEOBJFUSN_TRACKABLE_U32SENSORSCURR_MIN <
                  pCurTrkbl->u32SensorsCurr) &&
                 (pCurTrkbl->u16Lifespan <
                  TUEOBJFUSN_TRACKABLE_U16LIFESPAN_NEW)) {
            pCurTrkbl->u16Lifespan++;
            // update age of track
            u32Success |= updateAge(pCurTrkbl, f32CycleDt);
        } else if ((TUEOBJFUSN_TRACKABLE_U16LIFESPAN_DEAD ==
                    pCurTrkbl->u16Lifespan) &&
                   (0u == pCurTrkbl->u32SensorsCurr)) {
            /* this object has been dead for a while: clear objectID
             * (and all other values that might influence object2track
             * association)
             * clear slot of the dead track */
            baMarkedToDrop[u16i] = TRUE;
        } else if (TUEOBJFUSN_TRACKABLE_U16LIFESPAN_DEAD <
                   pCurTrkbl->u16Lifespan) {
            // update age of track
            u32Success |= updateAge(pCurTrkbl, f32CycleDt);
        } else {
            /* MISRA */
        }

        /** Is update check really required here ?? */
        if ((baMarkedToDrop[u16i] == FALSE) && (pCurTrkbl->bUpdated == TRUE) &&
            (pCurTrkbl->u8CyclesNoVision >
             TUE_PRV_LKF_TRACK_MANAGEMENT_NUM_CYCLES_TO_CLEAR_VISION) &&
            ((pCurTrkbl->u32SensorsHist &
              TUEOBJFUSN_TRACKABLE_U32SENSOR_CAMERA) != 0u)) {
            u32Success |= lkfTrackManagement_clearVisionInformation(pCurTrkbl);
        } else {
            /* MISRA */
        }

        if ((baMarkedToDrop[u16i] == FALSE) && (pCurTrkbl->bUpdated == TRUE) &&
            (pCurTrkbl->u8CyclesNoRadar >
             TUE_PRV_LKF_TRACK_MANAGEMENT_NUM_CYCLES_TO_CLEAR_RADAR) &&
            ((pCurTrkbl->u32SensorsHist &
              TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR) != 0u)) {
            u32Success |= lkfTrackManagement_clearRadarInformation(pCurTrkbl);
        } else {
            /* MISRA */
        }

        if (baMarkedToDrop[u16i] == FALSE) {
            u32Success |= tue_prv_motionType_calculateMotionType(pCurTrkbl);
        } else {
            /* MISRA */
        }
    }

    if (u32Success == TUEOBJFUSN_ERROR_NOERROR) {
        u32Success |= lkfTrackManagement_dropObjects(baMarkedToDrop);
        u32Success |= tue_prv_validate_trackable_list(&TRK_LIST);

        idProvider_finishCycle();
    } else {
        /* MISRA */
    }

    return u32Success;
}

/**
 * @fn   bool_t lkfTrackManagement_deleteSensorIds( u32_t const
 * u32SensorUpdatePattern )
 *
 * @brief   Resets the ids of the sensor id array to default values for provided
 * sensor pattern
 *
 * @param   u32SensorUpdatePattern[]             u32_t const, sensor pattern
 *
 * @return  TUEOBJFUSN_ERROR_NOERROR if no error occured and error code
 * otherwise.
 */

/* PRQA S 1532 2 */ /* Inteface function called from external AAU */
uint32 lkfTrackManagement_deleteSensorInformation(
    const uint32 u32SensorUpdatePattern) {
    uint16 u16i = 0u;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
#else
    const uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
#endif

    P2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pCurTrkbl;
    const uint16 u16SensPos = Trackable_getSensPos(u32SensorUpdatePattern);

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
    if (u16SensPos == TUEOBJFUSN_SENS_POS_INVALID) {
        u32Success = TUEOBJFUSN_ERROR_INVALID_SENSOR_PATTERN;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_LKF_TRACKMANAGEMENT,
            TUEOBJFUSN_AAU_LKF_TRACK_MANAGEMENT_DELETE_SENSOR_INFORMATION);
    } else
#endif
    {
        for (u16i = 0u; u16i < TRK_LIST.u16ValidTrackables; u16i++) {
            pCurTrkbl = &TRK_LIST.aTrackable[TRK_LIST.as16TrackableMap[u16i]];

            if (TUEOBJFUSN_TRACKABLE_U16LIFESPAN_FREE_SLOT <
                pCurTrkbl->u16Lifespan) {
                // Do a bitwise AND with u32SensorsCurr and the inverse of the
                // update pattern (invPattern) to erase all bits of sensors with
                // updates in the current cycle, but keep all bits set of
                // sensors
                // that do not provide updates.
                // create inverse bit mask of update pattern
                pCurTrkbl->u32SensorsCurr &= ~u32SensorUpdatePattern;

                /* increase cycle by 1 if it was ever seen (default) and is
                 * below max value */
                if ((0u != (u32SensorUpdatePattern &
                            TUEOBJFUSN_TRACKABLE_U32SENSOR_VISION)) &&
                    (pCurTrkbl->u8CyclesNoVision <
                     TUEOBJFUSN_TRACKABLE_U8CYCLESNOVISION_MAX)) {
                    pCurTrkbl->u8CyclesNoVision++;
                } else {
                    /* MISRA */
                }

                if ((0u != (u32SensorUpdatePattern &
                            TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR)) &&
                    (pCurTrkbl->u8CyclesNoRadar !=
                     TUEOBJFUSN_TRACKABLE_U8CYCLESNORADAR_DEFAULT)) {
                    pCurTrkbl->u8CyclesNoRadar++;
                } else {
                    /* MISRA */
                }

                pCurTrkbl->au16SensorID[u16SensPos] =
                    TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT;
            } else {
                /* MISRA */
            }
        }
    }

    return u32Success;
}

/* PRQA S 1532 2 */ /* Inteface function called from external AAU */
uint32 lkfTrackManagement_predictInternalTrkbl(float32 const f32PredictionDt) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    uint16 u16i;

    P2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pCurrTrkbl;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
    if (FLT_ZERO > f32PredictionDt) {
        u32Success = TUEOBJFUSN_ERROR_INVALID_INPUT;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_LKF_TRACKMANAGEMENT,
            TUEOBJFUSN_AAU_LKF_TRACK_MANAGEMENT_PREDICT_INTERNAL_TRKBL);
    } else
#endif
    {
        for (u16i = 0u; u16i < TUEOBJFUSN_TRACKABLELIST_U16VALIDTRACKABLES_MAX;
             u16i++) {
            if (TUEOBJFUSN_TRACKABLE_U16LIFESPAN_FREE_SLOT <
                TRK_LIST.aTrackable[u16i].u16Lifespan) {
                pCurrTrkbl = &(TRK_LIST.aTrackable[u16i]);

                u32Success |= LKF_DoPredict(pCurrTrkbl, f32PredictionDt);
                u32Success |=
                    LKF_CoordinatedTurn_DoPredict(pCurrTrkbl, f32PredictionDt);

/** Calculate gain if desired */
#if TUE_PRV_GAIN_ESTIMATION_ENABLE_GAIN_CALCULATION == STD_ON
                u32Success |= gain_prediction(pCurrTrkbl, f32PredictionDt);
#endif
            } else {
                /* MISRA */
            }
        }
    }

    /** Reduce Track list age by prediction time as track list now is "younger"
     */
    if (TUEOBJFUSN_ERROR_NOERROR == u32Success) {
        TRK_LIST.f32MeasurementLatency -= f32PredictionDt;
    } else {
        /* MISRA */
    }

    return u32Success;
}

/* PRQA S 1532 2 */ /* Inteface function called from external AAU */
uint32 lkfTrackManagement_compensateInternalTrkbleList(
    const float32 f32MeasurementLatency) {
    return EgoCoordCompensation_compensateTrackbleList(
        TRK_LIST.aTrackable, TRK_LIST.u16ValidTrackables,
        TRK_LIST.as16TrackableMap, TRUE,
        /** Use trackable map */ TRK_LIST.f32MeasurementLatency,
        f32MeasurementLatency, FALSE);
}

/* PRQA S 1532 2 */ /* Inteface function called from external AAU */
uint32 lkfTrackManagement_updateTrkList(
    CONSTP2VAR(TueObjFusn_ObjectListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pMeasList,
    CONSTP2CONST(stMatchIndexArrayType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pMatchIndexArray,
    VAR(boolean, ObjFusn_VAR_NOINIT) abObjectAssignedFlags[],
    CONSTP2CONST(TueObjFusn_EgoMotionType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pEgoMotion) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    uint16 u16Idx = 0u;
    uint16 u16i;
    P2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pMeas;
    P2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pTrkbl;
    uint16 u16SensPos = 0u;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON) && \
    (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if ((((NULL_PTR == pMeasList) || (NULL_PTR == pMatchIndexArray)) ||
         (NULL_PTR == abObjectAssignedFlags)) ||
        (NULL_PTR == pEgoMotion)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION,
            TUEOBJFUSN_AAU_LKF_TRACKMANAGEMENT,
            TUEOBJFUSN_AAU_LKF_TRACK_MANAGEMENT_UPDATE_TRKBLE);
    } else
#endif
    {
        u16SensPos = Trackable_getSensPos(pMeasList->u32SensorPattern);

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
        if (u16SensPos == TUEOBJFUSN_SENS_POS_INVALID) {
            u32Success = TUEOBJFUSN_ERROR_INVALID_SENSOR_PATTERN;
            (void)tue_prv_error_management_addError(
                TUEOBJFUSN_ERROR_INVALID_SENSOR_PATTERN,
                TUEOBJFUSN_AAU_LKF_TRACKMANAGEMENT,
                TUEOBJFUSN_AAU_LKF_TRACK_MANAGEMENT_UPDATE_TRKBLE);
        } else
#endif
        {
            /** Iterate over all matches */
            for (u16i = 0u; u16i < pMatchIndexArray->u16NumMatches; u16i++) {
#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
                if ((pMatchIndexArray->aMatchIndexArray[u16i].u16IndexCol >=
                     pMeasList->u16NumObjects) ||
                    (pMatchIndexArray->aMatchIndexArray[u16i].u16IndexRow >=
                     TRK_LIST.u16ValidTrackables)) {
                    u32Success =
                        TUEOBJFUSN_ERROR_LKF_TRACKMANAGEMENT_INVALID_MATCH_INDEX;
                    (void)tue_prv_error_management_addError(
                        TUEOBJFUSN_ERROR_LKF_TRACKMANAGEMENT_INVALID_MATCH_INDEX,
                        TUEOBJFUSN_AAU_LKF_TRACKMANAGEMENT,
                        TUEOBJFUSN_AAU_LKF_TRACK_MANAGEMENT_UPDATE_TRKBLE);
                } else
#endif
                {
                    u16Idx = (uint16)TRK_LIST
                                 .as16TrackableMap[pMatchIndexArray
                                                       ->aMatchIndexArray[u16i]
                                                       .u16IndexRow];

                    pMeas = &pMeasList->aTrackable[pMatchIndexArray
                                                       ->aMatchIndexArray[u16i]
                                                       .u16IndexCol];
                    pTrkbl = &(TRK_LIST.aTrackable[u16Idx]);

#if TUE_PRV_GAIN_ESTIMATION_ENABLE_GAIN_CALCULATION == STD_ON
                    u32Success = gain_update(pTrkbl, pMeas, pEgoMotion);
#endif
                    u32Success |= LKF_DoCorrect(pTrkbl, pMeas);

                    if (TUEOBJFUSN_ERROR_NOERROR != u32Success) {
                        /* Re-initialize trackable in case an error occurred
                         * during update step */
                        u32Success = LKF_AddNewTrkbl(pTrkbl, pMeas);

                        /* Reset existence probabilty */
                        pTrkbl->f32ExistenceQuality =
                            TUEOBJFUSN_TRACKABLE_F32EXISTENCEQUALITY_DEFAULT;
                    } else {
                        u32Success |= LKF_CoordinatedTurn_DoCorrect(pTrkbl);
                    }

                    if ((((pMeas->u32SensorsCurr) &
                          TUEOBJFUSN_TRACKABLE_U32SENSOR_CAMERA) > 0u) &&
                        (pTrkbl->au16SensorIDLast[u16SensPos] !=
                         pMeas->u16ID) &&
                        (pMeas->u16Age > 0u)) {
                        u32Success |=
                            lkfTrackManagement_clearVisionInformationFromList(
                                pMeas->u16ID, u16SensPos,
                                pMeas->u32SensorsCurr);
                    } else {
                        /* MISRA */
                    }

                    u32Success |= lkfTrackManagement_updateTrkbleInfos(
                        pMeas, pTrkbl, u16SensPos);

                    // indicate this object has been used to update a track
                    abObjectAssignedFlags
                        [pMatchIndexArray->aMatchIndexArray[u16i].u16IndexCol] =
                            TRUE;

                    lkfTrackManagement_increaseDiagonalP(
                        &TRK_LIST.aTrackable[u16Idx].matP);
                }
            }
        }
    }

    return u32Success;
}

/* PRQA S 1532 2 */ /* Inteface function called from external AAU */
uint32 lkfTrackManagement_setupNewLkfTrkble(
    CONSTP2CONST(TueObjFusn_ObjectListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pMeasList,
    CONST(boolean, ObjFusn_VAR_NOINIT) abObjectAssignedFlags[]) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    uint16 u16i;
    uint16 u16SensPos = 0u;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON) && \
    (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if ((NULL_PTR == pMeasList) || (abObjectAssignedFlags == NULL_PTR)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION,
            TUEOBJFUSN_AAU_LKF_TRACKMANAGEMENT,
            TUEOBJFUSN_AAU_LKF_TRACK_MANAGEMENT_SETUP_NEW_LKF_TRKBLE);
    } else
#endif
    {
        u16SensPos = Trackable_getSensPos(pMeasList->u32SensorPattern);

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
        if (u16SensPos == TUEOBJFUSN_SENS_POS_INVALID) {
            u32Success = TUEOBJFUSN_ERROR_INVALID_SENSOR_PATTERN;
            (void)tue_prv_error_management_addError(
                TUEOBJFUSN_ERROR_INVALID_SENSOR_PATTERN,
                TUEOBJFUSN_AAU_LKF_TRACKMANAGEMENT,
                TUEOBJFUSN_AAU_LKF_TRACK_MANAGEMENT_SETUP_NEW_LKF_TRKBLE);
        } else
#endif
        {
            // iterate through object list and find sensor objects not
            // associated to
            // any track
            for (u16i = 0u; u16i < pMeasList->u16NumObjects; ++u16i) {
                if ((FALSE == abObjectAssignedFlags[u16i])) {
                    // create new track
                    u32Success |= lkfTrackManagement_addTrkbl(
                        &pMeasList->aTrackable[u16i], u16SensPos);
                } else {
                    /* MISRA */
                }
            }  // end iterating through object list
        }
    }

    return u32Success;
}

/**
 * Takes all active tracks from TrackList and copies them into the given
 * TueObjFusn_TrackableListType erasing all former contents of that list.
 * Active tracks are those that have either been updated or started in the
 * last cylce.
 */
/* PRQA S 1532 2 */ /* Inteface function called from external AAU */
uint32 lkfTrackManagement_copyLkfTrkbl2Outputlist(
    CONSTP2VAR(TueObjFusn_ObjectListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pOutputList,
    const boolean abObjectsToBeDropped[]) {
    uint32 u32Success;
    uint16 u16i;
    uint16 u16NextSlot = 0u;
    sint16 s16Idx;

    P2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pCurTrkbl;
    P2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT)
    pCurrentObject;

    /* only need to copy the trackable list because all other functionalities
     * will be done in the trackable 2 objlist conversion */

    /* Copy Track List to Tracked Fusion Output & Set Track List back to default
     */

    u32Success = Trackable_initObjectList(pOutputList);
    pOutputList->f32MeasurementLatency = TRK_LIST.f32MeasurementLatency;

    for (u16i = 0u; u16i < TRK_LIST.u16ValidTrackables; u16i++) {
        if (FALSE == abObjectsToBeDropped[u16i]) {
            s16Idx = TRK_LIST.as16TrackableMap[u16i];
            pCurTrkbl = &TRK_LIST.aTrackable[s16Idx];

            pCurrentObject = &pOutputList->aTrackable[u16NextSlot];
            pOutputList->u16NumObjects++;
            // copy trackable, because 90% of the fields are identical
            u32Success |= Trackable_copyTrackable(pCurrentObject, pCurTrkbl);

            u16NextSlot += 1u;
        } else {
            /* MISRA */
        }
    } /* end for u16i */

    return u32Success;
}

/* PRQA S 1532 2 */ /* Inteface function called from external AAU */
uint32 lkfTrackManagement_runTrackMerge(CONSTP2CONST(
    TueObjFusn_EgoMotionType, AUTOMATIC, ObjFusn_VAR_NOINIT) pEgoMotion) {
    const boolean bUseTrackMergeTmp = Fusion_get_bUseTrackMerge();
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;

    /* Array indicating the objects to be removed after track merge */
    VAR(boolean, ObjFusn_VAR_NOINIT)
    abMarkedToDrop[TUEOBJFUSN_TRACKABLELIST_U16VALIDTRACKABLES_MAX];

    if (TRUE == bUseTrackMergeTmp) {
        u32Success =
            trackMerge_mergeTracks(&TRK_LIST, abMarkedToDrop, pEgoMotion);
        u32Success |= lkfTrackManagement_dropObjects(abMarkedToDrop);
    } else {
        /* MISRA */
    }

    return u32Success;
}

/* PRQA S 1532 2 */ /* Inteface function called from external AAU */
float32 lkfTrackManagement_getTrkListAge(void) {
    return TRK_LIST.f32MeasurementLatency;
}

/* PRQA S 1532 2 */ /* Inteface function called from external AAU */
P2VAR(TueObjFusn_TrackableListType, AUTOMATIC, ObjFusn_CODE)
lkfTrackManagement_getTrkbleList(void) { return &TRK_LIST; }

LOCAL uint32
lkfTrackManagement_dropObjects(const boolean abMarkedForDeletion[]) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    uint16 u16i;
    uint16 u16offset = 0u;
    sint16 s16Idx;
    uint16 _u16ID;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
    if (TRK_LIST.u16ValidTrackables >
        TUEOBJFUSN_TRACKABLELIST_U16VALIDTRACKABLES_MAX) {
        u32Success = TUEOBJFUSN_ERROR_NUMBER_OF_OBJECTS_EXCEEDS_MAXIMUM_SIZE;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NUMBER_OF_OBJECTS_EXCEEDS_MAXIMUM_SIZE,
            TUEOBJFUSN_AAU_LKF_TRACKMANAGEMENT,
            TUEOBJFUSN_AAU_LKF_TRACK_MANAGEMENT_DROP_OBJECTS);
    } else
#endif
    {
        for (u16i = 0u; u16i < TRK_LIST.u16ValidTrackables; u16i++) {
            if (TRUE == abMarkedForDeletion[u16i]) {
                s16Idx = TRK_LIST.as16TrackableMap[u16i];
                _u16ID = TRK_LIST.aTrackable[s16Idx].u16ID;
                u32Success = Trackable_init(&TRK_LIST.aTrackable[s16Idx]);
                u32Success |= idProvider_releaseFusionId(_u16ID);
                u16offset++;
            } else if (u16offset > 0u) {
                TRK_LIST.as16TrackableMap[u16i - u16offset] =
                    TRK_LIST.as16TrackableMap[u16i];
            } else {
                /* MISRA */
            }
        }

        TRK_LIST.u16ValidTrackables -= u16offset;
        // set as16TrackableMap to -1
        for (u16i = TRK_LIST.u16ValidTrackables;
             u16i < TUEOBJFUSN_TRACKABLELIST_U16VALIDTRACKABLES_MAX; ++u16i) {
            TRK_LIST.as16TrackableMap[u16i] = -1;
        }
    }

    return u32Success;
}

/************************************************
************** INTERNAL FUNCTIONS **************
************************************************/

/**
* \fn  bool_t updateAge(TueObjFusn_TrackableType * const pLkfTrkble, f32_t const
f32CycleDt)
* \brief increments the age by cycle time f32CycleDt

The age in ms saturates at TUEOBJFUSN_TRACKABLE_U16AGE_MAX.
*
* \param  [in, out] u16TrkAge     u16_t *, age which shall be adapted [ms]
* \param  [in] f32CycleDt    f32_t const, cycle time [s]
*
* \return  TUEOBJFUSN_ERROR_NOERROR if no error occured and error code
otherwise.
*/
LOCAL uint32 updateAge(CONSTP2VAR(TueObjFusn_TrackableType,
                                  AUTOMATIC,
                                  ObjFusn_VAR_NOINIT) pLkfTrkble,
                       const float32 f32CycleDt) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    /* PRQA S 3204 1 */ // variable may be set to other values depending on
                        //    run-time pointer check activation
    uint32 u32AgeIn;
    const float32 f32DtinMS =
        (f32CycleDt * TUE_PRV_LKF_MILISEC_PER_SEC) + FLT_ONE_HALF;
    uint32 u32Age;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON) && \
    (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if (NULL_PTR == pLkfTrkble) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION,
            TUEOBJFUSN_AAU_LKF_TRACKMANAGEMENT,
            TUEOBJFUSN_AAU_LKF_TRACK_MANAGEMENT_UPDATE_AGE);
    } else
#endif
    {
        u32AgeIn = (uint32)(pLkfTrkble->u16Age);
        u32Age = (uint32)f32DtinMS + u32AgeIn;

        if (u32AgeIn == TUEOBJFUSN_TRACKABLE_U16AGE_DEFAULT) {
            pLkfTrkble->u16Age = TUEOBJFUSN_TRACKABLE_U16AGE_MIN;
        } else if (u32Age > TUEOBJFUSN_TRACKABLE_U16AGE_MAX) {
            pLkfTrkble->u16Age = TUEOBJFUSN_TRACKABLE_U16AGE_MAX;
        } else {
            pLkfTrkble->u16Age = (uint16)u32Age;
        }
    }

    return u32Success;
}

LOCAL uint32 lkfTrackManagement_updateTrkbleInfos(
    CONSTP2CONST(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pMeas,
    CONSTP2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pLkfTrkble,
    const uint16 u16SensPos) {
#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
#else
    const uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
#endif

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if ((NULL_PTR == pMeas) || (NULL_PTR == pLkfTrkble)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION,
            TUEOBJFUSN_AAU_LKF_TRACKMANAGEMENT,
            TUEOBJFUSN_AAU_LKF_TRACK_MANAGEMENT_UPDATE_TRKBLE_INFOS);
    } else
#endif
        if (u16SensPos == TUEOBJFUSN_SENS_POS_INVALID) {
        u32Success = TUEOBJFUSN_ERROR_INVALID_SENSOR_PATTERN;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_INVALID_SENSOR_PATTERN,
            TUEOBJFUSN_AAU_LKF_TRACKMANAGEMENT,
            TUEOBJFUSN_AAU_LKF_TRACK_MANAGEMENT_UPDATE_TRKBLE_INFOS);
    } else
#endif
    {
        pLkfTrkble->bUpdated = TRUE;

        /* accumulate sensor current */
        pLkfTrkble->u32SensorsCurr |= pMeas->u32SensorsCurr;

        /* accumulate sensor history */
        pLkfTrkble->u32SensorsHist |= pLkfTrkble->u32SensorsCurr;

        /* Update Sensor ID */
        pLkfTrkble->au16SensorID[u16SensPos] = pMeas->u16ID;
        pLkfTrkble->au16SensorIDLast[u16SensPos] = pMeas->u16ID;

        if (0u !=
            (pMeas->u32SensorsCurr & TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR)) {
            pLkfTrkble->u8CyclesNoRadar = 0u;
            pLkfTrkble->u8VisionIdx = 1u - (uint8)u16SensPos;

            pLkfTrkble->f32ObstacleProbability = pMeas->f32ObstacleProbability;

            // use sensor's POE as the reference, rather than the fusion's
            // existence logic itself. POE = MAX(CameraPOE,RadarPOE) guotao
            // 20190605
            // pLkfTrkble->f32ExistenceQuality = pMeas->f32ExistenceQuality;
            if (pLkfTrkble->u8CyclesNoVision == 0u) {
                pLkfTrkble->f32ExistenceQuality =
                    tue_prv_fusion_max_F32(pLkfTrkble->f32ExistenceQuality,
                                           pMeas->f32ExistenceQuality);
            } else {
                /* do not overwrite ClassProb with default value, as default
                 * value would be invalid */
                if (pMeas->u16ClassProb >= pLkfTrkble->u16ClassProb) {
                    // set track's class to radar sensor input object class
                    // property
                    pLkfTrkble->u16Class = pMeas->u16Class;
                    pLkfTrkble->u16ClassProb = pMeas->u16ClassProb;
                }
                // Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/150
                // changed by guotao for the camera confirmed logic change start
                else if (pMeas->u16Class != pLkfTrkble->u16Class)
                // Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/150
                // changed by guotao for the camera confirmed logic change end
                {
                    // 20200106 car, pedestrian and camera object could have a
                    // higher class probability, probability deceleration every
                    // cycle.
                    // for example, first input radar object is car and changed
                    // to be obstacle objcet in the next 20 cycles, the track's
                    // class would
                    // be car class, and change to obstacle class in the 20th
                    // cycles.
                    pLkfTrkble->u16ClassProb--;
                }

                pLkfTrkble->f32ExistenceQuality = pMeas->f32ExistenceQuality;
            }
            // add new value for tuerme project FPS module, guotao 20190507
            pLkfTrkble->fRCS = pMeas->fRCS;
            pLkfTrkble->eObjMaintenanceState = pMeas->eObjMaintenanceState;
            pLkfTrkble->u8RadarMotionTypeInput = pMeas->u8RadarMotionTypeInput;
        } else if (0u != (pMeas->u32SensorsCurr &
                          TUEOBJFUSN_TRACKABLE_U32SENSOR_VISION)) {
            pLkfTrkble->u8CyclesNoVision = 0u;
            pLkfTrkble->u8VisionIdx = (uint8)u16SensPos;

            pLkfTrkble->u16Class =
                pMeas->u16Class;  // \todo: handle/filter class and motion type
                                  // correctly, e.g. by using DS, HMM etc.
            /************* GAC new feature *************/
            pLkfTrkble->bMCPFlag = pMeas->bMCPFlag;
            pLkfTrkble->bCIPVFlag = pMeas->bCIPVFlag;
            pLkfTrkble->f32Height = pMeas->f32Height;
            /************* GAC new feature *************/
            pLkfTrkble->f32Width = pMeas->f32Width;

            pLkfTrkble->f32Length = pMeas->f32Length;
            pLkfTrkble->u16RefPoint = pMeas->u16RefPoint;
            /* keep lowest obstacle probability */
            pLkfTrkble->f32ObstacleProbability =
                tue_prv_fusion_min_F32(pMeas->f32ObstacleProbability,
                                       pLkfTrkble->f32ObstacleProbability);

            // use sensor's POE as the reference, rather than the fusion's
            // existence logic itself. POE = MAX(CameraPOE,RadarPOE) guotao
            // 20190605
            // pLkfTrkble->f32ExistenceQuality = pMeas->f32ExistenceQuality;
            if (pLkfTrkble->u8CyclesNoRadar == 0u) {
                pLkfTrkble->f32ExistenceQuality =
                    tue_prv_fusion_max_F32(pLkfTrkble->f32ExistenceQuality,
                                           pMeas->f32ExistenceQuality);
            } else {
                pLkfTrkble->f32ExistenceQuality = pMeas->f32ExistenceQuality;
            }
            // add new value for tuerme project FPS module, guotao 20190507
            pLkfTrkble->fRCS = pMeas->fRCS;
            pLkfTrkble->eObjMaintenanceState = pMeas->eObjMaintenanceState;

            /* do not overwrite ClassProb with default value, as default value
             * would be invalid */
            if (pMeas->u16ClassProb !=
                TUEOBJFUSN_TRACKABLE_U16CLASSPROB_DEFAULT) {
                pLkfTrkble->u16ClassProb = pMeas->u16ClassProb;
            } else {
                /* MISRA */
            }
        } else {
            /* MISRA */
        }
    }

    return u32Success;
}

/* PRQA S 1532 2 */ /* Function called from external AAU */
uint32 lkfTrackManagement_updateExistenceProbability(
    const uint32 u32SensorUpdatePattern) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    uint16 u16i = 0u;
    uint32 au32SensorInfos[TUE_PRV_FUSION_MAX_INPUTS];

    Trackable_getSensorInfos(au32SensorInfos);

    for (u16i = 0u; u16i < TUE_PRV_FUSION_MAX_INPUTS; u16i++) {
        if (0u != (u32SensorUpdatePattern & au32SensorInfos[u16i])) {
            u32Success |= tue_prv_update_existence_probability(
                &TRK_LIST, au32SensorInfos[u16i]);
        } else {
            /* MISRA */
        }
    }

    return u32Success;
}

/* PRQA S 1532 2 */ /* Function called from external AAU */
uint32 lkfTrackManagement_selectVioTrackables(const uint16 u16NumMeasurements,
                                              const uint16 u16nBestMatches) {
    uint16 u16MaxTrackables;
    sint16 s16TrackablesToDelete;
    uint32 u32Success;
    VAR(boolean, ObjFusn_VAR_NOINIT)
    abObjectsToBeDropped[TUEOBJFUSN_TRACKABLELIST_U16VALIDTRACKABLES_MAX];

    u16MaxTrackables =
        (TRK_LIST.u16ValidTrackables + u16NumMeasurements) - u16nBestMatches;
    s16TrackablesToDelete =
        (sint16)u16MaxTrackables -
        (sint16)TUEOBJFUSN_TRACKABLELIST_U16VALIDTRACKABLES_MAX;

    u32Success = tue_prv_object_selection_select_objects(
        &TRK_LIST, s16TrackablesToDelete, abObjectsToBeDropped);
    u32Success |= lkfTrackManagement_dropObjects(abObjectsToBeDropped);

    return u32Success;
}

void lkfTrackManagement_increaseDiagonalP(CONSTP2VAR(stf32SymMatrix_t,
                                                     AUTOMATIC,
                                                     ObjFusn_VAR_NOINIT) matP) {
    const uint16 _u16Size = matP->u16Size;
    const float32 f32QOnDiagonal =
        TUE_PRV_LKF_TRACK_MANAGEMENT_ADDITIONAL_Q_ON_DIAGONAL;

    if (f32QOnDiagonal > FLT_ZERO) {
        matP->data[TRACKABLE_INDEX_VARIANCE_POSX] +=
            TUE_PRV_LKF_TRACK_MANAGEMENT_ADDITIONAL_Q_ON_DIAGONAL;
        matP->data[TRACKABLE_INDEX_VARIANCE_POSY] +=
            TUE_PRV_LKF_TRACK_MANAGEMENT_ADDITIONAL_Q_ON_DIAGONAL;
        matP->data[TRACKABLE_INDEX_VARIANCE_VELX] +=
            TUE_PRV_LKF_TRACK_MANAGEMENT_ADDITIONAL_Q_ON_DIAGONAL;

        if (_u16Size > TRACKABLE_ACCY) {
            matP->data[TRACKABLE_INDEX_VARIANCE_VELY] +=
                TUE_PRV_LKF_TRACK_MANAGEMENT_ADDITIONAL_Q_ON_DIAGONAL;
            matP->data[TRACKABLE_INDEX_VARIANCE_ACCX] +=
                TUE_PRV_LKF_TRACK_MANAGEMENT_ADDITIONAL_Q_ON_DIAGONAL;
            matP->data[TRACKABLE_INDEX_VARIANCE_ACCY] +=
                TUE_PRV_LKF_TRACK_MANAGEMENT_ADDITIONAL_Q_ON_DIAGONAL;
        } else if (_u16Size > TRACKABLE_ACCX) {
            matP->data[TRACKABLE_INDEX_VARIANCE_VELY] +=
                TUE_PRV_LKF_TRACK_MANAGEMENT_ADDITIONAL_Q_ON_DIAGONAL;
            matP->data[TRACKABLE_INDEX_VARIANCE_ACCX] +=
                TUE_PRV_LKF_TRACK_MANAGEMENT_ADDITIONAL_Q_ON_DIAGONAL;
        } else if (_u16Size > TRACKABLE_VELY) {
            matP->data[TRACKABLE_INDEX_VARIANCE_VELY] +=
                TUE_PRV_LKF_TRACK_MANAGEMENT_ADDITIONAL_Q_ON_DIAGONAL;
        } else {
            /* MISRA */
        }
    } else {
        /* MISRA */
    }
}

/* PRQA S 1532 2 */ /* Library Function */
uint32 lkfTrackManagement_removeObsoleteTracks(const uint32 u32SensorPattern) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    uint16 u16i;
    uint32 _u32SensorCurr;

    boolean abMarkToDrop[TUEOBJFUSN_TRACKABLELIST_U16VALIDTRACKABLES_MAX];
    P2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pCurTrkbl;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
    const uint32 u32FusionMode = Fusion_get_u32SensorMode();
#endif
    const uint16 u16SensIndex = Trackable_getSensPos(u32SensorPattern);
    tue_prv_common_array_utils_defaultInit_abool(
        abMarkToDrop, TUEOBJFUSN_TRACKABLELIST_U16VALIDTRACKABLES_MAX, FALSE);

    if (TUEOBJFUSN_TRACKABLE_U32SENSOR_INVALID == u32SensorPattern) {
        /* Do nothing as this sensor has not been seen before */
    }
#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
    else if (0u == (u32FusionMode & u32SensorPattern)) {
        u32Success = TUEOBJFUSN_ERROR_INVALID_INPUT;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_INVALID_INPUT, TUEOBJFUSN_AAU_LKF_TRACKMANAGEMENT,
            TUEOBJFUSN_AAU_LKF_TRACK_MANAGEMENT_REMOVE_OBSOLETE_OBJECTS);
    }
#endif
    else {
        for (u16i = 0u; u16i < TRK_LIST.u16ValidTrackables; u16i++) {
            pCurTrkbl = &TRK_LIST.aTrackable[TRK_LIST.as16TrackableMap[u16i]];

            /* Check for non fused tracks that match the given sensor pattern */
            _u32SensorCurr = pCurTrkbl->u32SensorsCurr;

            if (u32SensorPattern == _u32SensorCurr) {
                /* Sensor only track */
                abMarkToDrop[u16i] = TRUE;
            } else if (0u == (_u32SensorCurr & u32SensorPattern)) {
                /* Internal track is not confirmed by this pattern, ignore */
            } else {
                pCurTrkbl->u32SensorsCurr &= ~u32SensorPattern;

                /* Clear sensor id's */
                pCurTrkbl->au16SensorID[u16SensIndex] =
                    TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT;
                pCurTrkbl->au16SensorIDLast[u16SensIndex] =
                    TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT;

                /** ToDo: Think about class type, motion type, width, height
                 * that is provided by camera only */
            }
        }

        u32Success = lkfTrackManagement_dropObjects(abMarkToDrop);
    }

    return u32Success;
}

LOCAL uint32 lkfTrackManagement_addTrkbl(CONSTP2CONST(TueObjFusn_TrackableType,
                                                      AUTOMATIC,
                                                      ObjFusn_VAR_NOINIT) pMeas,
                                         const uint16 u16SensPos) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    sint16 s16Index = TUEOBJFUSN_TRACKABLELIST_AS16TRACKABLEMAP_DEFAULT;
    uint16 u16i = 0u;

    P2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pTrkbl;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if (NULL_PTR == pMeas) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION,
            TUEOBJFUSN_AAU_TRACKABLE_LIST_UTILS,
            TUEOBJFUSN_AAU_LKF_TRACK_MANAGEMENT_ADD_TRKBL);
    } else
#endif
        if (TRK_LIST.u16ValidTrackables ==
            TUEOBJFUSN_TRACKABLELIST_U16VALIDTRACKABLES_MAX) {
        u32Success = TUEOBJFUSN_ERROR_INTERNAL_ERROR;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_INTERNAL_ERROR, TUEOBJFUSN_AAU_LKF_TRACKMANAGEMENT,
            TUEOBJFUSN_AAU_LKF_TRACK_MANAGEMENT_ADD_TRKBL);
    } else
#endif
    {
        /* Search next available index */
        for (u16i = 0u;
             (TUEOBJFUSN_TRACKABLELIST_AS16TRACKABLEMAP_DEFAULT == s16Index) &&
             (u16i < TUEOBJFUSN_TRACKABLELIST_U16VALIDTRACKABLES_MAX);
             ++u16i) {
            if (TUEOBJFUSN_TRACKABLE_U16LIFESPAN_FREE_SLOT ==
                TRK_LIST.aTrackable[u16i].u16Lifespan) {
                s16Index = (sint16)u16i;
            } else {
                /* MISRA */
            }
        }

        /** Check if free slot is found */
        if (s16Index != TUEOBJFUSN_TRACKABLELIST_AS16TRACKABLEMAP_DEFAULT) {
            pTrkbl = &(TRK_LIST.aTrackable[s16Index]);

            u32Success = Trackable_init(pTrkbl);
            u32Success |= LKF_AddNewTrkbl(pTrkbl, pMeas);

            pTrkbl->u16Lifespan = TUEOBJFUSN_TRACKABLE_U16LIFESPAN_NEW;
            pTrkbl->bUpdated = TRUE;

            pTrkbl->u16ID = idProvider_getNewFusionId();

            /* new track starts only with history made up of current sensors */
            pTrkbl->u32SensorsCurr = pMeas->u32SensorsCurr;
            pTrkbl->u32SensorsHist = pTrkbl->u32SensorsCurr;

            /* Set sensor id */
            pTrkbl->au16SensorID[u16SensPos] = pMeas->u16ID;
            pTrkbl->au16SensorIDLast[u16SensPos] = pMeas->u16ID;

            /* set obstacle probability */
            pTrkbl->f32ObstacleProbability = pMeas->f32ObstacleProbability;

            /* initialize values which are radar or camera specific to
             * reasonable default */
            pTrkbl->u16MotionType = TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_UNKNOWN;
            pTrkbl->u16Class = TUEOBJFUSN_TRACKABLE_U16CLASS_UNKNOWN;
            pTrkbl->u16ClassProb = TUEOBJFUSN_TRACKABLE_U16CLASSPROB_MIN;
            pTrkbl->f32Width = TUEOBJFUSN_TRACKABLE_F32WIDTH_MIN;
            pTrkbl->f32Length = TUEOBJFUSN_TRACKABLE_F32LENGTH_MIN;
            pTrkbl->u16RefPoint = TUEOBJFUSN_TRACKABLE_U16REFPOINT_POINT_OBJECT;
            pTrkbl->u8CoordSystem = pMeas->u8CoordSystem;
            /************* GAC new feature *************/
            pTrkbl->f32Height = TUEOBJFUSN_TRACKABLE_F32HEIGHT_MIN;
            pTrkbl->bMCPFlag = TUEOBJFUSN_TRACKABLE_bMCPFlag_DEFAULT;
            pTrkbl->bCIPVFlag = TUEOBJFUSN_TRACKABLE_bCIPVFlag_DEFAULT;
            /************* GAC new feature *************/

            /* if object has been seen by camera */
            if (0u != (pMeas->u32SensorsCurr &
                       TUEOBJFUSN_TRACKABLE_U32SENSOR_CAMERA)) {
                pTrkbl->u16Class = pMeas->u16Class;
                pTrkbl->u16ClassProb = pMeas->u16ClassProb;
                pTrkbl->f32Width = pMeas->f32Width;
                /************* GAC new feature *************/
                pTrkbl->f32Height = pMeas->f32Height;
                pTrkbl->bMCPFlag = pMeas->bMCPFlag;
                pTrkbl->bCIPVFlag = pMeas->bCIPVFlag;
                /************* GAC new feature *************/
                pTrkbl->f32Length = pMeas->f32Length;
                pTrkbl->u16RefPoint = pMeas->u16RefPoint;
                pTrkbl->u8VisionIdx = (uint8)u16SensPos;
                pTrkbl->u8CyclesNoVision = 0u;

                // use sensor's POE as the reference, rather than the fusion's
                // existence logic itself. guotao 20190517
                pTrkbl->f32ExistenceQuality = pMeas->f32ExistenceQuality;
                // add new value for tuerme project FPS module, guotao 20190507
                pTrkbl->fRCS = pMeas->fRCS;
                pTrkbl->eObjMaintenanceState =
                    1U;  // MT_STATE_NEW = 1U, set to new object

                /* Track has been used before */
                if ((pMeas->u16Age) > 0u) {
                    u32Success |=
                        lkfTrackManagement_clearVisionInformationFromList(
                            pMeas->u16ID, u16SensPos, pMeas->u32SensorsCurr);
                } else {
                    /* MISRA */
                }
            }
            /* if object has been seen by radar */
            else if (0u != (pMeas->u32SensorsCurr &
                            TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR)) {
                // set track's class to radar input object class property
                /* do not overwrite ClassProb with default value, as default
                 * value would be invalid */
                if (pMeas->u16ClassProb >= pTrkbl->u16ClassProb) {
                    // set track's class to radar sensor input object class
                    // property
                    pTrkbl->u16Class = pMeas->u16Class;
                    pTrkbl->u16ClassProb = pMeas->u16ClassProb;
                }
                // Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/150
                // changed by guotao for the camera confirmed logic change start
                else if (pMeas->u16Class != pTrkbl->u16Class) {
                // Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/150
                // changed by guotao for the camera confirmed logic change end
                    // 20200106 car, pedestrian and camera object could have a
                    // higher class probability, probability deceleration every
                    // cycle.
                    // for example, first input radar object is car and changed
                    // to be obstacle objcet in the next 20 cycles, the track's
                    // class would
                    // be car class, and change to obstacle class in the 20th
                    // cycles.
                    pTrkbl->u16ClassProb--;
                }
                // use sensor's POE as the reference, rather than the fusion's
                // existence logic itself. guotao 20190517
                pTrkbl->f32ExistenceQuality = pMeas->f32ExistenceQuality;
                // add new value for tuerme project FPS module, guotao 20190507
                pTrkbl->fRCS = pMeas->fRCS;
                pTrkbl->eObjMaintenanceState = pMeas->eObjMaintenanceState;
                pTrkbl->u8RadarMotionTypeInput = pMeas->u8RadarMotionTypeInput;

                pTrkbl->u16MotionType = pMeas->u16MotionType;
                pTrkbl->u8CyclesNoRadar = 0u;
                pTrkbl->u8VisionIdx = 1u - (uint8)u16SensPos;
            } else {
                /* MISRA */
            }

            TRK_LIST.as16TrackableMap[TRK_LIST.u16ValidTrackables] = s16Index;
            TRK_LIST.u16ValidTrackables++;
        } else {
            /* MISRA */
        }
    }

    return u32Success;
}

LOCAL uint32 lkfTrackManagement_clearVisionInformation(CONSTP2VAR(
    TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pTueTrkbl) {
#if STD_OFF == TUE_PRV_RUNTIME_ERROR_CHECK
    const uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
#else
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
#endif
    const uint16 u16SensPos =
        Trackable_getSensPos(TUEOBJFUSN_TRACKABLE_U32SENSOR_MVS_FRONT);

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)

    if (u16SensPos == TUEOBJFUSN_SENS_POS_INVALID) {
        u32Success = TUEOBJFUSN_ERROR_INTERNAL_ERROR;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_INTERNAL_ERROR, TUEOBJFUSN_AAU_LKF_TRACKMANAGEMENT,
            TUEOBJFUSN_AAU_LKF_TRACK_MANAGEMENT_CLEAR_VISION);
    } else if (pTueTrkbl->u8CyclesNoVision == 0u) {
        u32Success = TUEOBJFUSN_ERROR_INVALID_INPUT;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_INVALID_INPUT, TUEOBJFUSN_AAU_LKF_TRACKMANAGEMENT,
            TUEOBJFUSN_AAU_LKF_TRACK_MANAGEMENT_CLEAR_VISION);
    } else
#endif
    {
        pTueTrkbl->au16SensorID[u16SensPos] =
            TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT;
        pTueTrkbl->au16SensorIDLast[u16SensPos] =
            TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT;

        // pTueTrkbl->u16Class                       =
        // TUEOBJFUSN_TRACKABLE_U16CLASS_UNKNOWN;
        pTueTrkbl->f32Width = TUEOBJFUSN_TRACKABLE_F32WIDTH_MIN;

        /* Should be sufficient to simply set sensor hist to sensor current as
         * u8CyclesNoVision > 0 at this point */
        pTueTrkbl->u32SensorsHist = pTueTrkbl->u32SensorsCurr;
    }

    return u32Success;
}

LOCAL uint32 lkfTrackManagement_clearRadarInformation(CONSTP2VAR(
    TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pTueTrkbl) {
#if STD_OFF == TUE_PRV_RUNTIME_ERROR_CHECK
    const uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
#else
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
#endif
    const uint16 u16SensPos =
        Trackable_getSensPos(TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR_FRONT_CENTER);

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)

    if (u16SensPos == TUEOBJFUSN_SENS_POS_INVALID) {
        u32Success = TUEOBJFUSN_ERROR_INTERNAL_ERROR;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_INTERNAL_ERROR, TUEOBJFUSN_AAU_LKF_TRACKMANAGEMENT,
            TUEOBJFUSN_AAU_LKF_TRACK_MANAGEMENT_CLEAR_RADAR);
    } else if (pTueTrkbl->u8CyclesNoRadar == 0u) {
        u32Success = TUEOBJFUSN_ERROR_INVALID_INPUT;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_INVALID_INPUT, TUEOBJFUSN_AAU_LKF_TRACKMANAGEMENT,
            TUEOBJFUSN_AAU_LKF_TRACK_MANAGEMENT_CLEAR_RADAR);
    } else
#endif
    {
        pTueTrkbl->au16SensorID[u16SensPos] =
            TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT;
        pTueTrkbl->au16SensorIDLast[u16SensPos] =
            TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT;

        pTueTrkbl->f32ObstacleProbability =
            TUEOBJFUSN_TRACKABLE_F32OBSTACLEPROBABILITY_MAX;

        /* Should be sufficient to simply set sensor hist to sensor current as
         * u8CyclesNoRadar > 0 at this point */
        pTueTrkbl->u32SensorsHist = pTueTrkbl->u32SensorsCurr;
    }

    return u32Success;
}

LOCAL uint32
lkfTrackManagement_clearVisionInformationFromList(const uint16 u16SensorID,
                                                  const uint16 u16SensPos,
                                                  const uint32 u32SensPattern) {
#if STD_OFF == TUE_PRV_RUNTIME_ERROR_CHECK
    const uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
#else
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
#endif
    uint16 u16i = 0u;
    boolean bFound = FALSE;
    P2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pCurrTrkbl;

    for (u16i = 0u; (u16i < TRK_LIST.u16ValidTrackables) && (FALSE == bFound);
         u16i++) {
        pCurrTrkbl = &(TRK_LIST.aTrackable[TRK_LIST.as16TrackableMap[u16i]]);

        /* Check for last sensor id as current one is deleted before */
        if (pCurrTrkbl->au16SensorIDLast[u16SensPos] == u16SensorID) {
            bFound = TRUE;
            pCurrTrkbl->au16SensorID[u16SensPos] =
                TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT;
            pCurrTrkbl->au16SensorIDLast[u16SensPos] =
                TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT;

            // pCurrTrkbl->u16Class                       =
            // TUEOBJFUSN_TRACKABLE_U16CLASS_UNKNOWN;
            pCurrTrkbl->f32Width = TUEOBJFUSN_TRACKABLE_F32WIDTH_MIN;

            pCurrTrkbl->u32SensorsCurr &= ~u32SensPattern;
            pCurrTrkbl->u32SensorsHist = pCurrTrkbl->u32SensorsCurr;
        } else {
            /* MISRA */
        }
    }

    return u32Success;
}

#define ObjFusn_STOP_SEC_SLOW_CODE

#ifdef UNITTEST
TueObjFusn_TrackableListType* accessTrkMgmtTrackList(
    void) { /* PRQA S 3219 */ /**< used for tests only */
    return &TRK_LIST;
}

#endif /* UNITTEST */

/** \} */
