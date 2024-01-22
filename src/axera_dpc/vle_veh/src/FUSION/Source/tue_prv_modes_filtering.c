/** \defgroup tuePrvmodes TUE Modes Filtering
 *  \{
 *  The Modes Filtering AAU has the main functionality of
 *  object fusion using filterint techniques. This includes
 *  the whole OOSM functionality.
 *
 * \file       tue_prv_modes_filtering.c
 * \brief Filtering Mode
 *
 */
/* PRQA S 0292 ++ */ /* MKS */
/*
 */
/* PRQA S 0292 -- */
/*
 *
 *         (C) Copyright Tuerme Inc. All rights reserved.
 *
 */
/*==================[inclusions]============================================*/
#include "tue_prv_modes_filtering.h"
#include "tue_prv_modes_filtering_int.h"

#include "TueObjFusn_Eps.h"
#include "TueObjFusn_ParameterInterface.h"
#include "tue_prv_association.h"
#include "tue_prv_distance_score.h"
#include "tue_prv_idProvider.h"
#include "tue_prv_lkf_trackManagement.h"
#include "tue_prv_fusion_tools.h"
#include "tue_prv_quality_management.h"
#include "TueObjFusn_Kinematics.h"
#include "tue_prv_egoMotion.h"
#include "tue_prv_egoCoordCompensation.h"
#include "TueObjFusn_TrackableListUtils.h"
#include "TueObjFusn_TrackableListProps.h"
#include "tue_prv_common_array_utils.h"
#include "tue_prv_fusion_math.h"
#include "tue_prv_objectSelection.h"
#include "TueObjFusn_ErrorCodes.h"
#include "TueObjFusn_AAU_Codes.h"
#include "tue_prv_error_management.h"
#include "TueObjFusn_ConfigConstants.h"
#include "tue_prv_lkf.h"
#include "tue_prv_lkf_coordinatedTurn.h"

/*==================[variables]=============================================*/
/** \name Global Variables */
/**
 * \brief current value of update counter, used for output lists
 */
//#define ObjFusn_START_SEC_VAR16
#define ASW_QM_CORE5_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
LOCAL VAR(uint16, ObjFusn_VAR_ZERO_INIT) s_u16UpdateCounter = 0u;
#define ASW_QM_CORE5_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
//#define ObjFusn_STOP_SEC_VAR16

/*==================[functions]=============================================*/

#define ObjFusn_START_SEC_SLOW_CODE

/** \name Public Functions */
/**
 * \brief Initializes modes cycle
 * \return TRUE (ok) or FALSE (error occured; output invalid)
 */
/** initializer method */
/* PRQA S 1532 2 */ /* Interface function called from Fusion AAU */
void modes_filtering_init(void) {
    // init AAUs
    lkfTrackManagement_init();
    idProvider_init();
    EgoMotion_historyInit();

    s_u16UpdateCounter = TUEOBJFUSN_OBJECTLIST_U16LISTUPDATECOUNTER_MIN;
}
#define ObjFusn_STOP_SEC_SLOW_CODE

/**
 * \brief Starts the cycle of the filter mode.
 * Has to be called always at the start of every fusion cycle.
 *
 * \param f32dt cycle time
 * \return TRUE (ok) or FALSE (error occured; output invalid)
 */
#define ObjFusn_START_SEC_CODE

/* PRQA S 1532 2 */ /* Interface function called from Fusion AAU */
uint32 modes_filtering_preCycle(const float32 f32dt,
                                CONSTP2CONST(TueObjFusn_EgoMotionType,
                                             AUTOMATIC,
                                             ObjFusn_VAR_NOINIT) pEgoMotion) {
    const uint32 u32Success = EgoMotion_addItemToHistory(pEgoMotion, f32dt);
    lkfTrackManagement_startCycle(f32dt);

    return u32Success;
}

/**
 * \brief Ends the cycle of the filter.
 * Has to be called always at the end of every fusion cycle.
 *
 * \param f32dt cycle time
 * \param u32SensorsUpdatePattern Sensor update pattern for current fusion cycle
 * \return TRUE (ok) or FALSE (error occured; output invalid)
 */
/* PRQA S 1532 2 */ /* Interface function called from Fusion AAU */
uint32 modes_filtering_postCycle(const float32 f32dt,
                                 const uint32 u32SensorsUpdatePattern) {
    uint32 u32Success;

    P2VAR(TueObjFusn_EgoMotionType, AUTOMATIC, ObjFusn_VAR_NOINIT) pEgoMotion;
    const float32 f32TrackListAge = lkfTrackManagement_getTrkListAge();

    u32Success = EgoMotion_getEgoMotionHistory(&pEgoMotion, f32TrackListAge);

    if (TUEOBJFUSN_ERROR_NOERROR == u32Success) {
        u32Success |= lkfTrackManagement_runTrackMerge(pEgoMotion);
    } else {
        /* MISRA */
    }

    u32Success |=
        lkfTrackManagement_updateExistenceProbability(u32SensorsUpdatePattern);

    /* update motion types in track list */
    u32Success |= lkfTrackManagement_finalizeCycle(f32dt);

    return u32Success;
}

/**
 * \brief Part of the main cycle of fusion process using filtering.
 * Has to be called whenever a new sensor list is available
 * Within a cycle has to be called in descending latency order
 *
 * \param pTueObjListInput sensor input list
 * \param u32InputSensorPattern sensor pattern of sensor input lists
 * \return TRUE (ok) or FALSE (error occured; output invalid)
 */
/* PRQA S 1532 2 */ /* Interface function called from Fusion AAU */
uint32 modes_filtering_cycle_SensorEvent(CONSTP2VAR(
    TueObjFusn_ObjectListType, AUTOMATIC, ObjFusn_VAR_NOINIT) pObjectList) {
    uint32 u32Success;
    P2VAR(TueObjFusn_EgoMotionType, AUTOMATIC, ObjFusn_VAR_NOINIT) pEgoMotion;

    const float32 f32inputLat = pObjectList->f32MeasurementLatency;
    const float32 f32TrkListAge = lkfTrackManagement_getTrkListAge();
    const float32 f32LatencyDiff = f32inputLat - f32TrkListAge;

    u32Success = EgoMotion_getEgoMotionHistory(
        &pEgoMotion, pObjectList->f32MeasurementLatency);
    u32Success |= Kinematics_Relative2OverGround(
        pEgoMotion, pObjectList,
        TUE_PRV_VEHICLE_DISTANCE_REAR_AXLE_TO_FRONT_BUMBER);

    if (FLT_ZERO < f32LatencyDiff) {
        /* compensate movement of ego vehicle by substracting dx, dy
        * and rotating vx,vy,ax,ay if they are in object list
        -> if sensor has not acceleration in x,y the states are excluded inside
        */
        u32Success |= EgoCoordCompensation_compensateTrackbleList(
            pObjectList->aTrackable, pObjectList->u16NumObjects,
            (sint16*)NULL_PTR, FALSE, pObjectList->f32MeasurementLatency,
            f32TrkListAge, TRUE);

        /* Predict input according to measurement latency difference */
        u32Success |=
            modes_predictObjectList(pObjectList, f32LatencyDiff, FALSE);
    } else {
        /* do not predict input list but instead
         * predict track to measurement time (non-OOSM-case) */

        /* compensate ego motion between current and new track point */
        u32Success |=
            lkfTrackManagement_compensateInternalTrkbleList(f32inputLat);
        /* predict all tracks to current time of filter using their motion state
         */
        u32Success |= lkfTrackManagement_predictInternalTrkbl(-f32LatencyDiff);
    }

    /* object to track association and filter update */
    u32Success |= modes_measToLkfUpdate(pObjectList);

    return u32Success;
}

/**
 * \brief Part of the main cycle of fusion process using filtering.
 * Has to be called once during each cycle.
 * Refines estimate and generates the desired output
 *
 * \param pTueObjectListOutput[out] fusion output trackable list
 * \return TRUE (ok) or FALSE (error occured; output invalid)
 */

/* PRQA S 1532 2 */ /* Interface function called from Fusion AAU */
uint32 modes_filtering_cycle_TimerEvent(CONSTP2VAR(
    TueObjFusn_ObjectListType, AUTOMATIC, ObjFusn_VAR_NOINIT) pObjList_Output) {
    uint32 u32Success;
    const float32 f32TrkListAge = lkfTrackManagement_getTrkListAge();

    CONSTP2VAR(TueObjFusn_TrackableListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
    pTrackList = lkfTrackManagement_getTrkbleList();
    const sint16 s16NumObjectsToReduce =
        (sint16)(pTrackList->u16ValidTrackables) -
        (sint16)TUE_PRV_FUSION_MAX_OUTPUT_OBJECTS;

    VAR(boolean, ObjFusn_VAR_NOINIT)
    abObjectsToBeDropped[TUEOBJFUSN_TRACKABLELIST_U16VALIDTRACKABLES_MAX];

    /* Reduce internal trackable list such that the number of out objects does
     * not exceed the maximal number of output objects */
    u32Success = tue_prv_object_selection_select_objects(
        pTrackList, s16NumObjectsToReduce, abObjectsToBeDropped);

    /* extract object list from track list */
    u32Success |= lkfTrackManagement_copyLkfTrkbl2Outputlist(
        pObjList_Output, abObjectsToBeDropped);

    /* compensate & predict to current point in time */
    u32Success |= EgoCoordCompensation_compensateTrackbleList(
        pObjList_Output->aTrackable, pObjList_Output->u16NumObjects,
        (sint16*)NULL_PTR, FALSE, f32TrkListAge, FLT_ZERO, TRUE);

    u32Success |= modes_predictObjectList(pObjList_Output, f32TrkListAge, TRUE);

    /* update counter of output lists */
    modes_updateCounter(pObjList_Output);

    /* object list post processing, only required for unit/integration tests and
     * can be removed in serices code */
    u32Success |= modes_postprocessing(pObjList_Output);

    return u32Success;
}

/**
 * \brief update update-counter of output list
 *
 * \param[in, out] pObjectList    pointer to object list (write access)
 */
LOCAL void modes_updateCounter(CONSTP2VAR(TueObjFusn_ObjectListType,
                                          AUTOMATIC,
                                          ObjFusn_VAR_NOINIT) pObjList) {
    pObjList->u16ListUpdateCounter = s_u16UpdateCounter;

    s_u16UpdateCounter =
        ((s_u16UpdateCounter + 1u) %
         (TUEOBJFUSN_OBJECTLIST_U16LISTUPDATECOUNTER_MAX + 1u));
}

LOCAL uint32 modes_postprocessing(CONSTP2VAR(TueObjFusn_ObjectListType,
                                             AUTOMATIC,
                                             ObjFusn_VAR_NOINIT) outList) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    const boolean bOutputIsOvergroundCur = Fusion_get_bOutputIsOverground();
    P2VAR(TueObjFusn_EgoMotionType, AUTOMATIC, ObjFusn_VAR_NOINIT) stEgoMotion;

    // add ego motion so that ouput is relative motion
    if (FALSE == bOutputIsOvergroundCur) {
        u32Success = EgoMotion_getEgoMotionHistory(&stEgoMotion, FLT_ZERO);
        u32Success |= Kinematics_OverGround2Relative(
            stEgoMotion, outList,
            TUE_PRV_VEHICLE_DISTANCE_REAR_AXLE_TO_FRONT_BUMBER);
    } else {
        /* MISRA */
    }

    return u32Success;
}

LOCAL uint32 modes_measToLkfUpdate(CONSTP2VAR(TueObjFusn_ObjectListType,
                                              AUTOMATIC,
                                              ObjFusn_VAR_NOINIT) pObjectList) {
    uint32 u32Success;

    VAR(stMatchIndexArrayType, ObjFusn_VAR_NOINIT) stFusionMatches;
    VAR(boolean, ObjFusn_VAR_NOINIT)
    abObjectAssignedFlags[TUE_PRV_MATCH_INDEX_ARRAY_SIZE];
    P2VAR(TueObjFusn_EgoMotionType, AUTOMATIC, ObjFusn_VAR_NOINIT) pEgoMotion;

    CONSTP2VAR(TueObjFusn_TrackableListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
    pTrackList = lkfTrackManagement_getTrkbleList();
    const boolean bUseCoastingTmp = Fusion_get_bUseCoasting();
    const float32 f32TrackListAge = lkfTrackManagement_getTrkListAge();

    u32Success = EgoMotion_getEgoMotionHistory(&pEgoMotion, f32TrackListAge);

    tue_prv_common_array_utils_defaultInit_abool(
        abObjectAssignedFlags, TUE_PRV_MATCH_INDEX_ARRAY_SIZE, FALSE);

    u32Success |= modes_runAssociation(pTrackList, pObjectList,
                                       &stFusionMatches, pEgoMotion);

    /** Clear sensor information from valid trackables for internal fusion
     * object list */
    /** Sensor information includes to clear the sensor update pattern if the
     * object was updated by this pattern before */
    u32Success |= lkfTrackManagement_deleteSensorInformation(
        pObjectList->u32SensorPattern);

    // update tracks based on association results
    u32Success |= lkfTrackManagement_updateTrkList(
        pObjectList, &stFusionMatches, abObjectAssignedFlags, pEgoMotion);

    if (TRUE == bUseCoastingTmp) {
        u32Success |= lkfTrackManagement_selectVioTrackables(
            pObjectList->u16NumObjects, stFusionMatches.u16NumMatches);
    } else {
        /* MISRA */
    }

    /* setup new tracks using sensor input that could not be associated to
     * existing tracks */
    u32Success |= lkfTrackManagement_setupNewLkfTrkble(pObjectList,
                                                       abObjectAssignedFlags);

    return u32Success;
}

/* PRQA S 1532 2 */ /* Library Function */
uint32 modes_filtering_removeDeadSensor(const uint32 u32Sensor) {
    return lkfTrackManagement_removeObsoleteTracks(u32Sensor);
}

LOCAL uint32 modes_runAssociation(
    CONSTP2VAR(TueObjFusn_TrackableListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pTrackListInternal,
    CONSTP2VAR(TueObjFusn_ObjectListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pTrackListSensor,
    CONSTP2VAR(stMatchIndexArrayType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pMatchesArray,
    CONSTP2CONST(TueObjFusn_EgoMotionType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pEgoMotion) {
    uint32 u32Success;
    uint16 u16i;
    const float32 _f32MatchGate = Fusion_get_f32MatchGate();
    VAR(uint16, ObjFusn_VAR_NOINIT)
    au16IndexMapInternal[TUE_PRV_DISTSCORE_MAX_ROW_SIZE];
    VAR(uint16, ObjFusn_VAR_NOINIT)
    au16IndexMapMeasurements[TUE_PRV_DISTSCORE_MAX_COL_SIZE];

    CONSTP2VAR(stDistMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT)
    pDistMatrix = getDistMat();

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
    /* input validity checks */
    if ((TUEOBJFUSN_TRACKABLELIST_U16VALIDTRACKABLES_MAX <
         pTrackListInternal->u16ValidTrackables) ||
        (TUE_PRV_FUSION_MAX_INPUT_OBJECTS < pTrackListSensor->u16NumObjects)) {
        u32Success = TUEOBJFUSN_ERROR_NUMBER_OF_OBJECTS_EXCEEDS_MAXIMUM_SIZE;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NUMBER_OF_OBJECTS_EXCEEDS_MAXIMUM_SIZE,
            TUEOBJFUSN_AAU_DISTANCE_SCORE,
            TUEOBJFUSN_AAU_DISTANCE_SCORE_DISTANCE_SCORE_TRACK_TO_TRACK);
    } else
#endif
    {
        u32Success = distance_score_track2track(
            pTrackListInternal, pTrackListSensor, _f32MatchGate, pDistMatrix,
            au16IndexMapMeasurements, au16IndexMapInternal, pEgoMotion);

        /* call association algorithm */
        u32Success |= tue_prv_association_runAssociation(
            pDistMatrix, _f32MatchGate, pMatchesArray);

        /* Remap from index array to matches array */
        for (u16i = 0u; u16i < pMatchesArray->u16NumMatches; u16i++) {
            pMatchesArray->aMatchIndexArray[u16i].u16IndexCol =
                au16IndexMapMeasurements[pMatchesArray->aMatchIndexArray[u16i]
                                             .u16IndexCol];
            pMatchesArray->aMatchIndexArray[u16i].u16IndexRow =
                au16IndexMapInternal[pMatchesArray->aMatchIndexArray[u16i]
                                         .u16IndexRow];
        }
    }

    return u32Success;
}

LOCAL uint32 modes_predictObjectList(CONSTP2VAR(TueObjFusn_ObjectListType,
                                                AUTOMATIC,
                                                ObjFusn_VAR_NOINIT) pObjectList,
                                     const float32 f32_dTforward,
                                     const boolean bPredictHeading) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    uint16 u16i = 0u;
    P2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pTrkbl;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if (NULL_PTR == pObjectList) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION,
            TUEOBJFUSN_AAU_MODES_FILTERING,
            TUEOBJFUSN_AAU_MODES_FILTERING_PREDICT_OBJECT_LIST);
    } else
#endif
        if (pObjectList->u16NumObjects > TUE_PRV_FUSION_OBJECT_LIST_SIZE) {
        u32Success = TUEOBJFUSN_ERROR_NUMBER_OF_OBJECTS_EXCEEDS_MAXIMUM_SIZE;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NUMBER_OF_OBJECTS_EXCEEDS_MAXIMUM_SIZE,
            TUEOBJFUSN_AAU_MODES_FILTERING,
            TUEOBJFUSN_AAU_MODES_FILTERING_PREDICT_OBJECT_LIST);
    } else if (f32_dTforward < FLT_ZERO) {
        u32Success = TUEOBJFUSN_ERROR_INVALID_INPUT;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_INVALID_INPUT, TUEOBJFUSN_AAU_MODES_FILTERING,
            TUEOBJFUSN_AAU_MODES_FILTERING_PREDICT_OBJECT_LIST);
    } else
#endif
    {
        for (u16i = 0u; u16i < pObjectList->u16NumObjects; u16i++) {
            pTrkbl = &(pObjectList->aTrackable[u16i]);

            u32Success |= LKF_DoPredict(pTrkbl, f32_dTforward);

            if (TRUE == bPredictHeading) {
                u32Success |=
                    LKF_CoordinatedTurn_DoPredict(pTrkbl, f32_dTforward);
            } else {
                /* MISRA */
            }
        }

        if (TUEOBJFUSN_ERROR_NOERROR == u32Success) {
            pObjectList->f32MeasurementLatency -= f32_dTforward;
        } else {
            /* MISRA */
        }
    }

    return u32Success;
}

#define ObjFusn_STOP_SEC_CODE

/** /} */
