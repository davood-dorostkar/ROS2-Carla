/** \defgroup tueFusion TUE Fusion
\brief This is the main module for Fusion

Please give a more detailed description of the module here\n

* \addtogroup tueFusion
* \{
*
* \file     tue_Fusion.c
*
* \brief This is the main module for Fusion
*/
/* PRQA S 0292 ++ */ /* MKS */
                     /*
                      */
/* PRQA S 0292 -- */ /* MKS */
                     /*
                      *          (C) Copyright Tuerme Inc. All rights reserved.
                      *
                      */
/*==================[inclusions]============================================*/
#include "tue_Fusion.h"
#include "tue_prv_fusion_internal.h"
/* perform assertions here */
#include "TueObjFusn_Assertions.h"

#include "tue_prv_motionType.h"

#include "tue_prv_common_types.h"
#include "TueObjFusn_Eps.h"

#include "tue_prv_fusion_tools.h"

#include "tue_prv_common_array_utils.h"
#include "tue_prv_fusion_math.h"

#include "tue_prv_modes_filtering.h"
#include "tue_prv_error_management.h"

#include "TueObjFusn_TrackableListUtils.h"
#include "TueObjFusn_TrackableListProps.h"
#include "TueObjFusn_ObjectListProps.h"
#include "TueObjFusn_ObjectListType.h"
#include "TueObjFusn_ErrorCodes.h"
#include "TueObjFusn_AAU_Codes.h"

/*==================[variables]=============================================*/

//#define ObjFusn_START_SEC_VAR32

/**
 * For each of these inputs store to which sensor it belongs so that
 */
#define ASW_QM_CORE5_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
LOCAL VAR(uint32,
          ObjFusn_VAR_NOINIT) s_sensorPatternList[TUE_PRV_FUSION_MAX_INPUTS] = {
    0u};
//#define ObjFusn_STOP_SEC_VAR32

//#define ObjFusn_START_SEC_VAR16

/**
 * \brief Memory for list update counters
 */
LOCAL VAR(uint16, ObjFusn_VAR_NOINIT)
    s_lastListUpdateCounters[TUE_PRV_FUSION_MAX_INPUTS] = {0u};

//#define ObjFusn_STOP_SEC_VAR16

//#define ObjFusn_START_SEC_VAR32

/**
 * implements some basic error management. As soon as one function returns
 * FALSE fusion will stay in the error state until it is reinitialized.
 */
LOCAL VAR(uint32, ObjFusn_VAR_NOINIT) s_u32NoError = TUEOBJFUSN_ERROR_NOERROR;
LOCAL VAR(float32, ObjFusn_VAR_NOINIT)
    s_af32SensorUpdateAge[TUE_PRV_FUSION_MAX_INPUTS] = {FLT_ZERO, FLT_ZERO};

/*
 * \brief   Sensor pattern to collect all sensors that provide an update in the
 * current fusion cycle.
 *          Shall be resetted in each startCycle() event.
 */
LOCAL VAR(uint32, ObjFusn_VAR_NOINIT)
    s_u32SensorUpdatePattern = TUEOBJFUSN_TRACKABLE_U32SENSOR_INVALID;
#define ASW_QM_CORE5_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"

#define ASW_QM_CORE5_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
sint8 inputIdxsInProcessingOrder[TUE_PRV_FUSION_MAX_INPUTS];
boolean sensorListsUpdatedList[TUE_PRV_FUSION_MAX_INPUTS];
#define ASW_QM_CORE5_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
//#define ObjFusn_STOP_SEC_VAR32

/// \}

/*==================[functions]=============================================*/
#define ObjFusn_START_SEC_SLOW_CODE

/// @name Fusion Main functions
void Fusion_init(void)  // PRQA S 1503
{
    /* all AAUs */
    modes_filtering_init();
    Fusion_set_DefaultParameters();
    /* Init sensor info array */
    Trackable_initSensorPatternBuffer();

    /* local variables */
    tue_prv_common_array_utils_defaultInit_au32(
        s_sensorPatternList, TUE_PRV_FUSION_MAX_INPUTS,
        TUEOBJFUSN_TRACKABLE_U32SENSOR_INVALID);
    tue_prv_common_array_utils_defaultInit_au16(
        s_lastListUpdateCounters, TUE_PRV_FUSION_MAX_INPUTS,
        TUEOBJFUSN_OBJECTLIST_U16LISTUPDATECOUNTER_MAX);
    tue_prv_common_array_utils_defaultInit_af32(
        s_af32SensorUpdateAge, TUE_PRV_FUSION_MAX_INPUTS, FLT_ZERO);
    s_u32NoError = TUEOBJFUSN_ERROR_NOERROR;

    /* Init error buffer */
    tue_prv_error_management_init();
}
#define ObjFusn_STOP_SEC_SLOW_CODE

#define ObjFusn_START_SEC_CODE

uint32 PerformFusionCycle(
    CONSTP2VAR(TueObjFusn_ObjectListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pObjectLists_Input[],  // PRQA S 1503
    CONSTP2CONST(TueObjFusn_EgoMotionType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pEgoMotion,
    const float32 f32dt,
    CONSTP2VAR(TueObjFusn_ObjectListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pObjectList_Output,
    CONSTP2VAR(TueObjFusn_ErrorBufferType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pErrorBuffer) {
    uint16 u16nLists;
    uint16 u16i;
    sint8 u8Idx;
    uint16 u16SensPos;
    uint32 u32SensPattern;
    /* in this pattern each set bit indicates a sensor that has provided an
     * updated object list (indicated by a change in the update counter). */
    /* array of object list indices that indicates in which order the object
     * lists are to be processed. */
    // sint8 inputIdxsInProcessingOrder[TUE_PRV_FUSION_MAX_INPUTS];

    /* ToDo: Clarify where to put this piece of code to be more compatible to
     * RTE runnable conecpt */
    if (TUEOBJFUSN_ERROR_NOERROR != s_u32NoError) {
        Fusion_reInit();
    } else {
        /* MISRA */
    }

    /* compute update pattern & input ordering  */
    s_u32NoError |= analyzeInputLists(pObjectLists_Input, s_sensorPatternList,
                                      &u16nLists, inputIdxsInProcessingOrder);

    s_u32NoError |=
        Fusion_startCycle(s_sensorPatternList, u16nLists, f32dt, pEgoMotion);

    for (u16i = 0u; u16i < TUE_PRV_FUSION_MAX_INPUTS; u16i++) {
        /** Only increment timeout monitoring for sensors seen before */
        if (s_sensorPatternList[u16i] !=
            TUEOBJFUSN_TRACKABLE_U32SENSOR_INVALID) {
            s_af32SensorUpdateAge[u16i] += f32dt;
        } else {
            /* MISRA */
        }
    }

    for (u16i = 0u; u16i < u16nLists; u16i++) {
        u8Idx = inputIdxsInProcessingOrder[u16i];

        if ((-1 < u8Idx) && (0u < s_sensorPatternList[u8Idx])) {
            /* Reset age count for this sensor */
            u16SensPos = Trackable_getSensPos(
                pObjectLists_Input[u8Idx]->u32SensorPattern);
            s_af32SensorUpdateAge[u16SensPos] = FLT_ZERO;

            s_u32NoError |=
                Fusion_PerformSensorEvent(pObjectLists_Input[u8Idx]);
        } else {
            /* MISRA */
        }
    }

    /* Increment error age in each cycle */
    for (u16i = 0u; u16i < TUE_PRV_FUSION_MAX_INPUTS; u16i++) {
        if (s_af32SensorUpdateAge[u16i] >= TUE_PRV_VEHICLE_SENSOR_TIMEOUT) {
            u32SensPattern = Trackable_getSensPattern(u16i);

            s_u32NoError |= modes_filtering_removeDeadSensor(u32SensPattern);
        } else {
            /* MISRA */
        }
    }

    s_u32NoError |=
        Fusion_performTimerEvent(pObjectList_Output, f32dt, pErrorBuffer);

    return s_u32NoError;
}

/* PRQA S 1505 2 */ /* Function declared in public header as function is used in
                       RTE Runnable */
uint32 Fusion_startCycle(const uint32 au32SensPatList[],
                         const uint16 u16NumLists,
                         const float32 f32dt,
                         CONSTP2CONST(TueObjFusn_EgoMotionType,
                                      AUTOMATIC,
                                      ObjFusn_VAR_NOINIT) pEgoMotion) {
    uint32 u32Error = TUEOBJFUSN_ERROR_NOERROR;

    /* Increment age for each error stored in buffer */
    s_u32NoError |= tue_prv_error_management_incrementAge();

    s_u32SensorUpdatePattern = TUEOBJFUSN_TRACKABLE_U32SENSOR_INVALID;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
    if (f32dt < TUE_PRV_FUSION_DT_EPS) {
        /* ensure we have a valid positive dt */
        u32Error = TUEOBJFUSN_ERROR_INVALID_INPUT;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_INVALID_INPUT, TUEOBJFUSN_AAU_FUSION,
            TUEOBJFUSN_AAU_FUSION_START_CYCLE);
    } else
#endif
    {
        u32Error = Trackable_setSensorInfos(au32SensPatList, u16NumLists);
        u32Error |= modes_filtering_preCycle(f32dt, pEgoMotion);
    }

    return u32Error;
}

/* PRQA S 1505 2 */ /* Function declared in public header as function is used in
                       RTE Runnable */
uint32 Fusion_performTimerEvent(
    CONSTP2VAR(TueObjFusn_ObjectListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pObjectList_out,
    const float32 f32dt,
    CONSTP2VAR(TueObjFusn_ErrorBufferType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pErrorBuffer) {
    uint32 u32Error;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
/** NULL pointer check */
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if ((NULL_PTR == pObjectList_out) || (NULL_PTR == pErrorBuffer)) {
        u32Error = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION, TUEOBJFUSN_AAU_FUSION,
            TUEOBJFUSN_AAU_FUSION_PERFORM_TIMER_EVENT);
    } else
#endif
        if (f32dt < TUE_PRV_FUSION_DT_EPS) {
        /* ensure we have a valid positive dt */
        u32Error = TUEOBJFUSN_ERROR_INVALID_INPUT;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_INVALID_INPUT, TUEOBJFUSN_AAU_FUSION,
            TUEOBJFUSN_AAU_FUSION_PERFORM_TIMER_EVENT);
    } else
#endif
    {
        /* LKF post cycle */
        u32Error = modes_filtering_postCycle(f32dt, s_u32SensorUpdatePattern);

        /* generate output from filtering mode */
        u32Error |= modes_filtering_cycle_TimerEvent(pObjectList_out);

        /* error output */
        if (TUEOBJFUSN_ERROR_NOERROR != s_u32NoError) {
            (void)Trackable_initObjectList(pObjectList_out);
        } else {
            /* MISRA */
        }

        (void)tue_prv_error_management_copyBuffer(pErrorBuffer);
    }

    (void)tue_prv_error_management_clearBuffer();

    return u32Error;
}

/* PRQA S 1505 2 */ /* Function declared in public header as function is used in
                       RTE Runnable */
uint32 Fusion_PerformSensorEvent(CONSTP2VAR(TueObjFusn_ObjectListType,
                                            AUTOMATIC,
                                            ObjFusn_VAR_NOINIT) pObjList) {
    uint32 u32Error = TUEOBJFUSN_ERROR_NOERROR;

#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if (NULL_PTR == pObjList) {
        u32Error = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION, TUEOBJFUSN_AAU_FUSION,
            TUEOBJFUSN_AAU_FUSION_SENSOR_EVENT);
    } else
#endif
    {
        u32Error = modes_filtering_cycle_SensorEvent(pObjList);

        s_u32SensorUpdatePattern |= pObjList->u32SensorPattern;

        // CONSTP2VAR(TueObjFusn_TrackableListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        // pTrackList = lkfTrackManagement_getTrkbleList();
    }

    return u32Error;
}

LOCAL uint16 countNrOfInputLists(CONSTP2VAR(TueObjFusn_ObjectListType,
                                            AUTOMATIC,
                                            ObjFusn_VAR_NOINIT)
                                     pObjectListInput[]) {
    uint16 u16i = 0u;
    uint16 u16NrInputLists = 0u;

    while ((NULL_PTR != pObjectListInput[u16i]) &&
           (u16i < TUE_PRV_FUSION_MAX_INPUTS)) {
        u16NrInputLists++;
        u16i++;
    }

    return u16NrInputLists;
}

LOCAL uint32 analyzeInputLists(
    CONSTP2VAR(TueObjFusn_ObjectListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pObjectListInput[],
    VAR(uint32, ObjFusn_VAR_NOINIT) au32InputSensorPatterns[],
    CONSTP2VAR(uint16, AUTOMATIC, ObjFusn_VAR_NOINIT) u16nInputLists,
    VAR(sint8, ObjFusn_VAR_NOINIT) as8ProcessingOrderOut[]) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    uint16 u16i;
    const uint16 u16NumLists = countNrOfInputLists(pObjectListInput);
    const uint32 u32FsnSensorMode = Fusion_get_u32SensorMode();
    uint32 u32NumberOfSensorBits = 0u;

    /* array that indicates for each input object list whether list update
     * counter incremented from the last cycle */
    // boolean sensorListsUpdatedList[TUE_PRV_FUSION_MAX_INPUTS];

    /* check changes in update counters to determine where new lists are
     * available */
    identifyUpdatedLists(pObjectListInput, sensorListsUpdatedList, u16NumLists);

/* verify null termination of shortened input list */
#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if ((NULL_PTR == as8ProcessingOrderOut) ||
        (NULL_PTR == au32InputSensorPatterns)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION, TUEOBJFUSN_AAU_FUSION,
            TUEOBJFUSN_AAU_FUSION_ANALYZE_INPUT_LISTS);
    } else
#endif
        if ((TUE_PRV_FUSION_MAX_INPUTS > u16NumLists) &&
            (NULL_PTR != pObjectListInput[u16NumLists])) {
        u32Success = TUEOBJFUSN_ERROR_INTERNAL_ERROR;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_INTERNAL_ERROR, TUEOBJFUSN_AAU_FUSION,
            TUEOBJFUSN_AAU_FUSION_ANALYZE_INPUT_LISTS);
    } else
#endif
    {
        for (u16i = 0u; u16i < u16NumLists; u16i++) {
            u32NumberOfSensorBits =
                getNumberOfSetBits(pObjectListInput[u16i]->u32SensorPattern);
            /* Check if Fusion is configured to receive input list */
            if ((u32FsnSensorMode &
                 (pObjectListInput[u16i]->u32SensorPattern)) ==
                TUEOBJFUSN_OBJECTLIST_U32SENSORPATTERN_MIN) {
                /* Ignore List */
            } else if (u32NumberOfSensorBits == 0u) {
#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
                u32Success = TUEOBJFUSN_ERROR_INVALID_OBJECT_LIST_PROVIDED;
                (void)tue_prv_error_management_addError(
                    TUEOBJFUSN_ERROR_INVALID_OBJECT_LIST_PROVIDED,
                    TUEOBJFUSN_AAU_FUSION,
                    TUEOBJFUSN_AAU_FUSION_ANALYZE_INPUT_LISTS);
#endif
            } else if (u32NumberOfSensorBits != 1u) {
                /* Ignore List */
            } else if (0u == au32InputSensorPatterns[u16i]) {
                /* Store first occurence in case sensor pattern is seen */
                au32InputSensorPatterns[u16i] =
                    pObjectListInput[u16i]->u32SensorPattern;
            } else {
                /* Orignially, the input list order check was done here. No
                 * longer needed due to split in sensor event */
            }
        }

        /* evaluate the processing order for input lists */
        u32Success |=
            computeInputListOrdering(pObjectListInput, sensorListsUpdatedList,
                                     u16NumLists, as8ProcessingOrderOut);
        *u16nInputLists = u16NumLists;
    }

    return u32Success;
}

LOCAL void identifyUpdatedLists(
    CONSTP2VAR(TueObjFusn_ObjectListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pObjectListInput[],
    VAR(boolean, ObjFusn_VAR_NOINIT) sensorListsUpdatedList[],
    const uint16 u16nLists) {
    uint16 u16i;
    const uint32 u32FsnSensorMode = Fusion_get_u32SensorMode();
    uint32 u32NumberOfSensorBits;
    tue_prv_common_array_utils_defaultInit_abool(
        sensorListsUpdatedList, TUE_PRV_FUSION_MAX_INPUTS, FALSE);

    /* go through all input lists */
    for (u16i = 0u; u16i < u16nLists; ++u16i) {
        u32NumberOfSensorBits =
            getNumberOfSetBits(pObjectListInput[u16i]->u32SensorPattern);
        /* Check if Fusion is configured to receive input list */
        if ((u32FsnSensorMode & (pObjectListInput[u16i]->u32SensorPattern)) ==
            TUEOBJFUSN_OBJECTLIST_U32SENSORPATTERN_MIN) {
            /* Ignore this list */
        } else if ((u32NumberOfSensorBits != 1u)) {
            /* ignore this list */
        }
        /* sanity check of static buffer */
        else if (s_lastListUpdateCounters[u16i] >
                 TUEOBJFUSN_OBJECTLIST_U16LISTUPDATECOUNTER_MAX)  // no object
                                                                  // have ever
                                                                  // been seen
        {
            /* this branch should never be accessed */
            sensorListsUpdatedList[u16i] = FALSE;
        } else if (pObjectListInput[u16i]->u16ListUpdateCounter ==
                   TUEOBJFUSN_OBJECTLIST_U16LISTUPDATECOUNTER_DEFAULT) {
            sensorListsUpdatedList[u16i] = FALSE;
        } else if (pObjectListInput[u16i]->u16ListUpdateCounter !=
                   s_lastListUpdateCounters[u16i]) {
            sensorListsUpdatedList[u16i] = TRUE;

            /* now store new value internally */
            s_lastListUpdateCounters[u16i] =
                pObjectListInput[u16i]->u16ListUpdateCounter;
        } else {
            sensorListsUpdatedList[u16i] = FALSE;
        }
    } /* do this for all valid input lists */
}

LOCAL uint32 computeInputListOrdering(
    CONSTP2VAR(TueObjFusn_ObjectListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pObjectList_Input[],
    const boolean sensorListsValidList[],
    const uint16 u16nLists,
    VAR(sint8, ObjFusn_VAR_NOINIT) pInputProcessOrder[]) {
    uint8 u8i;
    boolean bBreak = FALSE;
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    /* PRQA S 3204 1 */ /* variable may be set to other values depending on
                           run-time check activation */
    sint16 largestIdx;
    float32 largestValue = FLT_ZERO;
    float32 sensorLatencies[TUE_PRV_FUSION_MAX_INPUTS];

    /* set sensor update pattern to zero */
    tue_prv_common_array_utils_defaultInit_as8(pInputProcessOrder,
                                               TUE_PRV_FUSION_MAX_INPUTS, -1);
    tue_prv_common_array_utils_defaultInit_af32(
        sensorLatencies, TUE_PRV_FUSION_MAX_INPUTS, -FLT_MAX);

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
    if (u16nLists > TUE_PRV_FUSION_MAX_INPUTS) {
        u32Success = TUEOBJFUSN_ERROR_INVALID_INPUT;
    } else
#endif
    {
        /* now take latencies from lists */
        for (u8i = 0u; u8i < u16nLists; ++u8i) {
            if (FALSE == sensorListsValidList[u8i]) {
                // if list holds no updates, overwrite sensor latency entry with
                // -FLT_MAX to ensure we don't use this sensor when setting up
                // processing
                // order below
                sensorLatencies[u8i] = -FLT_MAX;
            } else {
                // otherwise save sensor latency
                sensorLatencies[u8i] =
                    pObjectList_Input[u8i]->f32MeasurementLatency;
            }
        }

        // generate processing order based on sensor latencies
        for (u8i = 0u; (u8i < u16nLists) && (FALSE == bBreak); ++u8i) {
            largestValue = tue_prv_common_array_utils_findLargestIndex_f32(
                sensorLatencies, u16nLists, &largestIdx);

            // return value equals -FLT_MAX or smallestIdx equals -1, i.e. found
            // all
            // sensors with updates already
            /* PRQA S 3416 1 */ /* simple fcn */
            if ((FLT_EPSILON >=
                 tue_prv_fusion_abs(largestValue - (-FLT_MAX))) ||
                (0 > largestIdx)) {
                bBreak = TRUE;
            } else {
                // add found index to processing order list
                pInputProcessOrder[u8i] = (sint8)largestIdx;

                // set value to an invalid latency value to avoid getting the
                // same index twice
                sensorLatencies[(uint16)largestIdx] = -FLT_MAX;
            }
        }
    }

    return u32Success;
}
#define ObjFusn_STOP_SEC_CODE

#define ObjFusn_START_SEC_SLOW_CODE

LOCAL void Fusion_reInit(void) {
    /* all AAUs */
    modes_filtering_init();
    Trackable_initSensorPatternBuffer();

    /* local variables */
    tue_prv_common_array_utils_defaultInit_au32(
        s_sensorPatternList, TUE_PRV_FUSION_MAX_INPUTS,
        TUEOBJFUSN_TRACKABLE_U32SENSORSCURR_MIN);
    tue_prv_common_array_utils_defaultInit_au16(
        s_lastListUpdateCounters, TUE_PRV_FUSION_MAX_INPUTS,
        TUEOBJFUSN_OBJECTLIST_U16LISTUPDATECOUNTER_MAX);
    s_u32NoError = TUEOBJFUSN_ERROR_NOERROR;
}
#define ObjFusn_STOP_SEC_SLOW_CODE

#ifdef UNITTEST
/// used for tests only
uint16* idmodes_s_lastListUpdateCounters(
    const uint16 index) /* PRQA S 1503 */ /* used for tests only */
{
    return &s_lastListUpdateCounters[index];
}
#endif /* UNITTEST */

/**
 * \}
 */
