/** \defgroup TueObjFusn_TrackableListutils Trackable list utils
 *  \{
 *
 * \file       TueObjFusn_TrackableListUtilsTmp.c
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

#include "TueObjFusn_TrackableListUtils.h"
#include "TueObjFusn_ErrorCodes.h"
#include "TueObjFusn_AAU_Codes.h"
#include "tue_prv_error_management.h"
#include "tue_prv_fusion_memory.h"
#include "TueObjFusn_TrackableConstants.h"
#include "TueObjFusn_TrackableListProps.h"
#include "TueObjFusn_TrackableProps.h"
#include "TueObjFusn_ObjectListProps.h"

/*==================[variables]=============================================*/
/// @name TrackableListUtils global variables

/**
 * Save the sensor pattern for each input.
 * Used to map an inputted object list to the corresponding index
 */
//#define ObjFusn_START_SEC_VAR32
#define ASW_QM_CORE5_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
LOCAL VAR(uint32, ObjFusn_VAR_ZERO_INIT)
    s_sensorPatternListTrkbl[TUE_PRV_FUSION_MAX_INPUTS] = {0u};
#define ASW_QM_CORE5_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
//#define ObjFusn_STOP_SEC_VAR32

/// @}

/*==================[functions]=============================================*/

/**
 * @fn   void Trackable_initSensorPatternBuffer(void)
 *
 * @brief   Inits the static sensor pattern array to default values.
 *
 * @return  void
 */
/* PRQA S 1532 4 */ /* External Interface to static buffer */
#define ObjFusn_START_SEC_SLOW_CODE

void Trackable_initSensorPatternBuffer(void) {
    uint16 u16i = 0u;

    for (u16i = 0u; u16i < TUE_PRV_FUSION_MAX_INPUTS; u16i++) {
        s_sensorPatternListTrkbl[u16i] = TUEOBJFUSN_TRACKABLE_U32SENSOR_INVALID;
    }
}
#define ObjFusn_STOP_SEC_SLOW_CODE

/**
 * @fn   uint32 Trackable_setSensorInfos(u32_t const au32SensPatList[],
 * u16_t const u16NumLists)
 *
 * @brief   Copies the inputted sensor pattern array to the local static
 * buffer
 *
 * @param   au32SensPatList[]    u32_t const, sensor pattern array
 * @param   u16NumLists          u16_t const, length of sensor pattern array
 *
 * @return  TUEOBJFUSN_ERROR_NOERROR if no error occured and error code
 * otherwise.
 */
/* PRQA S 1532 4 */ /* External Interface to static buffer called by Fusion
                       AAU */
#define ObjFusn_START_SEC_CODE

uint32 Trackable_setSensorInfos(const uint32 au32SensPatList[],
                                const uint16 u16NumLists) {
    uint16 u16Sens;
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    /* PRQA S 3204 1 */ /* variable may be set to other values depending on
                           run-time pointer check activation */

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if (NULL_PTR == au32SensPatList) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_TRACKABLE_LIST_UTILS,
            TUEOBJFUSN_AAU_TRACKABLE_LIST_UTILS_SET_SENSOR_INFOS);
    } else
#endif
        if (TUE_PRV_FUSION_MAX_INPUTS < u16NumLists) {
        u32Success = TUEOBJFUSN_ERROR_INVALID_INPUT;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_TRACKABLE_LIST_UTILS,
            TUEOBJFUSN_AAU_TRACKABLE_LIST_UTILS_SET_SENSOR_INFOS);
    } else
#endif
    {
        for (u16Sens = 0u; u16Sens < u16NumLists; ++u16Sens) {
            if (TUEOBJFUSN_TRACKABLE_U32SENSORSCURR_MIN !=
                au32SensPatList[u16Sens]) {
                s_sensorPatternListTrkbl[u16Sens] = au32SensPatList[u16Sens];
            } else {
                /* MISRA */
            }
        }
    }

    return u32Success;
}

/**
 * @fn   uint16 Trackable_getSensPos(uint32 const u32SensorCurr)
 *
 * @brief   Get the index of the inputted sensor pattern in the sensor pattern
 array
 *
 * @param   u32SensorCurr     u32_t const, sensor pattern array
 *
 * @return  the index in the sensor pattern array if the corresponding pattern
 war found and TUE_PRV_FUSION_MAX_INPUTS + 1u
            if not found
 */
uint16 Trackable_getSensPos(const uint32 u32SensorCurr) {
    uint16 u16Senspos = TUEOBJFUSN_SENS_POS_INVALID;
    uint16 u16i;

    for (u16i = 0u; u16i < TUE_PRV_FUSION_MAX_INPUTS; ++u16i) {
        if (u32SensorCurr == s_sensorPatternListTrkbl[u16i]) {
            u16Senspos = u16i;
        } else {
            /* MISRA */
        }
    }
    return u16Senspos;
}

/* PRQA S 1532 2 */ /* Library Function */
uint32 Trackable_getSensPattern(const uint16 u16Idx) {
    uint32 _u32SensorPattern;

    if (u16Idx < TUE_PRV_FUSION_MAX_INPUTS) {
        _u32SensorPattern = s_sensorPatternListTrkbl[u16Idx];
    } else {
        _u32SensorPattern = TUEOBJFUSN_TRACKABLE_U32SENSOR_INVALID;
    }

    return _u32SensorPattern;
}

/**
 * @fn   uint32 Trackable_init(TueObjFusn_TrackableType * const pTrkble)
 *
 * @brief    Initialization of a trackable.
 *           Set all values to default values
 *
 * @param   pTrkble    TueObjFusn_TrackableType * const, Input trackable
 *
 * @return  TUEOBJFUSN_ERROR_NOERROR if no error occured and error code
 * otherwise.
 */
uint32 Trackable_init(CONSTP2VAR(TueObjFusn_TrackableType,
                                 AUTOMATIC,
                                 ObjFusn_VAR_NOINIT) pTrkble) {
    uint32 u32Success;
    uint16 u16i;

#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if (NULL_PTR == pTrkble) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_TRACKABLE_LIST_UTILS,
            TUEOBJFUSN_AAU_TRACKABLE_LIST_UTILS_TRACKABLE_INIT);
    } else
#endif
    {
        /* set each trackable object values to default */
        pTrkble->bUpdated = TUEOBJFUSN_TRACKABLE_BUPDATED_DEFAULT;
        pTrkble->u8VisionIdx = TUEOBJFUSN_TRACKABLE_U8VISIONIDX_DEFAULT;
        pTrkble->u16ID = TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT;
        pTrkble->u16Age = TUEOBJFUSN_TRACKABLE_U16AGE_DEFAULT;
        pTrkble->u16Lifespan = TUEOBJFUSN_TRACKABLE_U16LIFESPAN_DEFAULT;
        pTrkble->u32SensorsCurr = TUEOBJFUSN_TRACKABLE_U32SENSORSCURR_DEFAULT;
        pTrkble->u32SensorsHist = TUEOBJFUSN_TRACKABLE_U32SENSORSHIST_DEFAULT;
        pTrkble->u8CyclesNoRadar = TUEOBJFUSN_TRACKABLE_U8CYCLESNORADAR_DEFAULT;
        pTrkble->u8CyclesNoVision =
            TUEOBJFUSN_TRACKABLE_U8CYCLESNOVISION_DEFAULT;
        pTrkble->fRCS = 0;
        pTrkble->eObjMaintenanceState = 0;
        pTrkble->u8RadarMotionTypeInput = TU_SENSOR_MOTIONG_TYPE_INPUT_UNKNOWN;

        for (u16i = 0u; u16i < TUE_PRV_FUSION_MAX_INPUTS; ++u16i) {
            pTrkble->au16SensorID[u16i] = TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT;
            pTrkble->au16SensorIDLast[u16i] =
                TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT;
        }

        u32Success = f32VecZeros(&pTrkble->vecX, TUEOBJFUSN_MATRIX_SIZE);
        u32Success |= f32SymMatZeros(&pTrkble->matP, TUEOBJFUSN_MATRIX_SIZE);
        pTrkble->f32ExistenceQuality =
            TUEOBJFUSN_TRACKABLE_F32EXISTENCEQUALITY_DEFAULT;
        pTrkble->f32ObstacleProbability =
            TUEOBJFUSN_TRACKABLE_F32OBSTACLEPROBABILITY_MAX;
        pTrkble->f32Width = TUEOBJFUSN_TRACKABLE_F32WIDTH_DEFAULT;
        pTrkble->f32Length = TUEOBJFUSN_TRACKABLE_F32LENGTH_DEFAULT;
        /************* GAC new feature *************/
        pTrkble->f32Height = TUEOBJFUSN_TRACKABLE_F32HEIGHT_MIN;
        pTrkble->bMCPFlag = TUEOBJFUSN_TRACKABLE_bMCPFlag_DEFAULT;
        pTrkble->bCIPVFlag = TUEOBJFUSN_TRACKABLE_bCIPVFlag_DEFAULT;
        pTrkble->uLifeCycle = TUEOBJFUSN_TRACKABLE_uLifeCycle_DEFAULT;
        /************* GAC new feature *************/
        pTrkble->u16RefPoint = TUEOBJFUSN_TRACKABLE_U16REFPOINT_DEFAULT;
        pTrkble->u8CoordSystem = TUEOBJFUSN_TRACKABLE_U8COORDSYSTEM_DEFAULT;
        pTrkble->u16MotionType = TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_DEFAULT;
        pTrkble->u16Class = TUEOBJFUSN_TRACKABLE_U16CLASS_DEFAULT;
        pTrkble->u16ClassProb = TUEOBJFUSN_TRACKABLE_U16CLASSPROB_DEFAULT;

        pTrkble->f32Gain = TUEOBJFUSN_TRACKABLE_F32GAIN_DEFAULT;
        pTrkble->f32GainVar = TUEOBJFUSN_TRACKABLE_F32GAINVAR_DEFAULT;
        pTrkble->u8TrkblBinPosition =
            TUEOBJFUSN_TRACKABLE_U8TRKBLBINPOSITION_DEFAULT;

        /** Coordinated Turn Model */
        pTrkble->f32Heading = TUEOBJFUSN_TRACKABLE_F32HEADING_DEFAULT;
        pTrkble->f32HeadingVar = TUEOBJFUSN_TRACKABLE_F32HEADINGVAR_DEFAULT;
        pTrkble->f32YawRate = TUEOBJFUSN_TRACKABLE_F32YAWRATE_DEFAULT;
        pTrkble->f32YawRateVar = TUEOBJFUSN_TRACKABLE_F32YAWRATEVAR_DEFAULT;
        pTrkble->f32CovarHeadingYawRate =
            TUEOBJFUSN_TRACKABLE_F32COVARHEADINGYAWRATE_DEFAULT;
    }

    return u32Success;
}

/**
 * @fn   uint32 Trackable_initObjectList(TueObjFusn_ObjectListType * const
 * pObjList)
 *
 * @brief    Initialization of an object list to default values
 *
 * @param   pObjList    TueObjFusn_ObjectListType * const, Input object list
 * list
 *
 * @return  TUEOBJFUSN_ERROR_NOERROR if no error occured and error code
 * otherwise.
 */

uint32 Trackable_initObjectList(CONSTP2VAR(TueObjFusn_ObjectListType,
                                           AUTOMATIC,
                                           ObjFusn_VAR_NOINIT) pObjList) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    uint16 u16it;

#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if (NULL_PTR == pObjList) {
        /* error handling */
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_TRACKABLE_LIST_UTILS,
            TUEOBJFUSN_AAU_TRACKABLE_LIST_UTILS_INIT_OBJECT_LIST);
    } else
#endif
    {
        /* set main list values to default */
        pObjList->f32MeasurementLatency =
            TUEOBJFUSN_OBJECTLIST_F32MEASUREMENTLATENCY_DEFAULT;
        pObjList->u16ListUpdateCounter =
            TUEOBJFUSN_OBJECTLIST_U16LISTUPDATECOUNTER_DEFAULT;
        /* no valid trackables (i.e. not default!)*/
        pObjList->u16NumObjects = TUEOBJFUSN_OBJECTLIST_U16NUMOBJECTS_MIN;
        pObjList->u32SensorPattern =
            TUEOBJFUSN_OBJECTLIST_U32SENSORPATTERN_DEFAULT;

        for (u16it = 0u; u16it < TUE_PRV_FUSION_OBJECT_LIST_SIZE; u16it++) {
            u32Success |= Trackable_init(&pObjList->aTrackable[u16it]);
        }
    }

    return u32Success;
}
#define ObjFusn_STOP_SEC_CODE

/**
 * @fn   uint32 Trackable_listInit(TueObjFusn_TrackableListType * const
 * pTueTrackableList)
 *
 * @brief    Initialization of a trackable list
 *           Set all values to default values
 *
 * @param   pTrkble    TueObjFusn_TrackableListType * const, Input trackable
 * list
 *
 * @return  TUEOBJFUSN_ERROR_NOERROR if no error occured and error code
 * otherwise.
 */
#define ObjFusn_START_SEC_SLOW_CODE

/* PRQA S 1532 2 */ /* Library Function */
uint32 Trackable_listInit(CONSTP2VAR(TueObjFusn_TrackableListType,
                                     AUTOMATIC,
                                     ObjFusn_VAR_NOINIT) pTueTrackableList) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    uint16 u16it;

#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if (NULL_PTR == pTueTrackableList) {
        /* error handling */
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_TRACKABLE_LIST_UTILS,
            TUEOBJFUSN_AAU_TRACKABLE_LIST_UTILS_TRACKABLE_LIST_INIT);
    } else
#endif
    {
        /* need a memset here to make tests pass because the trackable list is
         * not well-aligned */
        //(void)memset(pTueTrackableList, 0,
        //(size_t)sizeof(TueObjFusn_TrackableListType));

        /* set main list values to default */
        pTueTrackableList->f32MeasurementLatency =
            TUEOBJFUSN_TRACKABLELIST_F32MEASUREMENTLATENCY_DEFAULT;
        pTueTrackableList->u16ListUpdateCounter =
            TUEOBJFUSN_OBJECTLIST_U16LISTUPDATECOUNTER_DEFAULT;
        pTueTrackableList->u32SensorsCurr =
            TUEOBJFUSN_TRACKABLELIST_U32SENSORSCURR_DEFAULT;
        /* no valid trackables (i.e. not default!)*/
        pTueTrackableList->u16ValidTrackables =
            TUEOBJFUSN_TRACKABLELIST_U16VALIDTRACKABLES_MIN;

        for (u16it = 0u;
             u16it < TUEOBJFUSN_TRACKABLELIST_U16VALIDTRACKABLES_MAX; u16it++) {
            pTueTrackableList->as16TrackableMap[u16it] =
                TUEOBJFUSN_TRACKABLELIST_AS16TRACKABLEMAP_DEFAULT;
            u32Success |= Trackable_init(&pTueTrackableList->aTrackable[u16it]);
        }
    }
    return u32Success;
}
#define ObjFusn_STOP_SEC_SLOW_CODE

#define ObjFusn_START_SEC_CODE

uint32 Trackable_copyTrackable(
    CONSTP2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pDest,
    CONSTP2CONST(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pSrc) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    /* PRQA S 3204 1 */ /* variable may be set to other values depending on
                           run-time pointer check activation */
    const uint32 u32Size = (uint32)sizeof(TueObjFusn_TrackableType);

#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if ((NULL_PTR == pDest) || (NULL_PTR == pSrc)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_TRACKABLE_LIST_UTILS,
            TUEOBJFUSN_AAU_TRACKABLE_LIST_UTILS_COPY_TRACKABLE);
    } else
#endif
    {
        /* PRQA S 0314 1 */ /* Cast to void required for copy function */
        tue_prv_fusion_memcpy((void *)pDest, (const void *)pSrc, u32Size);
    }

    return u32Success;
}
#define ObjFusn_STOP_SEC_CODE

#define ObjFusn_START_SEC_CODE

/* PRQA S 1532 2 */ /* Library Function */
void Trackable_getSensorInfos(uint32 au32SensorInfos[]) {
    uint16 u16i = 0u;
    for (u16i = 0u; u16i < TUE_PRV_FUSION_MAX_INPUTS; u16i++) {
        au32SensorInfos[u16i] = s_sensorPatternListTrkbl[u16i];
    }
}
#define ObjFusn_STOP_SEC_CODE

/** /} */
