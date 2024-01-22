/** \defgroup tuePrvEgoMotion TUE Ego Motion Module
 * \brief  Store and provide ego motion history data and calculate path driven
 *
 * \addtogroup tuePrvEgoMotion
 * \{
 * \file    tue_prv_egoMotion.c
 * \brief   source code of ego motion module
 */
/* PRQA S 0292 ++ */ /* MKS */
                     /*  */
                     /* PRQA S 0292 -- */
                     /*          (C) Copyright Tuerme Inc. All rights reserved.
                      */
/*==================[inclusions]============================================*/

#include "tue_prv_egoMotion.h"
#include "tue_prv_egoMotion_int.h"
#include "TueObjFusn_ErrorCodes.h"
#include "TueObjFusn_AAU_Codes.h"
#include "tue_prv_error_management.h"
#include "tue_prv_fusion_memory.h"

#include "tue_prv_common_matrix.h" /** FLT_ZERO */
#include "tue_prv_fusion_math.h"
#include "TueObjFusn_Eps.h"

/** offset of loop index for second-to-last iteration, wrt number of iterations
 */
#define TUE_EGOMOTION_OFFSET_SECOND_TO_LAST (2u)

/*==================[variables]================================================*/
//#define ObjFusn_START_SEC_VAR_UNSPECIFIED
#define ASW_QM_CORE5_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
LOCAL VAR(TueObjFusn_EgoMotionHistory,
          ObjFusn_VAR_NOINIT) s_sEgoMotionHistory = {
    {
        {FALSE, 0u, 0u, FLT_ZERO, FLT_ZERO, FLT_ZERO, FLT_ZERO},
        {FALSE, 0u, 0u, FLT_ZERO, FLT_ZERO, FLT_ZERO, FLT_ZERO},
        {FALSE, 0u, 0u, FLT_ZERO, FLT_ZERO, FLT_ZERO, FLT_ZERO},
        {FALSE, 0u, 0u, FLT_ZERO, FLT_ZERO, FLT_ZERO, FLT_ZERO},
        {FALSE, 0u, 0u, FLT_ZERO, FLT_ZERO, FLT_ZERO, FLT_ZERO},
        {FALSE, 0u, 0u, FLT_ZERO, FLT_ZERO, FLT_ZERO, FLT_ZERO},
        {FALSE, 0u, 0u, FLT_ZERO, FLT_ZERO, FLT_ZERO, FLT_ZERO},
        {FALSE, 0u, 0u, FLT_ZERO, FLT_ZERO, FLT_ZERO, FLT_ZERO},
        {FALSE, 0u, 0u, FLT_ZERO, FLT_ZERO, FLT_ZERO, FLT_ZERO},
        {FALSE, 0u, 0u, FLT_ZERO, FLT_ZERO, FLT_ZERO, FLT_ZERO},
        {FALSE, 0u, 0u, FLT_ZERO, FLT_ZERO, FLT_ZERO, FLT_ZERO},
        {FALSE, 0u, 0u, FLT_ZERO, FLT_ZERO, FLT_ZERO, FLT_ZERO},
        {FALSE, 0u, 0u, FLT_ZERO, FLT_ZERO, FLT_ZERO, FLT_ZERO},
        {FALSE, 0u, 0u, FLT_ZERO, FLT_ZERO, FLT_ZERO, FLT_ZERO},
        {FALSE, 0u, 0u, FLT_ZERO, FLT_ZERO, FLT_ZERO, FLT_ZERO},
        {FALSE, 0u, 0u, FLT_ZERO, FLT_ZERO, FLT_ZERO, FLT_ZERO},
        {FALSE, 0u, 0u, FLT_ZERO, FLT_ZERO, FLT_ZERO, FLT_ZERO},
        {FALSE, 0u, 0u, FLT_ZERO, FLT_ZERO, FLT_ZERO, FLT_ZERO},
        {FALSE, 0u, 0u, FLT_ZERO, FLT_ZERO, FLT_ZERO, FLT_ZERO},
        {FALSE, 0u, 0u, FLT_ZERO, FLT_ZERO, FLT_ZERO, FLT_ZERO},
        {FALSE, 0u, 0u, FLT_ZERO, FLT_ZERO, FLT_ZERO, FLT_ZERO},
        {FALSE, 0u, 0u, FLT_ZERO, FLT_ZERO, FLT_ZERO, FLT_ZERO},
        {FALSE, 0u, 0u, FLT_ZERO, FLT_ZERO, FLT_ZERO, FLT_ZERO},
        {FALSE, 0u, 0u, FLT_ZERO, FLT_ZERO, FLT_ZERO, FLT_ZERO},
        {FALSE, 0u, 0u, FLT_ZERO, FLT_ZERO, FLT_ZERO, FLT_ZERO},
        {FALSE, 0u, 0u, FLT_ZERO, FLT_ZERO, FLT_ZERO, FLT_ZERO},
        {FALSE, 0u, 0u, FLT_ZERO, FLT_ZERO, FLT_ZERO, FLT_ZERO},
        {FALSE, 0u, 0u, FLT_ZERO, FLT_ZERO, FLT_ZERO, FLT_ZERO},
        {FALSE, 0u, 0u, FLT_ZERO, FLT_ZERO, FLT_ZERO, FLT_ZERO},
        {FALSE, 0u, 0u, FLT_ZERO, FLT_ZERO, FLT_ZERO, FLT_ZERO},
    },
    0u,
    0u,
    FLT_ZERO}; /**< ego motion history buffer */

/* PRQA S 3218 1 */ /* variable may be accessed depending on run-time pointer
                       check activation */
LOCAL VAR(TueObjFusn_EgoMotionType, ObjFusn_VAR_NOINIT) s_sEgoMotionDefault = {
    FALSE, 0u, 0u, FLT_ZERO, FLT_ZERO, FLT_ZERO, FLT_ZERO};
#define ASW_QM_CORE5_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
//#define ObjFusn_STOP_SEC_VAR_UNSPECIFIED

/**
 * @brief init ego motion item with default values
 * @param pEgoMotionItem item to be initialized
 */
#define ObjFusn_START_SEC_SLOW_CODE

LOCAL void EgoMotion_itemInit(CONSTP2VAR(TueObjFusn_EgoMotionType,
                                         AUTOMATIC,
                                         ObjFusn_VAR_NOINIT) pEgoMotionItem) {
    pEgoMotionItem->bIsValid = TUEOBJFUSN_EGOMOTION_BISVALID_DEFAULT;
    pEgoMotionItem->f32Acc = TUEOBJFUSN_EGOMOTION_F32ACC_DEFAULT;
    pEgoMotionItem->f32Age = TUEOBJFUSN_EGOMOTION_F32AGE_DEFAULT;
    pEgoMotionItem->f32Speed = TUEOBJFUSN_EGOMOTION_F32SPEED_DEFAULT;
    pEgoMotionItem->f32YawRate = TUEOBJFUSN_EGOMOTION_F32YAWRATE_DEFAULT;
}
#define ObjFusn_STOP_SEC_SLOW_CODE

/**
 * @brief init ego motion history buffer with default values
 */
#define ObjFusn_START_SEC_SLOW_CODE

void EgoMotion_historyInit(void) {
    uint16 u16Item;

    /* set all parameters of s_sEgoMotionHistory to default */
    s_sEgoMotionHistory.u16NextWriteIdx =
        TUEOBJFUSN_EGOMOTIONHISTORY_U16NEXTWRITEIDX_DEFAULT;
    s_sEgoMotionHistory.u16NumOfItems =
        TUEOBJFUSN_EGOMOTIONHISTORY_U16NUMOFITEMS_DEFAULT;
    s_sEgoMotionHistory.f32Spacer =
        TUEOBJFUSN_EGOMOTIONHISTORY_F32SPACER_DEFAULT;

    /* set all history items to default */
    for (u16Item = 0u; u16Item < TUEOBJFUSN_EGOMOTIONHISTORY_SIZE; ++u16Item) {
        EgoMotion_itemInit(&(s_sEgoMotionHistory.aEgoMotionItem[u16Item]));
    }

    EgoMotion_itemInit(&s_sEgoMotionDefault);
}
#define ObjFusn_STOP_SEC_SLOW_CODE

/**
 * @brief Get index of element in ego motion buffer with given age
 *
 * Finds nearest item to f32Age which is older or as old as f32Age.
 * If no such element is present, return the index of the oldest element.
 *
 * @param[out]  pu16Idx     index of requested ego motion element in buffer
 * @param       f32Age      age of requested ego motion [s]
 * @return      TUEOBJFUSN_ERROR_NOERROR if no error occured and error code
 * otherwise.
 */
#define ObjFusn_START_SEC_CODE

LOCAL uint32 EgoMotion_getEgoMotionHistoryIdx(
    CONSTP2VAR(uint16, AUTOMATIC, ObjFusn_VAR_NOINIT) pu16Idx,
    const float32 f32Age) {
    uint16 u16ReadIdx;
    uint32 u32Success;
    boolean bFound = FALSE;
    uint16 u16Item;

#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if (NULL_PTR == pu16Idx) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION, TUEOBJFUSN_AAU_EGOMOTION,
            TUEOBJFUSN_AAU_EGO_MOTION_GET_MOTION_HISTORY_INDEX);
    } else
#endif
        // check valid range of s_sEgoMotionHistory.u16NumOfItems
        if (TUEOBJFUSN_EGOMOTIONHISTORY_SIZE >=
            s_sEgoMotionHistory.u16NumOfItems) {
        u16ReadIdx = (s_sEgoMotionHistory.u16NextWriteIdx +
                      (TUEOBJFUSN_EGOMOTIONHISTORY_SIZE - 1u)) %
                     TUEOBJFUSN_EGOMOTIONHISTORY_SIZE;

        /* find nearest item to f32Age which is larger (older) or equal f32Age
         */
        if (0u < s_sEgoMotionHistory.u16NumOfItems) {
            /* start search with newest and proceed to older elements */
            for (u16Item = 0u;
                 (u16Item < (s_sEgoMotionHistory.u16NumOfItems - 1u)) &&
                 (FALSE == bFound);
                 ++u16Item) {
                if (s_sEgoMotionHistory.aEgoMotionItem[u16ReadIdx].f32Age >=
                    f32Age) {
                    bFound = TRUE;
                } else {
                    /* proceed to previous (older) element in ringbuffer,
                     * considering wrap around */
                    u16ReadIdx =
                        (u16ReadIdx + (TUEOBJFUSN_EGOMOTIONHISTORY_SIZE - 1u)) %
                        TUEOBJFUSN_EGOMOTIONHISTORY_SIZE;
                }
            }

            u32Success = TUEOBJFUSN_ERROR_NOERROR;
        } else {
#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
            /* else: no item stored in history, return error */
            u32Success = TUEOBJFUSN_ERROR_EGO_MOTION_BUFFER_EMTPY;
            (void)tue_prv_error_management_addError(
                TUEOBJFUSN_ERROR_EGO_MOTION_BUFFER_EMTPY,
                TUEOBJFUSN_AAU_EGOMOTION,
                TUEOBJFUSN_AAU_EGO_MOTION_GET_MOTION_HISTORY_INDEX);
#else
            u32Success = TUEOBJFUSN_ERROR_NOERROR;
#endif
        }

        *pu16Idx = u16ReadIdx;
    } else {
        /* EgoMotion buffer not initialized. Should never happen. Try to recover
         * by initializing it. */
        EgoMotion_historyInit();
#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
        u32Success = TUEOBJFUSN_ERROR_AAU_NOT_INITIALIZED;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_AAU_NOT_INITIALIZED, TUEOBJFUSN_AAU_EGOMOTION,
            TUEOBJFUSN_AAU_EGO_MOTION_GET_MOTION_HISTORY_INDEX);
#else
        u32Success = TUEOBJFUSN_ERROR_NOERROR;
#endif
    }

    return u32Success;
}

/**
 * @brief Get element from ego motion buffer with given age
 *
 * Finds nearest item to f32Age which is older or as old as f32Age.
 * If no such element is present, return the oldest element.
 *
 * @param[out]  pEgoMotion  requested ego motion
 * @param       f32Age      age of requested ego motion [s]
 * @return      TUEOBJFUSN_ERROR_NOERROR if no error occured and error code
 * otherwise.
 */
/* PRQA S 1532 2 */ /* Library Function */
uint32 EgoMotion_getEgoMotionHistory(CONSTP2VAR(TueObjFusn_EgoMotionType *,
                                                AUTOMATIC,
                                                ObjFusn_VAR_NOINIT) pEgoMotion,
                                     const float32 f32Age) {
    uint32 u32Success;
    uint16 u16Idx;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    /* check input validity */
    if (NULL_PTR == pEgoMotion) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION, TUEOBJFUSN_AAU_EGOMOTION,
            TUEOBJFUSN_AAU_EGO_MOTION_GET_EGO_MOTION_HISTORY);

        /* error occured -> provide default ego motion */
        *pEgoMotion = &s_sEgoMotionDefault;
    } else
#endif
#endif
    {
        u32Success = EgoMotion_getEgoMotionHistoryIdx(&u16Idx, f32Age);
#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
        if (TUEOBJFUSN_ERROR_NOERROR == u32Success)
#endif
        {
            *pEgoMotion = &s_sEgoMotionHistory.aEgoMotionItem[u16Idx];
        }
#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
        else {
            /* error occured -> provide default ego motion */
            *pEgoMotion = &s_sEgoMotionDefault;
        }
#endif
    }

    return u32Success;
}

/**
 * @brief Add element to ego motion buffer and increase buffer age
 *
 * @param       pNewEgoMotion  new ego motion element
 * @param[out]  f32Dt          age to be added to existing elements in buffer
 * [s]
 * @return      TUEOBJFUSN_ERROR_NOERROR if no error occured and error code
 * otherwise.
 */

/* PRQA S 1532 2 */ /* Interface function called from Fusion AAU */
uint32 EgoMotion_addItemToHistory(CONSTP2CONST(TueObjFusn_EgoMotionType,
                                               AUTOMATIC,
                                               ObjFusn_VAR_NOINIT)
                                      pNewEgoMotion,
                                  const float32 f32Dt) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    uint16 u16Item;
    uint16 u16ReadIdx;
    uint16 u16SetIdx;
    boolean bFound = FALSE;
    VAR(TueObjFusn_EgoMotionHistory, ObjFusn_VAR_NOINIT) EgoHistBuf;

/* check for all input values and for s_sEgoMotionHistory.u16NumOfItems of
 * validity; otherwise send error code */
#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if (NULL_PTR == pNewEgoMotion) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION, TUEOBJFUSN_AAU_EGOMOTION,
            TUEOBJFUSN_AAU_EGO_MOTION_ADD_ITEM_TO_HISTORY);
    } else
#endif
        if (FLT_ZERO > f32Dt) {
        u32Success = TUEOBJFUSN_ERROR_INVALID_INPUT;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_INVALID_INPUT, TUEOBJFUSN_AAU_EGOMOTION,
            TUEOBJFUSN_AAU_EGO_MOTION_ADD_ITEM_TO_HISTORY);
    } else if (TUEOBJFUSN_EGOMOTIONHISTORY_SIZE <
               s_sEgoMotionHistory.u16NumOfItems) {
        u32Success = TUEOBJFUSN_ERROR_AAU_NOT_INITIALIZED;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_AAU_NOT_INITIALIZED, TUEOBJFUSN_AAU_EGOMOTION,
            TUEOBJFUSN_AAU_EGO_MOTION_ADD_ITEM_TO_HISTORY);
    } else
#endif
    {
        /* increase bufferAge for all existing items */
        for (u16Item = 0u; u16Item < s_sEgoMotionHistory.u16NumOfItems;
             ++u16Item) {
            s_sEgoMotionHistory.aEgoMotionItem[u16Item].f32Age += f32Dt;
        }

        /* insert new element */
        /* if it fits to ringbuffer (else it is older that oldest element in
         * buffer and shall be ignored) */
        if ((TRUE == pNewEgoMotion->bIsValid) &&
            ((TUEOBJFUSN_EGOMOTIONHISTORY_SIZE !=
              s_sEgoMotionHistory.u16NumOfItems) ||
             (pNewEgoMotion->f32Age <=
              s_sEgoMotionHistory
                  .aEgoMotionItem[s_sEgoMotionHistory.u16NextWriteIdx]
                  .f32Age))) {
            /* determine read index: one element before write index:,
             * considering wrap-around */
            u16ReadIdx = ((s_sEgoMotionHistory.u16NextWriteIdx +
                           (TUEOBJFUSN_EGOMOTIONHISTORY_SIZE - 1u)) %
                          TUEOBJFUSN_EGOMOTIONHISTORY_SIZE);

            /* if buffer empty or new element is younger than youngest element
             * in buffer */
            if ((0u == s_sEgoMotionHistory.u16NumOfItems) ||
                (pNewEgoMotion->f32Age <=
                 s_sEgoMotionHistory.aEgoMotionItem[u16ReadIdx].f32Age)) {
                /* then insert at currrent write index */
                u32Success |= EgoMotion_copyEgoMotionItem(
                    &s_sEgoMotionHistory
                         .aEgoMotionItem[s_sEgoMotionHistory.u16NextWriteIdx],
                    pNewEgoMotion);
            } else {
                /* search for appropriate index */
                /* TODO: reduce nesting */
                u16SetIdx = s_sEgoMotionHistory.u16NextWriteIdx;
                u32Success |= EgoMotion_copyEgoMotionHistory(
                    &EgoHistBuf, &s_sEgoMotionHistory);

                /* search loop */
                for (u16Item = 0u;
                     (u16Item < s_sEgoMotionHistory.u16NumOfItems) &&
                     (FALSE == bFound);
                     ++u16Item) {
                    /* shift items further */
                    u32Success |= EgoMotion_copyEgoMotionItem(
                        &s_sEgoMotionHistory.aEgoMotionItem[u16SetIdx],
                        &EgoHistBuf.aEgoMotionItem[u16ReadIdx]);

                    /* proceed to previous (older) element, considering wrap
                     * around */
                    u16SetIdx = ((u16SetIdx +
                                  (TUEOBJFUSN_EGOMOTIONHISTORY_SIZE - 1u))) %
                                TUEOBJFUSN_EGOMOTIONHISTORY_SIZE;
                    u16ReadIdx = ((u16SetIdx +
                                   (TUEOBJFUSN_EGOMOTIONHISTORY_SIZE - 1u))) %
                                 TUEOBJFUSN_EGOMOTIONHISTORY_SIZE;

                    /* if appropriate index found, or if last buffer element has
                     * been reached */
                    if ((pNewEgoMotion->f32Age <=
                         EgoHistBuf.aEgoMotionItem[u16ReadIdx].f32Age) ||
                        (u16Item == (s_sEgoMotionHistory.u16NumOfItems - 1u))) {
                        u32Success |= EgoMotion_copyEgoMotionItem(
                            &s_sEgoMotionHistory.aEgoMotionItem[u16SetIdx],
                            pNewEgoMotion);
                        bFound = TRUE; /* exit loop (equiv. to break) */
                    } else {
                        /* MISRA */
                    }
                }
            }

            /* increase u16NextWriteIdx considering wrap around */
            s_sEgoMotionHistory.u16NextWriteIdx =
                ((s_sEgoMotionHistory.u16NextWriteIdx + 1u) %
                 TUEOBJFUSN_EGOMOTIONHISTORY_SIZE);

            /* increase u16NumItems if necessary, ie until max is reached */
            s_sEgoMotionHistory.u16NumOfItems =
                tue_prv_fusion_min_U16(s_sEgoMotionHistory.u16NumOfItems + 1u,
                                       TUEOBJFUSN_EGOMOTIONHISTORY_SIZE);
        } else {
            /* MISRA */
        }
    }

    return u32Success;
}

/**
 * @brief return the path driven by the ego vehicle in given time interval
 *
 * The output is relative to the position of the ego vehicle position and
 * direction at the start of the path.
 *
 * @param[out]  pf32Dx       Distance driven in X direction [m]
 * @param[out]  pf32Dy       Distance driven in Y direction [m]
 * @param[out]  pf32Dphi     Change of heading [rad]
 * @param       f32AgeStart  Start age of path [s]
 * @param       f32AgeEnd    End age of path, must be smaller (younger) than or
 * equal to start age [s]
 * @return      TUEOBJFUSN_ERROR_NOERROR if no error occured and error code
 * otherwise.
 */
/* PRQA S 1532 2 */ /* Interface function called from Fusion AAU */
uint32 EgoMotion_getDeltaPath(
    CONSTP2VAR(float32, AUTOMATIC, ObjFusn_VAR_NOINIT) pf32Dx,
    CONSTP2VAR(float32, AUTOMATIC, ObjFusn_VAR_NOINIT) pf32Dy,
    CONSTP2VAR(float32, AUTOMATIC, ObjFusn_VAR_NOINIT) pf32Dphi,
    CONSTP2VAR(float32, AUTOMATIC, ObjFusn_VAR_NOINIT) pf32Sin,
    CONSTP2VAR(float32, AUTOMATIC, ObjFusn_VAR_NOINIT) pf32Cos,
    float32 f32AgeStart,
    const float32 f32AgeEnd) {
    uint32 u32Success;
    uint16 u16Idx;
    uint16 u16NextIdx;
    uint16 u16LastIdx;
    uint16 u16Step;
    uint16 u16NumOfSteps;
    float32 f32AgeTmp;
    float32 f32Dt;
    float32 f32EgoSpeed;
    float32 f32EgoYawRate;
    float32 f32Dist;
    float32 f32DDphi;
    float32 f32Radius;
    float32 f32T;
    float32 f32Dx = FLT_ZERO;
    float32 f32Dy = FLT_ZERO;
    float32 f32Dphi = FLT_ZERO;
    float32 f32SinSum = FLT_ZERO;
    float32 f32CosSum = FLT_ONE;
    float32 f32SinTmp;
    float32 f32CosTmp;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if ((NULL_PTR == pf32Dx) || (NULL_PTR == pf32Dy) ||
        (NULL_PTR == pf32Dphi)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION, TUEOBJFUSN_AAU_EGOMOTION,
            TUEOBJFUSN_AAU_EGO_MOTION_GET_DELTA_PATH);
    } else
#endif
        if ((FLT_ZERO > f32AgeStart) || (FLT_ZERO > f32AgeEnd) ||
            (f32AgeEnd > f32AgeStart)) {
        u32Success = TUEOBJFUSN_ERROR_INVALID_INPUT;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_INVALID_INPUT, TUEOBJFUSN_AAU_EGOMOTION,
            TUEOBJFUSN_AAU_EGO_MOTION_GET_DELTA_PATH);
    } else
#endif
    {
        u32Success = EgoMotion_getEgoMotionHistoryIdx(
            &u16Idx, f32AgeStart); /* Start  index */
        u32Success |= EgoMotion_getEgoMotionHistoryIdx(
            &u16LastIdx, f32AgeEnd); /* End  index */

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
        if (TUEOBJFUSN_ERROR_NOERROR == u32Success)
#endif
        {
            /* calculate number of steps according to indices */
            if (u16Idx > u16LastIdx) {
                /* consider ring buffer wrap around */
                u16NumOfSteps = (TUEOBJFUSN_EGOMOTIONHISTORY_SIZE - u16Idx) +
                                (u16LastIdx + 1u);
            } else {
                u16NumOfSteps = (u16LastIdx - u16Idx) + 1u;
            }

            u16NextIdx = ((u16Idx + 1u) % TUEOBJFUSN_EGOMOTIONHISTORY_SIZE);

            f32AgeTmp = s_sEgoMotionHistory.aEgoMotionItem[u16NextIdx].f32Age;

            if ((s_sEgoMotionHistory.u16NextWriteIdx == u16NextIdx) ||
                (f32AgeTmp < f32AgeEnd)) {
                f32AgeTmp = f32AgeEnd;
            } else {
                /* MISRA */
            }

            /* integration loop */
            for (u16Step = 0u; u16Step < u16NumOfSteps; ++u16Step) {
                f32Dt = f32AgeStart - f32AgeTmp;
                f32EgoSpeed =
                    s_sEgoMotionHistory.aEgoMotionItem[u16Idx].f32Speed;
                f32EgoYawRate =
                    s_sEgoMotionHistory.aEgoMotionItem[u16Idx].f32YawRate;

                /* integrate velocities if f32Age1 is not exactly at a certain
                 * item (is only at first step possible) */
                if (0u == u16Step) {
                    f32T = s_sEgoMotionHistory.aEgoMotionItem[u16Idx].f32Age -
                           f32AgeStart;
                    f32EgoSpeed +=
                        s_sEgoMotionHistory.aEgoMotionItem[u16Idx].f32Acc *
                        f32T;
                } else {
                    /* MISRA */
                }

                /* integrate path */
                /* if driving straight */
                if (TUE_PRV_FUSION_POS_EPS_ABS >
                    tue_prv_fusion_abs(
                        s_sEgoMotionHistory.aEgoMotionItem[u16Idx]
                            .f32YawRate)) /* PRQA S 3416 */ /* simple fcn */
                {
                    /* f32Dist = Dt * v + 0.5 * Dt^2 * a */
                    f32Dist =
                        f32Dt *
                        (f32EgoSpeed +
                         (f32Dt * FLT_ONE_HALF *
                          s_sEgoMotionHistory.aEgoMotionItem[u16Idx].f32Acc));
                    f32Dx += (f32Dist * f32CosSum);
                    f32Dy += (f32Dist * f32SinSum);
                } else {
                    /* else circular path */
                    /* TODO: consider acceleration, once f32YawAcc is
                     * available/nonzero */
                    f32DDphi = f32EgoYawRate * f32Dt;
                    f32Radius = f32EgoSpeed / f32EgoYawRate;

                    f32SinTmp = f32SinSum;
                    f32CosTmp = f32CosSum;
                    f32SinSum = tue_prv_fusion_sin(f32Dphi + f32DDphi);
                    f32CosSum = tue_prv_fusion_cos(f32Dphi + f32DDphi);

                    f32Dx += (f32Radius * (f32SinSum - f32SinTmp));
                    f32Dy += (f32Radius * (f32CosTmp - f32CosSum));
                    f32Dphi = f32Dphi + f32DDphi;
                }

                /* next step */
                u16Idx = u16NextIdx;
                /* Increment index with wrap-around */
                u16NextIdx = ((u16Idx + 1u) % TUEOBJFUSN_EGOMOTIONHISTORY_SIZE);

                f32AgeStart = f32AgeTmp;
                f32AgeTmp =
                    s_sEgoMotionHistory.aEgoMotionItem[u16NextIdx].f32Age;

                /* if next step is last step, the last integration shall go to
                 * f32AgeEnd */
                if ((u16NumOfSteps - TUE_EGOMOTION_OFFSET_SECOND_TO_LAST) ==
                    u16Step) {
                    f32AgeTmp = f32AgeEnd;
                } else {
                    /* MISRA */
                }
            }

            *pf32Dx = f32Dx;
            *pf32Dy = f32Dy;
            *pf32Dphi = f32Dphi;
            *pf32Sin = f32SinSum;
            *pf32Cos = f32CosSum;
        }
#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
        else {
            /* MISRA */
        }
#endif
    }

    return u32Success;
}

LOCAL uint32 EgoMotion_copyEgoMotionItem(
    CONSTP2VAR(TueObjFusn_EgoMotionType, AUTOMATIC, ObjFusn_VAR_NOINIT) pDest,
    CONSTP2CONST(TueObjFusn_EgoMotionType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pSrc) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    /* PRQA S 3204 1 */ /* variable may be set to other values depending on
                           run-time pointer check activation */
    const uint32 u32Size = (uint32)sizeof(TueObjFusn_EgoMotionType);

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if ((NULL_PTR == pDest) || (NULL_PTR == pSrc)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION, TUEOBJFUSN_AAU_EGOMOTION,
            TUEOBJFUSN_AAU_EGO_MOTION_COPY_EGO_MOTION_ITEM);
    } else
#endif
#endif
    {
        /* PRQA S 0314 1 */ /* Cast to void required for copy function */
        tue_prv_fusion_memcpy((void *)pDest, (const void *)pSrc, u32Size);
    }

    return u32Success;
}

LOCAL uint32 EgoMotion_copyEgoMotionHistory(
    CONSTP2VAR(TueObjFusn_EgoMotionHistory, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pDest,
    CONSTP2CONST(TueObjFusn_EgoMotionHistory, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pSrc) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    /* PRQA S 3204 1 */ /* variable may be set to other values depending on
                           run-time pointer check activation */
    const uint32 u32Size = (uint32)sizeof(TueObjFusn_EgoMotionHistory);

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if ((NULL_PTR == pDest) || (NULL_PTR == pSrc)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION, TUEOBJFUSN_AAU_EGOMOTION,
            TUEOBJFUSN_AAU_EGO_MOTION_COPY_EGO_MOTION_HISTORY);
    } else
#endif
#endif
    {
        /* PRQA S 0314 1 */ /* Cast to void required for copy function */
        tue_prv_fusion_memcpy((void *)pDest, (const void *)pSrc, u32Size);
    }

    return u32Success;
}
#define ObjFusn_STOP_SEC_CODE

#ifdef UNITTEST
/* only needed for unit tests to get the static values */
TueObjFusn_EgoMotionHistory *EgoMotion_accessEgoMotionHistory(
    void) /* PRQA S 3219 */ /* used for tests only */
{
    return &s_sEgoMotionHistory;
}
#endif /* UNITTEST */

/**
 * \}
 */
