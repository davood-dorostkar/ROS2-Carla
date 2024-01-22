/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * zhang guanglin <zhang guanglin@senseauto.com>
 */
/** \addtogroup tue_prv_error_management
 * \{
 * \file tue_prv_error_management.c
 *
 * \brief Implementation file for error management AAU.
 *
 */
/* PRQA S 0292 ++ */ /* MKS */
                     /*
                      */
/* PRQA S 0292 -- */ /* MKS */
                     /*   (C) Copyright Tuerme Inc. All rights reserved.
                      *
                      */

/*==================[inclusions]============================================*/
#include "tue_prv_common_types.h"
#include "TueObjFusn_ConfigVehicle.h"
#include "TueObjFusn_ConfigAlgorithm.h"
#include "TueObjFusn_ErrorCodes.h"
#include "tue_prv_error_management.h"
#include "tue_prv_error_management_int.h"
#include "tue_prv_fusion_math.h"
#include "tue_prv_fusion_memory.h"

#ifdef TUE_PRV_DEBUG_OUTPUT
#include "support/debug/debug.h"
#endif

/*==================[macros]================================================*/
/*==================[type definitions]======================================*/

/*==================[static variables]====================================*/

//#define ObjFusn_START_SEC_VAR8
#define ASW_QM_CORE5_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/* PRQA S 3218 2 */
/* PRQA S 3229 1 */ // variable may be accessed depending on run-time pointer
                    //    check activation
LOCAL VAR(uint8, ObjFusn_VAR_ZERO_INIT)
    s_u8ErrorManagementState = TUE_PRV_ERROR_MANAGEMENT_STATE_NOT_INITIALIZED;
//#define ObjFusn_STOP_SEC_VAR8

//#define ObjFusn_START_SEC_VAR_UNSPECIFIED
LOCAL VAR(TueObjFusn_ErrorBufferType,
          ObjFusn_VAR_ZERO_INIT) stErrorManagementBuffer = {{{0}}, 0u, 0u};
//#define ObjFusn_STOP_SEC_VAR_UNSPECIFIED
#define ASW_QM_CORE5_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/*==================[forward declarations]==================================*/

/*==================[symbolic constants]====================================*/
/*==================[functions]============================================*/

#define ObjFusn_START_SEC_SLOW_CODE

/* PRQA S 1532 2 */ /* Function called by external AAU */
void tue_prv_error_management_init(void) {
    uint16 u16i;
    P2VAR(TueObjFusn_ErrorType, AUTOMATIC, ObjFusn_VAR_NOINIT) pError;

    stErrorManagementBuffer.u16NextWriteIdx = 0u;
    stErrorManagementBuffer.u16NumOfItems = 0u;

    for (u16i = 0u; u16i < TUE_PRV_ERROR_MANAGEMENT_BUFFER_SIZE; u16i++) {
        pError = &stErrorManagementBuffer.aErrorBuffer[u16i];
        (void)tue_prv_error_management_initErrorItem(pError);
    }

    s_u8ErrorManagementState = TUE_PRV_ERROR_MANAGEMENT_STATE_INITIALIZED;
}

LOCAL uint32 tue_prv_error_management_initErrorItem(CONSTP2VAR(
    TueObjFusn_ErrorType, AUTOMATIC, ObjFusn_VAR_NOINIT) pErrorItem) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    /* PRQA S 3204 1 */ // variable may be set to other values depending on
                        //    run-time pointer check activation

#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if (NULL_PTR == pErrorItem) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
    } else
#endif
    {
        pErrorItem->u16Age = TUEOBJFUSN_ERRORBUFFER_U16AGE_DEFAULT;
        pErrorItem->u32ErrorCode = TUEOBJFUSN_ERRORBUFFER_U32ERRORCODE_DEFAULT;
        pErrorItem->u8AAU_Code = TUEOBJFUSN_ERRORBUFFER_U8AAUCODE_DEFAULT;
        pErrorItem->u8FunctionCode =
            TUEOBJFUSN_ERRORBUFFER_U8FUNCTIONCODE_DEFAULT;
    }

    return u32Success;
}

/* PRQA S 1532 2 */ /* Function called by external AAU */
uint32 tue_prv_error_management_incrementAge(void) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    /* PRQA S 3204 1 */ // variable may be set to other values depending on
                        //    run-time check activation
    uint16 u16i;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
    if (TUE_PRV_ERROR_MANAGEMENT_STATE_NOT_INITIALIZED ==
        s_u8ErrorManagementState) {
        u32Success = TUEOBJFUSN_ERROR_AAU_NOT_INITIALIZED;
    } else if (stErrorManagementBuffer.u16NumOfItems >
               TUE_PRV_ERROR_MANAGEMENT_BUFFER_SIZE) {
        u32Success = TUEOBJFUSN_ERROR_NUMBER_OF_OBJECTS_EXCEEDS_MAXIMUM_SIZE;
    } else
#endif
    {
        for (u16i = 0u; u16i < TUE_PRV_ERROR_MANAGEMENT_BUFFER_SIZE; u16i++) {
            if (stErrorManagementBuffer.aErrorBuffer[u16i].u16Age <
                TUE_PRV_ERROR_MANAGEMENT_MAX_ERROR_AGE) {
                stErrorManagementBuffer.aErrorBuffer[u16i].u16Age++;
            } else {
                /* MISRA */
            }
        }
    }

    return u32Success;
}

/* PRQA S 1532 2 */ /* Function called by external AAU */
uint32 tue_prv_error_management_clearBuffer(void) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    uint16 u16i;
    uint16 u16NumElements = 0u;
    P2VAR(TueObjFusn_ErrorType, AUTOMATIC, ObjFusn_VAR_NOINIT) pCurrItem;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
    if (TUE_PRV_ERROR_MANAGEMENT_STATE_NOT_INITIALIZED ==
        s_u8ErrorManagementState) {
        u32Success = TUEOBJFUSN_ERROR_AAU_NOT_INITIALIZED;
    } else if (stErrorManagementBuffer.u16NumOfItems >
               TUE_PRV_ERROR_MANAGEMENT_BUFFER_SIZE) {
        u32Success = TUEOBJFUSN_ERROR_NUMBER_OF_OBJECTS_EXCEEDS_MAXIMUM_SIZE;
    } else
#endif
    {
        for (u16i = 0u; u16i < TUE_PRV_ERROR_MANAGEMENT_BUFFER_SIZE; u16i++) {
            pCurrItem = &((stErrorManagementBuffer.aErrorBuffer)[u16i]);

            if ((pCurrItem->u16Age) < TUE_PRV_ERROR_MANAGEMENT_MAX_ERROR_AGE) {
                u16NumElements++;
            } else if ((pCurrItem->u16Age) ==
                       TUE_PRV_ERROR_MANAGEMENT_MAX_ERROR_AGE) {
                u32Success |= tue_prv_error_management_initErrorItem(pCurrItem);
            } else {
                /* MISRA */
            }
        }

        if (TUE_PRV_ERROR_MANAGEMENT_BUFFER_SIZE_EMPTY == u16NumElements) {
            stErrorManagementBuffer.u16NextWriteIdx = 0u;
        } else {
            /* MISRA */
        }

        stErrorManagementBuffer.u16NumOfItems = u16NumElements;
    }

    return u32Success;
}

#if TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON
uint32 tue_prv_error_management_addError(const uint32 u32Error,
                                         const uint8 u8AAU,
                                         const uint8 u8Function) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    const uint32 u16WriteIndex = stErrorManagementBuffer.u16NextWriteIdx;
    P2VAR(TueObjFusn_ErrorType, AUTOMATIC, ObjFusn_VAR_NOINIT) pCurrError;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
    const boolean bErrorInBuffer =
        tue_prv_error_management_containsError(u32Error, u8AAU, u8Function);

    if ((TUEOBJFUSN_ERROR_NOERROR == u32Error) || (u8AAU == 0x00u) ||
        (u8Function == 0x00u)) {
        u32Success = TUEOBJFUSN_ERROR_INVALID_INPUT;
    } else if ((stErrorManagementBuffer.u16NumOfItems) >
               TUE_PRV_ERROR_MANAGEMENT_BUFFER_SIZE) {
        u32Success = TUEOBJFUSN_ERROR_NUMBER_OF_OBJECTS_EXCEEDS_MAXIMUM_SIZE;
    } else if (TRUE == bErrorInBuffer) {
        /* Do not store duplicate errors */
    } else
#endif
    {
#ifdef TUE_PRV_DEBUG_OUTPUT
        const uint16 u16AauCode = ((u8AAU << 8u) | u8Function);

        DEBUG_LOG("ObjFus AAU/Function: 0x%x ErrCode: 0x%x" NEWLINE, u16AauCode,
                  u32Error);
#endif
        pCurrError = &((stErrorManagementBuffer.aErrorBuffer)[u16WriteIndex]);

        /* Write index valid, insert new error */
        pCurrError->u16Age = TUE_PRV_ERROR_MANAGEMENT_U16AGE_NEW;
        pCurrError->u32ErrorCode = u32Error;
        pCurrError->u8AAU_Code = u8AAU;
        pCurrError->u8FunctionCode = u8Function;

        stErrorManagementBuffer.u16NumOfItems =
            tue_prv_fusion_min_U16(TUE_PRV_ERROR_MANAGEMENT_BUFFER_SIZE,
                                   stErrorManagementBuffer.u16NumOfItems + 1u);
        u32Success |= tue_prv_error_management_updateWriteIndex();
    }

    return u32Success;
}
#endif

#if TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON
LOCAL uint32 tue_prv_error_management_updateWriteIndex(void) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    const uint16 u16WriteIndex = stErrorManagementBuffer.u16NextWriteIdx;

    if (stErrorManagementBuffer.u16NumOfItems >
        TUE_PRV_ERROR_MANAGEMENT_BUFFER_SIZE) {
        u32Success = TUEOBJFUSN_ERROR_NUMBER_OF_OBJECTS_EXCEEDS_MAXIMUM_SIZE;
    } else if (stErrorManagementBuffer.u16NumOfItems ==
               TUE_PRV_ERROR_MANAGEMENT_BUFFER_SIZE_EMPTY) {
        stErrorManagementBuffer.u16NextWriteIdx = 0u;
    } else {
        stErrorManagementBuffer.u16NextWriteIdx =
            (u16WriteIndex + 1u) % TUE_PRV_ERROR_MANAGEMENT_BUFFER_SIZE;
    }

    return u32Success;
}
#endif

/* PRQA S 1532 2 */ /* Function called by external AAU */
uint32 tue_prv_error_management_copyBuffer(CONSTP2VAR(
    TueObjFusn_ErrorBufferType, AUTOMATIC, ObjFusn_VAR_NOINIT) pBuffer) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    /* PRQA S 3204 1 */ // variable may be set to other values depending on
                        //    run-time check activation
    const uint32 u32Size = (uint32)sizeof(TueObjFusn_ErrorBufferType);

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if (NULL_PTR == pBuffer) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
    } else
#endif
        if (stErrorManagementBuffer.u16NumOfItems >
            TUEOBJFUSN_ERRORBUFFER_U16NUMOFITEMS_MAX) {
        u32Success = TUEOBJFUSN_ERROR_NUMBER_OF_OBJECTS_EXCEEDS_MAXIMUM_SIZE;
    } else if (stErrorManagementBuffer.u16NextWriteIdx >
               TUEOBJFUSN_ERRORBUFFER_U16NEXTWRITEIDX_MAX) {
        u32Success = TUEOBJFUSN_ERROR_INTERNAL_ERROR;
    } else
#endif
    {
        /* PRQA S 0314 1 */ /* Cast to void required for copy function */
        tue_prv_fusion_memcpy((void *)pBuffer, (void *)&stErrorManagementBuffer,
                              u32Size);
    }

    return u32Success;
}

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
LOCAL boolean tue_prv_error_management_containsError(const uint32 u32Error,
                                                     const uint8 u8AAU,
                                                     const uint8 u8Function) {
    boolean bFound = FALSE;
    uint16 u16i;
    P2VAR(TueObjFusn_ErrorType, AUTOMATIC, ObjFusn_VAR_NOINIT) pCurrError;

    for (u16i = 0u; u16i < TUE_PRV_ERROR_MANAGEMENT_BUFFER_SIZE; u16i++) {
        if (FALSE == bFound) {
            pCurrError = &stErrorManagementBuffer.aErrorBuffer[u16i];
            if ((pCurrError->u32ErrorCode == u32Error) &&
                (pCurrError->u8AAU_Code == u8AAU) &&
                (pCurrError->u8FunctionCode == u8Function) &&
                (pCurrError->u16Age == TUE_PRV_ERROR_MANAGEMENT_U16AGE_NEW)) {
                bFound = TRUE;
            } else {
                /* MISRA */
            }
        } else {
            /* MISRA */
        }
    }

    return bFound;
}
#endif

#define ObjFusn_STOP_SEC_SLOW_CODE

#ifdef UNITTEST
TueObjFusn_ErrorBufferType *const tue_prv_error_management_getErrorBuffer(
    void) {
    return &stErrorManagementBuffer;
}
#endif

/**
 * \}
 */
