/** \addtogroup tue_prv_error_management
 *  @{
 * \file tue_prv_error_management_int.h
 *
 * \brief Internal header for error management AAU.
 *
 *
 *
 *   (C) Copyright Tuerme Inc. All rights reserved.
 *
 */

#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_SOURCE_TUE_PRV_ERROR_MANAGEMENT_INT_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_SOURCE_TUE_PRV_ERROR_MANAGEMENT_INT_H_

#ifdef __cplusplus
extern "C" {
#endif

/*==================[inclusions]============================================*/
#include "tue_prv_common_types.h"
#include "TueObjFusn_ErrorBufferType.h"

/*==================[macros]================================================*/
/**
 * Determines if AAU is not initialized
 */
#define TUE_PRV_ERROR_MANAGEMENT_STATE_NOT_INITIALIZED (0x00u)

/**
 * Determines if AAU is initialized
 */
#define TUE_PRV_ERROR_MANAGEMENT_STATE_INITIALIZED (0x01u)

/**
 * New error is initialized with Age zero
 */
#define TUE_PRV_ERROR_MANAGEMENT_U16AGE_NEW (0u)

/**
 * Indicates empty error management buffer
 */
#define TUE_PRV_ERROR_MANAGEMENT_BUFFER_SIZE_EMPTY (0u)

/*==================[type definitions]======================================*/

/*==================[forward declarations]==================================*/
/*==================[symbolic constants]====================================*/
/*==================[return codes]==========================================*/
/*==================[functions]=============================================*/

/**
 * @brief  Inits an TueObjFusn_ErrorType item to default values
 *
 * @param    pErrorItem, TueObjFusn_ErrorType * const  item to be initialized
 * @return TUEOBJFUSN_ERROR_NOERROR or corresponding error code
 */
#define ObjFusn_START_SEC_CODE

LOCAL uint32 tue_prv_error_management_initErrorItem(
    CONSTP2VAR(TueObjFusn_ErrorType, AUTOMATIC, ObjFusn_VAR_NOINIT) pErrorItem);
#define ObjFusn_STOP_SEC_CODE

/**
 * @brief  Updates wite index to next free item.
 *
 * @return TUEOBJFUSN_ERROR_NOERROR or corresponding error code
 */
#if TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON
#define ObjFusn_START_SEC_CODE

LOCAL uint32 tue_prv_error_management_updateWriteIndex(void);
#define ObjFusn_STOP_SEC_CODE

#endif

/**
 * @brief  Checks if provided error with has already been saved in current cycle
 *
 * @return TRUE if error is found in buffer with Age zero and FALSE otherwise
 */
#define ObjFusn_START_SEC_CODE

LOCAL boolean tue_prv_error_management_containsError(const uint32 u32Error,
                                                     const uint8 u8AAU,
                                                     const uint8 u8Function);
#define ObjFusn_STOP_SEC_CODE

#ifdef UNITTEST
/**
 * @brief  Returns pointer to error buffer.
 *
 * @return Address of error buffer structure.
 */
TueObjFusn_ErrorBufferType* const tue_prv_error_management_getErrorBuffer(void);
#endif

/*==================[external function declarations]========================*/
/*==================[external constants]====================================*/
/*==================[external data]=========================================*/

#ifdef __cplusplus
}
#endif

#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_SOURCE_TUE_PRV_ERROR_MANAGEMENT_INT_H_
       /**@}==================[end of
        * file]===========================================*/
