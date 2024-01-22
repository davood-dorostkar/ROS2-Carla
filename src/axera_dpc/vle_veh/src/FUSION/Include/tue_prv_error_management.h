/** \addtogroup tue_prv_error_management
 *  @{
 * \file tue_prv_error_management.h
 *
 * \brief Header for error management AAU.
 *
 *
 *   (C) Copyright Tuerme Inc. All rights reserved.
 *
 */

#ifndef TUE_PRV_ERROR_MANAGEMENT_H
#define TUE_PRV_ERROR_MANAGEMENT_H

#ifdef __cplusplus
extern "C" {
#endif

/*==================[inclusions]============================================*/
#include "tue_prv_common_types.h"
#include "TueObjFusn_ErrorBufferType.h"

/*==================[macros]================================================*/
/*==================[type definitions]======================================*/

/*==================[forward declarations]==================================*/
/*==================[symbolic constants]====================================*/
/*==================[return codes]==========================================*/
/*==================[functions]=============================================*/

#define ObjFusn_START_SEC_SLOW_CODE

/**
 *  @brief Inits the error management AAU
 *
 *  @return void
 */
void tue_prv_error_management_init(void);

/**
 *  @brief increments age counter of each valid error item
 *
 *  This function should be called within pre-cycle routine to ensure the age of
 * each existing error is
 *  incremented.
 *
 *  @return TUEOBJFUSN_ERROR_NOERROR or corresponding error code
 */

uint32 tue_prv_error_management_incrementAge(void);

/**
 *  @brief  Removes all error items with u16Age > @ref
 * TUE_PRV_ERROR_MANAGEMENT_MAX_ERROR_AGE
 *
 *  This function should be called within the finalize cycle to remove all error
 * items that occured
 *  @ref TUE_PRV_ERROR_MANAGEMENT_MAX_ERROR_AGE cycles ago
 *
 *  @return TUEOBJFUSN_ERROR_NOERROR or corresponding error code
 */

uint32 tue_prv_error_management_clearBuffer(void);

/**
 *  @brief  Adds a new error to the error buffer
 *
 *  This function adds a new error to the buffer by setting the item pointed by
 * @ref u16NextWriteIdx.
 *  In case the buffer is full, the 'oldest' item(s) are removed by calling the
 * function @ref tue_prv_error_management_clearObsoleteErrors
 *
 *  @param   u32Error, const u32_t Error Code
 *  @param   u8AAU, const u8_t AAU Code
 *  @param   u8Function, const u8_t Function Code
 *
 *  @return  TUEOBJFUSN_ERROR_NOERROR or corresponding error code
 */

#if TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON

uint32 tue_prv_error_management_addError(const uint32 u32Error,
                                         const uint8 u8AAU,
                                         const uint8 u8Function);

#endif

/**
 *  @brief  Copy internal buffer to provided pointer
 *
 *  @param   pBuffer, TueObjFusn_ErrorBufferType * const  Pointer to output
 * structure.
 *
 *  @return  TUEOBJFUSN_ERROR_NOERROR or corresponding error code
 */

uint32 tue_prv_error_management_copyBuffer(CONSTP2VAR(
    TueObjFusn_ErrorBufferType, AUTOMATIC, ObjFusn_VAR_NOINIT) pBuffer);

#define ObjFusn_STOP_SEC_SLOW_CODE

/*==================[external function declarations]========================*/
/*==================[external constants]====================================*/
/*==================[external data]=========================================*/

#ifdef __cplusplus
}
#endif

#endif /* TUE_PRV_ERROR_MANAGEMENT_H */
       /**@}==================[end of
        * file]===========================================*/
