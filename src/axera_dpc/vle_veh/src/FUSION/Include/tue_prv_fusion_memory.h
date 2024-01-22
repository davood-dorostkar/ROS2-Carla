/** \addtogroup tueFusion
 *  \{
 * \file tue_prv_fusion_memory.h
 *
 * \brief Defines common memory operations to be implemented in platform
 *        specific implementation
 *
 *
 *
 *
 *
 *   (C) Copyright Tuerme Inc. All rights reserved.
 *
 */

#ifndef TUE_PRV_FUSION_MEMORY
#define TUE_PRV_FUSION_MEMORY

#ifdef __cplusplus
extern "C" {
#endif

#include "tue_prv_common_types.h"

/**
 * Defines and functions from math.h
 **/
#define ObjFusn_START_SEC_CODE

void tue_prv_fusion_memcpy(CONSTP2VAR(void, AUTOMATIC, ObjFusn_VAR_NOINIT)
                               pDest,
                           CONSTP2CONST(void, AUTOMATIC, ObjFusn_VAR_NOINIT)
                               pSrc,
                           const uint32 u32Size);
#define ObjFusn_STOP_SEC_CODE

#ifdef __cplusplus
}
#endif

#endif /**\} TUE_PRV_FUSION_MEMORY */
