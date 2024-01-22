/** \defgroup array_utils Array Utils
 * \brief utilities for manipulating arrays
 *
 * \addtogroup array_utils
 *  @{
 * \file        tue_prv_common_array_utils.h
 * \brief       utilities for manipulating arrays
 *
 *
 *
 * <br>=====================================================<br>
 * <b>Copyright 2014 by Tuerme.</b>
 *
 *  All rights reserved. Property of Tuerme.<br>
 *  Restricted rights to use, duplicate or disclose of this code<br>
 *  are granted through contract.
 * <br>=====================================================<br>
 */

#ifndef TUE_PRV_COMMON_ARRAY_UTILS_H
#define TUE_PRV_COMMON_ARRAY_UTILS_H

/* standard types (uint16, float32, etc.) */
#include "tue_prv_common_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*write def to all elements of arr */
#define ObjFusn_START_SEC_CODE

void tue_prv_common_array_utils_defaultInit_as8(VAR(sint8, ObjFusn_VAR_NOINIT)
                                                    as8_arr[],
                                                const uint16 u16_n,
                                                const sint8 s8_def);

void tue_prv_common_array_utils_defaultInit_au16(uint16 au16_arr[],
                                                 const uint16 u16_n,
                                                 const uint16 u16_def);

void tue_prv_common_array_utils_defaultInit_au32(uint32 au32_arr[],
                                                 const uint16 u16_n,
                                                 const uint32 u32_def);

void tue_prv_common_array_utils_defaultInit_af32(VAR(float32,
                                                     ObjFusn_VAR_NOINIT)
                                                     af32_arr[],
                                                 const uint16 u16_n,
                                                 const float32 f32_def);

void tue_prv_common_array_utils_defaultInit_abool(VAR(boolean,
                                                      ObjFusn_VAR_NOINIT)
                                                      ab_arr[],
                                                  const uint16 u16_n,
                                                  const boolean b_def);

/** find first occurrence of element in array and return its index (else return
 * ARRAY_UTILS_NOTFOUND_IDX)*/

/** find largest element from array and also return its index */
float32 tue_prv_common_array_utils_findLargestIndex_f32(
    const float32 af32_arr[],
    const uint16 u16_n,
    CONSTP2VAR(sint16, AUTOMATIC, ObjFusn_VAR_NOINIT) s16_index);

#define ObjFusn_STOP_SEC_CODE

#ifdef __cplusplus
}
#endif

#endif /**@} TUE_PRV_COMMON_ARRAY_UTILS_H */