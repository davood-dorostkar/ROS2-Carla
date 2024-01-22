/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * zhang guanglin <zhang guanglin@senseauto.com>
 */
/** \addtogroup tueFusion
 * \{
 * \file tue_prv_fusion_memory.c
 *
 * \brief Implementation file for memory operations
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
#include <string.h>
#include "tue_prv_fusion_memory.h"
#include "tue_prv_fusion_memory_int.h"

/*==================[macros]================================================*/
/*==================[type definitions]======================================*/
/*==================[static variables]======================================*/
/*==================[forward declarations]==================================*/
/*==================[symbolic constants]====================================*/
/*==================[functions]=============================================*/
#define ObjFusn_START_SEC_CODE

LOCAL void intern_memcpy_byte(CONST(uint16, ObjFusn_VAR_NOINIT) size,
                              VAR(uint8, ObjFusn_VAR_NOINIT) dst[],
                              CONST(uint8, ObjFusn_VAR_NOINIT) src[]) {
    uint16 i;

    for (i = 0u; i < size; i++) {
        dst[i] = src[i];
    }
}

LOCAL P2VAR(uint64, AUTOMATIC, ObjFusn_CODE)
    intern_memcpy_doubleword(CONST(uint16, ObjFusn_VAR_NOINIT) size,
                             VAR(uint64, ObjFusn_VAR_NOINIT) dst[],
                             CONST(uint64, ObjFusn_VAR_NOINIT) src[]) {
    uint16 i;

    for (i = 0u; i < size; i++) {
        dst[i] = src[i];
    }

    return &dst[i];
}

LOCAL P2VAR(uint8, AUTOMATIC, ObjFusn_CODE)
    intern_memcpy(VAR(uint8, ObjFusn_VAR_NOINIT) dst[],
                  CONST(uint8, ObjFusn_VAR_NOINIT) src[],
                  CONST(uint16, ObjFusn_VAR_NOINIT) n) {
    uint16 idx;
    uint16 bytes_left;
    uint16 word_count;

    if (0u < n) {
        idx = 0u;
        /* PRQA S 0306 1 */ /* Cast required here */
        if ((n >= (uint16)LONGWORD_BYTES) &&
            (((uint64)(&dst[idx]) & INTEGER_ALIGNED) ==
             ((uint64)(&src[idx]) & INTEGER_ALIGNED))) {
            /* Byte alignment */
            /* PRQA S 0306 1 */ /* Cast required here */
            if (0u != (((uint64)(&dst[idx]) & BYTE_ALIGNED))) {
                dst[idx] = src[idx];
                idx++;
            } else {
                /* MISRA */
            }

            /* Half word alignment */
            /* PRQA S 0306 1 */ /* Cast required here */
            if (0u != ((uint64)(&dst[idx]) & HALFWORD_ALIGNED)) {
                dst[idx] = src[idx];
                idx++;
                dst[idx] = src[idx];
                idx++;
            } else {
                /* MISRA */
            }

            /* Word alignment */
            /* PRQA S 4130 1 */ /* Elements in expression are unsigned */
            bytes_left = (uint16)((n - idx) & (uint16)LONGWORD_ALIGNED);
            word_count = (uint16)((n - idx) / (uint16)LONGWORD_BYTES);
            if (0u < word_count) {
                /* PRQA S 3305 1 */ // dst/src element at 'idx' already assured
                                    //    to be correctly aligned
                (void)intern_memcpy_doubleword(word_count,
                                               (uint64 *)(&dst[idx]),
                                               (const uint64 *)(&src[idx]));
                /* PRQA S 0310 1 */ /* Cast required here */
                idx = idx + (word_count * (uint16)LONGWORD_BYTES);
            } else {
                /* MISRA */
            }
        } else {
            bytes_left = n;
        }

        if (0u < bytes_left) {
            intern_memcpy_byte(bytes_left, &dst[idx], &src[idx]);
        } else {
            /* MISRA */
        }
    } else {
        /* MISRA */
    }

    return &dst[0];
}

void tue_prv_fusion_memcpy(CONSTP2VAR(void, AUTOMATIC, ObjFusn_VAR_NOINIT)
                               pDest,
                           CONSTP2CONST(void, AUTOMATIC, ObjFusn_VAR_NOINIT)
                               pSrc,
                           CONST(uint32, ObjFusn_VAR_NOINIT) u32Size) {
#ifdef _ROS_ACC_VERSION
    memcpy(pDest, pSrc, u32Size);
#else
    (void)intern_memcpy((uint8 *)pDest, (const uint8 *)pSrc, (uint16)u32Size);
///* PRQA S 0316 */ /* Cast to uint8 required here */
#endif
}
#define ObjFusn_STOP_SEC_CODE

/**
 * \}
 */
