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
// #include "envm_ext.h"
// #include "envm_consts.h"
#include "fusion_ext.h"
#include "tue_common_libs.h"
// #include "TM_Global_Types.h"

//#include "Fusion_Std_Types.h"
#include "Std_Types.h"
#include "FusionCompiler.h"
/*==================[macros]================================================*/
#define LONGWORD_BYTES sizeof(uint64)
#define LONGWORD_ALIGNED (uint64)0x7u
#define INTEGER_ALIGNED (uint64)0x3u
#define HALFWORD_ALIGNED (uint64)0x2u
#define BYTE_ALIGNED (uint64)0x1u

/*==================[type definitions]======================================*/
/*==================[static variables]======================================*/
/*==================[forward declarations]==================================*/
/*==================[symbolic constants]====================================*/
/*==================[functions]=============================================*/
#define ObjFusn_START_SEC_CODE

/*!
 * \brief       Copies byte wise n bytes from source to destination.
 *
 * \param[in]   size   Number of bytes to copy. This number is unsigned integral
 * type.
 * \param[out]  dst    Pointer to the destination array where the content is to
 * be copied.
 * \param[in]   src    Pointer to the source of data to be copied.
 *
 * \return      void
 */
LOCAL void intern_memcpy_byte(CONST(uint16, ObjFusn_VAR_NOINIT) size,
                              VAR(uint8, ObjFusn_VAR_NOINIT) dst[],
                              CONST(uint8, ObjFusn_VAR_NOINIT) src[]);

/*!
 * \brief       Copies double word wise n bytes from source to destination.
 *
 * \param[in]   size   Number of bytes to copy. This number is unsigned integral
 * type.
 * \param[out]  dst    Pointer to the destination array where the content is to
 * be copied.
 * \param[in]   src    Pointer to the source of data to be copied.
 *
 * \return      Pointer to the destination.
 *
 * \retval      unsigned long long *   Pointer to the destinations memory
 * address.
 */
LOCAL P2VAR(uint64, AUTOMATIC, ObjFusn_CODE)
    intern_memcpy_doubleword(CONST(uint16, ObjFusn_VAR_NOINIT) size,
                             VAR(uint64, ObjFusn_VAR_NOINIT) dst[],
                             CONST(uint64, ObjFusn_VAR_NOINIT) src[]);

/*!
 * \brief       Copies n bytes from source to destination.
 *
 * \details     Copies data from default memory to default memory.
 *             Does not check for overlapping spaces.
 *
 * \param[out]  dst    Pointer to the destination array where the content is to
 * be copied,
 *                   type-casted to a pointer of type void*.
 * \param[in]   src    Pointer to the source of data to be copied, type-casted
 * to a pointer
 *                   of type const void*.
 * \param[in]   size   Number of bytes to copy. This number is unsigned integral
 * type.
 *
 * \return      Pointer to the destination.
 *
 * \retval      void*   void casted pointer to the destinations memory address.
 *
 * Example usage:
 * \code
 *              #define SIZE 15
 *              unsigned char   source[SIZE]      = "Hello World!";
 *              unsigned char   destination[SIZE] = "";
 *              unsigned char * result;
 *              result = (unsigned char *)memcpy(destination, source, SIZE);
 *              // Copies the characters from source to destination and return
 * destination,
 *              // result will point to destination[0].
 * \endcode
 */
LOCAL P2VAR(uint8, AUTOMATIC, ObjFusn_CODE)
    intern_memcpy(VAR(uint8, ObjFusn_VAR_NOINIT) dst[],
                  CONST(uint8, ObjFusn_VAR_NOINIT) src[],
                  CONST(uint16, ObjFusn_VAR_NOINIT) n);
#define ObjFusn_STOP_SEC_CODE

/**
 * \}
 */
