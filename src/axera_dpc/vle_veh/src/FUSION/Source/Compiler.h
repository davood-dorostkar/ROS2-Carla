/**********************************************************************************************************************
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: Configuration header file of ComStack
 * Author: VOS
 * Create: 2019-12-21
 *
 *********************************************************************************************************************/

#ifndef COMPILER_H
#define COMPILER_H
/**********************************************************************************************************************
 * Based on AUTOSAR_SWS_CompilerAbstraction.pdf; Document Version 4.4.0
 *********************************************************************************************************************/
// #include "Compiler_Cfg.h"
#define COMPILER_AR_RELEASE_MAJOR_VERSION    (4u)
#define COMPILER_AR_RELEASE_MINOR_VERSION    (4u)
#define COMPILER_AR_RELEASE_REVISION_VERSION (0u)
/*
 * compiler V1.0.0
 */
#define COMPILER_SW_MAJOR_VERSION (1u)
#define COMPILER_SW_MINOR_VERSION (0u)
#define COMPILER_SW_PATCH_VERSION (0u)

#if defined (__TASKING__)
#define _TASKING_C_TRICORE_ 1U

#elif defined(__DCC__)
#define _DIABDATA_C_TRICORE_ 1U

#elif defined (__ghs__)
#define _GHS_C_TRICORE_ 1U
#define _GREENHILLS_C_TRICORE_ 1U
#elif defined(__clang__ )
#define _DEFAULT_C_TRICORE_ 1U
#define _CLANG_C_TRICORE_ 1U
#elif defined(__GNUC__)
#define _GNU_C_TRICORE_ 1U
#define _HITECH_C_TRICORE_ 1U
#endif

#if defined (__ghs__)
#define COMPILER_PACKED __packed
#else
#define COMPILER_PACKED
#endif

/**********************************************************************************************************************
 * Types and Defines
 *********************************************************************************************************************/
/* SWS_COMPILER_00046 */
#define AUTOMATIC
/* SWS_COMPILER_00059 */
#define TYPEDEF
/* SWS_COMPILER_00051 */
#ifndef NULL_PTR
#define NULL_PTR ((void *)0)
#endif
/* SWS_COMPILER_00057 */
#define INLINE inline
/* SWS_COMPILER_00060 */
#define LOCAL_INLINE static inline

/**********************************************************************************************************************
 * Macros for Functions
 *********************************************************************************************************************/
/* SWS_COMPILER_00001 */
#define FUNC(rettype, memclass) memclass rettype
/* SWS_COMPILER_00061 */
#define FUNC_P2CONST(rettype, ptrclass, memclass) const ptrclass rettype * memclass
/* SWS_COMPILER_00063 */
#define FUNC_P2VAR(rettype, ptrclass, memclass) ptrclass rettype * memclass

/**********************************************************************************************************************
 * Macros for Pointers
 *********************************************************************************************************************/
/* SWS_COMPILER_00006 */
#define P2VAR(ptrtype, memclass, ptrclass) ptrclass ptrtype * memclass
/* SWS_COMPILER_00013 */
#define P2CONST(ptrtype, memclass, ptrclass) const ptrtype memclass * ptrclass
/* SWS_COMPILER_00031 */
#define CONSTP2VAR(ptrtype, memclass, ptrclass) ptrclass ptrtype * const memclass
/* SWS_COMPILER_00032 */
#define CONSTP2CONST(ptrtype, memclass, ptrclass) const memclass ptrtype * const ptrclass
/* SWS_COMPILER_00039 */
#define P2FUNC(rettype, ptrclass, fctname) rettype (*ptrclass fctname)
/* SWS_COMPILER_00065 */
#define CONSTP2FUNC(rettype, ptrclass, fctname) rettype (* const ptrclass fctname)

/**********************************************************************************************************************
 * Keywords for constants
 *********************************************************************************************************************/
/* SWS_COMPILER_00023 */
#define CONST(type, memclass) memclass const type

/**********************************************************************************************************************
 * Keywords for variables
 *********************************************************************************************************************/
/* SWS_COMPILER_00026 */
#define VAR(type, memclass) memclass type

#endif
