/*
 * Copyright (C) 2017-2021 by SenseTime Group Limited. All rights reserved.
 * He Qiushu <heqiushu@senseauto.com>
 */

#ifndef glob_type_H
#define glob_type_H

#include "Platform_Types.h"

#define __IPL_CANTPP__ 1

#ifdef __cplusplus
extern "C" {
#endif

#define PLATFORM_TYPES_H

#if ((!defined(PLATFORM_TYPES_H)) && \
     (!(defined(_MSC_VER) && (!defined(__IPL_CANTPP__)))))

typedef unsigned char boolean;
#endif

#if ((!defined(PLATFORM_TYPES_H)) && \
     (!(defined(_MSC_VER) && (!defined(__IPL_CANTPP__)))))

typedef signed char sint8;
#endif

#if ((!defined(PLATFORM_TYPES_H)) && \
     (!(defined(_MSC_VER) && (!defined(__IPL_CANTPP__)))))

typedef unsigned char uint8;
#endif

#if ((!defined(PLATFORM_TYPES_H)) && \
     (!(defined(_MSC_VER) && (!defined(__IPL_CANTPP__)))))

typedef signed short sint16;
#endif

#if ((!defined(PLATFORM_TYPES_H)) && \
     (!(defined(_MSC_VER) && (!defined(__IPL_CANTPP__)))))

typedef unsigned short uint16;
#endif

#if ((!defined(PLATFORM_TYPES_H)) && \
     (!(defined(_MSC_VER) && (!defined(__IPL_CANTPP__)))))

typedef signed long sint32;
#endif

#if ((!defined(PLATFORM_TYPES_H)) && \
     (!(defined(_MSC_VER) && (!defined(__IPL_CANTPP__)))))

typedef unsigned long uint32;
#endif

#if ((!defined(PLATFORM_TYPES_H)) && \
     (!(defined(_MSC_VER) && (!defined(__IPL_CANTPP__)))))

typedef signed long long sint64;
#endif

// #if ( (!defined(PLATFORM_TYPES_H)) && ( !( defined(_MSC_VER) &&
// (!defined(__IPL_CANTPP__)) ) ) )

// typedef unsigned long long uint64;
// #endif

#if ((!defined(PLATFORM_TYPES_H)) && \
     (!(defined(_MSC_VER) && (!defined(__IPL_CANTPP__)))))

typedef float float32;
#endif

#if ((!defined(PLATFORM_TYPES_H)) && \
     (!(defined(_MSC_VER) && (!defined(__IPL_CANTPP__)))))

typedef double float64;
#endif

typedef sint8 *psint8;

typedef uint8 *puint8;

typedef sint16 *psint16;

typedef uint16 *puint16;

typedef sint32 *psint32;

typedef uint32 *puint32;

typedef void *pvoid;

#if 0
#if ((!defined(DISABLE_LEAST_TYPE_GENERATION)) && \
     (!defined(PLATFORM_TYPES_H)) &&              \
     (!(defined(_MSC_VER) && (!defined(__IPL_CANTPP__)))))

typedef signed int sint8_least;
#endif

#if ((!defined(DISABLE_LEAST_TYPE_GENERATION)) && \
     (!defined(PLATFORM_TYPES_H)) &&              \
     (!(defined(_MSC_VER) && (!defined(__IPL_CANTPP__)))))

typedef unsigned int uint8_least;
#endif

#if ((!defined(DISABLE_LEAST_TYPE_GENERATION)) && \
     (!defined(PLATFORM_TYPES_H)) &&              \
     (!(defined(_MSC_VER) && (!defined(__IPL_CANTPP__)))))

typedef signed int sint16_least;
#endif

#if ((!defined(DISABLE_LEAST_TYPE_GENERATION)) && \
     (!defined(PLATFORM_TYPES_H)) &&              \
     (!(defined(_MSC_VER) && (!defined(__IPL_CANTPP__)))))

typedef unsigned int uint16_least;
#endif

#if ((!defined(DISABLE_LEAST_TYPE_GENERATION)) && \
     (!defined(PLATFORM_TYPES_H)) &&              \
     (!(defined(_MSC_VER) && (!defined(__IPL_CANTPP__)))))

typedef signed int sint32_least;
#endif

#if ((!defined(DISABLE_LEAST_TYPE_GENERATION)) && \
     (!defined(PLATFORM_TYPES_H)) &&              \
     (!(defined(_MSC_VER) && (!defined(__IPL_CANTPP__)))))

typedef unsigned int uint32_least;
#endif
#endif
typedef unsigned char osuint8;
typedef unsigned short osuint16;
typedef unsigned int osuint32;
typedef signed char osint8;
typedef signed short osint16;
typedef signed int osint32;

typedef signed int sbit32_t;
typedef signed short sbit16_t;
typedef signed char sbit8_t;
typedef unsigned int ubit32_t;
typedef unsigned short ubit16_t;
typedef unsigned char ubit8_t;

typedef uint8 ui8_t;
typedef uint16 ui16_t;
typedef uint32 ui32_t;

typedef sint8 i8_t;
typedef sint16 i16_t;
typedef sint32 i32_t;

typedef float32 f32_t;
typedef boolean bool_t;

typedef osuint32 PhysicalTimeType;
typedef unsigned long long osTPTimeStampType;
typedef osTPTimeStampType* osTPTimeStampRefType;
typedef long long osTPTimeType;
typedef osTPTimeType* osTPTimeRefType;

typedef osuint8 ProtectionReturnType;

typedef osuint16 SpinlockIdType;
typedef osuint8 TryToGetSpinlockType;
typedef osuint32 CoreIdType;

typedef ui8_t enum_t;
typedef uint32 ubit32;

typedef signed char int8_T;
typedef unsigned char uint8_T;
typedef short int16_T;
typedef unsigned short uint16_T;
typedef int int32_T;
typedef unsigned int uint32_T;
typedef float real32_T;
typedef double real64_T;

typedef double real_T;
typedef double time_T;
typedef unsigned char boolean_T;
typedef int int_T;
typedef unsigned int uint_T;
typedef unsigned long ulong_T;
typedef char char_T;
typedef char_T byte_T;
typedef signed short int16;

#ifndef RTE_TYPE_H
typedef boolean Boolean;
#endif

#ifndef b_FALSE
#define b_FALSE ((boolean)0)
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef b_TRUE
#define b_TRUE ((boolean)1)
#endif

#ifndef TRUE
#define TRUE 1
#endif

#if 0
#ifndef NULL
#define NULL ((void*)0)
#endif
#endif
#define CONST(consttype, memclass) const consttype
#define CONST(consttype, memclass) const consttype
#define CONSTP2CONST(ptrtype, memclass, ptrclass) const ptrtype* const
#define P2VAR(ptrtype, memclass, ptrclass) ptrtype*

#define CT(type) CONST(type, AUTOMATIC)
#define VR(type) VAR(type, AUTOMATIC)

#define CP2C(type) CONSTP2CONST(type, AUTOMATIC, COM_APPL_CONST)
#define P2C(type) P2CONST(type, AUTOMATIC, COM_APPL_CONST)

#define CP2V(type) CONSTP2VAR(type, AUTOMATIC, COMM_APPL_DATA)
#define P2V(type) P2VAR(type, AUTOMATIC, COMM_APPL_DATA)
#define VAR(vartype, memclass) vartype

#define P2CONST(ptrtype, memclass, ptrclass) const ptrtype*
#define CONSTP2VAR(ptrtype, memclass, ptrclass) ptrtype* const

#ifdef MPC574xG
typedef unsigned int size_t;
#endif

#ifndef NULL
#ifdef __cplusplus
#define NULL (0)
#else
#define NULL ((void*)0)
#endif
#endif

#ifdef TU_ALGO_SIMULATION_VS_CONF_200
#define S32DS_SECTION_internal1
#else
#define S32DS_SECTION_internal1 __attribute__((section(".internal1")))
#endif

#define LOG_ERR 1
#define LOG_DEBUG 2
#define LOG_INFO 3

//#undef NULL
//#ifdef __cplusplus
//#define NULL 0
//#else
//#define NULL ((void *)0)
//#endif

#ifdef __cplusplus
}
#endif

#endif
