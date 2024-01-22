/**********************************************************************************************************************
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: Configuration header file of ComStack
 * Author: VOS
 * Create: 2019-12-21
 *
 *********************************************************************************************************************/

#ifndef PLATFORMTYPES_H
#define PLATFORMTYPES_H
/* this is the vos vendor */
#define PLATFORM_VENDOR_ID (0x007F)
#define PLATFORM_TYPES_AR_RELEASE_MAJOR_VERSION       4
#define PLATFORM_TYPES_AR_RELEASE_MINOR_VERSION       4
#define PLATFORM_TYPES_AR_RELEASE_REVISION_VERSION    0
#define PLATFORM_TYPES_SW_MAJOR_VERSION               1
#define PLATFORM_TYPES_SW_MINOR_VERSION               0
#define PLATFORM_TYPES_SW_PATCH_VERSION               0

#ifndef AUTOSAR_CP_42
#define AUTOSAR_CP_42
#endif
/**********************************************************************************************************************
 * Based on AUTOSAR_SWS_PlatformTypes.pdf; Document Version 4.4.0
 *********************************************************************************************************************/
/**********************************************************************************************************************
 * Types and Defines
 *********************************************************************************************************************/
/* SWS_Platform_00064 */
#define CPU_TYPE_8 8u
#define CPU_TYPE_16 16u
#define CPU_TYPE_32 32u
#define CPU_TYPE_64 64u
/* SWS_Platform_00038 */
#define MSB_FIRST 0u
#define LSB_FIRST 1u
/* SWS_Platform_00039 */
#define HIGH_BYTE_FIRST 0u /* Big endian byte ordering */
#define LOW_BYTE_FIRST 1u /* Little endian byte ordering */

#define CPU_TYPE CPU_TYPE_32
#define CPU_BYTE_ORDER LOW_BYTE_FIRST
#define CPU_BIT_ORDER LSB_FIRST

/**********************************************************************************************************************
 * Standard Integer Data Types
 *********************************************************************************************************************/
/* SWS_Platform_00013 */
typedef unsigned char uint8;
/* SWS_Platform_00014 */
typedef unsigned short uint16;
/* SWS_Platform_00015 */
typedef unsigned int uint32;
/* SWS_Platform_00016 */
typedef signed char sint8;
/* SWS_Platform_00017 */
typedef signed short sint16;
/* SWS_Platform_00018 */
typedef signed int sint32;
/* SWS_Platform_00066 */
typedef unsigned long long uint64;
/* SWS_Platform_00067 */
typedef long long sint64;

/**********************************************************************************************************************
 * Standard Float Data Types
 *********************************************************************************************************************/
/* SWS_Platform_00041 */
typedef float float32;
/* SWS_Platform_00042 */
typedef double float64;

/**********************************************************************************************************************
 * Boolean Float Data Types
 *********************************************************************************************************************/
/* SWS_Platform_00026 */
typedef unsigned char boolean;

/* SWS_Platform_00056 */
#ifndef TRUE
#define TRUE 1u
#endif

#ifndef FALSE
#define FALSE 0u
#endif

/**********************************************************************************************************************
 * Optimized Integer Data Types
 *********************************************************************************************************************/
/* SWS_Platform_00020 */
typedef unsigned long uint8_least;
/* SWS_Platform_00021 */
typedef unsigned long uint16_least;
/* SWS_Platform_00022 */
typedef unsigned long uint32_least;
/* SWS_Platform_00023 */
typedef signed long sint8_least;
/* SWS_Platform_00024 */
typedef signed long sint16_least;
/* SWS_Platform_00025 */
typedef signed long sint32_least;

/**********************************************************************************************************************
 * custom def Data Types
 *********************************************************************************************************************/
typedef unsigned long long uint8_8_iso_8859_1;

#endif
