/**********************************************************************************************************************
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: Configuration header file of std type
 * Author: VOS
 * Create: 2019-12-21
 *
 *********************************************************************************************************************/

#ifndef STANDARDTYPES_H
#define STANDARDTYPES_H

/**********************************************************************************************************************
 * Based on AUTOSAR_SWS_StandardTypes.pdf; Document Version 4.4.0
 *********************************************************************************************************************/
/*********************************************************************************************************************
 * Includes
 *********************************************************************************************************************/
#include "PlatformTypes.h"
#include "Compiler.h"

/**********************************************************************************************************************
 * Types and Defines
 *********************************************************************************************************************/
/* SWS_Std_00005 */
typedef uint8 Std_ReturnType;

/* SWS_Std_00015 */
// typedef struct {
//     uint16 vendorID;
//     uint16 moduleID;
//     uint8 sw_major_version;
//     uint8 sw_minor_version;
//     uint8 sw_patch_version;
// } Std_VersionInfoType;

/**********************************************************************************************************************
 * Symbol and Defines
 *********************************************************************************************************************/
/* SWS_Std_00006 */
#ifndef STATUSTYPEDEFINED
#define STATUSTYPEDEFINED
#define E_OK 0x00u /* No error occurs */
typedef unsigned char StatusType; /* OSEK compliance */
#endif
#define E_NOT_OK 0x01u /* An error occurs */
/* SWS_Std_00007 */
#define STD_LOW 0x00u /* Physical state 0V */
#define STD_HIGH 0x01u /* Physical state 5V or 3.3V */
/* SWS_Std_00013 */
#define STD_IDLE 0x00u /* Logical state idle */
#define STD_ACTIVE 0x01u /* Logical state active */
/* SWS_Std_00010 */
#define STD_OFF 0x00u /* Off state */
#define STD_ON 0x01u /* On state */

/* for post build witch */
#if (!defined(VARIANT_PRE_COMPILE))
#define VARIANT_PRE_COMPILE                 0x00u
#endif
#if (!defined(VARIANT_LINK_TIME))
#define VARIANT_LINK_TIME                   0x01u
#endif
#if (!defined(VARIANT_POST_BUILD))
#define VARIANT_POST_BUILD                  0x02u
#endif

#endif
