/*
* Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
* wen mingshu <wenmingshu@senseauto.com>
*/
/*****************************************************************************

AUTOLIV ELECTRONIC document.

-------------------------------------

DUPLICATION or DISCLOSURE PROHIBITED without prior written consent

******************************************************************************/
/* PRQA S 0288 ++ */
/*
 * Explanation:
 *    Disabled for MKS keywords
 */
/*

*/
/* PRQA S 0288 -- */
/*****************************************************************************/
/*****************************************************************************/
/* STD_TYPES.H v5.2.0A13098                                                  */
/*                                                                           */
/*                                                                           */
/*  Redistribution and  use in source  and binary forms, with  or without    */
/*  modification,  are permitted provided  that the  following conditions    */
/*  are met:                                                                 */
/*                                                                           */
/*     Redistributions  of source  code must  retain the  above copyright    */
/*     notice, this list of conditions and the following disclaimer.         */
/*                                                                           */
/*     Redistributions in binary form  must reproduce the above copyright    */
/*     notice, this  list of conditions  and the following  disclaimer in    */
/*     the  documentation  and/or   other  materials  provided  with  the    */
/*     distribution.                                                         */
/*                                                                           */
/*     Neither the  name of Texas Instruments Incorporated  nor the names    */
/*     of its  contributors may  be used to  endorse or  promote products    */
/*     derived  from   this  software  without   specific  prior  written    */
/*     permission.                                                           */
/*                                                                           */
/*  THIS SOFTWARE  IS PROVIDED BY THE COPYRIGHT  HOLDERS AND CONTRIBUTORS    */
/*  "AS IS"  AND ANY  EXPRESS OR IMPLIED  WARRANTIES, INCLUDING,  BUT NOT    */
/*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR    */
/*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT    */
/*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,    */
/*  SPECIAL,  EXEMPLARY,  OR CONSEQUENTIAL  DAMAGES  (INCLUDING, BUT  NOT    */
/*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,    */
/*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY    */
/*  THEORY OF  LIABILITY, WHETHER IN CONTRACT, STRICT  LIABILITY, OR TORT    */
/*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE    */
/*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.     */
/*                                                                           */
/*****************************************************************************/
#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_SOURCE_FUSION_STD_TYPES_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_SOURCE_FUSION_STD_TYPES_H_
/******************************************************************************
 *  AUTOSAR Standard Types
 *  Document reference: AUTOSAR_SWS_PlatformTypes.pdf
 *
 *                         Non COM SW modules
 *                                 |
 *                            Std_Types.h
 *                             |       |
 *                 Platform_Types.h   FusionCompiler.h
 *                                        |
 *                                  FusionCompiler_Cfg.h
 ******************************************************************************/
// #include "TM_Global_Types.h"
#include "fusion_ext.h"
#include "FusionCompiler.h"

/******************************************************************************
Extension to AUTOSAR Standard Types for 64bit variables
******************************************************************************/
// typedef signed long long sint64;
// typedef unsigned long long uint64;

/******************************************************************************
   TYPE DEFINITIONS
******************************************************************************/
// typedef uint8 Std_ReturnType;

// typedef struct {
//     uint16 vendorID;
//     uint16 moduleID;
//     uint8 instanceID;
//     uint8 sw_major_version;
//     uint8 sw_minor_version;
//     uint8 sw_patch_version;
// } Std_VersionInfoType;

/*****************************************************************************/
/* SYMBOL DEFINITIONS                                                        */
/*****************************************************************************/
#ifndef STATUSTYPEDEFINED
#define STATUSTYPEDEFINED
#define E_OK 0x00U

typedef unsigned char StatusType;
#endif

#define E_NOT_OK 0x01U

#define STD_HIGH 0x01U /* Physical state 5V or 3.3V */
#define STD_LOW 0x00U  /* Physical state 0v         */

#define STD_ACTIVE 0x01U /* Logical state active */
#define STD_IDLE 0x00U   /* Logical state idle   */

#define STD_ON 0x01U
#define STD_OFF 0x00U

/******************************************************************************
Evolution of the component
******************************************************************************/
/* PRQA S 0288 ++ */
/*
 * Explanation:
 *    Disabled for MKS keywords
 */
/*
$Log: Std_Types.h  $
/*****************************************************************************/

#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_SOURCE_FUSION_STD_TYPES_H_
