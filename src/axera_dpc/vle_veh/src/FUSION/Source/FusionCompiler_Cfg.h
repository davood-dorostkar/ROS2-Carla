/*************************************************************************

AUTOLIV ELECTRONIC document.

-------------------------------------

DUPLICATION or DISCLOSURE PROHIBITED without prior written consent

**************************************************************************


*************************************************************************/
/*****************************************************************************/
/* FusionCompiler_Cfg.h v5.2.0A13098 */
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
#ifndef _COMPILER_CFG_H_
#define _COMPILER_CFG_H_

/*************************************************************************
Helper Macro for removing compiler warning
            #880-D: parameter "par" was never referenced
*************************************************************************/
#define COMPILER_UNUSED_PRM_DIS(par) (void)(par)
#define TS_PARAM_UNUSED(par) COMPILER_UNUSED_PRM_DIS((par))
/*
 * Definition of static keyword for enabling external visibility of
 * translation unit objects. The default resolution is 'static' and
 * should be used for production builds.
 * For unit test environment LOCAL shall be defined as LOCAL= on the
 * command line. This will allow unit test harness to access UUT
 * static functions and objects
 */
#ifndef LOCAL
#define LOCAL static
#endif /* LOCAL */

#ifndef EXTERN
#define EXTERN extern
#endif /* EXTERN */

/*************************************************************************
   This is an empty header file which should be replaced by the end user.

   Compiler_Cfg.c is normally a system-wide header file.
   Typically the system integrator will take the settings
   from each module and construct this by hand.
   The contents of "Os_Compiler_Cfg.h" can be used by
   the system integrator for this purpose.

   According to the AUTOSAR document "Specification of Compiler
   Abstraction" version 2.0.1, as part of AUTOSAR release 3.1,
   COMPILER55:
     The file FusionCompiler_Cfg.h shall contain the module specific parameters
     (ptrclass and memclass) that are passed to the macros defined in
     FusionCompiler.h.

   The following is the form of these macro definitions.  MSN stands for
   "Module Short Name of BSW module list"

   #define <MSN>_CODE
   #define <MSN>_CONST
   #define <MSN>_APPL_DATA
   #define <MSN>_APPL_CONST
   #define <MSN>_APPL_CODE
   #define <MSN>_VAR_NOINIT
   #define <MSN>_VAR_POWER_ON_INIT
   #define <MSN>_VAR_FAST
   #define <MSN>_VAR

   NOTE: On 32 bit architectures these definitions are required
        to be empty
*************************************************************************/
/*************************************************************************
OS
*************************************************************************/
//#include "Os_Compiler_Cfg.h"
/*************************************************************************
RTE
*************************************************************************/
#define RTE_CODE
#define RTE_DATA
#define RTE_CONST
#define RTE_OS_CDATA
#define RTE_APPL_DATA
#define RTE_APPL_CONST
#define RTE_OFFLINE
#define RTE_ONLINE
/*************************************************************************
CRC
*************************************************************************/
#define CRC_CODE
#define CRC_APPL_DATA
/*************************************************************************
E2E
*************************************************************************/
#define E2E_CODE
#define E2E_CONST
#define E2E_APPL_DATA
#define E2E_APPL_CONST
#define E2E_APPL_CODE
#define E2E_VAR_NOINIT
#define E2E_VAR_POWER_ON_INIT
#define E2E_VAR_FAST
#define E2E_VAR
/*************************************************************************
InpAdpr
*************************************************************************/
#define InpAdpr_CODE
/*************************************************************************
OutpAdpr
*************************************************************************/
#define OutpAdpr_CODE
/*************************************************************************
AdasEcuModeMgr
*************************************************************************/
#define AdasEcuModeMgr_CODE
/*************************************************************************
Dcm
*************************************************************************/
#define Dcm_CODE
/*************************************************************************
Dem
*************************************************************************/
#define Dem_CODE
/*************************************************************************
NvM
*************************************************************************/
#define NvM_CODE
/*************************************************************************
WdgM
*************************************************************************/
#define WdgM_CODE
/*************************************************************************
AsyMgrForDataRec
*************************************************************************/
#define AsyMgrForDataRec_CODE
/*************************************************************************
HznRgnElectc
*************************************************************************/
#define HznRgnElectc_CODE
/*************************************************************************
InpAdpr
*************************************************************************/
#define IfAdpr_CODE

/*************************************************************************
IfAdprAMfDR_CODE
*************************************************************************/
#define IfAdprAMfDR_CODE

/**********************************************************************************************************************
 *  ObjFsn START
 *********************************************************************************************************************/
//#pragma section("fusion_data", read, write)

/* used for SWC specific code */
#define ObjFusn_CODE

/* used for code that shall go into fast code memory segments */
#define ObjFusn_CODE_FAST

/* used for global or static SWC specific constants */
#define ObjFusn_CONST

/* used for all global or static SWC specific variables that are not initialized
 * by the startup code of the compiler */
#define ObjFusn_VAR_NOINIT

/* used for global or static SWC specific variables that are initialized by the
 * startup code of the compiler */
#define ObjFusn_VAR_INIT

/* used for global or static SWC specific variables that are initialized with
 * zero by the startup code of the compiler */
#define ObjFusn_VAR_ZERO_INIT

/* used for all global or static SWC specific variables that are not initialized
 * by the startup code of the compiler */
#define ObjFusn_VAR_FAST_NOINIT

/* used for global or static SWC specific variables that are initialized by the
 * startup code of the compiler */
#define ObjFusn_VAR_FAST_INIT

/* used for all global or static SWC specific variables that are not initialized
 * by the startup code of the compiler */
#define ObjFusn_VAR_SLOW_NOINIT

/* used for global or static SWC specific variables that are initialized by the
 * startup code of the compiler */
#define ObjFusn_VAR_SLOW_INIT

#define ObjFusn_VAR_NOSTACK

#define ObjFusn_APPL_CODE

/* To be used for references on application data (expected to be in RAM or ROM)
 * passed via API */
#define ObjFusn_APPL_DATA

/* To be used for references on application constants (expected to be certainly
   in ROM, for instance pointer of Init-function)
   passed via API */
#define ObjFusn_APPL_CONST

/**********************************************************************************************************************
 *  ObjFsn END
 *********************************************************************************************************************/

#endif /* _COMPILER_CFG_H_ */
