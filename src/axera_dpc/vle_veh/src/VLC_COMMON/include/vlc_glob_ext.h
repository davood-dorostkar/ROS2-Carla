
/** @defgroup frame FRAME (Global Variable Framework)
   @ingroup sim_swc_vlc

@{ */

#ifndef VLC_GLOB_EXT_H
#define VLC_GLOB_EXT_H

/*use internal global header files*/
#include "TM_Global_TypeDefs.h"
//#include "TM_Global_Types.h"
// TEST

#include "stddef.h"
#include "vlc_config.h"
#include "vlc_types.h"
#include "vlc_ver.h"

/*******************************************************************************************/
/*! @cond Doxygen_Suppress */
#ifndef DLLEXPORT
#ifdef VLC_SIMULATION
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT
#endif
#endif
/*! @endcond Doxygen_Suppress */
/* includes */

/*****************************************************************************
  MACROS
*****************************************************************************/
/*! Makros for declaration and definition of TP application parameters
    @description:

    TP_DECL_ADJ_PARAM : use in <sub-comp>_par.h file to DECLARE a
   "adjustable" parameter in RAM
    TP_DECL_FIX_PARAM : use in <sub-comp>_par.h file to DECLARE a "NOT
   adjustable" parameter in ROM
      usage:
            \#define <sub-comp>_PARAM_NAME   <sub-comp>_PARAM_NAME_c      // (
   \#define parameter name and map it to variable
                                                                         //   in
   code only name of parameter is used, NOT variable name (_c)
            TP_DECL_FIX_PARAM( [type],      <sub-comp>_PARAM_NAME_c )    // (
   declaration of fix/adjustable parameter )

    TP_DEF_ADJ_PARAM : use in <sub-comp>_par.c file to DEFINE a     "adjustable"
   parameter in RAM
    TP_DEF_FIX_PARAM : use in <sub-comp>_par.c file to DEFINE a "NOT adjustable"
   parameter in ROM
      usage:
            TP_DEF_FIX_PARAM( [type] , <sub-comp>_PARAM_NAME_c , <value> )

     notice: declaration AND definition must be of same type (data type and
   FIX/ADJ)!!*/

/* see description above */
#define VLC_DECL_ADJ_PARAM(type_, name_) extern type_ name_;
/*! alias for not changable parameters, see VLC_DECL_ADJ_PARAM */
#define VLC_DECL_FIX_PARAM(type_, name_) extern const type_ name_;

/*! alias for changable parameters, see VLC_DECL_ADJ_PARAM */
#define VLC_DEF_ADJ_PARAM(type_, name_, value_) type_ name_ = (value_);
/*! alias for not changable parameters, see VLC_DECL_ADJ_PARAM */
#define VLC_DEF_FIX_PARAM(type_, name_, value_) const type_ name_ = (value_);

/* MACROS (GLOBAL) */
#ifndef SET_MEMSEC_VAR_A
#define SET_MEMSEC_VAR_A(v)
#endif
#ifndef SET_MEMSEC_VAR
#define SET_MEMSEC_VAR(v)
#endif
#ifndef SET_MEMSEC_CONST_A
#define SET_MEMSEC_CONST_A(v)
#endif
#ifndef SET_MEMSEC_CONST
#define SET_MEMSEC_CONST(v)
#endif
#ifndef MEMSEC_REF
#define MEMSEC_REF
#endif
/* SYMBOLIC CONSTANTS (GLOBAL) */

#ifndef _PARAM_UNUSED
#define _PARAM_UNUSED(x) \
    (void)(x) /*!< macro to void parameter (not used interface solution)*/
#ifdef PRQA_SIZE_T

#pragma PRQA_MACRO_MESSAGES_OFF "_PARAM_UNUSED" 3112

#endif
#endif /* #ifndef _PARAM_UNUSED */

#endif
/** @} end defgroup */
