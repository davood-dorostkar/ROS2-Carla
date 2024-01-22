/*! \file **********************************************************************

  COMPONENT:              MAT (math functions)

  MODULENAME:             mat_std_ext.h

  @brief                  This module contains all standard math functions like
  min/max operators


  ---*/
#ifndef MATSTD_INCLUDED
#define MATSTD_INCLUDED

#include "vlc_glob_ext.h"

/** @defgroup vlc_mat_std VLC_MAT_STD ( mathematical library for standard
operations )
containes methods for standard mathematical operations
   @ingroup vlc_veh

@{ */

/* Special min/max defines previously present in vlc_types.h, but only used by
MAT and
modules that use MAT, thus moved here */
#define Signed_int16_min (-32767)
#define Signed_int16_max 32766
#define Signed_int32_max 2147483647L

/* FUNKTION PROTOTYPES (KOMPONENT EXTERNAL) */
extern sint32 MAT_MIN(sint32 Val1, sint32 Val2);
extern sint32 MAT_MAX(sint32 Val1, sint32 Val2);
extern sint32 MAT_LIM(sint32 Val, sint32 Lim1, sint32 Lim2);
extern sint32 MAT_FILT(sint32 NewVal, sint32 OldVal, sint32 FilterDepth);
extern sint32 MAT_ABS(sint32 i);
extern sint32 MAT_DIFF_DT(sint32 New, sint32 Old, times_t Cycle);
extern sint32 MAT_INT_DT(sint32 New, sint32 Old, sint32 OldInt, times_t Cycle);
extern sint32 MAT_LIM_GRAD(sint32 New,
                           sint32 Old,
                           sint32 max_neg_grad,
                           sint32 max_pos_grad,
                           times_t Cycle);
extern sint32 MAT_PT1_FILTER(const times_t cycle_time,
                             times_t time_constant,
                             sint32 input_value,
                             sint32 output_value_last_cycle);
extern sint32 MAT_SQRT(sint32 Val);
extern sint8 MAT_SIGN(sint32 Val);
extern sint32 MAT_MUL(sint32 Val1,
                      sint32 Val2,
                      sint32 Val1Scale,
                      sint32 Val2Scale,
                      sint32 ReturnScale);
extern sint32 MAT_DIV(sint32 Val1,
                      sint32 Val2,
                      sint32 Val1Scale,
                      sint32 Val2Scale,
                      sint32 ReturnScale);
extern sint16 MAT_SIN(sint32 Val);
extern sint16 MAT_COS(sint32 Val);
extern sint16 MAT_TAN(sint32 Val);
extern sint32 MAT_QUANT(sint32 Val,
                        sint32 LastVal,
                        sint32 MinVal,
                        sint32 MaxVal,
                        sint32 Res,
                        percentage_t DutyCycle);

#endif
/** @} end defgroup */
