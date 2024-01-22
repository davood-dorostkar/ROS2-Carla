/**********************************Model Property********************************
 *
 * Company             : SENSETIME
 *
 * Version             : Ver1.9
 *
 * Model               : LaDMC_model
 *
 ************************************Auto Coder**********************************
 *
 * File                             : plook_u32ff_binc.c
 *
 * FileType                         : Code Source File
 *
 * Real-Time Workshop file version  : 9.4 (R2020b) 29-Jul-2020
 *
 * TLC version                      : 9.4 (Aug 20 2020)
 *
 * C source code generated on       : Wed Feb 16 19:23:00 2022
 *******************************************************************************/
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" 

#include "rtwtypes.h"
#include "binsearch_u32f.h"
#include "plook_u32ff_binc.h"

uint32_T plook_u32ff_binc(real32_T u, const real32_T bp[], uint32_T maxIndex,
  real32_T *fraction)
{
  uint32_T bpIndex;

  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Clip'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u <= bp[0U]) {
    bpIndex = 0U;
    *fraction = 0.0F;
  } else if (u < bp[maxIndex]) {
    bpIndex = binsearch_u32f(u, bp, maxIndex >> 1U, maxIndex);
    *fraction = (u - bp[bpIndex]) / (bp[bpIndex + 1U] - bp[bpIndex]);
  } else {
    bpIndex = maxIndex - 1U;
    *fraction = 1.0F;
  }

  return bpIndex;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h"