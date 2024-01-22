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
 * File                             : intrp1d_fu32fl_pw.c
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
#include "intrp1d_fu32fl_pw.h"

real32_T intrp1d_fu32fl_pw(uint32_T bpIndex, real32_T frac, const real32_T
  table[])
{
  real32_T yL_0d0;

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'portable wrapping'
   */
  yL_0d0 = table[bpIndex];
  return (table[bpIndex + 1U] - yL_0d0) * frac + yL_0d0;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h"