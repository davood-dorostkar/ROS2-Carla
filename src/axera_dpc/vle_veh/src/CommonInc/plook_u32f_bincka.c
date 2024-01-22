/**********************************Model Property********************************
 *
 * Company             : SENSETIME
 *
 * Tool Version        : Ver2.0
 *
 * Model Name          : TJASA_model
 *
 * Model Long Name     : Traffic Jam Assist

 *

 * Model Advisor       : Not Check

 *

 * Model Version       : Ver_02

 *

 * Model Author        : WJ

 *

 * Model Reviewer      :

 *

 * Model Review Data   :

 *

 * Model Cycle Time    : 50ms


 ************************************Auto Coder**********************************
 *
 * File                             : plook_u32f_bincka.c
 *
 * FileType                         : Code Source File
 *
 * Real-Time Workshop file version  : 9.4 (R2020b) 29-Jul-2020
 *
 * TLC version                      : 9.4 (Aug 20 2020)
 *
 * C source code generated on       : Thu Feb  2 11:07:18 2023
 *
 * Copyright (C) by SenseTime Group Limited. All rights reserved.
 *******************************************************************************/

#include "rtwtypes.h"
#include "binsearch_u32f.h"
#include "plook_u32f_bincka.h"

uint32_T plook_u32f_bincka(real32_T u, const real32_T bp[], uint32_T maxIndex)
{
  uint32_T bpIndex;

  /* Prelookup - Index only
     Index Search method: 'binary'
     Extrapolation method: 'Clip'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'on'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u <= bp[0U]) {
    bpIndex = 0U;
  } else if (u < bp[maxIndex]) {
    bpIndex = binsearch_u32f(u, bp, maxIndex >> 1U, maxIndex);
  } else {
    bpIndex = maxIndex;
  }

  return bpIndex;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
