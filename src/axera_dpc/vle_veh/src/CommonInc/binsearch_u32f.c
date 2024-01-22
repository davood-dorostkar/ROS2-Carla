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
 * File                             : binsearch_u32f.c
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

uint32_T binsearch_u32f(real32_T u, const real32_T bp[], uint32_T startIndex,
  uint32_T maxIndex)
{
  uint32_T bpIdx;
  uint32_T bpIndex;
  uint32_T iRght;

  /* Binary Search */
  bpIdx = startIndex;
  bpIndex = 0U;
  iRght = maxIndex;
  while (iRght - bpIndex > 1U) {
    if (u < bp[bpIdx]) {
      iRght = bpIdx;
    } else {
      bpIndex = bpIdx;
    }

    bpIdx = (iRght + bpIndex) >> 1U;
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