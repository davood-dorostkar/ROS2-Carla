         

/*****************************************************************************
  INCLUDES
*****************************************************************************/
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" 

#include "TM_Global_Types.h" 
#include "TM_CML_Types.h" 
#include "TM_Base_Emul.h" 

/*****************************************************************************
  FUNCTIONS
*****************************************************************************/

/*****************************************************************************
  Functionname:    CML_v_FFT                                            */ /*!

  @brief           This computes an in-place complex-to-complex FFT

  @description     http://paulbourke.net/miscellaneous/dft/index.html
                   The output equals the output of a direct Fourier transform.
                   The direct transform requires less memory but more 
                   computation time.
                   The output of this function is NOT normalized by 1/N to save
                   computation time.

  @param[in,out]   a_Elements :        list of input elements, replaced by transformed elements
                                       Optimal Range for a_Elements[].f_Real is [-F_MAX..F_MAX]
                                       Optimal Range for a_Elements[].f_Imag is [-F_MAX..F_MAX]
                                       where F_MAX is one-third of the max range of float.
  @param[in]       u_LogNofElements :  logarithm of length of af_ListOfElements
                                       Range of values [0..32]
  @param[in]       b_Inverse :         true if inverse FFT shall be computed
                                       This value can be either TRUE or FALSE

  @return          void


*****************************************************************************/
void CML_v_FFT(t_Complexf32 a_Elements[], const uint8 u_LogNofElements, boolean b_Inverse)
{

  /* number of elements */
  uint32 u_NofElements = 1UL;

  float32 f_c1 = -1.f; 
  float32 f_c2 = 0.f;

  /* loop counters */
  uint32 u_ElementCounter1, u_ElementCounter2, u_ElementCounter3, u_ElementCounter4, u_ElementCounter5;

  {
    uint32 u_LogCounter;

    for(u_LogCounter = 0UL; u_LogCounter < (uint32)u_LogNofElements; u_LogCounter++)
    {
      u_NofElements *= 2UL;
    }
  }

  /* Do the bit reversal */
  u_ElementCounter2 = 0UL;

  for ( u_ElementCounter1 = 0UL; u_ElementCounter1 < (u_NofElements - 1UL); u_ElementCounter1++)
  {
    if (u_ElementCounter1 < u_ElementCounter2)
    {
      /* temporary variable for copying */
      t_Complexf32 Copy;

      Copy                           = a_Elements[u_ElementCounter1];
      a_Elements[u_ElementCounter1]  = a_Elements[u_ElementCounter2];
      a_Elements[u_ElementCounter2]  = Copy;
    }

    {
      /* loop counters */
      u_ElementCounter3 = u_NofElements / 2UL;

      while (u_ElementCounter3 <= u_ElementCounter2)
      {
        u_ElementCounter2 = u_ElementCounter2 - u_ElementCounter3;
        u_ElementCounter3 >>= 1;
      }
      u_ElementCounter2 = u_ElementCounter2 + u_ElementCounter3;
    }
  }

  /* Compute the FFT */

  u_ElementCounter4 = 1UL;
  for (u_ElementCounter1 = 0UL; u_ElementCounter1 < (uint32)u_LogNofElements; u_ElementCounter1++)
  {

    float32 f_u1 = 1.f; 
    float32 f_u2 = 0.f;

    u_ElementCounter5 = u_ElementCounter4;

    u_ElementCounter4 = (uint32) (u_ElementCounter4<<1);

    for (u_ElementCounter2 = 0UL; u_ElementCounter2 < u_ElementCounter5; u_ElementCounter2++)
    {
      for (u_ElementCounter3 = u_ElementCounter2; u_ElementCounter3 < u_NofElements;
        u_ElementCounter3 = (u_ElementCounter3 + u_ElementCounter4) )
      {
        t_Complexf32 *const CurrentElement =
          &(a_Elements[u_ElementCounter3 + u_ElementCounter5]);

        float32 const Internal1 =
          (f_u1 * CurrentElement->f_Real) - (f_u2 * CurrentElement->f_Imag);

        float32 const Internal2 =
          (f_u1 * CurrentElement->f_Imag) + (f_u2 * CurrentElement->f_Real);

        CurrentElement->f_Real = a_Elements[u_ElementCounter3].f_Real - Internal1; 
        CurrentElement->f_Imag = a_Elements[u_ElementCounter3].f_Imag - Internal2;

        a_Elements[u_ElementCounter3].f_Real += Internal1;
        a_Elements[u_ElementCounter3].f_Imag += Internal2;
      }

      {
        /* temporary variable for copying */
        float32 const f_Copy = (f_u1 * f_c1) - (f_u2 * f_c2);
        f_u2 = (f_u1 * f_c2) + (f_u2 * f_c1);
        f_u1 = f_Copy;
      }
    }

    f_c2 = BML_f_Sqrt((1.f - f_c1) / 2.f);

    if (b_Inverse == FALSE)
    {
      f_c2 = -f_c2;
    }

    f_c1 = BML_f_Sqrt((1.f + f_c1) / 2.f);
  }
}

/*****************************************************************************
  Functionname:    CML_v_InverseFFT16                                   */     

    
void CML_v_InverseFFT16(t_Complexf32 a_Elements[]) {

  static float32 const f_k2 = 0.382683432365090f;
  static float32 const f_k1 = 0.707106781186548f;
  static float32 const f_k3 = 0.923879532511287f;

  float32 f_Internal1;
  float32 f_Internal2;
  t_Complexf32 Copy;

  Copy           = a_Elements[1];
  a_Elements[1]  = a_Elements[8];
  a_Elements[8]  = Copy;

  Copy           = a_Elements[2];
  a_Elements[2]  = a_Elements[4];
  a_Elements[4]  = Copy;

  Copy           = a_Elements[3];
  a_Elements[3]  = a_Elements[12];
  a_Elements[12] = Copy;

  Copy           = a_Elements[5];
  a_Elements[5]  = a_Elements[10];
  a_Elements[10] = Copy;

  Copy           = a_Elements[7];
  a_Elements[7]  = a_Elements[14];
  a_Elements[14] = Copy;

  Copy            = a_Elements[11];
  a_Elements[11]  = a_Elements[13];
  a_Elements[13]  = Copy;

  f_Internal1 = a_Elements[1].f_Real;
  f_Internal2 = a_Elements[1].f_Imag;

  a_Elements[1].f_Real = a_Elements[0].f_Real - f_Internal1; 
  a_Elements[1].f_Imag = a_Elements[0].f_Imag - f_Internal2;

  a_Elements[0].f_Real += f_Internal1;
  a_Elements[0].f_Imag += f_Internal2;

  f_Internal1 = a_Elements[3].f_Real;
  f_Internal2 = a_Elements[3].f_Imag;

  a_Elements[3].f_Real = a_Elements[2].f_Real - f_Internal1; 
  a_Elements[3].f_Imag = a_Elements[2].f_Imag - f_Internal2;

  a_Elements[2].f_Real += f_Internal1;
  a_Elements[2].f_Imag += f_Internal2;

  f_Internal1 = a_Elements[5].f_Real;
  f_Internal2 = a_Elements[5].f_Imag;

  a_Elements[5].f_Real = a_Elements[4].f_Real - f_Internal1; 
  a_Elements[5].f_Imag = a_Elements[4].f_Imag - f_Internal2;

  a_Elements[4].f_Real += f_Internal1;
  a_Elements[4].f_Imag += f_Internal2;

  f_Internal1 = a_Elements[7].f_Real;
  f_Internal2 = a_Elements[7].f_Imag;

  a_Elements[7].f_Real = a_Elements[6].f_Real - f_Internal1; 
  a_Elements[7].f_Imag = a_Elements[6].f_Imag - f_Internal2;

  a_Elements[6].f_Real += f_Internal1;
  a_Elements[6].f_Imag += f_Internal2;

  f_Internal1 = a_Elements[9].f_Real;
  f_Internal2 = a_Elements[9].f_Imag;

  a_Elements[9].f_Real = a_Elements[8].f_Real - f_Internal1; 
  a_Elements[9].f_Imag = a_Elements[8].f_Imag - f_Internal2;

  a_Elements[8].f_Real += f_Internal1;
  a_Elements[8].f_Imag += f_Internal2;

  f_Internal1 = a_Elements[11].f_Real;
  f_Internal2 = a_Elements[11].f_Imag;

  a_Elements[11].f_Real = a_Elements[10].f_Real - f_Internal1; 
  a_Elements[11].f_Imag = a_Elements[10].f_Imag - f_Internal2;

  a_Elements[10].f_Real += f_Internal1;
  a_Elements[10].f_Imag += f_Internal2;

  f_Internal1 = a_Elements[13].f_Real;
  f_Internal2 = a_Elements[13].f_Imag;

  a_Elements[13].f_Real = a_Elements[12].f_Real - f_Internal1; 
  a_Elements[13].f_Imag = a_Elements[12].f_Imag - f_Internal2;

  a_Elements[12].f_Real += f_Internal1;
  a_Elements[12].f_Imag += f_Internal2;

  f_Internal1 = a_Elements[15].f_Real;
  f_Internal2 = a_Elements[15].f_Imag;

  a_Elements[15].f_Real = a_Elements[14].f_Real - f_Internal1; 
  a_Elements[15].f_Imag = a_Elements[14].f_Imag - f_Internal2;

  a_Elements[14].f_Real += f_Internal1;
  a_Elements[14].f_Imag += f_Internal2;

  f_Internal1 = a_Elements[2].f_Real;
  f_Internal2 = a_Elements[2].f_Imag;

  a_Elements[2].f_Real = a_Elements[0].f_Real - f_Internal1; 
  a_Elements[2].f_Imag = a_Elements[0].f_Imag - f_Internal2;

  a_Elements[0].f_Real += f_Internal1;
  a_Elements[0].f_Imag += f_Internal2;

  f_Internal1 = a_Elements[6].f_Real;
  f_Internal2 = a_Elements[6].f_Imag;

  a_Elements[6].f_Real = a_Elements[4].f_Real - f_Internal1; 
  a_Elements[6].f_Imag = a_Elements[4].f_Imag - f_Internal2;

  a_Elements[4].f_Real += f_Internal1;
  a_Elements[4].f_Imag += f_Internal2;

  f_Internal1 = a_Elements[10].f_Real;
  f_Internal2 = a_Elements[10].f_Imag;

  a_Elements[10].f_Real = a_Elements[8].f_Real - f_Internal1; 
  a_Elements[10].f_Imag = a_Elements[8].f_Imag - f_Internal2;

  a_Elements[8].f_Real += f_Internal1;
  a_Elements[8].f_Imag += f_Internal2;

  f_Internal1 = a_Elements[14].f_Real;
  f_Internal2 = a_Elements[14].f_Imag;

  a_Elements[14].f_Real = a_Elements[12].f_Real - f_Internal1; 
  a_Elements[14].f_Imag = a_Elements[12].f_Imag - f_Internal2;

  a_Elements[12].f_Real += f_Internal1;
  a_Elements[12].f_Imag += f_Internal2;

  f_Internal1 = -a_Elements[3].f_Imag;
  f_Internal2 = a_Elements[3].f_Real;

  a_Elements[3].f_Real = a_Elements[1].f_Real - f_Internal1; 
  a_Elements[3].f_Imag = a_Elements[1].f_Imag - f_Internal2;

  a_Elements[1].f_Real += f_Internal1;
  a_Elements[1].f_Imag += f_Internal2;

  f_Internal1 = -a_Elements[7].f_Imag;
  f_Internal2 = a_Elements[7].f_Real;

  a_Elements[7].f_Real = a_Elements[5].f_Real - f_Internal1; 
  a_Elements[7].f_Imag = a_Elements[5].f_Imag - f_Internal2;

  a_Elements[5].f_Real += f_Internal1;
  a_Elements[5].f_Imag += f_Internal2;

  f_Internal1 = -a_Elements[11].f_Imag;
  f_Internal2 = a_Elements[11].f_Real;

  a_Elements[11].f_Real = a_Elements[9].f_Real - f_Internal1; 
  a_Elements[11].f_Imag = a_Elements[9].f_Imag - f_Internal2;

  a_Elements[9].f_Real += f_Internal1;
  a_Elements[9].f_Imag += f_Internal2;

  f_Internal1 = -a_Elements[15].f_Imag;
  f_Internal2 = a_Elements[15].f_Real;

  a_Elements[15].f_Real = a_Elements[13].f_Real - f_Internal1; 
  a_Elements[15].f_Imag = a_Elements[13].f_Imag - f_Internal2;

  a_Elements[13].f_Real += f_Internal1;
  a_Elements[13].f_Imag += f_Internal2;

  f_Internal1 = a_Elements[4].f_Real;
  f_Internal2 = a_Elements[4].f_Imag;

  a_Elements[4].f_Real = a_Elements[0].f_Real - f_Internal1; 
  a_Elements[4].f_Imag = a_Elements[0].f_Imag - f_Internal2;

  a_Elements[0].f_Real += f_Internal1;
  a_Elements[0].f_Imag += f_Internal2;

  f_Internal1 = a_Elements[12].f_Real;
  f_Internal2 = a_Elements[12].f_Imag;

  a_Elements[12].f_Real = a_Elements[8].f_Real - f_Internal1; 
  a_Elements[12].f_Imag = a_Elements[8].f_Imag - f_Internal2;

  a_Elements[8].f_Real += f_Internal1;
  a_Elements[8].f_Imag += f_Internal2;

  f_Internal1 = f_k1 * (a_Elements[5].f_Real - a_Elements[5].f_Imag);
  f_Internal2 = f_k1 * (a_Elements[5].f_Imag + a_Elements[5].f_Real);

  a_Elements[5].f_Real = a_Elements[1].f_Real - f_Internal1; 
  a_Elements[5].f_Imag = a_Elements[1].f_Imag - f_Internal2;

  a_Elements[1].f_Real += f_Internal1;
  a_Elements[1].f_Imag += f_Internal2;

  f_Internal1 = f_k1 * (a_Elements[13].f_Real - a_Elements[13].f_Imag);
  f_Internal2 = f_k1 * (a_Elements[13].f_Imag + a_Elements[13].f_Real);

  a_Elements[13].f_Real = a_Elements[9].f_Real - f_Internal1; 
  a_Elements[13].f_Imag = a_Elements[9].f_Imag - f_Internal2;

  a_Elements[9].f_Real += f_Internal1;
  a_Elements[9].f_Imag += f_Internal2;

  f_Internal1 = -a_Elements[6].f_Imag;
  f_Internal2 = a_Elements[6].f_Real;

  a_Elements[6].f_Real = a_Elements[2].f_Real - f_Internal1; 
  a_Elements[6].f_Imag = a_Elements[2].f_Imag - f_Internal2;

  a_Elements[2].f_Real += f_Internal1;
  a_Elements[2].f_Imag += f_Internal2;

  f_Internal1 =-a_Elements[14].f_Imag;
  f_Internal2 =a_Elements[14].f_Real;

  a_Elements[14].f_Real = a_Elements[10].f_Real - f_Internal1; 
  a_Elements[14].f_Imag = a_Elements[10].f_Imag - f_Internal2;

  a_Elements[10].f_Real += f_Internal1;
  a_Elements[10].f_Imag += f_Internal2;

  f_Internal1 = -f_k1 * (a_Elements[7].f_Real + a_Elements[7].f_Imag);
  f_Internal2 = f_k1 * ( a_Elements[7].f_Real - a_Elements[7].f_Imag);

  a_Elements[7].f_Real = a_Elements[3].f_Real - f_Internal1; 
  a_Elements[7].f_Imag = a_Elements[3].f_Imag - f_Internal2;

  a_Elements[3].f_Real += f_Internal1;
  a_Elements[3].f_Imag += f_Internal2;

  f_Internal1 = -f_k1 * (a_Elements[15].f_Real + a_Elements[15].f_Imag);
  f_Internal2 = f_k1 * ( a_Elements[15].f_Real - a_Elements[15].f_Imag);

  a_Elements[15].f_Real = a_Elements[11].f_Real - f_Internal1; 
  a_Elements[15].f_Imag = a_Elements[11].f_Imag - f_Internal2;

  a_Elements[11].f_Real += f_Internal1;
  a_Elements[11].f_Imag += f_Internal2;

  f_Internal1 = a_Elements[8].f_Real;
  f_Internal2 = a_Elements[8].f_Imag;

  a_Elements[8].f_Real = a_Elements[0].f_Real - f_Internal1; 
  a_Elements[8].f_Imag = a_Elements[0].f_Imag - f_Internal2;

  a_Elements[0].f_Real += f_Internal1;
  a_Elements[0].f_Imag += f_Internal2;

  f_Internal1 = (f_k3 * a_Elements[9].f_Real) - (f_k2 * a_Elements[9].f_Imag);
  f_Internal2 = (f_k3 * a_Elements[9].f_Imag) + (f_k2 * a_Elements[9].f_Real);

  a_Elements[9].f_Real = a_Elements[1].f_Real - f_Internal1; 
  a_Elements[9].f_Imag = a_Elements[1].f_Imag - f_Internal2;

  a_Elements[1].f_Real += f_Internal1;
  a_Elements[1].f_Imag += f_Internal2;

  f_Internal1 = f_k1 * (a_Elements[10].f_Real - a_Elements[10].f_Imag);
  f_Internal2 = f_k1 * (a_Elements[10].f_Imag + a_Elements[10].f_Real);

  a_Elements[10].f_Real = a_Elements[2].f_Real - f_Internal1; 
  a_Elements[10].f_Imag = a_Elements[2].f_Imag - f_Internal2;

  a_Elements[2].f_Real += f_Internal1;
  a_Elements[2].f_Imag += f_Internal2;

  f_Internal1 = (f_k2 * a_Elements[11].f_Real) - (f_k3 * a_Elements[11].f_Imag);
  f_Internal2 = (f_k2 * a_Elements[11].f_Imag) + (f_k3 * a_Elements[11].f_Real);

  a_Elements[11].f_Real = a_Elements[3].f_Real - f_Internal1; 
  a_Elements[11].f_Imag = a_Elements[3].f_Imag - f_Internal2;

  a_Elements[3].f_Real += f_Internal1;
  a_Elements[3].f_Imag += f_Internal2;

  f_Internal1 = -a_Elements[12].f_Imag;
  f_Internal2 = a_Elements[12].f_Real;

  a_Elements[12].f_Real = a_Elements[4].f_Real - f_Internal1; 
  a_Elements[12].f_Imag = a_Elements[4].f_Imag - f_Internal2;

  a_Elements[4].f_Real += f_Internal1;
  a_Elements[4].f_Imag += f_Internal2;

  f_Internal1 = (-f_k2 * a_Elements[13].f_Real) - (f_k3 * a_Elements[13].f_Imag);
  f_Internal2 = (-f_k2 * a_Elements[13].f_Imag) + (f_k3 * a_Elements[13].f_Real);

  a_Elements[13].f_Real = a_Elements[5].f_Real - f_Internal1; 
  a_Elements[13].f_Imag = a_Elements[5].f_Imag - f_Internal2;

  a_Elements[5].f_Real += f_Internal1;
  a_Elements[5].f_Imag += f_Internal2;

  f_Internal1 = -f_k1 * (a_Elements[14].f_Real + a_Elements[14].f_Imag);
  f_Internal2 =  f_k1 * (a_Elements[14].f_Real - a_Elements[14].f_Imag);

  a_Elements[14].f_Real = a_Elements[6].f_Real - f_Internal1; 
  a_Elements[14].f_Imag = a_Elements[6].f_Imag - f_Internal2;

  a_Elements[6].f_Real += f_Internal1;
  a_Elements[6].f_Imag += f_Internal2;

  f_Internal1 = (-f_k3 * a_Elements[15].f_Real) - (f_k2 * a_Elements[15].f_Imag);
  f_Internal2 = (-f_k3 * a_Elements[15].f_Imag) + (f_k2 * a_Elements[15].f_Real);

  a_Elements[15].f_Real = a_Elements[7].f_Real - f_Internal1; 
  a_Elements[15].f_Imag = a_Elements[7].f_Imag - f_Internal2;

  a_Elements[7].f_Real += f_Internal1;
  a_Elements[7].f_Imag += f_Internal2;
}


// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h"
