/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "odpr_cml_mtrx.h"
#include "tue_common_libs.h"

/* Deactivate QA-C warnings 0488, 0489, 0491; Reviewer: S. Schwarzkopf; Date:
   09.12.2014;
   Reason: Matrix functions need pointer arithmetics due to interface issues and
   a high degree of optimisation. They are proven in use.
   Review-ID: 3942463 */
/* PRQA S 0488 EOF */
/* PRQA S 0489 EOF */
/* PRQA S 0491 EOF */

/*****************************************************************************
  LOCAL DEFINES
*****************************************************************************/
/*! number of elements in a 2x2 matrix */
#define MTRX_2X2_NOF_ELEMENTS 4
/*! number of elements in a 3x3 matrix */
#define MTRX_3X3_NOF_ELEMENTS 9

/*****************************************************************************
  LOCAL INLINES
*****************************************************************************/

/* Deactivate QA-C warning 3447; Reviewer: S. Schwarzkopf; Date: 05.12.2014;
   Reason: ALGO_INLINE is defined to have no external linkage on target
   platform.
   Review-ID: 3942463 */
/* PRQA S 3447 ODPR_CML_INLINES */

// ODPR_CML_INLINE boolean CML_b_invertMatrixCramer2(float32
// a_res[MTRX_2X2_NOF_ELEMENTS], float32 a_in[MTRX_2X2_NOF_ELEMENTS]);
// ODPR_CML_INLINE boolean CML_b_invertMatrixCramer3(float32
// a_res[MTRX_3X3_NOF_ELEMENTS], float32 a_in[MTRX_3X3_NOF_ELEMENTS]);

/* PRQA L:ODPR_CML_INLINES */

/*****************************************************************************
  FUNCTIONS
*****************************************************************************/

/*****************************************************************************
  Functionname:    CML_v_initMatrix                                      */ /*!

              @brief           Matrix initialization with a const value

              @description     This function initializes all the elements of the
            matrix
                               with a const value.
                               NOTE: RowNr and ColNr are expected to be values
            not
                               exceeding 8 bits.
                               CAUTION: u_RowNr*u_ColNr must not exceed
            A->Desc.maxsize

              @param[in,out]   p_Matrix :  matrix o be filled
                                           Range for p_Matrix->Desc.maxsize
            [Full
            range
            of uint16]
              @param[out]      f_Val :     value used for filling
                                           [Full range of float32]
              @param[in]       u_RowNr :   Row dimension of the matrix to be
            created
                                           [Full range of uint8]
              @param[in]       u_ColNr :   Column dimension of the matrix to be
            created
                                           [Full range of uint8]

              @return          void

              @author

            *****************************************************************************/
void ODPR_CML_v_InitMatrix(ODPR_CML_t_Matrix* p_Matrix,
                           uint32 u_RowNr,
                           uint32 u_ColNr,
                           float32 f_Val) {
    uint32 u_Idx;
    uint32 u_size = u_ColNr * u_RowNr;
    float32* p_MatrixData; /* pointer to matrix data */

#if ODPR_CML_MatrixBoundsCheckOn
    if (p_Matrix->Desc.maxsize >= u_size) {
#endif

        /* set new dimension */
        p_Matrix->Desc.col = (uint8)u_ColNr;
        p_Matrix->Desc.row = (uint8)u_RowNr;

        /* init elements */
        p_MatrixData = p_Matrix->pData;
        for (u_Idx = 0u; u_Idx < u_size; u_Idx++) {
            p_MatrixData[u_Idx] = f_Val;
        }

#if ODPR_CML_MatrixBoundsCheckOn
    } else {
        /* Return empty matrix */
        p_Matrix->Desc.col = (uint8)0;
        p_Matrix->Desc.row = (uint8)0;
        CML_ASSERT(b_FALSE);
    }
#endif
}

/*****************************************************************************
  Functionname:    ODPR_CML_v_createIdentityMatrix                           */ /*!

  @brief           Initialises matrix with identity matrix

  @description     This function initializes the given matrix with an
                   identity matrix of the provided size.
                   NOTE: Value for row/column is expected not to exceed 8 bits
                   CAUTION: Size of the matrix (rows x columns) must not
                   exceed A->Desc.maxsize

  @param[in,out]   p_Matrix :  matrix o be filled (square matrix)
                               Range for p_Matrix->Desc.maxsize [Full range of
uint16]
  @param[in]       u_Size :    no. of row/col square matrix (u_Size x u_Size)
                               [Full range of uint8]

  @return          void

  @author

*****************************************************************************/
void ODPR_CML_v_createIdentityMatrix(ODPR_CML_t_Matrix* p_Matrix,
                                     uint32 u_Size) {
    uint32 u_Idx;
    uint32 u_SizeSquare = TUE_CML_Sqr(u_Size);
    float32* p_MatrixData; /* pointer to matrix data */

#if ODPR_CML_MatrixBoundsCheckOn
    if (p_Matrix->Desc.maxsize >= u_SizeSquare) {
#endif

        /* set new dimension */
        p_Matrix->Desc.col = (uint8)u_Size;
        p_Matrix->Desc.row = (uint8)u_Size;

        /* fill with zero */
        ODPR_CML_v_InitMatrix(p_Matrix, u_Size, u_Size, 0.0F);

        /* set diagonal to one */
        p_MatrixData = p_Matrix->pData;
        for (u_Idx = 0u; u_Idx < u_SizeSquare;
             u_Idx += ((uint32)p_Matrix->Desc.col + 1u)) {
            p_MatrixData[u_Idx] = 1.0F;
        }

#if ODPR_CML_MatrixBoundsCheckOn
    } else {
        /* Set empty matrix */
        p_Matrix->Desc.col = (uint8)0;
        p_Matrix->Desc.row = (uint8)0;
        CML_ASSERT(b_FALSE);
    }
#endif
}

///*****************************************************************************
//  Functionname:    CML_v_addMatrices                                    */ /*!
//
//  @brief           Matrix addition (inplace/outplace). A or B can be same as
//  Res
//
//  @description     This function performs matrix addition (inplace/outplace)
//  of
//                   two matrices A and B with same dimesions and store the
//                   result
//                   in a resultant matrix.
//                   The matrix A or B can be same as resultant matrix.
//                   [Res] = [A] + [B]
//
//  @param[in]       p_MatrixA : First Addend matrix (source)
//                               Range for p_MatrixA->Desc.row [Full range of
//                               uint8]
//                               Range for p_MatrixA->Desc.col [Full range of
//                               uint8]
//                               Range for p_MatrixA->Desc.maxsize [Full range
//                               of uint16]
//                               Range for p_MatrixA->pData
//                               [Valid pointer with data in full range of
//                               float32]
//  @param[in]       p_MatrixB : Second Addend matrix (source)
//                               Range for p_MatrixB->Desc.row [Full range of
//                               uint8]
//                               Range for p_MatrixB->Desc.col [Full range of
//                               uint8]
//                               Range for p_MatrixB->Desc.maxsize [Full range
//                               of uint16]
//                               Range for p_MatrixB->pData
//                               [Valid pointer with data in full range of
//                               float32]
//                               Overflow may occur when one or more input
//                               values in both
//                               matrices are at the defined range extremities.
//
//  @param[out]      p_MatrixRes : Result Sum matrix
//
//  @return          void
//
//  @author
//
//*****************************************************************************/
// void CML_v_addMatrices(ODPR_CML_t_Matrix* p_MatrixRes, const
// ODPR_CML_t_Matrix* p_MatrixA, const ODPR_CML_t_Matrix* p_MatrixB)
//{
//  uint32 u_Idx;
//  uint32 u_size = (uint32)p_MatrixA->Desc.col * (uint32)p_MatrixA->Desc.row;
//
//  float32 * ODPR_CML_RESTRICT p_DataA = p_MatrixA->pData;       /* get pointer
//  to matrix data */
//  float32* ODPR_CML_RESTRICT p_DataB = p_MatrixB->pData;       /* get pointer
//  to matrix data */
//  float32* ODPR_CML_RESTRICT p_DataRes = p_MatrixRes->pData;   /* get pointer
//  to result matrix data */
//
//#if ODPR_CML_MatrixBoundsCheckOn
//  /* check if matrices are equal and Res is big enough */
//  if (  (p_MatrixA->Desc.col== p_MatrixB->Desc.col)
//     && (p_MatrixA->Desc.row == p_MatrixB->Desc.row)
//     && (p_MatrixRes->Desc.maxsize >= u_size))
//  {
//#endif
//
//    /* add elements */
//    if (p_DataA == p_DataRes)
//    {
//      for (u_Idx = 0u; u_Idx < u_size; u_Idx++)
//      {
//        p_DataRes[u_Idx] += p_DataB[u_Idx];
//      }
//    }
//    else if (p_DataB == p_DataRes)
//    {
//      for (u_Idx = 0u; u_Idx < u_size; u_Idx++)
//      {
//        p_DataRes[u_Idx] += p_DataA[u_Idx];
//      }
//    }
//    else
//    {
//      for (u_Idx = 0u; u_Idx < + u_size; u_Idx++)
//      {
//        p_DataRes[u_Idx] = p_DataA[u_Idx] + p_DataB[u_Idx];
//      }
//    }
//
//    /* create description for result matrix */
//    p_MatrixRes->Desc.col = p_MatrixA->Desc.col;
//    p_MatrixRes->Desc.row = p_MatrixA->Desc.row;
//
//
//#if ODPR_CML_MatrixBoundsCheckOn
//  }
//  else
//  {
//    /* set empty matrix */
//    p_MatrixRes->Desc.col = (uint8)0;
//    p_MatrixRes->Desc.row = (uint8)0;
//    CML_ASSERT(b_FALSE);
//  }
//#endif
//}
//
///*****************************************************************************
//  Functionname:    CML_v_subtractMatrices                               */ /*!
//
//  @brief           Matrix substraction Res = A - B
//
//  @description     This function performs matrix subtraction
//  (inplace/outplace)
//                   of two matrices A and B with same dimesions and store the
//                   result
//                   in a resultant matrix.
//                   [Res] = [A] - [B]
//
//  @param[in]       p_MatrixA : Minuend matrix
//                               Range for p_MatrixA->Desc.row [Full range of
//                               uint8]
//                               Range for p_MatrixA->Desc.col [Full range of
//                               uint8]
//                               Range for p_MatrixA->Desc.maxsize [Full range
//                               of uint16]
//                               Range for p_MatrixA->pData
//                               [Valid pointer with data in full range of
//                               float32]
//  @param[in]       p_MatrixB : Subtrahend matrix
//                               Range for p_MatrixB->Desc.row [Full range of
//                               uint8]
//                               Range for p_MatrixB->Desc.col [Full range of
//                               uint8]
//                               Range for p_MatrixB->Desc.maxsize [Full range
//                               of uint16]
//                               Range for p_MatrixB->pData
//                               [Valid pointer with data in full range of
//                               float32]
//                               Overflow may occur when one or more input
//                               values in both
//                               matrices are at the defined range extremities.
//  @param[out]      p_MatrixRes : Result Difference matrix
//
//  @return          void
//
//  @author
//
//*****************************************************************************/
// void CML_v_subtractMatrices(ODPR_CML_t_Matrix* p_MatrixRes, const
// ODPR_CML_t_Matrix* p_MatrixA, const ODPR_CML_t_Matrix* p_MatrixB)
//{
//  uint32 u_Idx;
//  uint32 u_size = (uint32)p_MatrixA->Desc.col * (uint32)p_MatrixA->Desc.row;
//
//  float32 * ODPR_CML_RESTRICT p_DataA = p_MatrixA->pData;       /* get pointer
//  to matrix data */
//  float32* ODPR_CML_RESTRICT p_DataB = p_MatrixB->pData;       /* get pointer
//  to matrix data */
//  float32* ODPR_CML_RESTRICT p_DataRes = p_MatrixRes->pData;   /* get pointer
//  to result matrix data */
//
//#if ODPR_CML_MatrixBoundsCheckOn
//  /* check if matrices are equal */
//  if (  (p_MatrixA->Desc.col== p_MatrixB->Desc.col)
//     && (p_MatrixA->Desc.row == p_MatrixB->Desc.row)
//     && (p_MatrixRes->Desc.maxsize >= p_MatrixA->Desc.col *
//     p_MatrixA->Desc.row) )
//  {
//#endif
//
//    /* substract elements */
//    if (p_DataA == p_DataRes)
//    {
//      for (u_Idx = 0u; u_Idx < u_size; u_Idx++)
//      {
//        p_DataRes[u_Idx] -= p_DataB[u_Idx];
//      }
//    }
//    else
//    {
//      for (u_Idx = 0u; u_Idx < u_size; u_Idx++)
//      {
//        p_DataRes[u_Idx] = p_DataA[u_Idx] - p_DataB[u_Idx];
//      }
//    }
//
//    /* create description for result matrix */
//    p_MatrixRes->Desc.col = p_MatrixA->Desc.col;
//    p_MatrixRes->Desc.row = p_MatrixA->Desc.row;
//
//#if ODPR_CML_MatrixBoundsCheckOn
//  }
//  else
//  {
//    /* set empty matrix */
//    p_MatrixRes->Desc.col = (uint8)0;
//    p_MatrixRes->Desc.row = (uint8)0;
//    CML_ASSERT(b_FALSE);
//  }
//#endif
//}
//
/*****************************************************************************
  Functionname:    ODPR_CML_v_MultiplyMatrices                               */ /*!

  @brief           Matrix multiplication

  @description     This function performs matrix multiplication (outplace)
                   of two matrices A and B and store the result in a
                   resultant matrix.
                   [Res] = [A] X [B]

  @param[in]       p_MatrixA : Multiplicand matrix (source)
                               Range for p_MatrixA->Desc.row [Full range of
uint8]
                               Range for p_MatrixA->Desc.col [Full range of
uint8]
                               Range for p_MatrixA->Desc.maxsize [Full range of
uint16]
                               Range for p_MatrixA->pData
                               [Valid pointer with data in full range of
float32]
  @param[in]       p_MatrixB : Multiplier matrix (source)
                               Range for p_MatrixB->Desc.row [Full range of
uint8]
                               Range for p_MatrixB->Desc.col [Full range of
uint8]
                               Range for p_MatrixB->Desc.maxsize [Full range of
uint16]
                               Range for p_MatrixB->pData
                               [Valid pointer with data in full range of
float32]
                               Overflow may occur when one or more input values
in both
                               matrices are at the defined range extremities.
  @param[out]      p_MatrixRes : Result Product matrix

  @return          void

  @author

*****************************************************************************/
void ODPR_CML_v_MultiplyMatrices(ODPR_CML_t_Matrix* p_MatrixRes,
                                 const ODPR_CML_t_Matrix* p_MatrixA,
                                 const ODPR_CML_t_Matrix* p_MatrixB) {
    uint8 u_RegAcol = p_MatrixA->Desc.col;
    uint8 u_RegBcol = p_MatrixB->Desc.col;
    uint8 u_RegArow = p_MatrixA->Desc.row;
    uint32 u_count;
    float32* p_DataA;
    float32* p_DataB;
    float32* p_LineA = p_MatrixA->pData;
    float32* p_LineB = p_MatrixB->pData;
    float32* p_LineBk;
    float32* p_DataRes = p_MatrixRes->pData; /* get pointer to matrix data */
    float32 f_Tmp;

#if ODPR_CML_MatrixBoundsCheckOn
    /* check if matrix dimensions fit */
    /* and matrices are distinct */
    if ((p_MatrixA->Desc.col != (uint8)0) &&
        (p_MatrixA->Desc.col == p_MatrixB->Desc.row) &&
        (p_MatrixA->pData != p_MatrixRes->pData) &&
        (p_MatrixB->pData != p_MatrixRes->pData) &&
        (p_MatrixRes->Desc.maxsize >=
         p_MatrixB->Desc.col * p_MatrixA->Desc.row)) {
#endif

        if ((u_RegAcol > 0U) && (u_RegBcol > 0U) && (u_RegArow > 0U)) {
            /* multiply matrix elements */
            u_count = u_RegArow;
            do {
                p_LineBk = p_LineB;
                do {
                    float32 f_DataA;
                    float32 f_DataB;

                    p_DataA = p_LineA;
                    p_DataB = p_LineBk;
                    p_LineBk++;
                    f_Tmp = 0.0F;
                    do {
                        f_DataA = *p_DataA;
                        f_DataB = *p_DataB;
                        p_DataA++;
                        p_DataB += u_RegBcol; /* goto next line */
                        /* Floating point multiply and add: y = a * b + d */
                        f_Tmp = TUE_CML_MultAdd(f_DataA, f_DataB, f_Tmp);

                        /* <ln_offset:+2 MISRA Rule 17.2: reviewer name: Daniel
                         * Meschenmoser date: 2012-09-12 reason: matrix
                         * multiplication runtime optimized by Uwe-Juergen
                         * Zunker */
                        /* <ln_offset:+1 MISRA Rule 17.3: reviewer name: Daniel
                         * Meschenmoser date: 2012-09-12 reason: matrix
                         * multiplication runtime optimized by Uwe-Juergen
                         * Zunker */
                    } while (p_DataA < (p_LineA + u_RegAcol));
                    *p_DataRes = f_Tmp;
                    p_DataRes++; /* go to next pRes element */
                    /* <ln_offset:+2 MISRA Rule 17.2: reviewer name: Daniel
                     * Meschenmoser date: 2012-09-12 reason: matrix
                     * multiplication runtime optimized by Uwe-Juergen Zunker */
                    /* <ln_offset:+1 MISRA Rule 17.3: reviewer name: Daniel
                     * Meschenmoser date: 2012-09-12 reason: matrix
                     * multiplication runtime optimized by Uwe-Juergen Zunker */
                } while (p_LineBk < (p_LineB + u_RegBcol));
                p_LineA += (uint32)u_RegAcol; /* go to next line of A */

                u_count--;
            } while (u_count > 0UL);
        } else {
            /* error */
        }

        /* create description for result matrix */
        p_MatrixRes->Desc.col = u_RegBcol;
        p_MatrixRes->Desc.row = u_RegArow;

#if ODPR_CML_MatrixBoundsCheckOn
    } else {
        /* set empty matrix */
        p_MatrixRes->Desc.col = (uint8)0;
        p_MatrixRes->Desc.row = (uint8)0;
        CML_ASSERT(b_FALSE);
    }
#endif
}

///*****************************************************************************
//  Functionname:    CML_v_multiplyMatricesToSymResult                    */ /*!
//
//  @brief           Matrix multiplication if result is symmetric
//
//  @description     This function performs matrix multiplication (outplace)
//                   of two matrices A and B and store the result in a
//                   resultant matrix, if the result is known to be a symmetric
//                   matrix.
//                   [Res] = [A] X [B],
//                   [A] is of the order MxN,
//                   [B] is of the order NxM,
//                   [Res] will of the order MxM.
//
//
//  @param[in]       p_MatrixA : Multiplicand matrix
//                               Range for p_MatrixA->Desc.row [Full range of
//                               uint8]
//                               Range for p_MatrixA->Desc.col [Full range of
//                               uint8]
//                               Range for p_MatrixA->Desc.maxsize [Full range
//                               of uint16]
//                               Range for p_MatrixA->pData
//                               [Valid pointer with data in full range of
//                               float32]
//  @param[in]       p_MatrixB : Multiplier matrix
//                               Range for p_MatrixB->Desc.row [Full range of
//                               uint8]
//                               Range for p_MatrixB->Desc.col [Full range of
//                               uint8]
//                               Range for p_MatrixB->Desc.maxsize [Full range
//                               of uint16]
//                               Range for p_MatrixB->pData
//                               [Valid pointer with data in full range of
//                               float32]
//                               Overflow may occur when one or more input
//                               values in both
//                               matrices are at the defined range extremities.
//  @param[out]      p_MatrixRes : symmetric result matrix
//
//  @return          void
//
//  @author
//
//*****************************************************************************/
// void CML_v_multiplyMatricesToSymResult(ODPR_CML_t_Matrix* ODPR_CML_RESTRICT
// p_MatrixRes, const ODPR_CML_t_Matrix* p_MatrixA, const ODPR_CML_t_Matrix*
// p_MatrixB)
//{
//  uint32 u_Idx1, u_Idx2;
//  float32* ODPR_CML_RESTRICT p_DataA;
//  float32* ODPR_CML_RESTRICT p_DataB;
//  float32* ODPR_CML_RESTRICT pLineA;
//  float32* ODPR_CML_RESTRICT p_DataRes = p_MatrixRes->pData;       /* get
//  pointer to matrix data */
//
//#if ODPR_CML_MatrixBoundsCheckOn
//  /* check if matrix dimensions fit */
//  /* and matrices are distinct */
//  if (  (p_MatrixA->Desc.col != (uint8)0)
//     && (p_MatrixA->Desc.col == p_MatrixB->Desc.row)
//     && (p_MatrixA->Desc.row == p_MatrixB->Desc.col)
//     && (p_MatrixA->pData != p_MatrixRes->pData)
//     && (p_MatrixB->pData != p_MatrixRes->pData)
//     && (p_MatrixRes->Desc.maxsize >= p_MatrixB->Desc.col *
//     p_MatrixA->Desc.row) )
//  {
//#endif
//
//    /* multiply matrix elements */
//    pLineA = p_MatrixA->pData;
//    for (u_Idx1=0UL; u_Idx1<p_MatrixA->Desc.row; u_Idx1++)
//    {
//      for (u_Idx2=0UL; u_Idx2<p_MatrixB->Desc.col; u_Idx2++)
//      {
//        p_DataB = &p_MatrixB->pData[u_Idx2];
//        *p_DataRes = 0.0F;
//        if (u_Idx1 > u_Idx2)
//        {
//          *p_DataRes = p_MatrixRes->pData[u_Idx1 + (u_Idx2 *
//          p_MatrixB->Desc.col)];
//        }
//        else
//        {
//          /* <ln_offset:+2 MISRA Rule 17.2: reviewer name: Daniel Meschenmoser
//          date: 2012-09-12 reason: matrix multiplication runtime optimized by
//          Uwe-Juergen Zunker */
//          /* <ln_offset:+1 MISRA Rule 17.3: reviewer name: Daniel Meschenmoser
//          date: 2012-09-12 reason: matrix multiplication runtime optimized by
//          Uwe-Juergen Zunker */
//          for (p_DataA = pLineA; p_DataA < (pLineA + p_MatrixA->Desc.col);
//          p_DataA++)
//          {
//            *p_DataRes += (*p_DataA) * (*p_DataB);
//            p_DataB+=p_MatrixB->Desc.col;        /* goto next line */
//          }
//        }
//        p_DataRes++;                   /* go to next pRes element */
//      }
//      pLineA += (uint32)p_MatrixA->Desc.col;  /* go to next line of A */
//    }
//
//    /* create description for result matrix */
//    p_MatrixRes->Desc.col = p_MatrixB->Desc.col;
//    p_MatrixRes->Desc.row = p_MatrixA->Desc.row;
//
//#if ODPR_CML_MatrixBoundsCheckOn
//  }
//  else
//  {
//    /* set empty matrix */
//    p_MatrixRes->Desc.col = (uint8)0;
//    p_MatrixRes->Desc.row = (uint8)0;
//    CML_ASSERT(b_FALSE);
//  }
//#endif
//}
//
///*****************************************************************************
//  Functionname:    CML_v_multiplyMatrixWithTranspose                    */ /*!
//
//  @brief           Matrix multiplication with transpose
//
//  @description     This function performs matrix multiplication (outplace)
//                   of matrix A with transpose of matrix B and store the
//                   result in a resultant matrix.
//                   [Res] = [A] X [B]'
//
//  @param[in]       p_MatrixA : Multiplicand matrix
//                               Range for p_MatrixA->Desc.row [Full range of
//                               uint8]
//                               Range for p_MatrixA->Desc.col [Full range of
//                               uint8]
//                               Range for p_MatrixA->Desc.maxsize [Full range
//                               of uint16]
//                               Range for p_MatrixA->pData
//                               [Valid pointer with data in full range of
//                               float32]
//  @param[in]       p_MatrixB : Transpose of which is the multiplier matrix
//                               Range for p_MatrixB->Desc.row [Full range of
//                               uint8]
//                               Range for p_MatrixB->Desc.col [Full range of
//                               uint8]
//                               Range for p_MatrixB->Desc.maxsize [Full range
//                               of uint16]
//                               Range for p_MatrixB->pData
//                               [Valid pointer with data in full range of
//                               float32]
//                               Overflow may occur when one or more input
//                               values in both
//                               matrices are at the defined range extremities.
//  @param[out]      p_MatrixRes : result matrix
//
//  @return          void
//
//  @author
//
//*****************************************************************************/
// void CML_v_multiplyMatrixWithTranspose(ODPR_CML_t_Matrix* p_MatrixRes, const
// ODPR_CML_t_Matrix* p_MatrixA, const ODPR_CML_t_Matrix* p_MatrixB)
//{
//  uint8  u_RegAcol = p_MatrixA->Desc.col;
//  uint8  u_RegBrow = p_MatrixB->Desc.row;
//  uint8  u_RegArow = p_MatrixA->Desc.row;
//  uint32 uIdx1, uIdx2;
//  float32* p_DataA;
//  float32* p_DataB;
//  float32* p_LineA=p_MatrixA->pData;
//  float32* p_LineB=p_MatrixB->pData;
//  float32* p_DataRes = p_MatrixRes->pData;         /* get pointer to matrix
//  data */
//  float32  f_Tmp;
//
//#if ODPR_CML_MatrixBoundsCheckOn
//  /* check if matrix dimensions fit */
//  /* and matrices are distinct */
//  if (  (p_MatrixA->Desc.col != (uint8)0)
//     && (p_MatrixA->Desc.col == p_MatrixB->Desc.col)
//     && (p_MatrixA->pData != p_MatrixRes->pData)
//     && (p_MatrixB->pData != p_MatrixRes->pData)
//     && (p_MatrixRes->Desc.maxsize >= p_MatrixB->Desc.col *
//     p_MatrixA->Desc.row) )
//  {
//#endif
//
//    if ((u_RegAcol > 0U) && (u_RegBrow > 0U) && (u_RegArow > 0U))
//    {
//      /* multiply matrix elements */
//      uIdx1 = u_RegArow;
//      do
//      {
//        uIdx2 = u_RegBrow;
//        p_DataB = p_LineB;
//        do
//        {
//          float32 fA;
//          float32 fB;
//
//          p_DataA = p_LineA;
//          f_Tmp = 0.0F;
//          do
//          {
//            fA = *p_DataA;               /* preread */
//            fB = *p_DataB;
//            p_DataA++;
//            p_DataB++;          /* goto next line */
//            /* Floating point multiply and add: y = a * b + d */
//            f_Tmp = CML_f_MultAdd(fA, fB, f_Tmp);
//          } while (p_DataA < (p_LineA + u_RegAcol));
//          *p_DataRes = f_Tmp;
//          p_DataRes++;                   /* go to next pRes element */
//
//          uIdx2--;
//        } while(uIdx2 > 0UL);
//        p_LineA += (uint32)u_RegAcol;  /* go to next line of A */
//
//        uIdx1--;
//      } while(uIdx1 > 0UL);
//    }
//    else
//    {
//      /* error */
//    }
//
//    /* create description for result matrix */
//    p_MatrixRes->Desc.col = u_RegBrow;
//    p_MatrixRes->Desc.row = u_RegArow;
//
//#if ODPR_CML_MatrixBoundsCheckOn
//  }
//  else
//  {
//    /* set empty matrix */
//    p_MatrixRes->Desc.col = (uint8)0;
//    p_MatrixRes->Desc.row = (uint8)0;
//    CML_ASSERT(b_FALSE);
//  }
//#endif
//}
//
///*****************************************************************************
//  Functionname:    CML_v_invertMatrix                                   */ /*!
//
//  @brief           Compute matrix inverse.
//
//  @description     Compute matrix inverse: Res = inv(A)
//                   Uses Gauss-Jordan elimination with partial pivoting.
//                   For matrices upto 3x3, determinant is found, singularity is
//                   checked and processing is done, whereas for higher order
//                   matrices, matrix singularity is determined during the
//                   processing.
//
//  @param[in,out]   p_MatrixA : matrix to be inversed.
//                               Range for p_MatrixA->Desc.row [Full range of
//                               uint8]
//                               Range for p_MatrixA->Desc.col [Full range of
//                               uint8]
//                               Range for p_MatrixA->Desc.maxsize [Full range
//                               of uint16]
//                               Range for p_MatrixA->pData
//                               [Valid pointer with data in full range of
//                               float32]
//                   The largest element on the each column MUST BE greater than
//                   the tolerance value (1e-10F). Otherwise function call will
//                   result in assertion fail.
//                   ATTENTION! This matrix is overwritten with the identity
//                   matrix.
//  @param[in,out]   p_MatrixRes : result matrix, containing the inverse of A
//
//  @return          void
//
//  @author
//
//*****************************************************************************/
// void CML_v_invertMatrix(ODPR_CML_t_Matrix* p_MatrixRes, ODPR_CML_t_Matrix*
// p_MatrixA)
//{
//  boolean bRet = b_FALSE;
//  uint32 u_Idx1, u_Idx2, u_col, u_row, u_pos1, u_pos2;
//  float32 f_Temp, f_MaxElem;
//  float32 f_PivElem       = 1.0F;
//  float32 f_InvPivElem    = 1.0F;
//  const float32 f_Tol     = 1e-10F;               /* tolerance */
//  float32* p_DataA        = p_MatrixA->pData;     /* get pointer to matrix
//  data */
//  float32* p_DataRes      = p_MatrixRes->pData;   /* get pointer to matrix
//  data */
//
//#if ODPR_CML_MatrixBoundsCheckOn
//  /* check if matrix is square */
//  if (  (p_MatrixA->Desc.col != (uint8)0)
//     && (p_MatrixA->Desc.col == p_MatrixA->Desc.row)
//     && (p_MatrixA->pData != p_MatrixRes->pData)
//     && (p_MatrixRes->Desc.maxsize >= (p_MatrixA->Desc.col *
//     p_MatrixA->Desc.row)) )
//  {
//#endif
//
//    if (p_MatrixA->Desc.col == (uint8)1U)
//    {
//      /* simple division for matrix size == 1 */
//      if(TUE_CML_IsNonZero(p_DataA[0]))
//      {
//        *p_DataRes = 1.0F / (*p_DataA);
//        bRet = b_TRUE;
//      }
//    }
//    else if (p_MatrixA->Desc.col == (uint8)2U)
//    {
//      /* Cramers Rule for matrix size == 2 */
//      bRet = CML_b_invertMatrixCramer2(p_DataRes, p_DataA);
//    }
//    else if(p_MatrixA->Desc.col == (uint8)3U)
//    {
//      /* Cramers Rule for matrix size == 3 */
//      bRet = CML_b_invertMatrixCramer3(p_DataRes, p_DataA);
//    }
//    else /* (A->Desc.col > (uint8)2U) */
//    {
//      /* Gauss-Jordan elimination with partial pivoting */
//
//      bRet = b_TRUE;
//      u_col = 0UL;
//      ODPR_CML_v_createIdentityMatrix(p_MatrixRes,
//      (uint32)p_MatrixA->Desc.row);  /* set result matrix as identity */
//      do
//      {
//        /* find largest element on the selected column */
//        /* and use as pivot element                    */
//        u_row = u_col;
//        u_pos1 = u_col + (u_col * (uint32)p_MatrixA->Desc.col);
//        f_MaxElem = 0.0F;
//        for (u_Idx1 = u_col; u_Idx1<(uint32)p_MatrixA->Desc.col; u_Idx1++)
//        {
//          f_Temp = CML_f_Abs(p_DataA[u_pos1]);
//          if (f_Temp > f_MaxElem)
//          {
//            f_MaxElem = f_Temp;
//            f_PivElem = p_DataA[u_pos1];
//            u_row = u_Idx1;
//          }
//          u_pos1 += (uint32)p_MatrixA->Desc.col;
//        }
//
//        /* exit routine if pivot element is very small => matrix not
//        inversible */
//        if (f_MaxElem >= f_Tol)
//        {
//          /* do pivoting to reduce column to identity matrix */
//          bRet = b_TRUE;
//
//          /* now swap rows to put the pivot element on the diagonal */
//          /* do the same operation for the result matrix */
//          if (u_row != u_col)
//          {
//            /* get pointer to matrix data */
//            u_pos1 = (uint32)p_MatrixA->Desc.col * u_row ;
//            u_pos2 = (uint32)p_MatrixA->Desc.col * u_col;
//
//            for (u_Idx1 = u_col; u_Idx1<(uint32)p_MatrixA->Desc.col; u_Idx1++)
//            /* only nonzero elements */
//            {
//              f_Temp = p_DataA[u_Idx1 + u_pos1];
//              p_DataA[u_Idx1 + u_pos1] = p_DataA[u_Idx1 + u_pos2];
//              p_DataA[u_Idx1 + u_pos2] = f_Temp;
//            }
//            for (u_Idx1 = 0U; u_Idx1<(uint32)p_MatrixA->Desc.col; u_Idx1++)
//            /* all elements */
//            {
//              f_Temp = p_DataRes[u_Idx1 + u_pos1];
//              p_DataRes[u_Idx1 + u_pos1] = p_DataRes[u_Idx1 + u_pos2];
//              p_DataRes[u_Idx1 + u_pos2] = f_Temp;
//            }
//          }
//
//          /* divide row by the pivot element => pivot becomes 1 */
//          /* do the same operation for the result matrix */
//          u_pos1 = u_col*(uint32)p_MatrixA->Desc.col;
//          f_InvPivElem = 1.0F/f_PivElem;
//          for (u_Idx1 = u_col; u_Idx1<(uint32)p_MatrixA->Desc.col; u_Idx1++)
//          /* only nonzero elements */
//          {
//            p_DataA[u_Idx1+u_pos1] *= f_InvPivElem;
//          }
//          for (u_Idx1 = 0UL; u_Idx1<(uint32)p_MatrixA->Desc.col; u_Idx1++)  /*
//          all elements */
//          {
//            p_DataRes[u_Idx1+u_pos1] *= f_InvPivElem;
//          }
//
//          /* now multiply the row by the right amount and substract from    */
//          /* each other row to make all the remaining elements in the pivot */
//          /* column zero                                                    */
//          for (u_Idx1 = 0UL; u_Idx1<(uint32)p_MatrixA->Desc.col; u_Idx1++)
//          /* loop other rows */
//          {
//            if (u_Idx1 != u_col)
//            {
//              u_pos1  = u_Idx1  *(uint32)p_MatrixA->Desc.col;
//              u_pos2  = u_col*(uint32)p_MatrixA->Desc.col;
//
//              /* use first element is row as scaling coefficient */
//              f_Temp = p_DataA[u_col + u_pos1];
//
//              /* substract pivot row multiplied by scaling from other row */
//              /* do the same operation for the result matrix */
//              for (u_Idx2 = u_col; u_Idx2<(uint32)p_MatrixA->Desc.col;
//              u_Idx2++)  /* only nonzero elements */
//              {
//                p_DataA[u_Idx2 + u_pos1] -= p_DataA[u_Idx2 + u_pos2] * f_Temp;
//              }
//              for (u_Idx2 = 0UL; u_Idx2<(uint32)p_MatrixA->Desc.col; u_Idx2++)
//              /* all elements */
//              {
//                p_DataRes[u_Idx2 + u_pos1] -= p_DataRes[u_Idx2 + u_pos2] *
//                f_Temp;
//              }
//            }
//          }
//
//          /* goto next column */
//          u_col++;
//        }
//
//        else
//        {
//          bRet = b_FALSE;
//        }
//      } while ( bRet && (u_col < (uint32)p_MatrixA->Desc.col));  /* quit if
//      finished or if matrix isn't inversible */
//    }
//
//#if ODPR_CML_MatrixBoundsCheckOn
//  }
//#endif
//
//  if (bRet)
//  {
//    /* create description for result matrix */
//    p_MatrixRes->Desc.col = p_MatrixA->Desc.col;
//    p_MatrixRes->Desc.row = p_MatrixA->Desc.row;
//  }
//  else
//  {
//    /* set empty matrix */
//    p_MatrixRes->Desc.col = (uint8)0;
//    p_MatrixRes->Desc.row = (uint8)0;
//    /* Deactivate QA-C warning 3112; Reviewer: S. Schwarzkopf;
//       Date: 04.12.2014; Reason: macro will be routed to assert() in
//       simulation
//       environment.
//       Review-ID: 3942463 */
//    /* PRQA S 3112 1 */
//    CML_ASSERT(b_FALSE);
//  }
//}
//
//
///*****************************************************************************
//  Functionname:    CML_b_invertMatrixCramer2                        */ /*!
//
//  @brief           Compute matrix inverse for matrix size 2x2
//
//  @description     This function compute matrix inverse: Res = inv(A)
//                   Uses Cramers Rule for matrix size 2x2.
//                   If A = |a b| and det(A) = (ad-bc),
//                          |c d|
//                   det(A) should be a non-zero value, then,
//                   Inverse of A, inv(A) = (1/det(A))* | d -b|
//                                                      |-c  a|
//
//  @param[in,out]   a_in : matrix to be inversed.
//                          [Full range of float32]
//                       ATTENTION! This matrix is overwritten with the identity
//                       matrix.
//  @param[in,out]   a_res : result matrix, containing the inverse of A
//
//  @return          void
//
//  @author
//
//*****************************************************************************/
// ODPR_CML_INLINE boolean CML_b_invertMatrixCramer2(float32
// a_res[MTRX_2X2_NOF_ELEMENTS], float32 a_in[MTRX_2X2_NOF_ELEMENTS])
//{
//  float32 f_temp  = 0.0f;   /* temporary variable */
//  boolean b_ret   = b_FALSE;  /* return value */
//
//
//  /* Cramers Rule for matrix size == 2 */
//  f_temp = (a_in[0] * a_in[3]) - (a_in[1] * a_in[2]);
//  if(TUE_CML_IsNonZero(f_temp))
//  {
//    f_temp = 1.0F / f_temp;
//
//    a_res[0] = a_in[3]  * f_temp;
//    a_res[1] = -a_in[1] * f_temp;
//    a_res[2] = -a_in[2] * f_temp;
//    a_res[3] = a_in[0]  * f_temp;
//
//    b_ret = b_TRUE;
//  } /* if(TUE_CML_IsNonZero(f_temp)) */
//
//
//  return b_ret;
//} /* b_InvertMatrixCramer2() */
//
//
///*****************************************************************************
//  Functionname:    CML_b_invertMatrixCramer3                                */
//  /*!
//
//  @brief           Compute matrix inverse for matrix size 3x3
//
//  @description     This function compute matrix inverse: Res = inv(A)
//                   Uses Cramers Rule for matrix size 3x3.
//                   If A is a 3x3 matrix and det(A) is the determinant of
//                   matrix,
//                   which must be non-zero, then inverse of A will be equal to
//                   Adjoint matrix of A divided by det(A).
//
//  @param[in,out]   a_in : matrix to be inversed.
//                          [Full range of float32]
//                       ATTENTION! This matrix is overwritten with the identity
//                       matrix.
//  @param[in,out]   a_res : result matrix, containing the inverse of A
//
//  @return          void
//
//  @author
//
//*****************************************************************************/
// ODPR_CML_INLINE boolean CML_b_invertMatrixCramer3(float32
// a_res[MTRX_3X3_NOF_ELEMENTS], float32 a_in[MTRX_3X3_NOF_ELEMENTS])
//{
//  float32 f_temp  = 0.0f;   /* temporary variable */
//  boolean b_ret   = b_FALSE;  /* return value */
//
//
//  /* Cramers Rule for matrix size == 3 */
//  f_temp = (((a_in[0] * a_in[4]) - (a_in[3] * a_in[1])) * a_in[8]) +
//           (((a_in[3] * a_in[7]) - (a_in[6] * a_in[4])) * a_in[2]) +
//           (((a_in[6] * a_in[1]) - (a_in[0] * a_in[7])) * a_in[5]);
//
//  if(TUE_CML_IsNonZero(f_temp))
//  {
//    f_temp = 1.0F / f_temp;
//
//    a_res[0] = ((a_in[4] * a_in[8]) - (a_in[5] * a_in[7])) * f_temp;
//    a_res[1] = ((a_in[2] * a_in[7]) - (a_in[1] * a_in[8])) * f_temp;
//    a_res[2] = ((a_in[1] * a_in[5]) - (a_in[2] * a_in[4])) * f_temp;
//    a_res[3] = ((a_in[5] * a_in[6]) - (a_in[3] * a_in[8])) * f_temp;
//    a_res[4] = ((a_in[0] * a_in[8]) - (a_in[2] * a_in[6])) * f_temp;
//    a_res[5] = ((a_in[2] * a_in[3]) - (a_in[0] * a_in[5])) * f_temp;
//    a_res[6] = ((a_in[3] * a_in[7]) - (a_in[4] * a_in[6])) * f_temp;
//    a_res[7] = ((a_in[1] * a_in[6]) - (a_in[0] * a_in[7])) * f_temp;
//    a_res[8] = ((a_in[0] * a_in[4]) - (a_in[1] * a_in[3])) * f_temp;
//
//    b_ret = b_TRUE;
//  } /* if(TUE_CML_IsNonZero(f_temp)) */
//
//
//  return b_ret;
//} /* b_InvertMatrixCramer3() */
//
//
///*****************************************************************************
//  Functionname:    CML_v_scaleMatrix                                    */ /*!
//
//  @brief           Matrix multiplication with scalar
//
//  @description     This function does an inplace matrix multiplication
//                   with a given scalar. If [A] is the matrix, and p is
//                   the scalar, then,
//                   [A] = p * [A]
//
//  @param[in,out]   p_MatrixA : Matrix o be multiplied
//                               Range for p_MatrixA->Desc.row [Full range of
//                               uint8]
//                               Range for p_MatrixA->Desc.col [Full range of
//                               uint8]
//                               Range for p_MatrixA->pData
//                               [Valid pointer with data in full range of
//                               float32]
//                               Overflow may occur when one or more input
//                               values in both
//                               matrices are at the defined range extremities.
//  @param[in]       f_Val :     scalar
//                               [Full range of float32]
//
//  @return          void
//
//  @author
//
//*****************************************************************************/
// void CML_v_scaleMatrix(ODPR_CML_t_Matrix* p_MatrixA, float32 f_Val)
//{
//  uint32 u_Idx;
//  uint32 u_size = (uint32)p_MatrixA->Desc.col * (uint32)p_MatrixA->Desc.row;
//  float32* p_DataA;        /* get pointer to matrix data */
//
//  /* scale elements */
//  p_DataA = p_MatrixA->pData;
//  for (u_Idx = 0u; u_Idx < u_size; u_Idx++)
//  {
//    p_DataA[u_Idx] *= f_Val;
//  }
//}
//
/*****************************************************************************
  Functionname:    ODPR_CML_v_copyMatrix                                     */ /*!

  @brief           Matrix copy

  @description     This function copies data from one matrix to another.

  @param[in]       p_MatrixA : matrix to be copied
                               Range for p_MatrixA->Desc.row [Full range of
uint8]
                               Range for p_MatrixA->Desc.col [Full range of
uint8]
                               Range for p_MatrixA->Desc.maxsize [Full range of
uint16]
                               Range for p_MatrixA->pData
                               [Valid pointer with data in full range of
float32]
  @param[out]      p_MatrixRes : destination matrix
                                 Range for p_MatrixRes->Desc.maxsize [Full range
of uint16]

  @return          void

  @author

*****************************************************************************/
void ODPR_CML_v_copyMatrix(ODPR_CML_t_Matrix* p_MatrixRes,
                           const ODPR_CML_t_Matrix* p_MatrixA) {
    uint32 u_Idx;
    uint32 size = (uint32)p_MatrixA->Desc.col * (uint32)p_MatrixA->Desc.row;

    float32* ODPR_CML_RESTRICT p_DataA =
        p_MatrixA->pData; /* get pointer to matrix data */
    float32* ODPR_CML_RESTRICT p_DataRes =
        p_MatrixRes->pData; /* get pointer to result matrix data */

#if ODPR_CML_MatrixBoundsCheckOn
    if (p_MatrixRes->Desc.maxsize >= size) {
#endif

        /* copy elements */
        for (u_Idx = 0u; u_Idx < size; u_Idx++) {
            p_DataRes[u_Idx] = p_DataA[u_Idx];
        }

        /* create description for result matrix */
        p_MatrixRes->Desc.col = p_MatrixA->Desc.col;
        p_MatrixRes->Desc.row = p_MatrixA->Desc.row;

#if ODPR_CML_MatrixBoundsCheckOn
    } else {
        /* set empty matrix */
        p_MatrixRes->Desc.col = (uint8)0;
        p_MatrixRes->Desc.row = (uint8)0;
        CML_ASSERT(b_FALSE);
    }
#endif
}

///*****************************************************************************
//  Functionname:    CML_v_copyArrayToSymMatrix                           */ /*!
//
//  @brief           Fill matrix with data from array
//
//  @description     This functon copies data from an array to a matrix.
//                   The resultant matrix is a symmetric matrix.
//
//  @param[in]       p_Data :  data to be copied
//                             [Valid pointer with data in full range of
//                             float32]
//  @param[out]      p_MatrixRes : destination matrix
//  @param[in]       u_RowNr : no. of rows
//                             [Full range of uint8]
//  @param[in]       u_ColNr : no. of columns
//                             [Full range of uint8]
//
//  @return          void
//
//  @author
//
//*****************************************************************************/
// void CML_v_copyArrayToSymMatrix(ODPR_CML_t_Matrix* ODPR_CML_RESTRICT
// p_MatrixRes, uint32 u_RowNr, uint32 u_ColNr, const float32* ODPR_CML_RESTRICT
// p_Data)
//{
//  uint32 u_Idx1, u_Idx2;
//
//  /* get pointer to matrix data */
//  const float32* p_MatrixData = p_Data;
//
//#if ODPR_CML_MatrixBoundsCheckOn
//  /*Check for Square Matrix*/
//  if(u_RowNr == u_ColNr)
//  {
//#endif
//  /* copy elements */
//  for (u_Idx1=0UL; u_Idx1<u_RowNr; u_Idx1++)
//  {
//    for (u_Idx2=u_Idx1; u_Idx2<u_ColNr; u_Idx2++)
//    {
//      p_MatrixRes->pData[u_Idx2 + (u_Idx1*u_RowNr)] = *p_MatrixData;
//      if (u_Idx2 != u_Idx1)
//      {
//        p_MatrixRes->pData[u_Idx1 + (u_Idx2*u_RowNr)] = *p_MatrixData;
//      }
//      p_MatrixData++;
//    }
//  }
//    p_MatrixRes->Desc.row = (uint8)u_RowNr;
//    p_MatrixRes->Desc.col = (uint8)u_ColNr;
//#if ODPR_CML_MatrixBoundsCheckOn
//  } /*if(u_RowNr == u_ColNr)*/
//  else
//  {
//    /* set empty matrix */
//    p_MatrixRes->Desc.col = (uint8)0;
//    p_MatrixRes->Desc.row = (uint8)0;
//    CML_ASSERT(b_FALSE);
//  }
//#endif
//}
//
///*****************************************************************************
//  Functionname:    CML_v_choleskyMatrix                                 */ /*!
//
//  @brief           Compute Cholesky factor of a p.d. hermitian matrix
//
//  @description     The Cholesky factorization is decomposing the hermitian
//                   positive definite matrix (A) into product of a lower
//                   triangular
//                   matrix (L) and its conjugare transpose (L*).
//                   A = LL*
//                   This function returns this lower triangular square root of
//                   the positive definite real symmetric matrix, no exception
//                   handling for any kind of rank deficiency rather direct
//                   regularization
//                   (associated m-file for unit testing: slow_chol2.m)
//
//  @param[in,out]   p_MatrixA :   matrix whose Cholesky type square root is
//  wanted
//                                 Range for p_MatrixA->Desc.row [Full range of
//                                 uint8]
//                                 Range for p_MatrixA->Desc.col [Full range of
//                                 uint8]
//                                 Range for p_MatrixA->Desc.maxsize [Full range
//                                 of uint16]
//                                 Range for p_MatrixA->pData
//                                 [Valid pointer with data in full range of
//                                 float32]
//  @param[in,out]   p_MatrixRes : lower triangular result matrix
//                                 Range for p_MatrixRes->Desc.maxsize [Full
//                                 range of uint16]
//
//  @return          void
//
//  @author
//
//*****************************************************************************/
// void CML_v_choleskyMatrix(ODPR_CML_t_Matrix* p_MatrixRes, const
// ODPR_CML_t_Matrix* p_MatrixA)
//{
//  uint32   u_Idx1, u_Idx2, u_Idx3, u_ColRowNr;
//  float32  f_Tol = (float32) 1e-10F;
//  float32  f_Temp;
//  float32* p_DataRes = p_MatrixRes->pData;   /* get pointer to matrix data */
//
//#if ODPR_CML_MatrixBoundsCheckOn
//  /* check if matrix is square */
//  if (  (p_MatrixA->Desc.col != (uint8)0)
//    && (p_MatrixA->Desc.col == p_MatrixA->Desc.row)
//    && ((p_MatrixA->pData) != (p_MatrixRes->pData))
//    && (p_MatrixRes->Desc.maxsize >= (p_MatrixA->Desc.col *
//    p_MatrixA->Desc.row)) )
//  {
//#endif
//    CML_v_CopyMatrix(p_MatrixRes, p_MatrixA);
//    u_ColRowNr = (uint32) p_MatrixA->Desc.col; // ColNr = RowNr
//    for (u_Idx2=0UL; u_Idx2<u_ColRowNr; u_Idx2++)
//    {
//      if (p_DataRes[u_Idx2 + (u_Idx2*u_ColRowNr)] < f_Tol) // should never
//      happen for p.d. input matrix
//      {
//        p_DataRes[u_Idx2 + (u_Idx2*u_ColRowNr)]  = f_Tol; // just "brutal"
//        pseudo regularization good for KAFI
//      }
//    }
//    for (u_Idx1=0UL; u_Idx1<u_ColRowNr; u_Idx1++)
//    {
//      for (u_Idx2=0UL; u_Idx2<u_Idx1; u_Idx2++)
//      {
//        f_Temp = p_DataRes[u_Idx2 + (u_Idx1*u_ColRowNr)];
//        for (u_Idx3=0UL; u_Idx3<u_Idx2; u_Idx3++)
//        {
//          f_Temp -= p_DataRes[u_Idx3 + (u_Idx1*u_ColRowNr)] * p_DataRes[u_Idx3
//          + (u_Idx2*u_ColRowNr)];
//        }
//        p_DataRes[u_Idx2 + (u_Idx1*u_ColRowNr)] = f_Temp/p_DataRes[u_Idx2 +
//        (u_Idx2*u_ColRowNr)];
//        p_DataRes[u_Idx1 + (u_Idx2*u_ColRowNr)] = 0.0F;
//      }
//      f_Temp = p_DataRes[u_Idx1 + (u_Idx1*u_ColRowNr)];
//      for (u_Idx3=0UL; u_Idx3<u_Idx1; u_Idx3++)
//      {
//        f_Temp -= p_DataRes[u_Idx3 + (u_Idx1*u_ColRowNr)] * p_DataRes[u_Idx3 +
//        (u_Idx1*u_ColRowNr)];
//      }
//      if ( f_Temp > f_Tol)
//      {
//        p_DataRes[u_Idx1 + (u_Idx1*u_ColRowNr)] = (float32)
//        CML_f_Sqrt(f_Temp);
//      }
//      else
//      { // this should never happen for p.d. input matrix
//        p_DataRes[u_Idx1 + (u_Idx1*u_ColRowNr)] = f_Tol; // it is caused by
//        rank-deficient input matrix
//      }
//
//    } // for (u_Idx1=0UL; u_Idx1<ColRowNr; u_Idx1++)
//#if ODPR_CML_MatrixBoundsCheckOn
//  }
//  else
//  {
//    CML_ASSERT(b_FALSE);
//  }
//#endif
//}
//
//
///*****************************************************************************
//  Functionname:    CML_v_lowTriaInvMatrix                               */ /*!
//
//  @brief           Inverse of p.d. lower triangular matrix  by forward
//  substitution
//
//  @description     The method of choice for inversion of positive definite
//                   symmetric matrices consists of Cholesky factorization
//                   followed
//                   by forward substitution which is efficiently implemented
//                   in this function, no exception handling in case of
//                   indefinite
//                   input rather brute force regularization which is
//                   appropriate
//                   for square-root Kalman filter applications but does a poor
//                   job in approximating the Pseudo-Inverse for least-squares.
//
//  @param[in,out]   p_MatrixA :   lower triangular p.d. matrix (upper
//  triangular entries are irrelevant)
//                                 Range for p_MatrixA->Desc.row [Full range of
//                                 uint8]
//                                 Range for p_MatrixA->Desc.col [Full range of
//                                 uint8]
//                                 Range for p_MatrixA->Desc.maxsize [Full range
//                                 of uint16]
//                                 Range for p_MatrixA->pData
//                                 [Valid pointer with data in full range of
//                                 float32]
//  @param[in,out]   p_MatrixRes : lower triangular inverse of A (upper
//  triangular entries are invalid)
//                                 Range for p_MatrixRes->Desc.maxsize [Full
//                                 range of uint16]
//
//  @return          void
//
//  @author
//
//*****************************************************************************/
// void CML_v_lowTriaInvMatrix(ODPR_CML_t_Matrix* p_MatrixRes, const
// ODPR_CML_t_Matrix* p_MatrixA)
//{
//  uint32   u_Idx3, u_Idx2, u_Idx1, u_ColRowNr;
//  float32  f_Tol = (float32) 1e-10F;
//  float32  f_Temp;
//  float32* p_DataA   = p_MatrixA->pData;     /* get pointer to matrix data */
//  float32* p_DataRes = p_MatrixRes->pData;   /* get pointer to matrix data */
//
//#if ODPR_CML_MatrixBoundsCheckOn
//  /* check if matrix is square */
//  if (  (p_MatrixA->Desc.col != (uint8)0)
//    && (p_MatrixA->Desc.col == p_MatrixA->Desc.row)
//    && (p_DataA != p_DataRes)
//    && (p_MatrixRes->Desc.maxsize >= (p_MatrixA->Desc.col *
//    p_MatrixA->Desc.row)) )
//  {
//#endif
//    u_ColRowNr = (uint32) p_MatrixA->Desc.col; // ColNr = RowNr
//    for (u_Idx1=0UL; u_Idx1<u_ColRowNr; u_Idx1++)
//    {
//      if (p_DataA[u_Idx1 + (u_Idx1*u_ColRowNr)] < f_Tol)
//      {
//        p_DataA[u_Idx1 + (u_Idx1*u_ColRowNr)] = f_Tol; // regularization
//      }
//      p_DataA[u_Idx1 + (u_Idx1*u_ColRowNr)] = 1.0F/p_DataA[u_Idx1 +
//      (u_Idx1*u_ColRowNr)];  // in place predivision for efficiency
//      for (u_Idx2=u_Idx1; u_Idx2<u_ColRowNr; u_Idx2++)
//      {
//        p_DataRes[u_Idx2 + (u_Idx1*u_ColRowNr)] = 0.0F;
//      }
//    }
//    for (u_Idx1=0UL; u_Idx1<u_ColRowNr; u_Idx1++)
//    {
//      for (u_Idx2=u_Idx1; u_Idx2<u_ColRowNr; u_Idx2++)
//      {
//        f_Temp = 0.0F;
//        for (u_Idx3=0UL; u_Idx3<u_Idx2; u_Idx3++)
//        {
//          f_Temp += p_DataA[u_Idx3 + (u_Idx2*u_ColRowNr)] * p_DataRes[u_Idx1 +
//          (u_Idx3*u_ColRowNr)];
//        }
//        if (u_Idx2 == u_Idx1)
//        {
//          p_DataRes[u_Idx1 + (u_Idx2*u_ColRowNr)] = p_DataA[u_Idx2 +
//          (u_Idx2*u_ColRowNr)] * (1.0F - f_Temp);
//        }
//        else
//        {
//          p_DataRes[u_Idx1 + (u_Idx2*u_ColRowNr)] = p_DataA[u_Idx2 +
//          (u_Idx2*u_ColRowNr)] * (-f_Temp);
//        }
//      }
//    }
//    /* create description for result matrix */
//  p_MatrixRes->Desc.col = p_MatrixA->Desc.col;
//  p_MatrixRes->Desc.row = p_MatrixA->Desc.row;
//#if ODPR_CML_MatrixBoundsCheckOn
//  }
//  else
//  {
//    CML_ASSERT(b_FALSE);
//  }
//#endif
//}
//
///*****************************************************************************
//  Functionname:    CML_v_lowTriaSqrMatrix                               */ /*!
//
//  @brief           Square of Cholesky Res = A*transpose(A), with A lower
//  triangular
//
//  @description     The method of choice for squaring of positive definite
//                   symmetric matrices is from Cholesky method. This function
//                   computes the product of the lower traingular matrix and its
//                   transpose.
//
//
//  @param[in,out]   p_MatrixA :   lower triangular p.d. matrix
//                                 (upper triangular entries are irrelevant)
//                                 Range for p_MatrixA->Desc.row [Full range of
//                                 uint8]
//                                 Range for p_MatrixA->Desc.col [Full range of
//                                 uint8]
//                                 Range for p_MatrixA->Desc.maxsize [Full range
//                                 of uint16]
//                                 Range for p_MatrixA->pData
//                                 [Valid pointer with data in full range of
//                                 float32]
//  @param[in,out]   p_MatrixRes : square matrix A*transpose(A) symmetric and
//                                 positive definite
//                                 Range for p_MatrixRes->Desc.maxsize [Full
//                                 range of uint16]
//
//  @return          void
//
//  @author
//
//*****************************************************************************/
// void CML_v_lowTriaSqrMatrix(ODPR_CML_t_Matrix* p_MatrixRes, const
// ODPR_CML_t_Matrix* p_MatrixA)
//{
//  uint32   u_Idx1,u_Idx2,u_Idx3,u_ColRowNr;
//  float32  f_Temp;
//  float32* p_DataA = p_MatrixA->pData;
//  float32* p_DataRes = p_MatrixRes->pData;
//
//#if ODPR_CML_MatrixBoundsCheckOn
//  /* check if matrix dimensions fit */
//  /* and matrices are distinct */
//  if (  (p_MatrixA->Desc.col != (uint8)0)
//    && (p_MatrixA->Desc.col == p_MatrixA->Desc.row)
//    && (p_MatrixA->pData != p_MatrixRes->pData)
//    && (p_MatrixRes->Desc.maxsize >= p_MatrixA->Desc.col *
//    p_MatrixA->Desc.row) )
//  {
//#endif
//    u_ColRowNr = p_MatrixA->Desc.row;
//    for (u_Idx1=0UL; u_Idx1<u_ColRowNr; u_Idx1++)
//    {
//      for (u_Idx2=0UL; u_Idx2<=u_Idx1; u_Idx2++)
//      {
//        f_Temp = 0.0F;
//        for (u_Idx3=0UL; (u_Idx3<=u_Idx1)&&(u_Idx3<=u_Idx2); u_Idx3++)
//        {
//          f_Temp += p_DataA[u_Idx3 + (u_Idx1*u_ColRowNr)] * p_DataA[u_Idx3 +
//          (u_Idx2*u_ColRowNr)];
//        }
//        p_DataRes[u_Idx1 + (u_Idx2*u_ColRowNr)] = f_Temp;
//        p_DataRes[u_Idx2 + (u_Idx1*u_ColRowNr)] = f_Temp;  //redundant at the
//        diagonal but faster on pipelined DSPs
//      }
//    }
//
//    /* create description for result matrix */
//    p_MatrixRes->Desc.col = p_MatrixA->Desc.col;
//    p_MatrixRes->Desc.row = p_MatrixA->Desc.row;
//
//#if ODPR_CML_MatrixBoundsCheckOn
//  }
//  else
//  {
//    /* set empty matrix */
//    p_MatrixRes->Desc.col = (uint8)0;
//    p_MatrixRes->Desc.row = (uint8)0;
//  }
//#endif
//}
//
///*****************************************************************************
//  Functionname:    CML_v_uppTriaSqrMatrix                               */ /*!
//
//  @brief           Res = A*transpose(A), with A upper triangular
//  @description     This function computes the product of the upper triangular
//                   matrix and its transpose
//
//  @param[in,out]   p_MatrixA   : upper triangular p.d. matrix
//                                 (lower triangular entries are irrelevant)
//                                 Range for p_MatrixA->Desc.row [Full range of
//                                 uint8]
//                                 Range for p_MatrixA->Desc.col [Full range of
//                                 uint8]
//                                 Range for p_MatrixA->Desc.maxsize [Full range
//                                 of uint16]
//                                 Range for p_MatrixA->pData
//                                 [Valid pointer with data in full range of
//                                 float32]
//  @param[in,out]   p_MatrixRes : square matrix A*transpose(A) symmetric and
//                                 positive definite
//                                 Range for p_MatrixRes->Desc.maxsize [Full
//                                 range of uint16]
//
//  @return          void
//
//  @author
//
//*****************************************************************************/
// void CML_v_uppTriaSqrMatrix(ODPR_CML_t_Matrix* p_MatrixRes, const
// ODPR_CML_t_Matrix* p_MatrixA)
//{
//  uint32   u_Idx1, u_Idx2, u_Idx3, u_ColRowNr;
//  float32  f_Temp;
//  float32* p_DataA   = p_MatrixA->pData;
//  float32* p_DataRes = p_MatrixRes->pData;
//
//#if ODPR_CML_MatrixBoundsCheckOn
//  /* check if matrix dimensions fit */
//  /* and matrices are distinct */
//  if (  (p_MatrixA->Desc.col != (uint8)0)
//    && (p_MatrixA->Desc.col == p_MatrixA->Desc.row)
//    && (p_MatrixA->pData != p_MatrixRes->pData)
//    && (p_MatrixRes->Desc.maxsize >= p_MatrixA->Desc.col *
//    p_MatrixA->Desc.row) )
//  {
//#endif
//    u_ColRowNr = p_MatrixA->Desc.row;
//    for (u_Idx1=0UL; u_Idx1<u_ColRowNr; u_Idx1++)
//    {
//      for (u_Idx2=0UL; u_Idx2<=u_Idx1; u_Idx2++)  // <lint left brace
//      {
//        f_Temp = 0.0F;
//        for (u_Idx3=u_Idx1; u_Idx3<u_ColRowNr; u_Idx3++)
//        {
//          f_Temp += p_DataA[u_Idx3 + (u_Idx1*u_ColRowNr)] * p_DataA[u_Idx3 +
//          (u_Idx2*u_ColRowNr)];
//        }
//        p_DataRes[u_Idx1 + (u_Idx2*u_ColRowNr)] = f_Temp;
//        p_DataRes[u_Idx2 + (u_Idx1*u_ColRowNr)] = f_Temp;  //redundant at the
//        diagonal but faster on pipelined DSPs
//      }
//    }
//
//    /* create description for result matrix */
//    p_MatrixRes->Desc.col = p_MatrixA->Desc.col;
//    p_MatrixRes->Desc.row = p_MatrixA->Desc.row;
//
//#if ODPR_CML_MatrixBoundsCheckOn
//  }
//  else
//  {
//    /* set empty matrix */
//    p_MatrixRes->Desc.col = (uint8)0;
//    p_MatrixRes->Desc.row = (uint8)0;
//    CML_ASSERT(b_FALSE);
//  }
//#endif
//}
//
/* ***************************************************************************
  Functionname:    CML_v_transposeMatrix                                */ /*!

                 @brief           Matrix transposition, outplace

                 @description     This function calculates the transpose of a
               matrix
                                  (outplace)
                                  If
                                      |a b|
                                  A = |c d| , then transpose of A is

                                         |a c|
                                  A(t) = |b d|

                 @param[in]       p_MatrixA :    matrix o be transposed (source)
                                                 Range for p_MatrixA->Desc.row
               [Full
               range
               of uint8]
                                                 Range for p_MatrixA->Desc.col
               [Full
               range
               of uint8]
                                                 Range for
               p_MatrixA->Desc.maxsize
               [Full
               range of uint16]
                                                 Range for p_MatrixA->pData
                                                 [Valid pointer with data in
               full
               range of
               float32]
                 @param[out]      p_MatrixRes :  destination matrix
                                                 Range for
               p_MatrixRes->Desc.maxsize
               [Full
               range of uint16]
                                                 Range for p_MatrixRes->pData
                                                 [Valid pointer with data in
               full
               range of
               float32]

                 @return          none

                 @author
               ****************************************************************************
               */
void ODPR_CML_v_TransposeMatrix(ODPR_CML_t_Matrix* p_MatrixRes,
                                const ODPR_CML_t_Matrix* p_MatrixA) {
    uint32 u_Idx2, u_Idx1;

    float32* ODPR_CML_RESTRICT p_DataA =
        p_MatrixA->pData; /* get pointer to matrix data */
    float32* ODPR_CML_RESTRICT p_DataRes =
        p_MatrixRes->pData; /* get pointer to result matrix data */

#if ODPR_CML_MatrixBoundsCheckOn
    /* check if source is different from destination */
    if ((p_MatrixA->pData != p_MatrixRes->pData) &&
        (p_MatrixRes->Desc.maxsize >=
         p_MatrixA->Desc.col * p_MatrixA->Desc.row)) {
#endif

        /* transpose while copying */
        for (u_Idx1 = 0UL; u_Idx1 < p_MatrixA->Desc.row; u_Idx1++) {
            for (u_Idx2 = 0UL; u_Idx2 < p_MatrixA->Desc.col; u_Idx2++) {
                p_DataRes[u_Idx1 + (u_Idx2 * (uint32)p_MatrixA->Desc.row)] =
                    *p_DataA;
                p_DataA++;
            }
        }
        /* create description for result matrix */
        p_MatrixRes->Desc.col = p_MatrixA->Desc.row;
        p_MatrixRes->Desc.row = p_MatrixA->Desc.col;

#if ODPR_CML_MatrixBoundsCheckOn
    } else {
        /* set empty matrix */
        p_MatrixRes->Desc.col = (uint8)0;
        p_MatrixRes->Desc.row = (uint8)0;
        CML_ASSERT(b_FALSE);
    }
#endif
}

ODPR_CML_t_Matrix ODPR_CML_initMatrixHeader(uint32 u_ColNr,
                                            uint32 u_RowNr,
                                            float32* p_f_MtrxData) {
    ODPR_CML_t_Matrix Ret;
    Ret.Desc.col = (uint8)u_ColNr;
    Ret.Desc.row = (uint8)u_RowNr;
    Ret.Desc.maxsize = (uint16)(u_RowNr * u_ColNr);
    Ret.pData = p_f_MtrxData;

    return Ret;
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */