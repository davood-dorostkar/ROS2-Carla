/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
#include "odpr_cml_adapted.h"
#include "tue_common_libs.h"
#include "odpr_cml_mtrx.h"

// StAr
/*****************************************************************************
  Functionname:    LCF_b_InvertMatrixCramer2                        */ /*!

                             @brief           Compute matrix inverse for matrix
                           size 2x2

                             @description     This function compute matrix
                           inverse: Res =
                           inv(A)
                                              Uses Cramers Rule for matrix size
                           2x2.
                                              If A = |a b| and det(A) = (ad-bc),
                                                     |c d|
                                              det(A) should be a non-zero value,
                           then,
                                              Inverse of A, inv(A) = (1/det(A))*
                           | d -b|
                                                                                 |-c  a|

                             @param[in,out]   a_in : matrix to be inversed.
                                                     [Full range of float32]
                                                  ATTENTION! This matrix is
                           overwritten
                           with the
                           identity matrix.
                             @param[in,out]   a_res : result matrix, containing
                           the
                           inverse of A

                             @return          void


                           *****************************************************************************/
boolean_T LCF_b_InvertMatrixCramer2(float32 a_res[LCF_MTRX_2X2_NOF_ELEMENTS],
                                    float32 a_in[LCF_MTRX_2X2_NOF_ELEMENTS]) {
    float32 f_temp = 0.0f;   /* temporary variable */
    boolean_T b_ret = FALSE; /* return value */

    /* Cramers Rule for matrix size == 2 */
    f_temp = (a_in[0] * a_in[3]) - (a_in[1] * a_in[2]);
    if (TUE_CML_IsNonZero(f_temp))  // GeGr: Modified precision
    {
        f_temp = 1.0F / f_temp;

        a_res[0] = a_in[3] * f_temp;
        a_res[1] = -a_in[1] * f_temp;
        a_res[2] = -a_in[2] * f_temp;
        a_res[3] = a_in[0] * f_temp;

        b_ret = TRUE;
    } /* if(TUE_CML_IsNonZero(f_temp)) */

    return b_ret;
} /* LCF_b_InvertMatrixCramer2() */

/*****************************************************************************
  Functionname:    LCF_b_InvertMatrixCramer3                                */ /*!

     @brief           Compute matrix inverse for matrix size 3x3

     @description     This function compute matrix inverse: Res = inv(A)
                      Uses Cramers Rule for matrix size 3x3.
                      If A is a 3x3 matrix and det(A) is the determinant of
   matrix,
                      which must be non-zero, then inverse of A will be equal to
                      Adjoint matrix of A divided by det(A).

     @param[in,out]   a_in : matrix to be inversed.
                             [Full range of float32]
                          ATTENTION! This matrix is overwritten with the
   identity
   matrix.
     @param[in,out]   a_res : result matrix, containing the inverse of A

     @return          void


   *****************************************************************************/
boolean_T LCF_b_InvertMatrixCramer3(float32 a_res[LCF_MTRX_3X3_NOF_ELEMENTS],
                                    float32 a_in[LCF_MTRX_3X3_NOF_ELEMENTS]) {
    float32 f_temp;          /* temporary variable */
    boolean_T b_ret = FALSE; /* return value */

    /* Cramers Rule for matrix size == 3 */
    f_temp = (((a_in[0] * a_in[4]) - (a_in[3] * a_in[1])) * a_in[8]) +
             (((a_in[3] * a_in[7]) - (a_in[6] * a_in[4])) * a_in[2]) +
             (((a_in[6] * a_in[1]) - (a_in[0] * a_in[7])) * a_in[5]);

    if (TUE_CML_IsNonZero(f_temp))  // AlFe: Modified precision
    {
        f_temp = 1.0F / f_temp;

        a_res[0] = ((a_in[4] * a_in[8]) - (a_in[5] * a_in[7])) * f_temp;
        a_res[1] = ((a_in[2] * a_in[7]) - (a_in[1] * a_in[8])) * f_temp;
        a_res[2] = ((a_in[1] * a_in[5]) - (a_in[2] * a_in[4])) * f_temp;
        a_res[3] = ((a_in[5] * a_in[6]) - (a_in[3] * a_in[8])) * f_temp;
        a_res[4] = ((a_in[0] * a_in[8]) - (a_in[2] * a_in[6])) * f_temp;
        a_res[5] = ((a_in[2] * a_in[3]) - (a_in[0] * a_in[5])) * f_temp;
        a_res[6] = ((a_in[3] * a_in[7]) - (a_in[4] * a_in[6])) * f_temp;
        a_res[7] = ((a_in[1] * a_in[6]) - (a_in[0] * a_in[7])) * f_temp;
        a_res[8] = ((a_in[0] * a_in[4]) - (a_in[1] * a_in[3])) * f_temp;

        b_ret = TRUE;
    } /* if(TUE_CML_IsNonZero(f_temp)) */

    return b_ret;
} /* LCF_b_InvertMatrixCramer3() */

boolean_T LCF_CML_CalcInvertMatrix_M(ODPR_CML_t_Matrix* p_MatrixA,
                                     float32* p_DataA,
                                     float32* p_DataRes) {
    boolean_T bRet = TRUE;
    uint32 u_Idx1, u_Idx2, u_col, u_row, u_pos1, u_pos2;
    float32 f_Temp, f_MaxElem;
    float32 f_PivElem = 1.0F;
    float32 f_InvPivElem = 1.0F;
    const float32 f_Tol = 1e-20F;
    /* tolerance */  // StAR - avoid division by zero

    u_col = 0UL;
    do {
        /* find largest element on the selected column */
        /* and use as pivot element                    */
        u_row = u_col;
        u_pos1 = u_col + (u_col * (uint32)p_MatrixA->Desc.col);
        f_MaxElem = 0.0F;
        for (u_Idx1 = u_col; u_Idx1 < (uint32)p_MatrixA->Desc.col; u_Idx1++) {
            f_Temp = TUE_CML_Abs(p_DataA[u_pos1]);
            if (f_Temp > f_MaxElem) {
                f_MaxElem = f_Temp;
                f_PivElem = p_DataA[u_pos1];
                u_row = u_Idx1;
            }
            u_pos1 += (uint32)p_MatrixA->Desc.col;
        }

        /* exit routine if pivot element is very small => matrix not
         * inversible */
        if (f_MaxElem >= f_Tol) {
            /* do pivoting to reduce column to identity matrix */
            bRet = TRUE;

            /* now swap rows to put the pivot element on the diagonal */
            /* do the same operation for the result matrix */
            if (u_row != u_col) {
                /* get pointer to matrix data */
                u_pos1 = (uint32)p_MatrixA->Desc.col * u_row;
                u_pos2 = (uint32)p_MatrixA->Desc.col * u_col;

                for (u_Idx1 = u_col; u_Idx1 < (uint32)p_MatrixA->Desc.col;
                     u_Idx1++) /* only nonzero elements */
                {
                    f_Temp = p_DataA[u_Idx1 + u_pos1];
                    p_DataA[u_Idx1 + u_pos1] = p_DataA[u_Idx1 + u_pos2];
                    p_DataA[u_Idx1 + u_pos2] = f_Temp;
                }
                for (u_Idx1 = 0U; u_Idx1 < (uint32)p_MatrixA->Desc.col;
                     u_Idx1++) /* all elements */
                {
                    f_Temp = p_DataRes[u_Idx1 + u_pos1];
                    p_DataRes[u_Idx1 + u_pos1] = p_DataRes[u_Idx1 + u_pos2];
                    p_DataRes[u_Idx1 + u_pos2] = f_Temp;
                }
            }

            /* divide row by the pivot element => pivot becomes 1 */
            /* do the same operation for the result matrix */
            u_pos1 = u_col * (uint32)p_MatrixA->Desc.col;
            f_InvPivElem = 1.0F / f_PivElem;
            for (u_Idx1 = u_col; u_Idx1 < (uint32)p_MatrixA->Desc.col;
                 u_Idx1++) /* only nonzero elements */
            {
                p_DataA[u_Idx1 + u_pos1] *= f_InvPivElem;
            }
            for (u_Idx1 = 0UL; u_Idx1 < (uint32)p_MatrixA->Desc.col;
                 u_Idx1++) /* all elements */
            {
                p_DataRes[u_Idx1 + u_pos1] *= f_InvPivElem;
            }

            /* now multiply the row by the right amount and substract
             * from    */
            /* each other row to make all the remaining elements in the
             * pivot */
            /* column zero */
            for (u_Idx1 = 0UL; u_Idx1 < (uint32)p_MatrixA->Desc.col;
                 u_Idx1++) /* loop other rows */
            {
                if (u_Idx1 != u_col) {
                    u_pos1 = u_Idx1 * (uint32)p_MatrixA->Desc.col;
                    u_pos2 = u_col * (uint32)p_MatrixA->Desc.col;

                    /* use first element is row as scaling coefficient
                     */
                    f_Temp = p_DataA[u_col + u_pos1];

                    /* substract pivot row multiplied by scaling from
                     * other row */
                    /* do the same operation for the result matrix */
                    for (u_Idx2 = u_col; u_Idx2 < (uint32)p_MatrixA->Desc.col;
                         u_Idx2++) /* only nonzero elements */
                    {
                        p_DataA[u_Idx2 + u_pos1] -=
                            p_DataA[u_Idx2 + u_pos2] * f_Temp;
                    }
                    for (u_Idx2 = 0UL; u_Idx2 < (uint32)p_MatrixA->Desc.col;
                         u_Idx2++) /* all elements */
                    {
                        p_DataRes[u_Idx2 + u_pos1] -=
                            p_DataRes[u_Idx2 + u_pos2] * f_Temp;
                    }
                }
            }

            /* goto next column */
            u_col++;
        }

        else {
            bRet = FALSE;
        }
    } while (bRet && (u_col < (uint32)p_MatrixA->Desc.col)); /* quit if
                                                                finished or
                                                                if matrix
                                                                isn't
                                                                inversible */
    return bRet;
}

/*****************************************************************************
  Functionname:    CML_v_InvertMatrix                                   */ /*!

                 @brief           Compute matrix inverse.

                 @description     Compute matrix inverse: Res = inv(A)
                                  Uses Gauss-Jordan elimination with partial
               pivoting.
                                  For matrices upto 3x3, determinant is found,
               singularity
               is
                                  checked and processing is done, whereas for
               higher
               order
                                  matrices, matrix singularity is determined
               during
               the
                                  processing.

                 @param[in,out]   p_MatrixA : matrix to be inversed.
                                              Range for p_MatrixA->Desc.row
               [Full
               range of
               uint8]
                                              Range for p_MatrixA->Desc.col
               [Full
               range of
               uint8]
                                              Range for p_MatrixA->Desc.maxsize
               [Full
               range of uint16]
                                              Range for p_MatrixA->pData
                                              [Valid pointer with data in full
               range
               of
               float32]
                                  The largest element on the each column MUST BE
               greater
               than
                                  the tolerance value (1e-10F). Otherwise
               function
               call
               will
                                  result in assertion fail.
                                  ATTENTION! This matrix is overwritten with the
               identity
               matrix.
                 @param[in,out]   p_MatrixRes : result matrix, containing the
               inverse
               of A

                 @return          void


               *****************************************************************************/
void LCF_CML_v_InvertMatrix(ODPR_CML_t_Matrix* p_MatrixRes,
                            ODPR_CML_t_Matrix* p_MatrixA) {
    boolean_T bRet = FALSE;
    float32* p_DataA = p_MatrixA->pData;     /* get pointer to matrix data */
    float32* p_DataRes = p_MatrixRes->pData; /* get pointer to matrix data */

#if ODPR_CML_MatrixBoundsCheckOn
    /* check if matrix is square */
    if ((p_MatrixA->Desc.col != (uint8)0) &&
        (p_MatrixA->Desc.col == p_MatrixA->Desc.row) &&
        (p_MatrixA->pData != p_MatrixRes->pData) &&
        (p_MatrixRes->Desc.maxsize >=
         (p_MatrixA->Desc.col * p_MatrixA->Desc.row))) {
#endif

        if (p_MatrixA->Desc.col == (uint8)1U) {
            /* simple division for matrix size == 1 */
            if (TUE_CML_IsNonZero(p_DataA[0])) {
                *p_DataRes = 1.0F / (*p_DataA);
                bRet = TRUE;
            }
        } else if (p_MatrixA->Desc.col == (uint8)2U) {
            /* Cramers Rule for matrix size == 2 */
            bRet = LCF_b_InvertMatrixCramer2(p_DataRes, p_DataA);
        } else if (p_MatrixA->Desc.col == (uint8)3U) {
            /* Cramers Rule for matrix size == 3 */
            bRet = LCF_b_InvertMatrixCramer3(p_DataRes, p_DataA);
        } else /* (A->Desc.col > (uint8)2U) */
        {
            /* Gauss-Jordan elimination with partial pivoting */

            bRet = TRUE;
            // u_col = 0UL;
            ODPR_CML_v_createIdentityMatrix(
                p_MatrixRes, (uint32)p_MatrixA->Desc
                                 .row); /* set result matrix as identity */
            bRet = LCF_CML_CalcInvertMatrix_M(p_MatrixA, p_DataA, p_DataRes);
        }

#if ODPR_CML_MatrixBoundsCheckOn
    }
#endif

    if (bRet) {
        /* create description for result matrix */
        p_MatrixRes->Desc.col = p_MatrixA->Desc.col;
        p_MatrixRes->Desc.row = p_MatrixA->Desc.row;
    } else {
        /* set empty matrix */
        p_MatrixRes->Desc.col = (uint8)0;
        p_MatrixRes->Desc.row = (uint8)0;

        // CML_ASSERT(FALSE);
    }
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */