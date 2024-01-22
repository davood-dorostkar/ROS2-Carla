/** \defgroup matrix Matrix Utilities
 * \brief Utilities for manipulating matrixes
 *
 * \addtogroup matrix
 *  \{
 * \file        tue_prv_common_matrix.c
 */
/* PRQA S 0292 ++ */ /* MKS */
                     /*
                      */
/* PRQA S 0292 -- */ /* MKS */
                     /*
                      * <br>=====================================================<br>
                      * <b>Copyright 2013 by Tuerme.</b>
                      *
                      *  All rights reserved. Property of Tuerme.<br>
                      *  Restricted rights to use, duplicate or disclose of this code<br>
                      *  are granted through contract.
                      * <br>=====================================================<br>
                      */

/**********************************************************************/
/* note that this is a library file so MISRA 1503 has been suppressed */
/**********************************************************************/

/*==================[inclusions]============================================*/
#include "tue_prv_common_matrix.h"
#include "tue_prv_common_matrix_int.h"
#include "tue_prv_fusion_math.h"
#include "TueObjFusn_ErrorCodes.h"
#include "TueObjFusn_AAU_Codes.h"
#include "tue_prv_error_management.h"
#include "tue_prv_fusion_memory.h"
#include "tue_prv_fusion_math.h"
#include "TueObjFusn_TrackableConstants.h"

/*==================[macros]================================================*/
/*==================[type definitions]======================================*/

/*==================[forward declarations]==================================*/
/*==================[symbolic constants]====================================*/
#define ObjFusn_START_SEC_ROM

LOCAL CONST(uint16, ObjFusn_CONST)
    au16SymMatrixLookup[TUEOBJFUSN_MATRIX_SIZE][TUEOBJFUSN_MATRIX_SIZE] = {
        {0u, 1u, 3u, 6u, 10u, 15u},     {1u, 2u, 4u, 7u, 11u, 16u},
        {3u, 4u, 5u, 8u, 12u, 17u},     {6u, 7u, 8u, 9u, 13u, 18u},
        {10u, 11u, 12u, 13u, 14u, 19u}, {15u, 16u, 17u, 18u, 19u, 20u}};

/* Static constant matrix used for "fast" zeroing */
/* PRQA S 3218 1 */ /**< Static buffer */
LOCAL CONST(stf32Matrix_t, ObjFusn_CONST) sf32ZeroMatrix = {
    {{0.0f}, {0.0f}}, 0u, 0u};
/* PRQA S 0686 */ /* variable initialisers */

/* Static constant vector used for "fast" zeroing */
/* PRQA S 3218 1 */ /**< Static buffer */
LOCAL CONST(stf32Vec_t, ObjFusn_CONST) sf32ZeroVector = {{0.0f}, 0u, 0u};
/* PRQA S 0686 */ /* variable initialisers */
#define ObjFusn_STOP_SEC_ROM

/*==================[functions]============================================*/
#define ObjFusn_START_SEC_CODE

/**
 * @brief performs matrix addition C = A+B
 *
 * - does not change matrix A or B
 * - matrices can be of dimensions nxm, 1 <= n,m <= @ref TUEOBJFUSN_MATRIX_SIZE
 * - matrices A and B must have same dimensions
 * - memory of matrix C must be allocated before function call
 * - C may point to A and/or B
 * - Complexity: O(n^2).
 * @param[in]  A   input matrix A
 * @param[in]  B   input matrix B
 * @param[out] C   output matrix C = A + B
 * @return TRUE (ok) or FALSE (error occured)
 */
uint32 f32MatAdd(CONSTP2CONST(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) A,
                 CONSTP2CONST(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) B,
                 CONSTP2VAR(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT)
                     C) /* PRQA S 1503 */
{
#if STD_OFF == TUE_PRV_RUNTIME_ERROR_CHECK
    const uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
#else
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
#endif

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if ((NULL_PTR == A) || (NULL_PTR == B) || (NULL_PTR == C)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32MATADD);
    } else
#endif
        if ((A->nCols != B->nCols) || (A->nRows != B->nRows)) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_SIZE_NOT_EQUAL;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_MATRIX_SIZE_NOT_EQUAL,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32MATADD);
    } else if ((A->nCols > TUEOBJFUSN_MATRIX_SIZE) ||
               (A->nRows > TUEOBJFUSN_MATRIX_SIZE)) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32MATADD);
    } else
#endif
    {
        /* we allow C and A or B to point to same data => do not erase C */
        C->nRows = A->nRows;
        C->nCols = A->nCols;
#if TUEOBJFUSN_MATRIX_SIZE != 6
        for (uint16 uRow = 0u; uRow < C->nRows; uRow++) {
            float32 *iRowA;
            float32 *iRowB;
            float32 *iRowC;
            iRowA = &A->data[uRow][0];
            iRowB = &B->data[uRow][0];
            iRowC = &C->data[uRow][0];

            for (uint16 uCol = 0u; uCol < C->nCols; uCol++) {
                iRowC[uCol] = iRowA[uCol] + iRowB[uCol];
            }
        }
#else
        const float32 *iRowA;
        const float32 *iRowB;
        float32 *iRowC;
        iRowA = &A->data[0][0];
        iRowB = &B->data[0][0];
        iRowC = &C->data[0][0];

        iRowC[0] = iRowA[0] + iRowB[0];
        iRowC[1] = iRowA[1] + iRowB[1];
        iRowC[2] = iRowA[2] + iRowB[2];
        iRowC[3] = iRowA[3] + iRowB[3];
        iRowC[4] = iRowA[4] + iRowB[4];
        iRowC[5] = iRowA[5] + iRowB[5];

        iRowA = &A->data[1][0];
        iRowB = &B->data[1][0];
        iRowC = &C->data[1][0];

        iRowC[0] = iRowA[0] + iRowB[0];
        iRowC[1] = iRowA[1] + iRowB[1];
        iRowC[2] = iRowA[2] + iRowB[2];
        iRowC[3] = iRowA[3] + iRowB[3];
        iRowC[4] = iRowA[4] + iRowB[4];
        iRowC[5] = iRowA[5] + iRowB[5];

        iRowA = &A->data[2][0];
        iRowB = &B->data[2][0];
        iRowC = &C->data[2][0];

        iRowC[0] = iRowA[0] + iRowB[0];
        iRowC[1] = iRowA[1] + iRowB[1];
        iRowC[2] = iRowA[2] + iRowB[2];
        iRowC[3] = iRowA[3] + iRowB[3];
        iRowC[4] = iRowA[4] + iRowB[4];
        iRowC[5] = iRowA[5] + iRowB[5];

        iRowA = &A->data[3][0];
        iRowB = &B->data[3][0];
        iRowC = &C->data[3][0];

        iRowC[0] = iRowA[0] + iRowB[0];
        iRowC[1] = iRowA[1] + iRowB[1];
        iRowC[2] = iRowA[2] + iRowB[2];
        iRowC[3] = iRowA[3] + iRowB[3];
        iRowC[4] = iRowA[4] + iRowB[4];
        iRowC[5] = iRowA[5] + iRowB[5];

        iRowA = &A->data[4][0];
        iRowB = &B->data[4][0];
        iRowC = &C->data[4][0];

        iRowC[0] = iRowA[0] + iRowB[0];
        iRowC[1] = iRowA[1] + iRowB[1];
        iRowC[2] = iRowA[2] + iRowB[2];
        iRowC[3] = iRowA[3] + iRowB[3];
        iRowC[4] = iRowA[4] + iRowB[4];
        iRowC[5] = iRowA[5] + iRowB[5];

        iRowA = &A->data[5][0];
        iRowB = &B->data[5][0];
        iRowC = &C->data[5][0];

        iRowC[0] = iRowA[0] + iRowB[0];
        iRowC[1] = iRowA[1] + iRowB[1];
        iRowC[2] = iRowA[2] + iRowB[2];
        iRowC[3] = iRowA[3] + iRowB[3];
        iRowC[4] = iRowA[4] + iRowB[4];
        iRowC[5] = iRowA[5] + iRowB[5];
#endif
    }

    return u32Success;
}

/**
 * @brief performs matrix substraction C = A-B
 *
 * - does not change matrix A or B
 * - matrices can be of dimensions nxm, 1 <= n,m <= @ref TUEOBJFUSN_MATRIX_SIZE
 * - matrices A and B must have same dimensions
 * - memory of matrix C must be allocated before function call
 * - C may point to A and/or B
 * - Complexity: O(n^2).
 * @param[in]  A   input matrix A
 * @param[in]  B   input matrix B
 * @param[out] C   output matrix C = A - B
 * @return TRUE (ok) or FALSE (error occured)
 */
/* PRQA S 1532 2 */ /* Library Function */
uint32 f32MatSub(CONSTP2CONST(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) A,
                 CONSTP2CONST(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) B,
                 CONSTP2VAR(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT)
                     C) /* PRQA S 1503 */
{
    // uint16 uRow;
    // uint16 uCol;
    uint32 u32Success;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if ((NULL_PTR == A) || (NULL_PTR == B) || (NULL_PTR == C)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32MATSUB);
    } else
#endif
        if ((A->nCols != B->nCols) || (A->nRows != B->nRows)) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_SIZE_NOT_EQUAL;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_MATRIX_SIZE_NOT_EQUAL,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32MATSUB);
    } else if ((A->nCols > TUEOBJFUSN_MATRIX_SIZE) ||
               (A->nRows > TUEOBJFUSN_MATRIX_SIZE)) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_MATRIX_SIZE_NOT_EQUAL,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE);
    } else
#endif
    {
        /* we allow C and A or B to point to same data => do not erase C */
        C->nRows = A->nRows;
        C->nCols = A->nCols;
#if TUEOBJFUSN_MATRIX_SIZE != 6
        for (uint16 uRow = 0u; uRow < C->nRows; uRow++) {
            float32 *iRowA;
            float32 *iRowB;
            float32 *iRowC;
            iRowA = &A->data[uRow][0];
            iRowB = &B->data[uRow][0];
            iRowC = &C->data[uRow][0];

            for (uint16 uCol = 0u; uCol < C->nCols; uCol++) {
                iRowC[uCol] = iRowA[uCol] - iRowB[uCol];
            }
        }
#else
        // this is a special calculation design for time saving
        const float32 *iRowA;
        const float32 *iRowB;
        float32 *iRowC;
        iRowA = &A->data[0][0];
        iRowB = &B->data[0][0];
        iRowC = &C->data[0][0];

        iRowC[0] = iRowA[0] - iRowB[0];
        iRowC[1] = iRowA[1] - iRowB[1];
        iRowC[2] = iRowA[2] - iRowB[2];
        iRowC[3] = iRowA[3] - iRowB[3];
        iRowC[4] = iRowA[4] - iRowB[4];
        iRowC[5] = iRowA[5] - iRowB[5];

        iRowA = &A->data[1][0];
        iRowB = &B->data[1][0];
        iRowC = &C->data[1][0];

        iRowC[0] = iRowA[0] - iRowB[0];
        iRowC[1] = iRowA[1] - iRowB[1];
        iRowC[2] = iRowA[2] - iRowB[2];
        iRowC[3] = iRowA[3] - iRowB[3];
        iRowC[4] = iRowA[4] - iRowB[4];
        iRowC[5] = iRowA[5] - iRowB[5];

        iRowA = &A->data[2][0];
        iRowB = &B->data[2][0];
        iRowC = &C->data[2][0];

        iRowC[0] = iRowA[0] - iRowB[0];
        iRowC[1] = iRowA[1] - iRowB[1];
        iRowC[2] = iRowA[2] - iRowB[2];
        iRowC[3] = iRowA[3] - iRowB[3];
        iRowC[4] = iRowA[4] - iRowB[4];
        iRowC[5] = iRowA[5] - iRowB[5];

        iRowA = &A->data[3][0];
        iRowB = &B->data[3][0];
        iRowC = &C->data[3][0];

        iRowC[0] = iRowA[0] - iRowB[0];
        iRowC[1] = iRowA[1] - iRowB[1];
        iRowC[2] = iRowA[2] - iRowB[2];
        iRowC[3] = iRowA[3] - iRowB[3];
        iRowC[4] = iRowA[4] - iRowB[4];
        iRowC[5] = iRowA[5] - iRowB[5];

        iRowA = &A->data[4][0];
        iRowB = &B->data[4][0];
        iRowC = &C->data[4][0];

        iRowC[0] = iRowA[0] - iRowB[0];
        iRowC[1] = iRowA[1] - iRowB[1];
        iRowC[2] = iRowA[2] - iRowB[2];
        iRowC[3] = iRowA[3] - iRowB[3];
        iRowC[4] = iRowA[4] - iRowB[4];
        iRowC[5] = iRowA[5] - iRowB[5];

        iRowA = &A->data[5][0];
        iRowB = &B->data[5][0];
        iRowC = &C->data[5][0];

        iRowC[0] = iRowA[0] - iRowB[0];
        iRowC[1] = iRowA[1] - iRowB[1];
        iRowC[2] = iRowA[2] - iRowB[2];
        iRowC[3] = iRowA[3] - iRowB[3];
        iRowC[4] = iRowA[4] - iRowB[4];
        iRowC[5] = iRowA[5] - iRowB[5];
#endif
        u32Success = TUEOBJFUSN_ERROR_NOERROR;
    }
    return u32Success;
}

/**
 * @brief performs vector substraction C = A+B
 *
 * - does not change vectors A or B
 * - vecotr can be of dimensions n, 1 <= n <= @ref TUEOBJFUSN_MATRIX_SIZE
 * - vectors A and B must have same dimensions
 * - memory of vector C must be allocated before function call
 * - C may point to A and/or B
 * - Complexity: O(n).
 *
 * @param[in]  A   const stf32Vec_t * const, input vector A
 * @param[in]  B   const stf32Vec_t * const, input vector  B
 * @param[out] C   stf32Vec_t * const, output vector C = A + B
 * @return TRUE (ok) or FALSE (error occured)
 */
uint32 f32VecAdd(CONSTP2CONST(stf32Vec_t, AUTOMATIC, ObjFusn_VAR_NOINIT) A,
                 CONSTP2CONST(stf32Vec_t, AUTOMATIC, ObjFusn_VAR_NOINIT) B,
                 CONSTP2VAR(stf32Vec_t, AUTOMATIC, ObjFusn_VAR_NOINIT) C) {
    uint32 u32Success;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if ((NULL_PTR == A) || (NULL_PTR == B) || (NULL_PTR == C)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32VECADD);
    } else
#endif
        if (A->nRows != B->nRows) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_SIZE_NOT_EQUAL;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_MATRIX_SIZE_NOT_EQUAL,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32VECADD);
    } else if (A->nRows > TUEOBJFUSN_MATRIX_SIZE) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32VECADD);
    } else
#endif
    {
#if TUEOBJFUSN_MATRIX_SIZE != 6
        for (uint16 i = 0u; i < A->nRows; i++) {
            C->data[i] = A->data[i] + B->data[i];
        }
#else
        C->data[0] = A->data[0] + B->data[0];
        C->data[1] = A->data[1] + B->data[1];
        C->data[2] = A->data[2] + B->data[2];
        C->data[3] = A->data[3] + B->data[3];
        C->data[4] = A->data[4] + B->data[4];
        C->data[5] = A->data[5] + B->data[5];
#endif
        C->nRows = A->nRows;
        u32Success = TUEOBJFUSN_ERROR_NOERROR;
    }

    return u32Success;
}

/**
 * @brief performs vector substraction C = A-B
 *
 * - does not change vectors A or B
 * - vecotr can be of dimensions n, 1 <= n <= @ref TUEOBJFUSN_MATRIX_SIZE
 * - vectors A and B must have same dimensions
 * - memory of vector C must be allocated before function call
 * - C may point to A and/or B
 * - Complexity: O(n).
 *
 * @param[in]  A   const stf32Vec_t * const, input vector A
 * @param[in]  B   const stf32Vec_t * const, input vector  B
 * @param[out] C   stf32Vec_t * const, output vector C = A - B
 * @return TRUE (ok) or FALSE (error occured)
 */
uint32 f32VecSub(CONSTP2CONST(stf32Vec_t, AUTOMATIC, ObjFusn_VAR_NOINIT) A,
                 CONSTP2CONST(stf32Vec_t, AUTOMATIC, ObjFusn_VAR_NOINIT) B,
                 CONSTP2VAR(stf32Vec_t, AUTOMATIC, ObjFusn_VAR_NOINIT) C) {
    uint32 u32Success;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if ((NULL_PTR == A) || (NULL_PTR == B) || (NULL_PTR == C)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32VECSUB);
    } else if ((A == C) || (A == B)) {
        u32Success = TUEOBJFUSN_ERROR_INVALID_POINTER;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_INVALID_POINTER, TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32VECSUB);
    } else
#endif
        if (A->nRows > TUEOBJFUSN_MATRIX_SIZE) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32VECSUB);
    } else
#endif
    {
#if TUEOBJFUSN_MATRIX_SIZE != 6
        for (uint16 i = 0u; i < A->nRows; i++) {
            C->data[i] = A->data[i] - B->data[i];
        }
#else
        C->data[0] = A->data[0] - B->data[0];
        C->data[1] = A->data[1] - B->data[1];
        C->data[2] = A->data[2] - B->data[2];
        C->data[3] = A->data[3] - B->data[3];
        C->data[4] = A->data[4] - B->data[4];
        C->data[5] = A->data[5] - B->data[5];
#endif
        C->nRows = A->nRows;
        u32Success = TUEOBJFUSN_ERROR_NOERROR;
    }

    return u32Success;
}

/**
 * @brief performs matrix multiplication C = A * B
 *
 * - does not change matrix A or B
 * - matrices can be of dimensions A:nxm, B:mxk, 1 <= n,m,k <= @ref
 * TUEOBJFUSN_MATRIX_SIZE, i.e. number of colums of A must be equal to number of
 * rows of B
 * - memory of matrix C must be allocated before function call
 * - C must not to A and/or B
 * - Complexity: O(n*m*k). Multiplication of two nxn-matrices requires: 2 * n^3
 * multiplications + 4 * n^3 additions + O(n^2)
 * @param[in]  A   input matrix A
 * @param[in]  B   input matrix B
 * @param[out] C   output matrix C = A * B
 * @return TRUE (ok) or FALSE (error occured)
 */
uint32 f32MatMul(CONSTP2CONST(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) A,
                 CONSTP2CONST(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) B,
                 CONSTP2VAR(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT)
                     C) /* PRQA S 1503 */
{
    // uint16 uRow       = 0u;      /* counts row of C */
    // uint16 uCol       = 0u;      /* counts column of C */
    // uint16 uCnt       = 0u;      /* counts column of A = row of B */
#if STD_OFF == TUE_PRV_RUNTIME_ERROR_CHECK
    const uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
#else
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
#endif

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    /* check dimensions and ensure C does not point to A or B */
    if ((NULL_PTR == A) || (NULL_PTR == B) || (NULL_PTR == C)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32MATMUL);
    } else if ((C == A) || (C == B)) {
        u32Success = TUEOBJFUSN_ERROR_INVALID_POINTER;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_INVALID_POINTER, TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32MATMUL);
    } else
#endif
        if (A->nCols != B->nRows) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_SIZE_NOT_EQUAL;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_MATRIX_SIZE_NOT_EQUAL,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32MATMUL);
    } else if ((A->nCols > TUEOBJFUSN_MATRIX_SIZE) ||
               (A->nRows > TUEOBJFUSN_MATRIX_SIZE)) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32MATMUL);
    } else
#endif
    {
        C->nRows = A->nRows;
        C->nCols = B->nCols;
#if TUEOBJFUSN_MATRIX_SIZE != 6
        for (uint16 i = 0u; i < C->nRows; i++) {
            float32 *iRowA;
            float32 *iRowC;
            iRowA = &A->data[i][0];
            iRowC = &C->data[i][0];
            // float32 *iRowA2;
            // iRowA2 = &A->data[uRow][0];
            // iRowA2[0] = 0;
            iRowC[0] = 0;
            iRowC[1] = 0;
            iRowC[2] = 0;
            iRowC[3] = 0;
            iRowC[4] = 0;
            iRowC[5] = 0;
            for (uint16 k = 0u; k < A->nCols; k++) {
                float32 *kRowB;
                kRowB = &B->data[k][0];
                float32 ikA = iRowA[k];
                if (ikA != 0) {
                    for (uint16 j = 0u; j < C->nCols; j++) {
                        iRowC[j] += (ikA * (kRowB[j]));
                    }
                }
            }
        }
#else
        for (uint16 i = 0u; i < C->nRows; i++) {
            const float32 *iRowA;
            float32 *iRowC;
            iRowA = &A->data[i][0];
            iRowC = &C->data[i][0];
            // float32 *iRowA2;
            // iRowA2 = &A->data[uRow][0];
            // iRowA2[0] = 0;
            iRowC[0] = 0;
            iRowC[1] = 0;
            iRowC[2] = 0;
            iRowC[3] = 0;
            iRowC[4] = 0;
            iRowC[5] = 0;
            for (uint16 k = 0u; k < A->nCols; k++) {
                const float32 *kRowB;
                kRowB = &B->data[k][0];
                float32 ikA = iRowA[k];
                if (ikA != 0) {
                    if (C->nCols == 6 && ikA != 0) {
                        iRowC[0] += (ikA * (kRowB[0]));
                        iRowC[1] += (ikA * (kRowB[1]));
                        iRowC[2] += (ikA * (kRowB[2]));
                        iRowC[3] += (ikA * (kRowB[3]));
                        iRowC[4] += (ikA * (kRowB[4]));
                        iRowC[5] += (ikA * (kRowB[5]));
                    } else if (C->nCols == 5) {
                        iRowC[0] += (ikA * (kRowB[0]));
                        iRowC[1] += (ikA * (kRowB[1]));
                        iRowC[2] += (ikA * (kRowB[2]));
                        iRowC[3] += (ikA * (kRowB[3]));
                        iRowC[4] += (ikA * (kRowB[4]));
                    } else {
                        for (uint16 j = 0u; j < C->nCols; j++) {
                            iRowC[j] += (ikA * (kRowB[j]));
                        }
                    }
                }
            }
        }
#endif
    }

    return u32Success;
}

/**
 * @brief performs matrix vector multiplication C = A * B where A is a matrix, B
 * and C are vectors
 *
 * - does not change matrix A or vector B
 * - matrices can be of dimensions A:nxm, B:m, 1 <= n,m <= @ref
 * TUEOBJFUSN_MATRIX_SIZE, i.e. number of colums of A must be equal to number of
 * rows of B
 * @param[in]  A   const stf32Matrix_t * const A, input matrix A
 * @param[in]  B   const stf32Vec_t * const B, input vector B
 * @param[out] C   stf32Vec_t * const C, output vector C = A * B
 * @return TRUE (ok) or FALSE (error occured)
 */
uint32 f32MatMulVec(CONSTP2CONST(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT)
                        A,
                    CONSTP2CONST(stf32Vec_t, AUTOMATIC, ObjFusn_VAR_NOINIT) B,
                    CONSTP2VAR(stf32Vec_t, AUTOMATIC, ObjFusn_VAR_NOINIT) C) {
#if STD_OFF == TUE_PRV_RUNTIME_ERROR_CHECK
    const uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
#else
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
#endif

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if ((NULL_PTR == A) || (NULL_PTR == B) || (NULL_PTR == C)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32MATMULVEC);
    } else
#endif
        if (A->nCols != B->nRows) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_SIZE_NOT_EQUAL;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_MATRIX_SIZE_NOT_EQUAL,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32MATMULVEC);
    } else if ((A->nCols > TUEOBJFUSN_MATRIX_SIZE) ||
               (A->nRows > TUEOBJFUSN_MATRIX_SIZE)) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32MATMULVEC);
    } else
#endif
    {
        for (uint16 u16i = 0u; u16i < A->nRows; u16i++) {
            C->data[u16i] = FLT_ZERO;
            const float32 *tempA;
            tempA = &A->data[u16i][0];
            for (uint16 u16j = 0u; u16j < A->nCols; u16j++) {
                C->data[u16i] += tempA[u16j] * B->data[u16j];
            }
        }

        C->nRows = A->nRows;
    }

    return u32Success;
}

/**
 * performs matrix transpose B = A^T
 *
 * - does not change matrix A
 * - matrix can be of dimensions nxm, 1 <= n,m <= @ref TUEOBJFUSN_MATRIX_SIZE
 * - memory of matrix B must be allocated before function call
 * - B must not point to A
 * - Complexity: O(n^2).
 * @param[in]  A           input matrix A
 * @param[out] B           output matrix B = A^T
 * @return TRUE (ok) or FALSE (error occured)
 */
uint32 f32MatTranspose(CONSTP2CONST(stf32Matrix_t,
                                    AUTOMATIC,
                                    ObjFusn_VAR_NOINIT) A,
                       CONSTP2VAR(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT)
                           B) /* PRQA S 1503 */
{
    // uint16 uRow; /* counts row of B = column of A */
    // uint16 uCol; /* counts column of B = row of A */
    uint32 u32Success;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    /* ensure B does not point to A */
    if ((NULL_PTR == A) || (NULL_PTR == B)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32MATTRANSPOSE);
    } else if (B == A) {
        u32Success = TUEOBJFUSN_ERROR_INVALID_POINTER;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_INVALID_POINTER, TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32MATTRANSPOSE);
    } else
#endif
        if ((A->nCols > TUEOBJFUSN_MATRIX_SIZE) ||
            (A->nRows > TUEOBJFUSN_MATRIX_SIZE)) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32MATTRANSPOSE);
    } else
#endif
    {
        B->nCols = A->nRows;
        B->nRows = A->nCols;
#if TUEOBJFUSN_MATRIX_SIZE != 6
        for (uint16 uCol = 0u; uCol < TUEOBJFUSN_MATRIX_SIZE; uCol++) {
            for (uint16 uRow = uCol; uRow < TUEOBJFUSN_MATRIX_SIZE; uRow++) {
                B->data[uRow][uCol] = A->data[uCol][uRow];
                B->data[uCol][uRow] = A->data[uRow][uCol];
            }
        }
#else
        const float32 *tempA;
        tempA = &A->data[0][0];

        B->data[0][0] = tempA[0];
        B->data[1][0] = tempA[1];
        B->data[2][0] = tempA[2];
        B->data[3][0] = tempA[3];
        B->data[4][0] = tempA[4];
        B->data[5][0] = tempA[5];

        tempA = &A->data[1][0];
        B->data[0][1] = tempA[0];
        B->data[1][1] = tempA[1];
        B->data[2][1] = tempA[2];
        B->data[3][1] = tempA[3];
        B->data[4][1] = tempA[4];
        B->data[5][1] = tempA[5];

        tempA = &A->data[2][0];
        B->data[0][2] = tempA[0];
        B->data[1][2] = tempA[1];
        B->data[2][2] = tempA[2];
        B->data[3][2] = tempA[3];
        B->data[4][2] = tempA[4];
        B->data[5][2] = tempA[5];

        tempA = &A->data[3][0];
        B->data[0][3] = tempA[0];
        B->data[1][3] = tempA[1];
        B->data[2][3] = tempA[2];
        B->data[3][3] = tempA[3];
        B->data[4][3] = tempA[4];
        B->data[5][3] = tempA[5];

        tempA = &A->data[4][0];
        B->data[0][4] = tempA[0];
        B->data[1][4] = tempA[1];
        B->data[2][4] = tempA[2];
        B->data[3][4] = tempA[3];
        B->data[4][4] = tempA[4];
        B->data[5][4] = tempA[5];

        tempA = &A->data[5][0];
        B->data[0][5] = tempA[0];
        B->data[1][5] = tempA[1];
        B->data[2][5] = tempA[2];
        B->data[3][5] = tempA[3];
        B->data[4][5] = tempA[4];
        B->data[5][5] = tempA[5];

#endif
        u32Success = TUEOBJFUSN_ERROR_NOERROR;
    }

    return u32Success;
}

/* PRQA S 1532 2 */ /* Library Function */
uint32 f32MatInvSym3x3_Sym(CONSTP2VAR(stf32SymMatrix_t,
                                      AUTOMATIC,
                                      ObjFusn_VAR_NOINIT) MatrixA) {
    uint32 u32Success;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if (NULL_PTR == MatrixA) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_INV_SYMMAT_3X3);
    } else
#endif
        if (MatrixA->u16Size != TUE_PRV_COMMON_MATRIX_MAT_INV_3X3_SIZE) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_SIZE_NOT_EQUAL;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_INV_SYMMAT_3X3);
    } else
#endif
    {
        float32 f32a11 =
            MatrixA->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_00];
        float32 f32a12 =
            MatrixA->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_01];
        float32 f32a22 =
            MatrixA->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_11];
        float32 f32a13 =
            MatrixA->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_02];
        float32 f32a23 =
            MatrixA->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_12];
        float32 f32a33 =
            MatrixA->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_22];

        float32 f32Temp1 = (f32a33 * f32a22) - (f32a23 * f32a23);
        float32 f32Temp2 = (f32a23 * f32a13) - (f32a33 * f32a12);
        float32 f32Temp3 = (f32a23 * f32a12) - (f32a22 * f32a13);

        float32 f32Det =
            (f32a11 * f32Temp1) + (f32a12 * f32Temp2) + (f32a13 * f32Temp3);
        float32 f32AbsDet = tue_prv_fusion_abs(f32Det);
        u32Success = TUEOBJFUSN_ERROR_NOERROR;

        if (f32AbsDet > TUE_PRV_COMMON_MATRIX_INV_EPS) {
            MatrixA->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_00] =
                f32Temp1 / f32Det;
            MatrixA->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_01] =
                f32Temp2 / f32Det;
            MatrixA->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_02] =
                f32Temp3 / f32Det;

            float32 f32m22 = (f32a33 * f32a11) - (f32a13 * f32a13);
            f32m22 /= f32Det;
            MatrixA->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_11] = f32m22;

            float32 f32m23 = (f32a12 * f32a13) - (f32a23 * f32a11);
            f32m23 /= f32Det;
            MatrixA->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_12] = f32m23;

            float32 f32m33 = (f32a22 * f32a11) - (f32a12 * f32a12);
            f32m33 /= f32Det;
            MatrixA->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_22] = f32m33;
        } else {
#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
            u32Success = TUEOBJFUSN_ERROR_COMMON_MATRIX_UNCONDITIONED;
            (void)tue_prv_error_management_addError(
                u32Success, TUEOBJFUSN_AAU_COMMON_MATRIX,
                TUEOBJFUSN_AAU_COMMON_MATRIX_INV_SYMMAT_3X3);
#endif
        }
    }

    return u32Success;
}

/**
 * @brief inverts a matrix
 *
 * - the input matrix is inverted and thus changed
 * - matrix can be of dimensions nxn, 1 <= n <= @ref TUEOBJFUSN_MATRIX_SIZE
 *    i.e. matrix must be square
 * - Inversion algorithm used is:
 *   LU factorization with partial pivoting,
 *   modified version of:
 * http://chandraacads.blogspot.jp/2015/12/c-program-for-matrix-inversion.html
 * - verifies the result by calculation the condition number and comparing it
 * against threshold @ref TUE_PRV_COMMON_MATRIX_INV_THRESHOLD
 * - complexity: O(n^3).
 * @param[in,out]  MatrixA       input and output matrix A
 * @return TRUE (ok) or FALSE (error occured)
 */
uint32 f32MatInv(CONSTP2VAR(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT)
                     MatrixA) {
    VAR(float32, ObjFusn_VAR_NOINIT)
    A[TUEOBJFUSN_MATRIX_SIZE][TUEOBJFUSN_MATRIX_SIZE];
    VAR(float32, ObjFusn_VAR_NOINIT)
    B[TUEOBJFUSN_MATRIX_SIZE][TUEOBJFUSN_MATRIX_SIZE];
    VAR(float32, ObjFusn_VAR_NOINIT) X[TUEOBJFUSN_MATRIX_SIZE];
    VAR(float32, ObjFusn_VAR_NOINIT) Y[TUEOBJFUSN_MATRIX_SIZE];
    VAR(uint8, ObjFusn_VAR_NOINIT) auP[TUEOBJFUSN_MATRIX_SIZE];
    VAR(sint8, ObjFusn_VAR_NOINIT) s8Size = (sint8)TUEOBJFUSN_MATRIX_SIZE;
    sint8 s8i;
    sint8 s8j;
    sint8 s8k;
    sint8 s8kd;
    uint8 u8Swap;
    float32 f32p;
    float32 f32t;
    float32 f32Swap;
    uint32 u32Success;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if (NULL_PTR == MatrixA) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_MATRIX_SIZE_NOT_EQUAL,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32MATINV);
    } else
#endif
        if (MatrixA->nCols != MatrixA->nRows) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_SIZE_NOT_EQUAL;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_MATRIX_SIZE_NOT_EQUAL,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32MATINV);
    } else if (MatrixA->nRows > TUEOBJFUSN_MATRIX_SIZE) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32MATINV);
    } else
#endif
    {
        s8Size = (sint8)(MatrixA->nRows);
        s8kd = 0;
        float32 *tempRowA, *tempRowB;

        for (s8i = 0; s8i < s8Size; s8i++) {
            tempRowA = &A[s8i][0];
            tempRowB = &MatrixA->data[s8i][0];
            for (s8k = 0; s8k < s8Size; s8k++) {
                tempRowA[s8k] = tempRowB[s8k];
            }

            /* Initializing. */
            auP[s8i] = (uint8)s8i;
        }

        /* Finding the pivot of the LUP decomposition. */
        for (s8k = 0; s8k < (s8Size - 1); s8k++) {
            f32p = FLT_ZERO;
            for (s8i = s8k; s8i < s8Size; s8i++) {
                f32t = tue_prv_fusion_abs(A[s8i][s8k]);
                if (f32t > f32p) {
                    f32p = f32t;
                    s8kd = s8i;
                } else {
                    /* MISRA */
                }
            }

            /* Exchanging the rows according to the pivot determined above. */
            u8Swap = auP[s8kd];
            auP[s8kd] = auP[s8k];
            auP[s8k] = u8Swap;
            for (s8i = 0; s8i < s8Size; s8i++) {
                f32Swap = A[s8kd][s8i];
                A[s8kd][s8i] = A[s8k][s8i];
                A[s8k][s8i] = f32Swap;
            }

            /* Performing substraction to decompose A as LU. */
            tempRowB = &A[s8k][0];
            for (s8i = s8k + 1; s8i < s8Size; s8i++) {
                tempRowA = &A[s8i][0];
                tempRowA[s8k] = tempRowA[s8k] / tempRowB[s8k];
                for (s8j = s8k + 1; s8j < s8Size; s8j++) {
                    tempRowA[s8j] -= tempRowA[s8k] * tempRowB[s8j];
                }
            }
        }
        /* Now, 'A' contains the L (without the diagonal elements, which are all
         * 1) and the U. */

        /* check that diagnoal elements are not zero */
        u32Success = TUEOBJFUSN_ERROR_NOERROR;
#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
        for (s8j = 0; s8j < s8Size; s8j++) {
            if (tue_prv_fusion_abs(A[s8j][s8j]) <
                TUE_PRV_COMMON_MATRIX_INV_EPS) /* PRQA S 3416 */ /* simple fcn
                                                                  */
            {
                u32Success |= TUEOBJFUSN_ERROR_COMMON_MATRIX_DIAG_ZERO;
                (void)tue_prv_error_management_addError(
                    TUEOBJFUSN_ERROR_COMMON_MATRIX_DIAG_ZERO,
                    TUEOBJFUSN_AAU_COMMON_MATRIX,
                    TUEOBJFUSN_AAU_COMMON_MATRIX_F32MATINV);
            } else {
                /* MISRA */
            }
        }

        if (u32Success == TUEOBJFUSN_ERROR_NOERROR)
#endif
        {
            /* Initializing X and Y. */
            for (s8j = 0; s8j < s8Size; s8j++) {
                X[s8j] = FLT_ZERO;
                Y[s8j] = FLT_ZERO;
            }

            /* Solving LUX = Pe, in order to calculate the inverse of 'A'. Here,
             * 'e' is a column
             * vector of the identity matrix of 4 '4-1'. Solving for all 'e'. */
            for (s8i = 0; s8i < s8Size; s8i++) {
                tempRowA = &B[s8i][0];
                /* Storing elements of the s8i-th column of the identity matrix
                 * in s8i-th row of 'B'. */
                for (s8j = 0; s8j < s8Size; s8j++) {
                    tempRowA[s8j] = FLT_ZERO;
                }
                tempRowA[s8i] = FLT_ONE;

                /* Solving Ly = Pb. */
                for (s8j = 0; s8j < s8Size; s8j++) {
                    f32t = FLT_ZERO;
                    tempRowB = &A[s8j][0];
                    for (s8k = 0; s8k <= (s8j - 1); s8k++) {
                        f32t += tempRowB[s8k] * Y[s8k];
                    }
                    Y[s8j] = B[s8i][auP[s8j]] - f32t;
                }

                /* Solving Ux = y. */
                for (s8j = (sint8)s8Size - 1; s8j >= 0; s8j--) {
                    f32t = FLT_ZERO;
                    tempRowB = &A[s8j][0];
                    for (s8k = s8j + 1; s8k < s8Size; s8k++) {
                        f32t += tempRowB[s8k] * X[s8k];
                    }

                    /* Now, X contains the solution. */
                    X[s8j] = (Y[s8j] - f32t) / tempRowB[s8j];

                    /* Copying transpose of 'B' into 'MatrixA', which would the
                     * inverse of 'A'. */
                    MatrixA->data[s8j][s8i] = X[s8j];
                }
            }
        }
#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
        else {
            /* MISRA */
        }
#endif
    }

    return u32Success;
}

/* PRQA S 1503 3 */ /* Library function, may not be used depending on
                       configuration */
/* PRQA S 1532 2 */ /* Library Function */
uint32 f32SymMatDet(CONSTP2CONST(stf32SymMatrix_t,
                                 AUTOMATIC,
                                 ObjFusn_VAR_NOINIT) A,
                    CONSTP2VAR(float32, AUTOMATIC, ObjFusn_VAR_NOINIT) f32det) {
    uint32 u32Success;
    VAR(stf32Matrix_t, ObjFusn_VAR_NOINIT) X;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if ((NULL_PTR == A) || (NULL_PTR == f32det)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32MATDET);
    } else
#endif
        if (A->u16Size > TUEOBJFUSN_MATRIX_SIZE) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32MATDET);
    } else if (A->u16Size == 0u) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_EMPTY;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_MATRIX_EMPTY, TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32MATDET);
    } else
#endif
    {
        u32Success = TUEOBJFUSN_ERROR_NOERROR;

        /* 1x1 */
        if (1u == A->u16Size) /* PRQA S 3120 */ /* math */
        {
            *f32det = A->data[au16SymMatrixLookup[0][0]];
        }
        /* 2x2 */
        else if (2u == A->u16Size) /* PRQA S 3120 */ /* math */
        {
            *f32det = (A->data[au16SymMatrixLookup[0][0]] *
                       A->data[au16SymMatrixLookup[1][1]]) -
                      (A->data[au16SymMatrixLookup[0][1]] *
                       A->data[au16SymMatrixLookup[0][1]]);
        }
        /* 3x3 */
        else if (3u == A->u16Size) /* PRQA S 3120 */ /* math */
        {
            /* PRQA S 3120 3 */ /* math */
            *f32det = ((A->data[au16SymMatrixLookup[0u][0u]] *
                        ((A->data[au16SymMatrixLookup[1u][1u]] *
                          A->data[au16SymMatrixLookup[2u][2u]]) -
                         (A->data[au16SymMatrixLookup[1u][2u]] *
                          A->data[au16SymMatrixLookup[2u][1u]]))) -
                       (A->data[au16SymMatrixLookup[0u][1u]] *
                        ((A->data[au16SymMatrixLookup[1u][0u]] *
                          A->data[au16SymMatrixLookup[2u][2u]]) -
                         (A->data[au16SymMatrixLookup[1u][2u]] *
                          A->data[au16SymMatrixLookup[2u][0u]])))) +
                      (A->data[au16SymMatrixLookup[0u][2u]] *
                       ((A->data[au16SymMatrixLookup[1u][0u]] *
                         A->data[au16SymMatrixLookup[2u][1u]]) -
                        (A->data[au16SymMatrixLookup[1u][1u]] *
                         A->data[au16SymMatrixLookup[2u][0u]])));
        }
        /* nxn */
        else {
            u32Success = f32SymMatToMat(A, &X);

            // float32 *temp1, *temp2;
            float32 f32detTmp = FLT_ONE;
            for (uint16 iter_k = 0u; iter_k < A->u16Size; ++iter_k) {
                float32 *temp2 = &X.data[iter_k][0];
                for (uint16 iter_i = iter_k + 1u; iter_i < A->u16Size;
                     iter_i++) {
                    float32 *temp1 = &X.data[iter_i][0];
                    temp1[iter_k] = temp1[iter_k] / temp2[iter_k];
                    for (uint16 iter_j = iter_k + 1u; iter_j < A->u16Size;
                         iter_j++) {
                        temp1[iter_j] -= temp1[iter_k] * temp2[iter_j];
                    }
                }

                /* multiplying diagonals of U = det */
                f32detTmp *= temp2[iter_k];
            }

            *f32det = f32detTmp;
        }
    }

    return u32Success;
}

/**
 * @brief calculates the determinant of a matrix
 *
 * - does not change matrix A
 * - matrix can be of dimensions nxn, 1 <= n <= @ref TUEOBJFUSN_MATRIX_SIZE,
 *   i.e. matrix must be square
 * - Algorithm depends on matrix size:
 *   - 1x1, 2x2, 3x3: direct calculation with appropriate formula
 *   - >3x3: LU decomposition
 * - Complexity: worst case O(n^3).
 * @param[in]  A       input matrix A
 * @param[out] f32det  calculated determinant
 * @return TRUE (ok) or FALSE (error occured)
 */
/* PRQA S 1503 2 */ /* Library function */
uint32 f32MatDet(CONSTP2CONST(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) A,
                 CONSTP2VAR(float32, AUTOMATIC, ObjFusn_VAR_NOINIT) f32det) {
    uint32 u32Success;
    VAR(stf32Matrix_t, ObjFusn_VAR_NOINIT) X;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if ((NULL_PTR == A) || (NULL_PTR == f32det)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32MATDET);
    } else
#endif
        if (A->nCols != A->nRows) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_SIZE_NOT_EQUAL;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_MATRIX_SIZE_NOT_EQUAL,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32MATDET);
    } else if (A->nCols > TUEOBJFUSN_MATRIX_SIZE) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32MATDET);
    } else if (A->nCols == 0u) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_EMPTY;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_MATRIX_EMPTY, TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32MATDET);
    } else
#endif
    {
        u32Success = TUEOBJFUSN_ERROR_NOERROR;

        /* 1x1 */
        if (1u == A->nCols) /* PRQA S 3120 */ /* math */
        {
            *f32det = A->data[0][0];
        }
        /* 2x2 */
        else if (2u == A->nCols) /* PRQA S 3120 */ /* math */
        {
            *f32det = (A->data[0][0] * A->data[1][1]) -
                      (A->data[0][1] * A->data[1][0]);
        }
        /* 3x3 */
        else if (3u == A->nCols) /* PRQA S 3120 */ /* math */
        {
            /* PRQA S 3120 3 */ /* math */
            *f32det =
                ((A->data[0u][0u] * ((A->data[1u][1u] * A->data[2u][2u]) -
                                     (A->data[1u][2u] * A->data[2u][1u]))) -
                 (A->data[0u][1u] * ((A->data[1u][0u] * A->data[2u][2u]) -
                                     (A->data[1u][2u] * A->data[2u][0u])))) +
                (A->data[0u][2u] * ((A->data[1u][0u] * A->data[2u][1u]) -
                                    (A->data[1u][1u] * A->data[2u][0u])));
        }
        /* nxn */
        else {
            /* LU decompostion */
            u32Success |= f32CopyMat(&X, A);

            float32 f32detTmp = FLT_ONE;
            for (uint16 iter_k = 0u; iter_k < A->nCols; ++iter_k) {
                for (uint16 iter_i = iter_k + 1u; iter_i < A->nCols; iter_i++) {
                    X.data[iter_i][iter_k] =
                        X.data[iter_i][iter_k] / X.data[iter_k][iter_k];
                    for (uint16 iter_j = iter_k + 1u; iter_j < A->nCols;
                         iter_j++) {
                        X.data[iter_i][iter_j] -=
                            X.data[iter_i][iter_k] * X.data[iter_k][iter_j];
                    }
                }

                /* multiplying diagonals of U = det */
                f32detTmp *= X.data[iter_k][iter_k];
            }

            *f32det = f32detTmp;
        }
    }

    return u32Success;
}

/**
 * @brief initializes the matrix with zeros and set rows and columns
 *
 * number of rows and columns must not exceed @ref TUEOBJFUSN_MATRIX_SIZE
 * @param[out] A         output matrix
 * @param[in]  u16nRows  desired number of rows
 * @param[in]  u16nCols  desired number of columns
 */
uint32 f32MatZeros(CONSTP2VAR(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) A,
                   const uint16 u16nRows,
                   const uint16 u16nCols) {
    uint32 u32Success;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if (NULL_PTR == A) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32MATZEROS);
    } else
#endif
        if ((u16nCols > TUEOBJFUSN_MATRIX_SIZE) ||
            (u16nRows > TUEOBJFUSN_MATRIX_SIZE)) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32MATZEROS);
    } else
#endif
    {
        /* PRQA S 0314 1 */ /* Cast to void required for copy function */
        tue_prv_fusion_memcpy((void *)A, (const void *)&sf32ZeroMatrix,
                              (uint32)sizeof(stf32Matrix_t));
        A->nCols = u16nCols;
        A->nRows = u16nRows;

        u32Success = TUEOBJFUSN_ERROR_NOERROR;
    }

    return u32Success;
}

/**
 * @brief initializes a vector with zeros and set rows
 *
 * number of rows must not exceed @ref TUEOBJFUSN_MATRIX_SIZE
 * @param[out] A         output vector
 * @param[in]  u16nRows  desired number of rows
 */
uint32 f32VecZeros(CONSTP2VAR(stf32Vec_t, AUTOMATIC, ObjFusn_VAR_NOINIT) A,
                   const uint16 u16nRows) {
    uint32 u32Success;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if (NULL_PTR == A) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32VECZEROS);
    } else
#endif
        if (u16nRows > TUEOBJFUSN_MATRIX_SIZE) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32VECZEROS);
    } else
#endif
    {
        /* PRQA S 0314 1 */ /* Cast to void required for copy function */
        tue_prv_fusion_memcpy((void *)A, (const void *)&sf32ZeroVector,
                              (uint32)sizeof(stf32Vec_t));
        A->nRows = u16nRows;
        u32Success = TUEOBJFUSN_ERROR_NOERROR;
    }

    return u32Success;
}

/**
 * @brief generates an identity matrix of certain size
 *
 * number of rows and columns must not exceed @ref TUEOBJFUSN_MATRIX_SIZE
 * @param[out] A         output matrix
 * @param[in]  u16nRows  desired number of rows
 * @param[in]  u16nCols  desired number of columns
 */
uint32 f32MatEye(CONSTP2VAR(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) A,
                 const uint16 u16nRows,
                 const uint16 u16nCols) {
    uint32 u32Success;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if (NULL_PTR == A) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32MATEYE);
    } else
#endif
        if ((u16nCols > TUEOBJFUSN_MATRIX_SIZE) ||
            (u16nRows > TUEOBJFUSN_MATRIX_SIZE)) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32MATEYE);
    } else
#endif
    {
        u32Success = f32MatZeros(A, u16nRows, u16nCols);
#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
        if (TUEOBJFUSN_ERROR_NOERROR != u32Success) {
            /* MISRA */
        } else
#endif
        {
            uint16 u16SizeTmp = tue_prv_fusion_min_U16(u16nRows, u16nCols);
            for (uint16 u16i = 0u; u16i < u16SizeTmp; u16i++) {
                A->data[u16i][u16i] = FLT_ONE;
            }
        }
    }

    return u32Success;
}

/* PRQA S 1503 2*/ /* Library function currently not used */
uint32 f32SymMatZeros(CONSTP2VAR(stf32SymMatrix_t,
                                 AUTOMATIC,
                                 ObjFusn_VAR_NOINIT) A,
                      const uint16 u16Size) {
    uint32 u32Success;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if (NULL_PTR == A) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32SYM_MAT_ZEROS);
    } else
#endif
        if (u16Size > TUEOBJFUSN_MATRIX_SIZE) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32SYM_MAT_ZEROS);
    } else
#endif
    {
        A->u16Size = u16Size;

        for (uint16 u16i = 0u; u16i < TUEOBJFUSN_SYMMETRIC_MATRIX_SIZE;
             u16i++) {
            A->data[u16i] = FLT_ZERO;
        }

        u32Success = TUEOBJFUSN_ERROR_NOERROR;
    }

    return u32Success;
}

/**
 * @brief converts a symmetrix matrix of type stf32Matrix_t into a matrix of
 * tpye stf32SymMatrix_t
 *
 * number of rows and columns for the matrix B must not exceed @ref
 * TUEOBJFUSN_MATRIX_SIZE
 * @param[out] B   output matrix,
 * @param[in]  A   input matrix that needs to be converted into an
 * stf32MatrxSym_t
 */
uint32 f32MatToSymMat(
    CONSTP2CONST(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) A,
    CONSTP2VAR(stf32SymMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) B) {
    uint32 u32Success;
    uint16 u16Row;
    uint16 u16Col;
    uint16 u16Idx;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if ((NULL_PTR == A) || (NULL_PTR == B)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
    } else
#endif
        if (A->nCols != A->nRows) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_SIZE_NOT_EQUAL;
    } else if (A->nRows > TUEOBJFUSN_MATRIX_SIZE) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE;
    } else
#endif
    {
        B->u16Size = A->nRows;
        u16Idx = 0u;
        const float32 *iRowA;
        for (u16Row = 0u; u16Row < A->nRows; u16Row++) {
            iRowA = &A->data[u16Row][0];
            for (u16Col = 0u; u16Col <= u16Row; u16Col++) {
                B->data[u16Idx] = iRowA[u16Col];
                u16Idx++;
            }
        }

        u32Success = TUEOBJFUSN_ERROR_NOERROR;
    }

    return u32Success;
}

/**
 * @brief Implements conversion of symmetric matrix A to regular matrix B
 *
 * number of rows and columns must not exceed @ref TUEOBJFUSN_MATRIX_SIZE
 * @param[out] B      stf32Matrix_t * const, regular matrix
 * @param[in]  A      const stf32SymMatrix_t * const, symmetric input matrix A
 */
uint32 f32SymMatToMat(
    CONSTP2CONST(stf32SymMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) A,
    CONSTP2VAR(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) B) {
    uint32 u32Success;
    uint16 u16Row;
    uint16 u16Col;
    uint16 u16Idx;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if ((NULL_PTR == A) || (NULL_PTR == B)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32SYM_MAT_TO_MAT);
    } else
#endif
        if (A->u16Size > TUEOBJFUSN_MATRIX_SIZE) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32SYM_MAT_TO_MAT);
    } else
#endif
    {
        B->nCols = A->u16Size;
        B->nRows = A->u16Size;

        for (u16Row = 0u; u16Row < B->nRows; u16Row++) {
            for (u16Col = u16Row; u16Col < B->nCols; u16Col++) {
                u16Idx = au16SymMatrixLookup[u16Row][u16Col];
                B->data[u16Row][u16Col] = A->data[u16Idx];
                B->data[u16Col][u16Row] = A->data[u16Idx];
            }
        }

        u32Success = TUEOBJFUSN_ERROR_NOERROR;
    }

    return u32Success;
}

/**
 * @brief Implements matrix matrix addition C = A + B whereas A,B and C are
 * assumed to be symmetric
 *
 * number of rows and columns must not exceed @ref TUEOBJFUSN_MATRIX_SIZE
 * @param[out] C      stf32SymMatrix_t * const, symetric output matrix C
 * @param[in]  A      const stf32SymMatrix_t * const, symmetric input matrix A
 * @param[in]  B      const stf32SymMatrix_t * const, symmetric input matrix B
 */
/* PRQA S 1503 2 */ /* Library function currently not used */
uint32 f32SymMatAddSymMat(
    CONSTP2CONST(stf32SymMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) A,
    CONSTP2CONST(stf32SymMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) B,
    CONSTP2VAR(stf32SymMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) C) {
    uint32 u32Success;
    uint16 u16MaxIndex;
    uint16 u16Idx;

    const uint16 au16SymMatrixSizes[TUEOBJFUSN_MATRIX_SIZE + 1u] = {
        0u, 1u, 3u, 6u, 10u, 15u, 21u};

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if ((NULL_PTR == A) || (NULL_PTR == B) || (NULL_PTR == C)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
    } else
#endif
        if (A->u16Size != B->u16Size) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_SIZE_NOT_EQUAL;
    } else if (A->u16Size > TUEOBJFUSN_MATRIX_SIZE) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE;
    } else
#endif
    {
        u16MaxIndex = au16SymMatrixSizes[A->u16Size];

        for (u16Idx = 0u; u16Idx < u16MaxIndex; u16Idx++) {
            C->data[u16Idx] = A->data[u16Idx] + B->data[u16Idx];
        }

        C->u16Size = A->u16Size;
        u32Success = TUEOBJFUSN_ERROR_NOERROR;
    }

    return u32Success;
}

/* PRQA S 1503 2 */ /* Library function currently not used */
uint32 f32SymMatEye(CONSTP2VAR(stf32SymMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT)
                        A,
                    const uint16 u16Size) {
    uint32 u32Success;
    uint16 u16i;
    uint16 u16Idx;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if (NULL_PTR == A) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32SYM_MAT_EYE);
    } else
#endif
        if (u16Size > TUEOBJFUSN_MATRIX_SIZE) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32SYM_MAT_EYE);
    } else
#endif
    {
        u32Success = f32SymMatZeros(A, u16Size);

        for (u16i = 0u; u16i < u16Size; u16i++) {
            u16Idx = au16SymMatrixLookup[u16i][u16i];
            A->data[u16Idx] = FLT_ONE;
        }
    }

    return u32Success;
}

/**
 * @brief    Copy matrix A to matrix B
 *
 * number of rows and columns must not exceed @ref TUEOBJFUSN_MATRIX_SIZE
 * @param[out] B      stf32Matrix_t * const, output matrix B
 * @param[in]  A      const stf32Matrix_t * const, regular input matrix A
 */
LOCAL uint32
f32CopyMat(CONSTP2VAR(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) pDest,
           CONSTP2CONST(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) pSrc) {
    uint32 u32Success;
    const uint32 u32Size = (uint32)sizeof(stf32Matrix_t);

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if ((NULL_PTR == pDest) || (NULL_PTR == pSrc)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_COPY_MAT);
    } else
#endif
        if ((pSrc->nCols > TUEOBJFUSN_MATRIX_SIZE) ||
            (pSrc->nRows > TUEOBJFUSN_MATRIX_SIZE)) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_COPY_MAT);
    } else
#endif
    {
        /* PRQA S 0314 1 */ /* Cast to void required for copy function */
        tue_prv_fusion_memcpy((void *)pDest, (const void *)pSrc, u32Size);
        u32Success = TUEOBJFUSN_ERROR_NOERROR;
    }

    return u32Success;
}

/**
 * @brief    Copy vector A to vector B
 *
 * number of rows must not exceed @ref TUEOBJFUSN_MATRIX_SIZE
 * @param[out] B      stf32Vec_t * const, output matrix B
 * @param[in]  A      const stf32Vec_t * const, regular input matrix A
 */
uint32 f32CopyVec(CONSTP2VAR(stf32Vec_t, AUTOMATIC, ObjFusn_VAR_NOINIT) pDest,
                  CONSTP2CONST(stf32Vec_t, AUTOMATIC, ObjFusn_VAR_NOINIT)
                      pSrc) {
    uint32 u32Success;
    const uint32 u32Size = (uint32)sizeof(stf32Vec_t);

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if ((NULL_PTR == pDest) || (NULL_PTR == pSrc)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_COPY_VEC);
    } else
#endif
        if (pSrc->nRows > TUEOBJFUSN_MATRIX_SIZE) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_COPY_VEC);
    } else
#endif
    {
        /* PRQA S 0314 1 */ /* Cast to void required for copy function */
        tue_prv_fusion_memcpy((void *)pDest, (const void *)pSrc, u32Size);
        u32Success = TUEOBJFUSN_ERROR_NOERROR;
    }

    return u32Success;
}

/* PRQA S 1532 2 */ /* library function used in LKF only*/
void f32MatInit(CONSTP2VAR(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) A) {
    A->nCols = 0u;
    A->nRows = 0u;
}

/* PRQA S 1532 2 */ /* library function */
void f32VecInit(CONSTP2VAR(stf32Vec_t, AUTOMATIC, ObjFusn_VAR_NOINIT) A) {
    A->nRows = 0u;
}

/* PRQA S 1503 2 */ /* library function */
void f32SymMatInit(CONSTP2VAR(stf32SymMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT)
                       A) {
    A->u16Size = 0u;
}

/**
 * @brief    Copy matrix A to matrix B
 *
 * number of rows and columns must not exceed @ref TUEOBJFUSN_MATRIX_SIZE
 * @param[out] B      stf32Matrix_t * const, output matrix B
 * @param[in]  A      const stf32Matrix_t * const, regular input matrix A
 */
/* PRQA S 1532 2 */ /* Library Function */
uint32 f32CopySymMat(
    CONSTP2VAR(stf32SymMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) pDest,
    CONSTP2CONST(stf32SymMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) pSrc) {
    uint32 u32Success;
    const uint32 u32Size = (uint32)sizeof(stf32SymMatrix_t);

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if ((NULL_PTR == pDest) || (NULL_PTR == pSrc)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_COPY_MAT);
    } else
#endif
        if (pSrc->u16Size > TUEOBJFUSN_MATRIX_SIZE) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_COPY_MAT);
    } else
#endif
    {
        /* PRQA S 0314 1 */ /* Cast to void required for copy function */
        tue_prv_fusion_memcpy((void *)pDest, (const void *)pSrc, u32Size);
        u32Success = TUEOBJFUSN_ERROR_NOERROR;
    }

    return u32Success;
}

/**
 * @brief Implements matrix matrix multiplication C = R * A *R  whereas A is a
 * sysmetric matrix, C is assumed to be symmetric and R is a rotaiton matrix
 * with rotation by angle r
 *
 * number of rows and columns must not exceed @ref TUEOBJFUSN_MATRIX_SIZE
 * @param[out] C      stf32Matrix_t * const, regular output matrix C
 * @param[in]  A      const stf32Matrix_t * const, regular input matrix A
 * @param[in]  f32rot float32 f32rot , angle for rotation in rad
 */
/* PRQA S 1503 2 */ /* Library function currently not used */
uint32 f32RotateSymMat(
    CONSTP2CONST(stf32SymMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) A,
    CONSTP2VAR(stf32SymMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) C,
    const float32 f32Sine,
    const float32 f32Cos) {
    uint32 u32Success;
    const float32 f32DiagSq = f32Cos * f32Cos;
    const float32 f32NoDiagSq = f32Sine * f32Sine;
    const float32 f32DiagNonDiag = f32Cos * f32Sine;

    const float32 f32Value00 =
        A->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_00];
    const float32 f32Value01 =
        A->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_01];
    const float32 f32Value11 =
        A->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_11];
    const float32 f32Value02 =
        A->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_02];
    const float32 f32Value12 =
        A->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_12];
    const float32 f32Value22 =
        A->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_22];
    const float32 f32Value03 =
        A->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_03];

    const float32 f32Value13 =
        A->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_13];
    const float32 f32Value32 =
        A->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_23];
    const float32 f32Value33 =
        A->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_33];
    const float32 f32Value04 =
        A->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_04];
    const float32 f32Value14 =
        A->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_14];
    const float32 f32Value24 =
        A->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_24];
    const float32 f32Value34 =
        A->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_34];
    const float32 f32Value44 =
        A->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_44];
    const float32 f32Value05 =
        A->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_05];
    const float32 f32Value15 =
        A->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_15];
    const float32 f32Value25 =
        A->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_25];
    const float32 f32Value35 =
        A->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_35];
    const float32 f32Value45 =
        A->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_45];
    const float32 f32Value55 =
        A->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_55];

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if ((A == NULL_PTR) || (C == NULL_PTR)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_ROTATE_SYMMAT);
    } else
#endif
        if (A->u16Size > TUEOBJFUSN_MATRIX_SIZE) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_ROTATE_SYMMAT);
    } else if (A->u16Size < TRACKABLE_VELX) {
        /* Matrix too small, add error handling */
        u32Success = TUEOBJFUSN_ERROR_INVALID_INPUT;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_ROTATE_SYMMAT);
    } else
#endif
    {
        C->u16Size = A->u16Size;

        if (A->u16Size == 3u) /* PRQA S 3120 */ /* math */
        {
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_00] =
                ((f32DiagSq * f32Value00) -
                 ((FLT_TWO * f32DiagNonDiag) * f32Value01)) +
                (f32NoDiagSq * f32Value11);
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_01] =
                ((-f32NoDiagSq + f32DiagSq) * f32Value01) +
                (f32DiagNonDiag * (f32Value00 - f32Value11));
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_11] =
                (f32DiagSq * f32Value11) +
                ((FLT_TWO * f32DiagNonDiag) * f32Value01) +
                (f32NoDiagSq * f32Value00);
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_02] =
                (f32Cos * f32Value02) - (f32Sine * f32Value12);
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_12] =
                (f32Sine * f32Value02) + (f32Cos * f32Value12);
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_22] = f32Value22;
        } else if (A->u16Size == 4u) /* PRQA S 3120 */ /* math */
        {
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_00] =
                ((f32DiagSq * f32Value00) -
                 ((FLT_TWO * f32DiagNonDiag) * f32Value01)) +
                (f32NoDiagSq * f32Value11);
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_01] =
                ((-f32NoDiagSq + f32DiagSq) * f32Value01) +
                (f32DiagNonDiag * ((f32Value00) - (f32Value11)));
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_11] =
                (f32DiagSq * f32Value11) +
                ((FLT_TWO * f32DiagNonDiag) * f32Value01) +
                (f32NoDiagSq * f32Value00);
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_02] =
                ((f32DiagSq * (f32Value02)) + (f32NoDiagSq * (f32Value13))) -
                (f32DiagNonDiag * ((f32Value12) + (f32Value03)));
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_12] =
                ((f32DiagSq * (f32Value12)) - (f32NoDiagSq * (f32Value03))) +
                (f32DiagNonDiag * ((-(f32Value13)) + f32Value02));
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_22] =
                ((f32DiagSq * f32Value22) -
                 ((FLT_TWO * f32DiagNonDiag) * f32Value32)) +
                (f32NoDiagSq * f32Value33);
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_03] =
                ((f32DiagSq * (f32Value03)) - (f32NoDiagSq * (f32Value12))) +
                (f32DiagNonDiag * ((f32Value02) - (f32Value13)));
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_13] =
                (f32DiagSq * f32Value13) + (f32NoDiagSq * f32Value02) +
                (f32DiagNonDiag * (f32Value12 + f32Value03));
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_23] =
                (((-f32NoDiagSq) + f32DiagSq) * (f32Value32)) +
                (f32DiagNonDiag * ((f32Value22) - (f32Value33)));
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_33] =
                (f32DiagSq * f32Value33) +
                ((FLT_TWO * f32DiagNonDiag) * f32Value32) +
                (f32NoDiagSq * f32Value22);
        } else if (A->u16Size == 5u) /* PRQA S 3120 */ /* math */
        {
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_00] =
                ((f32DiagSq * f32Value00) -
                 ((FLT_TWO * f32DiagNonDiag) * f32Value01)) +
                (f32NoDiagSq * f32Value11);
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_01] =
                (((-f32NoDiagSq) + f32DiagSq) * (f32Value01)) +
                (f32DiagNonDiag * ((f32Value00) - (f32Value11)));
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_11] =
                (f32DiagSq * f32Value11) +
                ((FLT_TWO * f32DiagNonDiag) * f32Value01) +
                (f32NoDiagSq * f32Value00);
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_02] =
                ((f32DiagSq * (f32Value02)) + (f32NoDiagSq * (f32Value13))) -
                (f32DiagNonDiag * ((f32Value12) + (f32Value03)));
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_12] =
                ((f32DiagSq * (f32Value12)) - (f32NoDiagSq * (f32Value03))) +
                (f32DiagNonDiag * ((-(f32Value13)) + (f32Value02)));
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_22] =
                ((f32DiagSq * f32Value22) -
                 ((FLT_TWO * f32DiagNonDiag) * f32Value32)) +
                (f32NoDiagSq * f32Value33);
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_03] =
                ((f32DiagSq * (f32Value03)) - (f32NoDiagSq * (f32Value12))) +
                (f32DiagNonDiag * ((f32Value02) - (f32Value13)));
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_13] =
                (f32DiagSq * f32Value13) + (f32NoDiagSq * f32Value02) +
                (f32DiagNonDiag * (f32Value12 + f32Value03));
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_23] =
                ((-f32NoDiagSq + f32DiagSq) * f32Value32) +
                (f32DiagNonDiag * ((f32Value22) - (f32Value33)));
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_33] =
                (f32DiagSq * f32Value33) +
                ((FLT_TWO * f32DiagNonDiag) * f32Value32) +
                (f32NoDiagSq * f32Value22);
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_04] =
                (f32Cos * f32Value04) - (f32Sine * f32Value14);
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_14] =
                (f32Sine * f32Value04) + (f32Cos * f32Value14);
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_24] =
                (f32Cos * f32Value24) - (f32Sine * f32Value34);
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_34] =
                (f32Sine * f32Value24) + (f32Cos * f32Value34);
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_44] = f32Value44;
        } else if (A->u16Size == 6u) /* PRQA S 3120 */ /* math */
        {
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_00] =
                ((f32DiagSq * f32Value00) -
                 ((FLT_TWO * f32DiagNonDiag) * f32Value01)) +
                (f32NoDiagSq * f32Value11);
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_01] =
                ((-f32NoDiagSq + f32DiagSq) * f32Value01) +
                (f32DiagNonDiag * ((f32Value00) - (f32Value11)));
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_11] =
                (f32DiagSq * f32Value11) +
                ((FLT_TWO * f32DiagNonDiag) * f32Value01) +
                (f32NoDiagSq * f32Value00);
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_02] =
                ((f32DiagSq * f32Value02) + (f32NoDiagSq * (f32Value13))) -
                (f32DiagNonDiag * ((f32Value12) + (f32Value03)));
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_12] =
                ((f32DiagSq * f32Value12) - (f32NoDiagSq * (f32Value03))) +
                (f32DiagNonDiag * ((-(f32Value13)) + (f32Value02)));
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_22] =
                ((f32DiagSq * f32Value22) -
                 ((FLT_TWO * f32DiagNonDiag) * f32Value32)) +
                (f32NoDiagSq * f32Value33);
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_03] =
                ((f32DiagSq * (f32Value03)) - (f32NoDiagSq * (f32Value12))) +
                (f32DiagNonDiag * ((f32Value02) - (f32Value13)));
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_13] =
                (f32DiagSq * f32Value13) + (f32NoDiagSq * f32Value02) +
                (f32DiagNonDiag * (f32Value12 + f32Value03));
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_23] =
                (((-f32NoDiagSq) + f32DiagSq) * (f32Value32)) +
                (f32DiagNonDiag * ((f32Value22) - (f32Value33)));
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_33] =
                (f32DiagSq * f32Value33) +
                ((FLT_TWO * f32DiagNonDiag) * f32Value32) +
                (f32NoDiagSq * f32Value22);
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_04] =
                ((f32DiagSq * (f32Value04)) + (f32NoDiagSq * (f32Value15))) -
                (f32DiagNonDiag * ((f32Value14) + (f32Value05)));
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_14] =
                ((f32DiagSq * (f32Value14)) - (f32NoDiagSq * (f32Value05))) +
                (f32DiagNonDiag * ((-(f32Value15)) + (f32Value04)));
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_24] =
                ((f32DiagSq * (f32Value24)) + (f32NoDiagSq * (f32Value35))) -
                (f32DiagNonDiag * ((f32Value34) + (f32Value25)));
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_34] =
                ((f32DiagSq * (f32Value34)) - (f32NoDiagSq * (f32Value25))) +
                (f32DiagNonDiag * ((-(f32Value35)) + (f32Value24)));
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_44] =
                ((f32DiagSq * f32Value44) -
                 ((FLT_TWO * f32DiagNonDiag) * f32Value45)) +
                (f32NoDiagSq * f32Value55);
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_05] =
                ((f32DiagSq * (f32Value05)) - (f32NoDiagSq * (f32Value14))) +
                (f32DiagNonDiag * ((f32Value04) - (f32Value15)));
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_15] =
                (f32DiagSq * f32Value15) + (f32NoDiagSq * f32Value04) +
                (f32DiagNonDiag * (f32Value14 + f32Value05));
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_25] =
                ((f32DiagSq * (f32Value25)) - (f32NoDiagSq * (f32Value34))) +
                (f32DiagNonDiag * ((f32Value24) - (f32Value35)));
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_35] =
                (f32DiagSq * f32Value35) + (f32NoDiagSq * f32Value24) +
                (f32DiagNonDiag * (f32Value34 + f32Value25));
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_45] =
                ((-f32NoDiagSq + f32DiagSq) * (f32Value45)) +
                (f32DiagNonDiag * ((f32Value44) - (f32Value55)));
            C->data[TUE_PRV_COMMOM_MATRIX_SYM_MATRIX_INDEX_55] =
                ((f32DiagSq * f32Value55) +
                 ((FLT_TWO * f32DiagNonDiag) * f32Value45)) +
                (f32NoDiagSq * f32Value44);
        } else {
            /* MISRA */
        }

        u32Success = TUEOBJFUSN_ERROR_NOERROR;
    }

    return u32Success;
}

/**
 * @brief performs vector substraction C = A-B, wheras it is only calculated for
 * mininmal number of rows from A,B
 *
 * - vector can be of any dimensions n, 1 <= n <= @ref TUEOBJFUSN_MATRIX_SIZE
 * - the minimal size of both vectors is taken for C
 * - memory of vector C must be allocated before function call
 * - Complexity: O(n).
 *
 * @param[in]  A   const stf32Vec_t * const, input vector A
 * @param[in]  B   const stf32Vec_t * const, input vector  B
 * @param[out] C   stf32Vec_t * const, output vector C = A - B
 * @return error code
 */
/* PRQA S 1532 2 */ /* Library Function */
uint32 f32VecSubPart(CONSTP2CONST(stf32Vec_t, AUTOMATIC, ObjFusn_VAR_NOINIT) A,
                     CONSTP2CONST(stf32Vec_t, AUTOMATIC, ObjFusn_VAR_NOINIT) B,
                     CONSTP2VAR(stf32Vec_t, AUTOMATIC, ObjFusn_VAR_NOINIT) C) {
    uint32 u32Success;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if ((NULL_PTR == A) || (NULL_PTR == B) || (NULL_PTR == C)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32VECSUBPART);
    } else if (A == B)  // C can be equal to A or B
    {
        u32Success = TUEOBJFUSN_ERROR_INVALID_POINTER;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_INVALID_POINTER, TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32VECSUBPART);
    } else
#endif
        if ((TUEOBJFUSN_MATRIX_SIZE < A->nRows) ||
            (TUEOBJFUSN_MATRIX_SIZE < B->nRows)) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_F32VECSUBPART);
    } else
#endif
    {
        C->nRows = tue_prv_fusion_min_U16(A->nRows, B->nRows);

#if TUEOBJFUSN_MATRIX_SIZE != 6
        uint16 u16i;
        for (u16i = 0u; u16i < C->nRows; u16i++) {
            C->data[u16i] = A->data[u16i] - B->data[u16i];
        }
#else
        C->data[0] = A->data[0] - B->data[0];
        C->data[1] = A->data[1] - B->data[1];
        C->data[2] = A->data[2] - B->data[2];
        C->data[3] = A->data[3] - B->data[3];
        C->data[4] = A->data[4] - B->data[4];
        C->data[5] = A->data[5] - B->data[5];
#endif
        u32Success = TUEOBJFUSN_ERROR_NOERROR;
    }

    return u32Success;
}

/**
 * @brief Implements matrix matrix addition C = A (DIAG) + B whereas A,B and C
 * are assumed to be symmetric CURRENTLY from A only the diagonal elements are
 * taken. It only accumulates the smallest size of A,B,
 *
 * number of rows and columns must not exceed @ref TUEOBJFUSN_MATRIX_SIZE
 * @param[out] C      stf32SymMatrix_t * const, symetric output matrix C - C
 * don't has to be initialized!!!s
 * @param[in]  A      const stf32SymMatrix_t * const, symmetric input matrix A
 * (only diagonal elements are taken !!!)
 * @param[in]  B      const stf32SymMatrix_t * const, symmetric input matrix B
 */
/* PRQA S 1532 2 */ /* Library function currently not used */
uint32 f32SymMatAddSymMatPart(
    CONSTP2CONST(stf32SymMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) A,
    CONSTP2CONST(stf32SymMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) B,
    CONSTP2VAR(stf32SymMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) C) {
    uint32 u32Success;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if ((NULL_PTR == A) || (NULL_PTR == B) || (NULL_PTR == C)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_ADD_SYM_MAT_PART);
    } else if ((A == B) || (A == C))  // C can be equal to A or B
    {
        u32Success = TUEOBJFUSN_ERROR_INVALID_POINTER;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_INVALID_POINTER, TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_ADD_SYM_MAT_PART);
    } else
#endif
        if ((TUEOBJFUSN_MATRIX_SIZE < B->u16Size) ||
            (TUEOBJFUSN_MATRIX_SIZE < A->u16Size)) {
        u32Success = TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE;
        (void)tue_prv_error_management_addError(
            TUEOBJFUSN_ERROR_MATRIX_EXCEEDS_MAXIMUM_SIZE,
            TUEOBJFUSN_AAU_COMMON_MATRIX,
            TUEOBJFUSN_AAU_COMMON_MATRIX_ADD_SYM_MAT_PART);
    } else
#endif
    {
        /* takes only minimum size of both matrices and stores them in C */
        C->u16Size = tue_prv_fusion_min_U16(A->u16Size, B->u16Size);

        C->data[TRACKABLE_INDEX_VARIANCE_POSX] =
            A->data[TRACKABLE_INDEX_VARIANCE_POSX] +
            B->data[TRACKABLE_INDEX_VARIANCE_POSX];
        C->data[TRACKABLE_INDEX_COVARIANCE_POSX_POSY] =
            B->data[TRACKABLE_INDEX_COVARIANCE_POSX_POSY];
        C->data[TRACKABLE_INDEX_VARIANCE_POSY] =
            A->data[TRACKABLE_INDEX_VARIANCE_POSY] +
            B->data[TRACKABLE_INDEX_VARIANCE_POSY];
        C->data[TRACKABLE_INDEX_COVARIANCE_POSX_VELX] =
            B->data[TRACKABLE_INDEX_COVARIANCE_POSX_VELX];
        C->data[TRACKABLE_INDEX_COVARIANCE_POSY_VELX] =
            B->data[TRACKABLE_INDEX_COVARIANCE_POSY_VELX];
        C->data[TRACKABLE_INDEX_VARIANCE_VELX] =
            A->data[TRACKABLE_INDEX_VARIANCE_VELX] +
            B->data[TRACKABLE_INDEX_VARIANCE_VELX];
        C->data[TRACKABLE_INDEX_COVARIANCE_POSX_VELY] =
            B->data[TRACKABLE_INDEX_COVARIANCE_POSX_VELY];
        C->data[TRACKABLE_INDEX_COVARIANCE_POSY_VELY] =
            B->data[TRACKABLE_INDEX_COVARIANCE_POSY_VELY];
        C->data[TRACKABLE_INDEX_COVARIANCE_VELX_VELY] =
            B->data[TRACKABLE_INDEX_COVARIANCE_VELX_VELY];
        C->data[TRACKABLE_INDEX_VARIANCE_VELY] =
            A->data[TRACKABLE_INDEX_VARIANCE_VELY] +
            B->data[TRACKABLE_INDEX_VARIANCE_VELY];
        C->data[TRACKABLE_INDEX_COVARIANCE_POSX_ACCX] =
            B->data[TRACKABLE_INDEX_COVARIANCE_POSX_ACCX];
        C->data[TRACKABLE_INDEX_COVARIANCE_POSY_ACCX] =
            B->data[TRACKABLE_INDEX_COVARIANCE_POSY_ACCX];
        C->data[TRACKABLE_INDEX_COVARIANCE_VELX_ACCX] =
            B->data[TRACKABLE_INDEX_COVARIANCE_VELX_ACCX];
        C->data[TRACKABLE_INDEX_COVARIANCE_VELY_ACCX] =
            B->data[TRACKABLE_INDEX_COVARIANCE_VELY_ACCX];
        C->data[TRACKABLE_INDEX_VARIANCE_ACCX] =
            A->data[TRACKABLE_INDEX_VARIANCE_ACCX] +
            B->data[TRACKABLE_INDEX_VARIANCE_ACCX];
        C->data[TRACKABLE_INDEX_COVARIANCE_POSX_ACCY] =
            B->data[TRACKABLE_INDEX_COVARIANCE_POSX_ACCY];
        C->data[TRACKABLE_INDEX_COVARIANCE_POSY_ACCY] =
            B->data[TRACKABLE_INDEX_COVARIANCE_POSY_ACCY];
        C->data[TRACKABLE_INDEX_COVARIANCE_VELX_ACCY] =
            B->data[TRACKABLE_INDEX_COVARIANCE_VELX_ACCY];
        C->data[TRACKABLE_INDEX_COVARIANCE_VELY_ACCY] =
            B->data[TRACKABLE_INDEX_COVARIANCE_VELY_ACCY];
        C->data[TRACKABLE_INDEX_COVARIANCE_ACCX_ACCY] =
            B->data[TRACKABLE_INDEX_COVARIANCE_ACCX_ACCY];
        C->data[TRACKABLE_INDEX_VARIANCE_ACCY] =
            A->data[TRACKABLE_INDEX_VARIANCE_ACCY] +
            B->data[TRACKABLE_INDEX_VARIANCE_ACCY];

        u32Success = TUEOBJFUSN_ERROR_NOERROR;
    }

    return u32Success;
}

#define ObjFusn_STOP_SEC_CODE

#ifdef UNITTEST
/**
 * @brief multiplies a vector with a scalar value
 *
 * number of rows must not exceed @ref TUEOBJFUSN_MATRIX_SIZE
 * @param[in,out]A       stf32Vec_t * const, input and output vector
 * @param[in]f32Scal     f32_t, the scalar
 *
 * @return TRUE (ok) or FALSE (error occured)
 */
boolean f32VecMulScal(stf32Vec_t *const A, float32 f32Scal) {
    boolean bSuccess = TRUE;
    uint16 u16i = 0u;
    if (A->nRows > TUEOBJFUSN_MATRIX_SIZE) {
        bSuccess = FALSE;
    } else {
        for (u16i = 0u; u16i < A->nRows; u16i++) {
            A->data[u16i] *= f32Scal;
        }
    }

    return bSuccess;
}
#endif /* UNITTEST */

/**
 * \}
 */
