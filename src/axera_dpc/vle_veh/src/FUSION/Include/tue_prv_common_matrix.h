/** \addtogroup matrix
 *  @{
 * \file        tue_prv_common_matrix.h
 *
 *
 *
 * <br>=====================================================<br>
 * <b>Copyright 2013 by Tuerme.</b>
 *
 *  All rights reserved. Property of Tuerme.<br>
 *  Restricted rights to use, duplicate or disclose of this code<br>
 *  are granted through contract.
 * <br>=====================================================<br>
 */

#ifndef TUE_PRV_COMMON_MATRIX_H
#define TUE_PRV_COMMON_MATRIX_H

#ifdef __cplusplus
extern "C" {
#endif

/*==================[inclusions]============================================*/
#include "tue_prv_common_types.h"

/*==================[macros]================================================*/

/**
 * maximum number of matrix rows/cols for matrix computaqtion.
 * this value depends on the number of object states that are processed in
 * kalman filter estimation and distance measurement.
 * for const-velocity applications choose 4
 * for const-acceleration applications choose 6
 */
#define TUEOBJFUSN_MATRIX_SIZE (6u)

/**
 * size of data buffer used to store a symmetric matrix of size
 * TUEOBJFUSN_MATRIX_SIZE x TUEOBJFUSN_MATRIX_SIZE
 */
#define TUEOBJFUSN_SYMMETRIC_MATRIX_SIZE \
    ((TUEOBJFUSN_MATRIX_SIZE * (TUEOBJFUSN_MATRIX_SIZE + 1u)) / 2u)

/*==================[type definitions]======================================*/

/**
 * @brief structure to store a matrix
 */
typedef struct {
    float32 data[TUEOBJFUSN_MATRIX_SIZE][TUEOBJFUSN_MATRIX_SIZE]; /**< data */
    uint16 nRows; /**< number of rows */
    uint16 nCols; /**< number of columns */
} stf32Matrix_t;

/**
 * @brief structure to store a vector
 */
typedef struct {
    float32 data[TUEOBJFUSN_MATRIX_SIZE]; /**< data */
    uint16 nRows;                         /**< number of rows */
    uint16 u16Reserved;
} stf32Vec_t;

/**
 * @brief structure to store a symmetric matrix
 */
typedef struct {
    float32 data[TUEOBJFUSN_SYMMETRIC_MATRIX_SIZE];
    uint16 u16Size;
    uint16 u16Reserved;
} stf32SymMatrix_t;

/*==================[forward declarations]==================================*/
/*==================[symbolic constants]====================================*/
/*==================[return codes]==========================================*/
/*==================[functions]=============================================*/

/*==================[functions]============================================*/

#define ObjFusn_START_SEC_CODE

#include "FusionCompiler.h"

uint32 f32MatAdd(CONSTP2CONST(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) A,
                 CONSTP2CONST(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) B,
                 CONSTP2VAR(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) C);

uint32 f32MatSub(CONSTP2CONST(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) A,
                 CONSTP2CONST(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) B,
                 CONSTP2VAR(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) C);

uint32 f32MatMul(CONSTP2CONST(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) A,
                 CONSTP2CONST(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) B,
                 CONSTP2VAR(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) C);

uint32 f32MatTranspose(
    CONSTP2CONST(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) A,
    CONSTP2VAR(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) B);

uint32 f32MatInvSym3x3_Sym(CONSTP2VAR(stf32SymMatrix_t,
                                      AUTOMATIC,
                                      ObjFusn_VAR_NOINIT) MatrixA);

uint32 f32SymMatDet(CONSTP2CONST(stf32SymMatrix_t,
                                 AUTOMATIC,
                                 ObjFusn_VAR_NOINIT) A,
                    CONSTP2VAR(float32, AUTOMATIC, ObjFusn_VAR_NOINIT) f32det);

uint32 f32MatInv(CONSTP2VAR(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT)
                     MatrixA);

uint32 f32MatDet(CONSTP2CONST(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) A,
                 CONSTP2VAR(float32, AUTOMATIC, ObjFusn_VAR_NOINIT) f32det);

uint32 f32MatZeros(CONSTP2VAR(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) A,
                   const uint16 u16nRows,
                   const uint16 u16nCols);

uint32 f32MatEye(CONSTP2VAR(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) A,
                 const uint16 u16nRows,
                 const uint16 u16nCols);

extern void f32MatInit(CONSTP2VAR(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT)
                           A);

uint32 f32MatMulVec(CONSTP2CONST(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT)
                        A,
                    CONSTP2CONST(stf32Vec_t, AUTOMATIC, ObjFusn_VAR_NOINIT) B,
                    CONSTP2VAR(stf32Vec_t, AUTOMATIC, ObjFusn_VAR_NOINIT) C);

uint32 f32CopySymMat(
    CONSTP2VAR(stf32SymMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) pDest,
    CONSTP2CONST(stf32SymMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) pSrc);

uint32 f32VecAdd(CONSTP2CONST(stf32Vec_t, AUTOMATIC, ObjFusn_VAR_NOINIT) A,
                 CONSTP2CONST(stf32Vec_t, AUTOMATIC, ObjFusn_VAR_NOINIT) B,
                 CONSTP2VAR(stf32Vec_t, AUTOMATIC, ObjFusn_VAR_NOINIT) C);

uint32 f32VecSub(CONSTP2CONST(stf32Vec_t, AUTOMATIC, ObjFusn_VAR_NOINIT) A,
                 CONSTP2CONST(stf32Vec_t, AUTOMATIC, ObjFusn_VAR_NOINIT) B,
                 CONSTP2VAR(stf32Vec_t, AUTOMATIC, ObjFusn_VAR_NOINIT) C);

uint32 f32VecZeros(CONSTP2VAR(stf32Vec_t, AUTOMATIC, ObjFusn_VAR_NOINIT) A,
                   const uint16 u16nRows);

uint32 f32CopyVec(CONSTP2VAR(stf32Vec_t, AUTOMATIC, ObjFusn_VAR_NOINIT) pDest,
                  CONSTP2CONST(stf32Vec_t, AUTOMATIC, ObjFusn_VAR_NOINIT) pSrc);

extern void f32VecInit(CONSTP2VAR(stf32Vec_t, AUTOMATIC, ObjFusn_VAR_NOINIT) A);

/* Function definitions for symmetric matrices */
uint32 f32SymMatZeros(CONSTP2VAR(stf32SymMatrix_t,
                                 AUTOMATIC,
                                 ObjFusn_VAR_NOINIT) A,
                      const uint16 u16Size);

uint32 f32MatToSymMat(
    CONSTP2CONST(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) A,
    CONSTP2VAR(stf32SymMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) B);

uint32 f32SymMatToMat(
    CONSTP2CONST(stf32SymMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) A,
    CONSTP2VAR(stf32Matrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) B);

uint32 f32SymMatAddSymMat(
    CONSTP2CONST(stf32SymMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) A,
    CONSTP2CONST(stf32SymMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) B,
    CONSTP2VAR(stf32SymMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) C);

uint32 f32RotateSymMat(
    CONSTP2CONST(stf32SymMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) A,
    CONSTP2VAR(stf32SymMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) C,
    const float32 f32Sine,
    const float32 f32Cos);

uint32 f32SymMatEye(CONSTP2VAR(stf32SymMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT)
                        A,
                    const uint16 u16Size);

extern void f32SymMatInit(CONSTP2VAR(stf32SymMatrix_t,
                                     AUTOMATIC,
                                     ObjFusn_VAR_NOINIT) A);

uint32 f32VecSubPart(CONSTP2CONST(stf32Vec_t, AUTOMATIC, ObjFusn_VAR_NOINIT) A,
                     CONSTP2CONST(stf32Vec_t, AUTOMATIC, ObjFusn_VAR_NOINIT) B,
                     CONSTP2VAR(stf32Vec_t, AUTOMATIC, ObjFusn_VAR_NOINIT) C);

uint32 f32SymMatAddSymMatPart(
    CONSTP2CONST(stf32SymMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) A,
    CONSTP2CONST(stf32SymMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) B,
    CONSTP2VAR(stf32SymMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) C);

#define ObjFusn_STOP_SEC_CODE

#ifdef UNITTEST
boolean f32VecMulScal(stf32Vec_t* const A, float32 f32Scal);
#endif

/*==================[external function declarations]========================*/
/*==================[external constants]====================================*/
/*==================[external data]=========================================*/

#ifdef __cplusplus
}
#endif

#endif /* TUE_PRV_COMMON_MATRIX_H */
       /**@}==================[end of
        * file]===========================================*/
