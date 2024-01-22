
/**
@defgroup mtrx MTRX (matrix computations)
  @ingroup cml
@{ */

/*****************************************************************************
  QA-C
*****************************************************************************/
/* Check if we are in QA-C: PRQA_SIZE_T is defined in QA-C environment. */
#ifdef PRQA_SIZE_T
/* Switch off QA-C warnings on side effects for macro, which don't have
   any side effects and will sure never have some. The macro definition
   does not hold good for C90 compiler and this case is taken care in its
   definition.*/
#pragma PRQA_MACRO_MESSAGES_OFF "ODPR_CML_CreateMatrixLocal" 1031
#endif
/*****************************************************************************
  INCLUDE PROTECTION
*****************************************************************************/

/* allow inclusion of cml sub-headers only if external cml header is included */
//#ifndef _ORPR_CML_MTRX_INCLUDED
//  #pragma message(__FILE__": Inclusion of orpr_cml_mtrx.h is discouraged. It
//  exists only for compatibility with ARS3xx and might be deleted without prior
//  notice. Include cml_ext.h instead.")
//#endif /* #ifdef _ORPR_CML_MTRX_INCLUDED */

#ifndef _ORPR_CML_MTRX_INCLUDED
#define _ORPR_CML_MTRX_INCLUDED

#include "TM_Global_Types.h"

#define ODPR_CML_INLINE __inline
#define ODPR_CML_RESTRICT
#define ODPR_CML_MatrixBoundsCheckOn (0)

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

///< \brief       The Matrix Descriptor Structure
///< \description The matrix descriptor structure contains the description
///<              of the matrix (row count, column count and max size) reqired
/// for matrix operations.
typedef struct {
    uint8 col;       ///< Number of columns in the matrix
    uint8 row;       ///< Number of rows in the matrix
    uint16 maxsize;  ///< Maximum memory size allocated for this matrix
} ODPR_CML_t_MatrixDescriptor;

/**/
///< \brief       The Matrix Data Structure
///< \description The matrix data structure contains the descriptor structure
/// element
///<              which contains row count, column count and max size reqired
/// for matrix operations.
///<              The matrix data is linked via a pointer. This allows using a
///<              single matrix datatype, for all matrix dimensions. \n
///<              It also allows the separation of data and header for saving
///<              memory space: the matrix data can be stored on the heap, while
///<              the wrapper can be created temporarily on the stack when
/// needed
///<              for computations.
typedef struct {
    ODPR_CML_t_MatrixDescriptor
        Desc;  ///< Matrix descriptor (dimensions, size, etc.)
    float32*
        pData;  ///< Pointer to the memory location where the data is stored */
} ODPR_CML_t_Matrix;

/*****************************************************************************
  MACROS
*****************************************************************************/

/*! creates matrix and allocates new payload data with global scope */
#define ODPR_CML_CreateMatrix(name, rows, cols)                      \
    static float32 fMtrxData##name[(uint32)(rows) * (uint32)(cols)]; \
    ODPR_CML_t_Matrix AlgoMtrxHeader##name = {                       \
        {(uint8)(cols), (uint8)(rows),                               \
         (uint16)((uint32)(rows) * (uint32)(cols))},                 \
        fMtrxData##name};                                            \
    ODPR_CML_t_Matrix* name =                                        \
        &AlgoMtrxHeader##name;  ///< Creates matrix and allocates new payload
                                /// data with global scope

#if (defined(_MSC_VER) || defined(__DCC__))
#define ODPR_CML_MATRIXLOCAL
#elif (defined(__STDC_VERSION__))
#if ((__STDC_VERSION__ >= 199901)) /* C99 compatible compiler */
#define ODPR_CML_MATRIXLOCAL
#endif
#endif

#ifdef ODPR_CML_MATRIXLOCAL
#define ODPR_CML_CreateMatrixLocal(name, rows, cols)                \
    float32 fMtrxData##name[(uint32)(rows) * (uint32)(cols)];       \
    ODPR_CML_t_Matrix AlgoMtrxHeader##name =                        \
        ODPR_CML_initMatrixHeader((cols), (rows), fMtrxData##name); \
    ODPR_CML_t_Matrix* name = &AlgoMtrxHeader##name;  ///< Creates matrix and
                                                      /// allocates new payload
/// data with local scope
#else
#define ODPR_CML_CreateMatrixLocal(name, rows, cols) \
    ODPR_CML_CreateMatrix(                           \
        name, rows, cols)  ///< Creates matrix and allocates new payload data
#endif

/* PRQA L:CREATEMATRIX */

#if (ODPR_CML_MatrixBoundsCheckOn)
#define ODPR_CML_GetMatrixElement(name, Row, Col) \
    (name)->pData[(uint32)(Col) +                 \
                  ((uint32)(Row) *                \
                   (name)->Desc.col)]  ///<  Access to matrix element
#else
/* (NO OVERFLOW CHECK!!) */
#define ODPR_CML_GetMatrixElement(name, Row, Col) \
    (name)->pData[(uint32)(Col) +                 \
                  ((uint32)(Row) *                \
                   (name)->Desc.col)]  ///<  Access to matrix element
#endif

/* Deactivate QA-C warning 3412; Reviewer: S. Schwarzkopf;
   Date: 04.12.2014; Reason: As this is an interface and proven in use, no
   changes shall be made here.
   Review-ID: 3942463 */
/* PRQA S 3412 5 */
#if (ODPR_CML_MatrixBoundsCheckOn)
#define ODPR_CML_SetMatrixSize(name, rows, cols) \
    if (rows * cols <= (name)->Desc.maxsize) {   \
        (name)->Desc.col = cols;                 \
        (name)->Desc.row = rows;                 \
    } else {                                     \
        CML_ASSERT(b_FALSE);                     \
    };  ///< Set new matrix size
#else
/* (NO OVERFLOW CHECK!!) */
#define ODPR_CML_SetMatrixSize(name, rows, cols) \
    (name)->Desc.col = (cols);                   \
    (name)->Desc.row = (rows);  ///< Set new matrix size
#endif

#define ODPR_CML_IsMatrixEmpty(name)   \
    (((name)->Desc.col == (uint8)0) || \
     ((name)->Desc.row == (uint8)0))  ///< This checks if matrix is empty

/*****************************************************************************
  FUNCTION DECLARATIONS
*****************************************************************************/

///*! helper function for creation of matrices with local scope */
// ODPR_CML_INLINE ODPR_CML_t_Matrix ODPR_CML_initMatrixHeader(uint32 u_ColNr,
// uint32 u_RowNr, float32* p_f_MtrxData);
//
/*****************************************************************************
  Functionname:    CML_v_initMatrix                                      */ /*!

               \brief           Matrix initialization with a const value

               \description     This function initializes all the elements of
             the
             matrix
                                with a const value(F).
               \InOutCorrelation
                                \f[ \begin{bmatrix}
                                M_{00}    &   M_{01}\\
                                M_{10}    &   M_{11}
                                \end{bmatrix}
                            =
                                \begin{bmatrix}
                                F   &   F\\
                                F   &   F
                                \end{bmatrix}
                                \f]
               \attention
                                RowNr and ColNr are expected to be values not
                                exceeding 8 bits.\\
                                u_RowNr*u_ColNr must not exceed A->Desc.maxsize.
               \param[in,out]   p_Matrix :  matrix o be filled
                                            Range for p_Matrix->Desc.maxsize
             [Full
             range
             of uint16]
               \param[out]      f_Val :     value used for filling
                                            [Full range of float32]
               \param[in]       u_RowNr :   Row dimension of the matrix to be
             created
                                            [Full range of uint8]
               \param[in]       u_ColNr :   Column dimension of the matrix to be
             created
                                            [Full range of uint8]

               \return          void

               \author

               \testmethod

               \traceability
             *****************************************************************************/
void ODPR_CML_v_InitMatrix(ODPR_CML_t_Matrix* p_Matrix,
                           uint32 u_RowNr,
                           uint32 u_ColNr,
                           float32 f_Val);

///*****************************************************************************
//  Functionname:    ODPR_CML_v_createIdentityMatrix *//*!
//
//  \brief           Initialises matrix with identity matrix
//
//  \description     This function initializes the given matrix with an
//                   identity matrix of the provided size.
//  \InOutCorrelation
//                   \f[ \begin{bmatrix}
//                   M_{00}    &   M_{01}\\
//                   M_{10}    &   M_{11}
//                   \end{bmatrix}
//               =
//                   \begin{bmatrix}
//                   1 &   0\\
//                   0 &   1
//                   \end{bmatrix}
//                   \f]
//  \attention
//                   NOTE: Value for row/column is expected not to exceed 8 bits
//                   CAUTION: Size of the matrix (rows x columns) must not
//                   exceed p_Matrix->Desc.maxsize
//                   p_Matrix->pData should hold a valid address.
//  \param[in,out]   p_Matrix :  matrix o be filled (square matrix)
//                               Range for p_Matrix->Desc.maxsize [Full range of
//                               uint16]
//  \param[in]       u_Size :    no. of row/col square matrix (u_Size x u_Size)
//                               [Full range of uint8]
//
//  \return          void
//
//  \author
//
//  \testmethod
//
//  \traceability
//*****************************************************************************/
void ODPR_CML_v_createIdentityMatrix(ODPR_CML_t_Matrix* p_Matrix,
                                     uint32 u_Size);
//
//
///*****************************************************************************
//  Functionname:    CML_v_addMatrices                                    *//*!
//
//  \brief           Matrix addition (inplace/outplace). A or B can be same as
//  Res
//
//  \description     This function performs matrix addition (inplace/outplace)
//  of
//                   two matrices A and B with same dimesions and store the
//                   result
//                   in a resultant matrix.
//                   The matrix A or B can be same as resultant matrix.
//                    \f[[Res] = [A] + [B]\f]
//  \InOutCorrelation
//                   \f[ \begin{bmatrix}
//                   A_{00}    &   A_{01}\\
//                   A_{10}    &   A_{11}
//                   \end{bmatrix}
//               +
//                   \begin{bmatrix}
//                   B_{00}    &   B_{01}\\
//                   B_{10}    &   B_{11}
//                   \end{bmatrix}
//               =
//                   \begin{bmatrix}
//                   A_{00} +B_{00}    &   A_{01} +B_{01}\\
//                   A_{10} +B_{10}    &   A_{11} +B_{11}
//                   \end{bmatrix}
//                   \f]
//  \attention
//                   NOTE: Value for row/column is expected not to exceed 8 bits
//                   CAUTION: Size of the matrix (rows x columns) must not
//                   exceed p_MatrixRes->Desc.maxsize
//                   p_MatrixRes->pData should hold a valid address.
//  \param[in]       p_MatrixA : First Addend matrix (source)
//                               Range for p_MatrixA->Desc.row [Full range of
//                               uint8]
//                               Range for p_MatrixA->Desc.col [Full range of
//                               uint8]
//                               Range for p_MatrixA->Desc.maxsize [Full range
//                               of uint16]
//                               Range for p_MatrixA->pData
//                               [Valid pointer with data in full range of
//                               float32]
//  \param[in]       p_MatrixB : Second Addend matrix (source)
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
//  \param[out]      p_MatrixRes : Result Sum matrix
//
//  \return          void
//
//  \author
//
//  \testmethod
//
//  \traceability
//*****************************************************************************/
// void CML_v_addMatrices(ODPR_CML_t_Matrix* p_MatrixRes, const
// ODPR_CML_t_Matrix* p_MatrixA, const ODPR_CML_t_Matrix* p_MatrixB);
//
///*****************************************************************************
//  Functionname:    CML_v_subtractMatrices                               *//*!
//
//  \brief           Matrix substraction Res = A - B
//
//  \description     This function performs matrix subtraction
//  (inplace/outplace)
//                   of two matrices A and B with same dimesions and store the
//                   result
//                   in a resultant matrix.
//                   [Res] = [A] - [B]
//  \InOutCorrelation
//                   \f[ \begin{bmatrix}
//                   A_{00}    &   A_{01}\\
//                   A_{10}    &   A_{11}
//                   \end{bmatrix}
//               -
//                   \begin{bmatrix}
//                   B_{00}    &   B_{01}\\
//                   B_{10}    &   B_{11}
//                   \end{bmatrix}
//               =
//                   \begin{bmatrix}
//                   A_{00} -B_{00}    &   A_{01} -B_{01}\\
//                   A_{10} -B_{10}    &   A_{11} -B_{11}
//                   \end{bmatrix}
//                   \f]
//  \attention
//                   NOTE: Value for row/column is expected not to exceed 8 bits
//                   CAUTION: Size of the matrix (rows x columns) must not
//                   exceed p_MatrixRes->Desc.maxsize
//                   p_MatrixRes->pData should hold a valid address.
//  \param[in]       p_MatrixA : Minuend matrix
//                               Range for p_MatrixA->Desc.row [Full range of
//                               uint8]
//                               Range for p_MatrixA->Desc.col [Full range of
//                               uint8]
//                               Range for p_MatrixA->Desc.maxsize [Full range
//                               of uint16]
//                               Range for p_MatrixA->pData
//                               [Valid pointer with data in full range of
//                               float32]
//  \param[in]       p_MatrixB : Subtrahend matrix
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
//  \param[out]      p_MatrixRes : Result Difference matrix
//
//  \return          void
//
//  \author
//
//  \testmethod
//
//  \traceability
//*****************************************************************************/
// void CML_v_subtractMatrices(ODPR_CML_t_Matrix* p_MatrixRes, const
// ODPR_CML_t_Matrix* p_MatrixA, const ODPR_CML_t_Matrix* p_MatrixB);
//
//
///*****************************************************************************
//  Functionname:    CML_v_multiplyMatrices                               *//*!
//
//  \brief           Matrix multiplication
//
//  \description     This function performs matrix multiplication (outplace)
//                   of two matrices A and B and store the result in a
//                   resultant matrix.
//                   \f[[Res]_{m\times p} = [A]_{m\times n}\ \times \
//                   [B]_{n\times p}\f]
//  \InOutCorrelation
//                   \f[ \begin{bmatrix}
//                   A_{00}    &   A_{01}  & A_{02}\\
//                   A_{10}    &   A_{11}  & A_{12}
//                   \end{bmatrix}
//               \times
//                   \begin{bmatrix}
//                   B_{00}    &   B_{01}\\
//                   B_{10}    &   B_{11}\\
//                   B_{20}    &   B_{21}\\
//                   \end{bmatrix}
//               =
//                   \begin{bmatrix}
//                   A_{00}\times  B_{00} + A_{01}\times B_{10}+A_{02}\times B_{20}   &&  A_{00}\times B_{01} + A_{01}\times B_{11}+A_{02}\times B_{21}\\
//                   A_{10}\times  B_{00} + A_{11}\times B_{10}+A_{12}\times B_{20}   &&  A_{10}\times B_{01} + A_{11}\times B_{11}+A_{12}\times B_{21}\\
//                   \end{bmatrix}
//                   \f]
//
//  \attention
//                   NOTE: Value for row/column is expected not to exceed 8 bits
//                   CAUTION: Size of the matrix (p_MatrixB->Desc.col X
//                   p_MatrixA->Desc.row) must not
//                   exceed p_MatrixRes->Desc.maxsize
//                   p_MatrixA->Desc.col should be equal to p_MatrixB->Desc.row\\
//                   p_MatrixRes->pData should hold a valid address
//  \param[in]       p_MatrixA : Multiplicand matrix (source)
//                               Range for p_MatrixA->Desc.row [Full range of
//                               uint8]
//                               Range for p_MatrixA->Desc.col [Full range of
//                               uint8]
//                               Range for p_MatrixA->Desc.maxsize [Full range
//                               of uint16]
//                               Range for p_MatrixA->pData
//                               [Valid pointer with data in full range of
//                               float32]
//  \param[in]       p_MatrixB : Multiplier matrix (source)
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
//  \param[out]      p_MatrixRes : Result Product matrix
//
//  \return          void
//
//  \author
//
//  \testmethod
//
//  \traceability
//*****************************************************************************/
void ODPR_CML_v_MultiplyMatrices(ODPR_CML_t_Matrix* p_MatrixRes,
                                 const ODPR_CML_t_Matrix* p_MatrixA,
                                 const ODPR_CML_t_Matrix* p_MatrixB);
//
//
///*****************************************************************************
//  Functionname:    CML_v_multiplyMatricesToSymResult                    *//*!
//
//  \brief           Matrix multiplication if result is symmetric
//
//  \description     This function performs matrix multiplication (outplace)
//                   of two matrices A and B and store the result in a
//                   resultant matrix, if the result is known to be a symmetric
//                   matrix.
//                   [Res] = [A] x [B],
//                   [A] is of the order MxN,
//                   [B] is of the order NxM,
//                   \f[[Res]_{m\times n} = [A]_{nXm}\ X\ [B]_{m\times m} \f]
//  \InOutCorrelation
//                    \f[\begin{bmatrix}
//                   A_{00}    &   A_{01}  & A_{02}\\
//                   A_{10}    &   A_{11}  & A_{12}\\
//                   A_{20}    &   A_{21}  & A_{22}
//                   \end{bmatrix}
//               \times
//                   \begin{bmatrix}
//                   B_{00}    &   B_{01}    &   B_{02}\\
//                   B_{10}    &   B_{11}    &   B_{12}\\
//                   B_{20}    &   B_{21}    &   B_{22}\\
//                   \end{bmatrix}
//               =
//                   \begin{bmatrix}
//                   A_{00}\times B_{00} + A_{01}\times B_{10}+A_{02}\times B_{20}   &&  A_{00}\times B_{01} + A_{01}\times B_{11}+A_{02}\times B_{21}   &&  A_{00}\times B_{02} + A_{01}\times B_{12}+A_{02}\times B_{22}\\
//                   A_{10}\times B_{00} + A_{11}\times B_{10}+A_{12}\times B_{20}   &&  A_{10}\times B_{01} + A_{11}\times B_{11}+A_{12}\times B_{21}   &&  A_{10}\times B_{02} + A_{11}\times B_{12}+A_{12}\times B_{22}\\
//                   A_{20}\times B_{00} + A_{21}\times B_{10}+A_{22}\times B_{20}   &&  A_{20}\times B_{01} + A_{21}\times B_{11}+A_{22}\times B_{21}   &&  A_{20}\times B_{02} + A_{21}\times B_{12}+A_{22}\times B_{22}\\
//                   \end{bmatrix}
//                   \f]
//
//  \attention
//                   NOTE: ODPR_CML_RESTRICT does not add any new functionality,
//                   it is used for optmizations by compiler.
//                         <b> Optimizations are effective with c66xx platform
//                         with switch CML_OPT_c66x </b>, there would be no
//                         change for others.
//                   CAUTION: Size of the matrix (p_MatrixB->Desc.col X
//                   p_MatrixA->Desc.row) must not
//                   exceed p_MatrixRes->Desc.maxsize
//                   p_MatrixA->Desc.col should be equal to p_MatrixB->Desc.row\\
//                   p_MatrixA->Desc.row should be equal to p_MatrixB->Desc.col\\
//                   p_MatrixRes->pData should hold a valid address
//  \param[in]       p_MatrixA : Multiplicand matrix
//                               Range for p_MatrixA->Desc.row [Full range of
//                               uint8]
//                               Range for p_MatrixA->Desc.col [Full range of
//                               uint8]
//                               Range for p_MatrixA->Desc.maxsize [Full range
//                               of uint16]
//                               Range for p_MatrixA->pData
//                               [Valid pointer with data in full range of
//                               float32]
//  \param[in]       p_MatrixB : Multiplier matrix
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
//  \param[out]      p_MatrixRes : symmetric result matrix
//
//  \return          void
//
//  \author
//
//  \testmethod
//
//  \traceability
//*****************************************************************************/
// void CML_v_multiplyMatricesToSymResult(ODPR_CML_t_Matrix* ODPR_CML_RESTRICT
// p_MatrixRes, const ODPR_CML_t_Matrix* p_MatrixA, const ODPR_CML_t_Matrix*
// p_MatrixB);
//
//
///*****************************************************************************
//  Functionname:    CML_v_multiplyMatrixWithTranspose                    *//*!
//
//  \brief           Matrix multiplication with transpose
//
//  \description     This function performs matrix multiplication (outplace)
//                   of matrix A with transpose of matrix B and store the
//                   result in a resultant matrix.
//                  \f[ [Res] = [A] \times [T] \\ \f]
//                  \f[ [B] = [T]' \\ \f]
//                  \f[ [Res] = [A] \times [B] \f]
//                   \f[[Res]_{m\times n} = [A]_{nXm}\ \times \ [B]_{m\times n}
//                   \\\f]
//  \InOutCorrelation
//                    \f[\begin{bmatrix}
//                   A_{00}    &   A_{01}  & A_{02}\\
//                   A_{10}    &   A_{11}  & A_{12}
//                   \end{bmatrix}
//               \times
//                   \begin{bmatrix}
//                   B_{00}    &   B_{01}\\
//                   B_{10}    &   B_{11}\\
//                   B_{20}    &   B_{21}\\
//                   \end{bmatrix}
//               =
//                   \begin{bmatrix}
//                   A_{00}\times B_{00} + A_{01}\times B_{10}+A_{02}\times B_{20} &&  A_{00}\times B_{01} + A_{01}\times B_{11}+A_{02}\times B_{21}\\
//                   A_{10}\times B_{00} + A_{11}\times B_{10}+A_{12}\times B_{20} &&  A_{10}\times B_{01} + A_{11}\times B_{11}+A_{12}\times B_{21}\\
//                   \end{bmatrix}
//                  \f]
//  \attention
//                   NOTE: Value for row/column is expected not to exceed 8 bits
//                   CAUTION: Size of the matrix (p_MatrixB->Desc.col X
//                   p_MatrixA->Desc.row) must not
//                   exceed p_MatrixRes->Desc.maxsize
//                   p_MatrixA->Desc.col should be equal to p_MatrixB->Desc.row\\
//                   p_MatrixRes->pData should hold a valid address
//  \param[in]       p_MatrixA : Multiplicand matrix
//                               Range for p_MatrixA->Desc.row [Full range of
//                               uint8]
//                               Range for p_MatrixA->Desc.col [Full range of
//                               uint8]
//                               Range for p_MatrixA->Desc.maxsize [Full range
//                               of uint16]
//                               Range for p_MatrixA->pData
//                               [Valid pointer with data in full range of
//                               float32]
//  \param[in]       p_MatrixB : Transpose of which is the multiplier matrix
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
//  \param[out]      p_MatrixRes : result matrix
//
//  \return          void
//
//  \author
//
//  \testmethod
//
//  \traceability
//*****************************************************************************/
// void CML_v_multiplyMatrixWithTranspose(ODPR_CML_t_Matrix* p_MatrixRes, const
// ODPR_CML_t_Matrix* p_MatrixA, const ODPR_CML_t_Matrix* p_MatrixB);
//
//
///*****************************************************************************
//  Functionname:    CML_v_invertMatrix                                   *//*!
//
//  \brief           Compute matrix inverse.
//
//  \description     Compute matrix inverse: Res = inv(A)
//                   Uses Gauss-Jordan elimination with partial pivoting.
//                   For matrices upto 3x3, determinant is found, singularity is
//                   checked and processing is done, whereas for higher order
//                   matrices, matrix singularity is determined during the
//                   processing.\n
//                   The function checks if the matrix has valid dimensions, is
//                   a square
//                   matrix and also if the resultant matrix has enough size to
//                   hold the data.
//                   The function will not process the data if the input
//                   conditions are not
//                   satisfied, instead returns the resultant matrix with both
//                   row and column
//                   dimensions set to zero. The checks can be disabled with the
//                   help of
//                   macro ODPR_CML_MatrixBoundsCheckOn if the conditions are
//                   prechecked before the
//                   function call. \n The function checks the size of the
//                   matrix. For single
//                   element matrix, a direct division of the element is done,
//                   provided the element
//                   is non-zero. For 2x2 and 3x3 matrices, Cramer's rule is
//                   applied to calculate
//                   the inverse. For matrix sizes of 4x4 and more, Gauss Jordan
//                   elimination method
//                   is used.
//                   @startuml
//                   legend
//                   Gauss - Jordan method
//                   endlegend
//                   (*) --> Set result matrix, R as an identity matrix
//                   --> Find largest element on the selected column and use as
//                   pivot element
//                   --> Check the pivot element value, p
//                   If p<p0
//                   note left : p0 is the minimum threshold value almost equal
//                   to zero (1e-10)
//                   --> [Yes] Set empty matrix
//                   Else
//                   --> [No] Swap rows to put the pivot element on the diagonal
//                   --> Same swap operation to be done on the result matrix, R
//                   --> Divide row by the pivot element so that pivot becomes
//                   1. \n Same operation should be repeated for result matrix,
//                   R
//                   --> Loop through the other rows
//                   note right #aqua
//                   Now multiply the row by the right amount and substract from
//                   each other row to make all the remaining elements in the
//                   pivot column zero
//                   end note
//                   --> Use the first element in row as scaling coeffiecient
//                   --> Substract pivot row multiplied by scaling from other
//                   row \n Repeat the same set of operation for result matrix,
//                   R
//                   If All Rows Covered?
//                   --> [Yes] Increment the column count and check if all
//                   columns are covered
//                   If All Columns covered ?
//                   --> [Yes] Check matrix inversion was successful
//                   If matrix invertible?
//                   --> [Yes] Set the dimension parameters for the resultant
//                   matrix
//                   --> (*)
//                   Else
//                   --> [No] Set empty matrix
//                   --> (*)
//                   EndIf
//                   Else
//                   --> [No] Find largest element on the selected column and
//                   use as pivot element
//                   EndIf
//                   Else
//                   --> [No] Loop through the other rows
//                   EndIf
//                   @enduml
//
//                   Cramer's Rule for matrix size 2x2.
//                  \f[ A =  \begin{bmatrix}
//                   A_0    &   A_1 \\
//                   A_2    &   A_3
//                   \end{bmatrix}  \f]
//                   \f[ Determinant, d = (A_0 \times A_3) - (A_2 \times A_1)
//                   \f]
//                   \f[ A^{-1} = \begin{bmatrix}
//                   (\frac{A_3}{d}    &   \frac{-A_1}{d} \\
//                   \frac{-A_2}{d}    &   \frac{A_0}{d}
//                   \end{bmatrix}  \f]
//                   Cramer's rule for matrix size 3x3.
//                   \f[ A = \begin{bmatrix}
//                   A_0     &   A_1  & A_2 \\
//                   A_3     &   A_4  & A_5 \\
//                   A_6     &   A_7  & A_8 \\
//                   \end{bmatrix}  \f]
//                   \f[ Determinant, d = (((A_0 \times A_4) - (A_3 \times A_1))
//                   \times A_8) +
//           (((A_3 \times A_7) - (A_6 \times A_4)) \times A_2) +
//           (((A_6 \times A_1) - (A_0 \times A_7)) \times A_5) \f]
//                   \f[ A = \begin{bmatrix}
//                   (\frac{(A_4 \times A_8) - (A_5 \times A_7))}{d}     &   \frac{((A_2 \times A_7) - (A_1 \times A_8))}{d}  &   \frac{((A_1 \times A_5) - (A_2 \times A_4))}{d}  \\
//                   (\frac{(A_5 \times A_6) - (A_3 \times A_8))}{d}     &   \frac{((A_0 \times A_8) - (A_2 \times A_6))}{d}  &   \frac{((A_2 \times A_3) - (A_0 \times A_5))}{d}  \\
//                   (\frac{(A_3 \times A_7) - (A_4 \times A_6))}{d}     &   \frac{((A_1 \times A_6) - (A_0 \times A_7))}{d}  &   \frac{((A_0 \times A_4) - (A_1 \times A_3))}{d}  \\
//                   \end{bmatrix}  \f]
//
//  \param[in,out]   p_MatrixA : matrix to be inversed.
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
//  \param[in,out]   p_MatrixRes : result matrix, containing the inverse of A
//
//  \return          void
//
//  \author
//
//  \testmethod
//
//  \traceability
//*****************************************************************************/
// void CML_v_invertMatrix(ODPR_CML_t_Matrix* p_MatrixRes, ODPR_CML_t_Matrix*
// p_MatrixA);
//
///*****************************************************************************
//  Functionname:    CML_v_scaleMatrix                                    *//*!
//
//  \brief           Matrix multiplication with scalar
//
//  \description     This function does an inplace matrix multiplication
//                   with a given scalar. If [A] is the matrix, and p is
//                   the scalar, then,
//  \InOutCorrelation
//                   \f[ [A]_{m\times n} = f_{Val} \times [A]_{m\times n}\\
//                    \begin{bmatrix}
//                   A_{00}    &   A_{01}  & A_{02}\\
//                   A_{10}    &   A_{11}  & A_{12}
//                   \end{bmatrix}
//               X
//                   \ p
//               =
//                   \begin{bmatrix}
//                   A_{00} \times p &   A_{01} \times p   &   A_{02} \times p\\
//                   A_{10} \times p &   A_{11} \times p   &   A_{12} \times p
//                   \end{bmatrix}
//                   \f]
//  \attention
//                   NOTE: Value for row/column is expected not to exceed 8 bits
//                   p_MatrixRes->pData should hold a valid address
//  \param[in,out]   p_MatrixA : Matrix o be multiplied
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
//  \param[in]       f_Val :     scalar
//                               [Full range of float32]
//
//  \return          void
//
//  \author
//
//  \testmethod
//
//  \traceability
//*****************************************************************************/
// void CML_v_scaleMatrix(ODPR_CML_t_Matrix* p_MatrixA, float32 f_Val);
//
//
/*****************************************************************************
  Functionname:    ODPR_CML_v_copyMatrix                                     */ /*!

   \brief           Matrix copy

   \description     This function copies data from one matrix to another.

   \InOutCorrelation
                    \f[[Res]_{m\times n} = [A]_{m\times n}\\
                     \begin{bmatrix}
                    Res_{00}  &   Res_{01}  & Res_{02}\\
                    Res_{10}  &   Res_{11}  & Res_{12}
                    \end{bmatrix}
                =
                    \begin{bmatrix}
                    A_{00}    &   A_{01}  &   A_{02} \\
                    A_{10}    &   A_{11}  &   A_{12}
                    \end{bmatrix}
                    \f]
   \attention
                    NOTE: Value for row/column is expected not to exceed 8 bits
                    CAUTION: Size of the matrix (p_MatrixB->Desc.col X
 p_MatrixA->Desc.row) must not
                    exceed p_MatrixRes->Desc.maxsize
                    p_MatrixRes->pData should hold a valid address
   \param[in]       p_MatrixA : matrix to be copied
                                Range for p_MatrixA->Desc.row [Full range of
 uint8]
                                Range for p_MatrixA->Desc.col [Full range of
 uint8]
                                Range for p_MatrixA->Desc.maxsize [Full range of
 uint16]
                                Range for p_MatrixA->pData
                                [Valid pointer with data in full range of
 float32]
   \param[out]      p_MatrixRes : destination matrix
                                  Range for p_MatrixRes->Desc.maxsize [Full
 range of uint16]

   \return          void

   \author

   \testmethod

   \traceability
 *****************************************************************************/
void ODPR_CML_v_copyMatrix(ODPR_CML_t_Matrix* p_MatrixRes,
                           const ODPR_CML_t_Matrix* p_MatrixA);

///*****************************************************************************
//  Functionname:    CML_v_copyArrayToSymMatrix                           *//*!
//
//  \brief           Fill matrix with data from array
//
//  \description     This functon copies data from an array to a matrix.
//                   The resultant matrix is a symmetric matrix.
//  \InOutCorrelation
//
//                   \f[[Res]_{m\times n} = [A]_{m\times n}\\
//                    \begin{bmatrix}
//                   Res_{00}  &   Res_{01}  & Res_{02}\\
//                   Res_{10}  &   Res_{11}  & Res_{12}
//                   \end{bmatrix}
//               =
//                   \begin{bmatrix}
//                   A_{00}    &   A_{01}  &   A_{02} \\
//                   A_{10}    &   A_{11}  &   A_{12}
//                   \end{bmatrix}
//                   \f]
//  \attention
//                   NOTE: ODPR_CML_RESTRICT does not add any new functionality,
//                   it is used for optmizations by compiler.
//                         <b> Optimizations are effective with c66xx platform
//                         with switch CML_OPT_c66x </b>, there would be no
//                         change for others.
//
//  \param[in]       p_Data :  data to be copied
//                             [Valid pointer with data in full range of
//                             float32]
//  \param[out]      p_MatrixRes : destination matrix
//  \param[in]       u_RowNr : no. of rows
//                             [Full range of uint8]
//  \param[in]       u_ColNr : no. of columns
//                             [Full range of uint8]
//
//  \return          void
//
//  \author
//
//  \testmethod
//
//  \traceability
//*****************************************************************************/
// void CML_v_copyArrayToSymMatrix(ODPR_CML_t_Matrix* ODPR_CML_RESTRICT
// p_MatrixRes, uint32 u_RowNr, uint32 u_ColNr, const float32* p_Data);
//
//
///*****************************************************************************
//  Functionname:    CML_v_choleskyMatrix                                 *//*!
//
//  \brief           Compute Cholesky factor of a p.d. hermitian matrix
//
//  \description     The Cholesky factorization is decomposing the hermitian
//                   positive definite matrix (A) into product of a lower
//                   triangular
//                   matrix (L) and its conjugare transpose (L*).
//                   A = LL*
//                   This function returns this lower triangular square root of
//                   the positive definite real symmetric matrix, no exception
//                   handling for any kind of rank deficiency rather direct
//                   regularization
//                   (associated m-file for unit testing: slow_chol2.m)
//                   \f[[A]_{m\times m} = [L]_{m\times m} \times [L^T]_{m\times m} \\
//                   \begin{bmatrix}
//                   A_{00}    &   A_{01}  &   A_{02} \\
//                   A_{10}    &   A_{11}  &   A_{12} \\
//                   A_{20}    &   A_{21}  &   A_{22}
//                   \end{bmatrix}
//                   =
//                   \begin{bmatrix}
//                   L_{11}    &   L_{21}      &   L_{31}\\
//                   0         &   L_{22}      &   L_{32}\\
//                   0         &   0           &   L_{33}
//                   \end{bmatrix}
//                   \times
//                   \begin{bmatrix}
//                   L_{11}    &   0           &   0\\
//                   L_{21}    &   L_{22}      &   0\\
//                   L_{31}    &   L_{32}      &   L_{33}\\
//                   \end{bmatrix}\f]
//  \InOutCorrelation
//                   Following formula is used for entries of L:
//                   \f[ L_{j,j} = \sqrt{A_{j,j} -
//                   \sum_{k=1}^{j-1}{L_{j,k}}^2}\f]
//                   \f[ L_{i,j} = \frac{A_{i,j} -
//                   \sum_{k=1}^{j-1}L_{i,k}L_{j,k}}{L_{j,j}}, for\ i>j.\f]
//  \attention
//                   NOTE: Value for row/column is expected not to exceed 8
//                   bits.
//                   CAUTION: Size of the matrix (p_MatrixA->Desc.col X
//                   p_MatrixA->Desc.row) must not
//                   exceed p_MatrixRes->Desc.maxsize.
//                   p_MatrixA->Desc.col should be equal to p_MatrixA->Desc.row
//                   p_MatrixRes->pData should hold a valid address.
//                   Regularization of values is done for matrix values less
//                   than 1e-10F.
//  \param[in,out]   p_MatrixA :   matrix whose Cholesky type square root is
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
//  \param[in,out]   p_MatrixRes : lower triangular result matrix
//                                 Range for p_MatrixRes->Desc.maxsize [Full
//                                 range of uint16]
//
//  \return          void
//
//  \author
//
//  \testmethod
//
//  \traceability
//*****************************************************************************/
// void CML_v_choleskyMatrix(ODPR_CML_t_Matrix* p_MatrixRes, const
// ODPR_CML_t_Matrix* p_MatrixA);
//
///*****************************************************************************
//  Functionname:    CML_v_lowTriaInvMatrix                               *//*!
//
//  \brief           Inverse of p.d. lower triangular matrix  by forward
//  substitution
//
//  \description     The method of choice for inversion of positive definite
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
//                   @startuml
//                   (*) --> Regularization of input values.
//                   note left : Done to avoid divide by 0 errors.
//                   -->Take inverse of only diagonal elements
//                   note right : in place predivision for efficiency.
//                   -->Assign all Result matrix elements to 0.
//                   -->loop through lower triangle elements only.
//                   note left: by restricting range of loops.
//                   -->calculate multiplication factor(f).
//                   note right: formula for calculation is mentioned below.
//                   If row_index == column_index
//                   --> [YES] Input_matrix element * (1.0 - f)
//                   -->check if all elements are processed
//                   else
//                   --> [NO] Input_matrix element * (-f)
//                   Endif
//                   -->check if all elements are processed
//                   if processing complete
//                   -->[YES](*)
//                   else
//                   -->[NO]loop through lower triangle elements only.
//                   Endif
//                   @enduml
//                   If A is the input matrix and Res is resultant matrix, j and
//                   i are column indices and row indices respectively then,
//                   \f[ F = \sum_{k = 0}^{j}{A_{j,k} \times Res_{k, i}} \f]
//
//  \param[in,out]   p_MatrixA :   lower triangular p.d. matrix (upper
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
//  \param[in,out]   p_MatrixRes : lower triangular inverse of A (upper
//  triangular entries are invalid)
//                                 Range for p_MatrixRes->Desc.maxsize [Full
//                                 range of uint16]
//
//  \return          void
//
//  \author
//
//  \testmethod
//
//  \traceability
//*****************************************************************************/
// void CML_v_lowTriaInvMatrix(ODPR_CML_t_Matrix* p_MatrixRes, const
// ODPR_CML_t_Matrix* p_MatrixA); //AlgoMtrxLowTriaInv
//
//
///*****************************************************************************
//  Functionname:    CML_v_lowTriaSqrMatrix                               *//*!
//
//  \brief           Square of Cholesky Res = A*transpose(A), with A lower
//  triangular
//
//  \description     The method of choice for squaring of positive definite
//                   symmetric matrices is from Cholesky method. This function
//                   computes the product of the lower traingular matrix and its
//                   transpose.\n
//                   The function checks if the matrix has valid dimensions, is
//                   a square
//                   matrix and also if the resultant matrix has enough size to
//                   hold the data.
//                   The function will not process the data if the input
//                   conditions are not
//                   satisfied, instead returns the resultant matrix with both
//                   row and column
//                   dimensions set to zero. The checks can be disabled with the
//                   help of
//                   macro ODPR_CML_MatrixBoundsCheckOn if the conditions are
//                   prechecked before the
//                   function call.
//                   @startuml
//                   (*)--> Check the validity of the input and output martices
//                   note left : This check can be turned off with the macro
//                   ODPR_CML_MatrixBoundsCheckOn
//                   --> Loop through the rows of the matrix for i=0 to no.of
//                   rows
//                   --> Loop through the columns of matrix for j = 0 to i
//                   note left #aqua
//                   The loop only need to process the lower triangular part of
//                   matrix, since we know upper matrix elements are zero.
//                   end note
//                   --> Compute the value 'T' as given in the equation (1)
//                   --> Set the resultant matrix R(i,j) = R(j,i) = T
//                   If Is end of loop for column?
//                   --> [Yes] If Is end of loop for row?
//                   --> [Yes] Set the dimensions for the resultant matrix
//                   --> (*)
//                   Else
//                   --> [No] Loop through the rows of the matrix for i=0 to
//                   no.of rows
//                   EndIf
//                   Else
//                   --> [No] Loop through the columns of matrix for j = 0 to i
//                   EndIf
//                   @enduml
//
//                   The T value for i-th row and j-th column is calculated as
//                   follows.
//                   \f[ T = \sum_{k = 0}^{p}{A_{j,k} \times A_{i,k}} \f]
//                   where p is the minimum of i and j.
//
//  \param[in,out]   p_MatrixA :   lower triangular p.d. matrix
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
//  \param[in,out]   p_MatrixRes : square matrix A*transpose(A) symmetric and
//                                 positive definite
//                                 Range for p_MatrixRes->Desc.maxsize [Full
//                                 range of uint16]
//
//  \return          void
//
//  \author
//
//  \testmethod
//
//  \traceability
//*****************************************************************************/
// void CML_v_lowTriaSqrMatrix(ODPR_CML_t_Matrix* p_MatrixRes, const
// ODPR_CML_t_Matrix* p_MatrixA); //AlgoMtrxLowTriaSqr
//
//
///*****************************************************************************
//  Functionname:    CML_v_uppTriaSqrMatrix                               *//*!
//
//  \brief           Res = A*transpose(A), with A upper triangular
//  \description     This function computes the product of the upper triangular
//                   matrix and its transpose. \n
//                   The function checks if the matrix has valid dimensions, is
//                   a square
//                   matrix and also if the resultant matrix has enough size to
//                   hold the data.
//                   The function will not process the data if the input
//                   conditions are not
//                   satisfied, instead returns the resultant matrix with both
//                   row and column
//                   dimensions set to zero. The checks can be disabled with the
//                   help of
//                   macro ODPR_CML_MatrixBoundsCheckOn if the conditions are
//                   prechecked before the
//                   function call.
//                   @startuml
//                   (*)--> Check the validity of the input and output martices
//                   note left : This check can be turned off with the macro
//                   ODPR_CML_MatrixBoundsCheckOn
//                   --> Loop through the rows of the matrix for i=0 to no.of
//                   rows
//                   --> Loop through the columns of matrix for j = 0 to i
//                   note left #aqua
//                   The loop only need to process the lower triangular part of
//                   matrix, since we know upper matrix elements are zero.
//                   end note
//                   --> Compute the value 'T' as given in the equation (1)
//                   --> Set the resultant matrix R(i,j) = R(j,i) = T
//                   If Is end of loop for column?
//                   --> [Yes] If Is end of loop for row?
//                   --> [Yes] Set the dimensions for the resultant matrix
//                   --> (*)
//                   Else
//                   --> [No] Loop through the rows of the matrix for i=0 to
//                   no.of rows
//                   EndIf
//                   Else
//                   --> [No] Loop through the columns of matrix for j = 0 to i
//                   EndIf
//                   @enduml
//
//                   The T value for i-th row and j-th column, is calculated as
//                   follows.
//                   \f[ T = \sum_{k = i1}^{R}{A_{j,k} \times A_{i,k}} \f]
//                   where R is the no. of rows.
//
//  \param[in,out]   p_MatrixA   : upper triangular p.d. matrix
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
//  \param[in,out]   p_MatrixRes : square matrix A*transpose(A) symmetric and
//                                 positive definite
//                                 Range for p_MatrixRes->Desc.maxsize [Full
//                                 range of uint16]
//
//  \return          void
//
//  \author
//
//  \testmethod
//
//  \testmethod
//
//  \traceability
//*****************************************************************************/
// void CML_v_uppTriaSqrMatrix(ODPR_CML_t_Matrix* p_MatrixRes, const
// ODPR_CML_t_Matrix* p_MatrixA); //AlgoMtrxUppTriaSqr
//
//
/* ***************************************************************************
  Functionname:    CML_v_transposeMatrix                                */ /*!

                  \brief           Matrix transposition, outplace

                  \description     This function calculates the transpose of a
                matrix
                                   (outplace)
                  \InOutCorrelation
                                   \f[A_{m\times m} =
                                   \begin{bmatrix}
                                   A_{00}    &   A_{01}  \\
                                   A_{10}    &   A_{11}  \\
                                   \end{bmatrix}
                                   \\then,
                                   [A^T]_{m\times m} =
                                   \begin{bmatrix}
                                   A_{00}    &   A_{10}  \\
                                   A_{01}    &   A_{11}  \\
                                   \end{bmatrix}
                                   \f]
                  \attention
                                   NOTE: Value for row/column is expected not to
                exceed 8
                bits.
                                   CAUTION: Size of the matrix
                (p_MatrixA->Desc.col X
                p_MatrixA->Desc.row) must not
                                   exceed p_MatrixRes->Desc.maxsize.
                                   p_MatrixRes->pData should hold a valid
                address.
                  \param[in]       p_MatrixA :    matrix o be transposed
                (source)
                                                  Range for p_MatrixA->Desc.row
                [Full
                range of uint8]
                                                  Range for p_MatrixA->Desc.col
                [Full
                range of uint8]
                                                  Range for
                p_MatrixA->Desc.maxsize
                [Full
                range of uint16]
                                                  Range for p_MatrixA->pData
                                                  [Valid pointer with data in
                full
                range
                of float32]
                  \param[out]      p_MatrixRes :  destination matrix
                                                  Range for
                p_MatrixRes->Desc.maxsize
                [Full range of uint16]
                                                  Range for p_MatrixRes->pData
                                                  [Valid pointer with data in
                full
                range
                of float32]

                  \return          none

                  \author

                  \testmethod

                  \traceability
                ****************************************************************************
                */
void ODPR_CML_v_TransposeMatrix(
    ODPR_CML_t_Matrix* p_MatrixRes,
    const ODPR_CML_t_Matrix* p_MatrixA);  // AlgoMtrxTrsp

/*****************************************************************************
  Functionname:    ODPR_CML_initMatrixHeader                                */ /*!

      \brief           helper function for creation of matrices with local scope

      \description     Return a copy of the matrix structure used for
                      setting up the user matrix. This is a work around
                      because a structure cannot be initialized via constructor
                      with non-static pointers.

      \param[in]       u_ColNr : number of columns
                                [Full range of uint8]
      \param[in]       u_RowNr : number of rows
                                [Full range of uint8]
      \param[in]       p_f_MtrxData : pointer to matrix data
                                     [Valid pointer with data in full range of
    float32]

      \return          Matrix structure initialized with given data

      \author

      \testmethod
      \traceability
    *****************************************************************************/

/* Deactivate QA-C warning 3406; Reviewer: S. Schwarzkopf;
   Date: 04.12.2014; Reason: Styleguide wants it like this and ODPR_CML_INLINE
   is
   defined compiler dependent for inlining without external linkage.
   Review-ID: 3942463 */
/* PRQA S 3406 1 */
ODPR_CML_t_Matrix ODPR_CML_initMatrixHeader(uint32 u_ColNr,
                                            uint32 u_RowNr,
                                            float32* p_f_MtrxData);

#endif /* #ifndef _CML_MTRX_INCLUDED */
//
//
///** @} end defgroup */
