
#ifdef PRQA_SIZE_T

#pragma PRQA_MACRO_MESSAGES_OFF "BML_CreatEnvmatrixLocal" 1031
#endif

#ifndef _BML_EXT_INCLUDED
#endif

#ifndef _BML_MTRX_INCLUDED
#define _BML_MTRX_INCLUDED

typedef struct {
    uint8 col;
    uint8 row;
    uint16 maxsize;
} BML_t_MatrixDescriptor;

typedef struct {
    BML_t_MatrixDescriptor Desc;
    float32* pData;
} BML_t_Matrix;

#define CML_CreateMatrix(name, rows, cols)                           \
    static float32 fMtrxData##name[(uint32)(rows) * (uint32)(cols)]; \
    BML_t_Matrix AlgoMtrxHeader##name = {                            \
        {(uint8)(cols), (uint8)(rows),                               \
         (uint16)((uint32)(rows) * (uint32)(cols))},                 \
        fMtrxData##name};                                            \
    BML_t_Matrix* name = &AlgoMtrxHeader##name;

#if (defined(_MSC_VER) || defined(__DVLC__))
#define BML_MATRIXLOCAL
#elif (defined(__STDC_VERSION__))
#if ((__STDC_VERSION__ >= 199901))
#define BML_MATRIXLOCAL
#endif
#endif

#ifdef BML_MATRIXLOCAL

#define BML_CreatEnvmatrixLocal(name, rows, cols)                \
    float32 fMtrxData##name[(uint32)(rows) * (uint32)(cols)];    \
    BML_t_Matrix AlgoMtrxHeader##name =                          \
        BML_a_InitMatrixHeader((cols), (rows), fMtrxData##name); \
    BML_t_Matrix* name = &AlgoMtrxHeader##name;
#else
#define BML_CreatEnvmatrixLocal(name, rows, cols) \
    CML_CreateMatrix(name, rows, cols)
#endif

#if (BML_MatrixBoundsCheckOn)
#define BML_GetMatrixElEnvment(name, Row, Col) \
    (name)->pData[(uint32)(Col) + ((uint32)(Row) * (name)->Desc.col)]
#else

#define BML_GetMatrixElEnvment(name, Row, Col) \
    (name)->pData[(uint32)(Col) + ((uint32)(Row) * (name)->Desc.col)]
#endif

#if (BML_MatrixBoundsCheckOn)
#define BML_SetMatrixSize(name, rows, cols)    \
    if (rows * cols <= (name)->Desc.maxsize) { \
        (name)->Desc.col = cols;               \
        (name)->Desc.row = rows;               \
    } else {                                   \
        BML_ASSERT(FALSE);                     \
    };
#else

#define BML_SetMatrixSize(name, rows, cols) \
    (name)->Desc.col = (cols);              \
    (name)->Desc.row = (rows);
#endif

#define BML_IsMatrixEnvmpty(name) \
    (((name)->Desc.col == (uint8)0) || ((name)->Desc.row == (uint8)0))

BML_INLINE BML_t_Matrix BML_a_InitMatrixHeader(uint32 u_ColNr,
                                               uint32 u_RowNr,
                                               float32* p_f_MtrxData);

void CML_v_InitMatrix(BML_t_Matrix* p_Matrix,
                      uint32 u_RowNr,
                      uint32 u_ColNr,
                      float32 f_Val);

void BML_v_CopyMatrix(BML_t_Matrix* p_MatrixRes, const BML_t_Matrix* p_MatrixA);

void CML_v_CopyArrayToSymMatrix(BML_t_Matrix* p_MatrixRes,
                                uint32 u_RowNr,
                                uint32 u_ColNr,
                                const float32* p_Data);

void BML_v_CreateIdentityMatrix(BML_t_Matrix* p_Matrix, uint32 u_Size);

void CML_v_AddMatrices(BML_t_Matrix* p_MatrixRes,
                       const BML_t_Matrix* p_MatrixA,
                       const BML_t_Matrix* p_MatrixB);

void CML_v_SubtractMatrices(BML_t_Matrix* p_MatrixRes,
                            const BML_t_Matrix* p_MatrixA,
                            const BML_t_Matrix* p_MatrixB);

void CML_v_MultiplyMatrices(BML_t_Matrix* p_MatrixRes,
                            const BML_t_Matrix* p_MatrixA,
                            const BML_t_Matrix* p_MatrixB);

void CML_v_MultiplyMatricesToSymResult(BML_t_Matrix* p_MatrixRes,
                                       const BML_t_Matrix* p_MatrixA,
                                       const BML_t_Matrix* p_MatrixB);

void CML_v_MultiplyMatrixWithTranspose(BML_t_Matrix* p_MatrixRes,
                                       const BML_t_Matrix* p_MatrixA,
                                       const BML_t_Matrix* p_MatrixB);

boolean CML_v_CalInvertMatrix_M(BML_t_Matrix* p_MatrixA,
                                float32* p_DataA,
                                float32* p_DataRes);

void CML_v_InvertMatrix(BML_t_Matrix* p_MatrixRes, BML_t_Matrix* p_MatrixA);

void CML_v_ScaleMatrix(BML_t_Matrix* p_MatrixA, float32 f_Val);

void BML_v_CholeskyMatrix(BML_t_Matrix* p_MatrixRes,
                          const BML_t_Matrix* p_MatrixA);

void BML_v_LowTriaInvMatrix(BML_t_Matrix* p_MatrixRes,
                            const BML_t_Matrix* p_MatrixA);

void BML_v_LowTriaSqrMatrix(BML_t_Matrix* p_MatrixRes,
                            const BML_t_Matrix* p_MatrixA);

void BML_v_UppTriaSqrMatrix(BML_t_Matrix* p_MatrixRes,
                            const BML_t_Matrix* p_MatrixA);

void CML_v_TransposeMatrix(BML_t_Matrix* p_MatrixRes,
                           const BML_t_Matrix* p_MatrixA);

BML_INLINE BML_t_Matrix BML_a_InitMatrixHeader(uint32 u_ColNr,
                                               uint32 u_RowNr,
                                               float32* p_f_MtrxData) {
    BML_t_Matrix Ret;
    Ret.Desc.col = (uint8)u_ColNr;
    Ret.Desc.row = (uint8)u_RowNr;
    Ret.Desc.maxsize = (uint16)(u_RowNr * u_ColNr);
    Ret.pData = p_f_MtrxData;

    return Ret;
}

#endif
