
#ifndef _BML_EXT_INCLUDED
// #pragma message( \
//     __FILE__     \
//     ": Inclusion of BML_kafi.h is discouraged. It exists only for compatibility with CR3xx and might be deleted without prior notice. Include BML_ext.h instead.")
#endif

#ifndef _BML_KAFI_INCLUDED
#define _BML_KAFI_INCLUDED

typedef BML_enum_t BML_t_KafiMatrixType;

#define CML_KafiMatrixTypeVector (BML_t_KafiMatrixType)0
#define BML_KafiMatrixTypeFull (BML_t_KafiMatrixType)1
#define BML_KafiMatrixTypeSymmetric (BML_t_KafiMatrixType)2
#define CML_KafiMatrixTypeUpperRight (BML_t_KafiMatrixType)3
#define BML_KafiMatrixTypeDiagonal (BML_t_KafiMatrixType)4

typedef struct {
    uint8 uiRows;
    uint8 uiColumns;
    uint16 uiElements;
    float32* pdData;
    BML_t_KafiMatrixType matType;
} BML_t_KafiMatrix;

typedef struct {
    BML_t_KafiMatrix AMat;
    BML_t_KafiMatrix QMat;
    BML_t_KafiMatrix XVec;
    BML_t_KafiMatrix PDiagMat;

    BML_t_KafiMatrix UDMat;
    BML_t_KafiMatrix XsVec;

    uint8 uiNbOfStates;
} BML_t_BaseKafi;

typedef struct {
    float32* A;
    float32* B;
    float32* u;
    float32* Q;
    float32* z;
    float32* H;
    float32* R;

    float32* P;

    float32* x;

    float32* K;
    float32* T;
    float32* T2;
    float32* tx;

    uint8 x_y, B_x;
} BML_t_SimpleKafi;

#define BML_VectMatIndex(index) (index)
#define BML_FullMatIndex(row, column, nbofcolumns) \
    (((row) * (nbofcolumns)) + (column))
#define BML_SymmMatIndex(row, column) \
    ((((column) * ((column) + 1u)) / 2u) + (row))
#define BML_UpriMatIndex(row, column) \
    ((((column) * ((column) + 1u)) / 2u) + (row))
#define BML_DiagMatrIndex(row) (row)

void BML_v_KTU_CaldUDdW(const float32 f_Tol,
                        const uint8 u_NbOfState2,
                        float32* f_Sum,
                        sint8 j,
                        uint32 u_Index1,
                        float32* p_dUD,
                        float32* p_dDAW,
                        float32* p_dW);

extern void BML_v_KalmanTimeUpdate(BML_t_BaseKafi const* const p_GDBKafi,
                                   BML_t_KafiMatrix WMat,
                                   BML_t_KafiMatrix DAMat,
                                   BML_t_KafiMatrix DAWVec);
extern void CML_v_KalmanMeasUpdate(BML_t_BaseKafi const* const p_GDBKafi,
                                   BML_t_KafiMatrix KMat,
                                   BML_t_KafiMatrix FVec,
                                   BML_t_KafiMatrix CMat,
                                   float32 f_Meas,
                                   float32 f_MeasNoise);
extern void CML_v_KalmanPDiagUpdate(BML_t_BaseKafi const* const p_GDBKafi);
extern void BML_v_KalmanCreatEnvmat(BML_t_KafiMatrix* p_matM,
                                    float32* p_Data,
                                    BML_t_KafiMatrixType MatType,
                                    uint8 u_Rows,
                                    uint8 u_Columns);
extern void BML_v_KalmanSetMat(const BML_t_KafiMatrix* p_matM,
                               uint8 u_Row,
                               uint8 u_Column,
                               float32 f_Value);
extern float32 BML_f_KalmanGetMatElEnvment(const BML_t_KafiMatrix* p_matM,
                                           uint8 u_Row,
                                           uint8 u_Column);

extern uint8 BML_u_SimpleKafiInit(BML_t_SimpleKafi const* p_kafi);
extern uint8 BML_u_SimpleKafiPrediction(BML_t_SimpleKafi const* p_kafi);
extern uint8 BML_u_SimpleKafiUpdate(BML_t_SimpleKafi const* p_kafi);

#endif
