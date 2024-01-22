
#ifndef _BML_EXT_INCLUDED
#endif

#ifndef _BML_MAT_INCLUDED
#define _BML_MAT_INCLUDED

typedef struct {
    float32 f00;
} BML_t_SymMatrix1;

typedef struct {
    float32 f00;
    float32 f01;
    float32 f11;
} CML_t_SymMatrix2;

typedef struct {
    float32 f00;
    float32 f01;
    float32 f02;
    float32 f03;
    float32 f04;
    float32 f05;
    float32 f11;
    float32 f12;
    float32 f13;
    float32 f14;
    float32 f15;
    float32 f22;
    float32 f23;
    float32 f24;
    float32 f25;
    float32 f33;
    float32 f34;
    float32 f35;
    float32 f44;
    float32 f45;
    float32 f55;
} BML_t_SymMatrix6s;

typedef enum { DEFAULT_, ROTATION_, TRANSLATION_ } BML_t_TrafoType;

typedef struct {
    BML_t_TrafoType TrafoType;
    float32 f00;

    float32 f02;
    float32 f10;

    float32 f12;

} BML_t_TrafoMatrix2D;

typedef struct {
    float32 f0;
} CML_t_Vector1D;

typedef struct {
    float32 f0;
    float32 f1;
} BML_t_Vector2D;

typedef struct {
    float32 X;
    float32 Y;
} BML_t_Point2D;

extern uint8 BML_u_MatrixCopy(float32 const a_MatrixA[],
                              float32 a_MatrixCpy[],
                              uint8 u_dim_Ax,
                              uint8 u_dim_Ay);
extern uint8 CML_u_MatrixTranspose(float32 const a_MatrixA[],
                                   float32 a_ATranspose[],
                                   uint8 u_dim_Ax,
                                   uint8 u_dim_Ay);
extern uint8 BML_u_MatrixMultiplication(float32 const a_MatrixA[],
                                        float32 const a_MatrixB[],
                                        float32 a_AxB[],
                                        uint8 u_dim_Ax,
                                        uint8 u_dim_Ay,
                                        uint8 u_dim_Bx);
void BML_u_MatrixInversionRoateRows(uint8 u_row,
                                    uint8 u_prow,
                                    uint8 u_dim_xy,
                                    float32 a_tempMatrix[],
                                    float32 a_InverseA[]);
extern uint8 BML_u_MatrixInversion(float32 const a_MatrixA[],
                                   float32 a_tempMatrix[],
                                   float32 a_InverseA[],
                                   uint8 u_dim_xy);
extern uint8 CML_u_MatrixAddition(float32 const a_MatrixA[],
                                  float32 const a_MatrixB[],
                                  float32 a_AplusB[],
                                  uint8 u_dim_x,
                                  uint8 u_dim_y);
extern uint8 BML_u_MatrixSubtraction(float32 const a_MatrixA[],
                                     float32 const a_MatrixB[],
                                     float32 a_AminusB[],
                                     uint8 u_dim_x,
                                     uint8 u_dim_y);

BML_t_TrafoMatrix2D BML_GetTrafoMatrixByAngle(float32 f_Angle);
BML_t_TrafoMatrix2D BML_GetTrafoMatrixByDisplacEnvment(float32 f_XDisplacement,
                                                       float32 f_YDisplacement);

BML_t_TrafoMatrix2D CML_TrafoMatrix2DMult(BML_t_TrafoMatrix2D MatrixA,
                                          BML_t_TrafoMatrix2D MatrixB);
BML_t_TrafoMatrix2D CML_TrafoMatrix2DInvert(BML_t_TrafoMatrix2D MatrixA);
extern void BML_v_TransformPosition2D(BML_t_TrafoMatrix2D const *p_Matrix,
                                      float32 *p_DistX,
                                      float32 *p_DistY);
extern float32 BML_f_TransformXPosition2D(BML_t_TrafoMatrix2D const *p_Matrix,
                                          float32 f_DistX,
                                          float32 f_DistY);
extern float32 BML_f_TransformYPosition2D(BML_t_TrafoMatrix2D const *p_Matrix,
                                          float32 f_DistX,
                                          float32 f_DistY);
extern void GDBsincos(float32 f_val, float32 *p_sin, float32 *p_cos);

#endif
