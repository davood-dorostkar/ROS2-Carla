
#ifndef _BML_EXT_INCLUDED
#endif

#ifndef _BML_INTERPOL_INCLUDED
#define _BML_INTERPOL_INCLUDED

extern float32 BML_f_LinearInterpolation(
    float32 f_X1, float32 f_Y1, float32 f_X2, float32 f_Y2, float32 f_XPos);
extern float32 BML_f_BoundedLinInterpol(
    BML_t_LinFunctionArgs const* const p_Params, const float32 f_Value);
extern float32 BML_f_BoundedLinInterpol3(BML_t_Point2D Point1,
                                         BML_t_Point2D Point2,
                                         float32 f_In);
extern float32 BML_f_BoundedLinInterpol4(Vector2_f32_t Point1,
                                         Vector2_f32_t Point2,
                                         float32 f_In);
extern float32 BML_f_BoundedLinInterpol5(Vector2_i32_t Point1,
                                         Vector2_i32_t Point2,
                                         float32 f_In);
extern float32 CML_f_CalculatePolygonValue(sint32 s_NrOfTableRows,
                                           const GDBVector2_t a_Table[],
                                           float32 f_InputValue);
void CML_CCSC_CalcBMatrix(const uint32 u_NrOfShapePoints,
                          float32 f_DeltaX_i,
                          float32 f_DeltaX_im1Abs,
                          float32 f_DeltaX_im1,
                          uint32 i,
                          float32 a_BMatrix[]);
extern boolean CML_CalculateCubicSplineClamped(
    const uint32 u_NrOfShapePoints,
    const Vector2_f32_t a_ShapePointTable[],
    const float32 f_SlopeStart,
    const float32 f_SlopeEnd,
    float32 a_BMatrix[],
    float32 a_BMatrixInv[],
    float32 a_Temp[],
    float32 a_YVector[],
    float32 a_BInvXYVec[],
    float32 a_ParaA[],
    float32 a_ParaB[],
    float32 a_ParaC[],
    float32 a_ParaD[],
    boolean a_SegmentSplineValid[]);

#endif
