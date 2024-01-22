         
#ifndef _BML_EXT_INCLUDED
  // #pragma message(__FILE__": Inclusion of BML_mapping.h is discouraged. It exists only for compatibility with CR3xx and might be deleted without prior notice. Include BML_ext.h instead.")
#endif 

#ifndef _BML_MAPPING_INCLUDED
#define _BML_MAPPING_INCLUDED

#undef C_PI 
#define C_PI           BML_f_Pi

#define GDB_SUBS_VAL_FOR_LOG_OF_ZERO CML_f_SubsForLogOfZero

#define MATTYPE_VECTOR          CML_KafiMatrixTypeVector

#define MATTYPE_FULL            BML_KafiMatrixTypeFull

#define MATTYPE_SYMMETRIC       BML_KafiMatrixTypeSymmetric

#define MATTYPE_DIAGONAL        BML_KafiMatrixTypeDiagonal

#define MATTYPE_UPPERTRIANGULAR CML_KafiMatrixTypeUpperRight

#define GDBSimpleKafi_t    BML_t_SimpleKafi

#define GDBBaseMatrix_t    BML_t_KafiMatrix

#define GDBBaseKafi_t      BML_t_BaseKafi

#define GDBLFunction_t     BML_t_LinFunctionArgs

#define GDBTrafoMatrix2D_t BML_t_TrafoMatrix2D

#define GDBSymMatrix1_t    BML_t_SymMatrix1

#define GDBSymMatrix2_t    CML_t_SymMatrix2

#define GDBSymMatrix6s_t   BML_t_SymMatrix6s

#define GDBVector1_t       CML_t_Vector1D
#ifndef GDBVector2_t
#define GDBVector2_t       BML_t_Vector2D
#endif
#define GDB_Math_Point_t   BML_t_Point2D

#define AlgoMatrix_t       BML_t_Matrix

#undef DEG2RAD 
#define DEG2RAD(deg_)           BML_Deg2Rad(deg_)

#undef RAD2DEG 
#define RAD2DEG(deg_)           BML_Rad2Deg(deg_)
#ifndef SQR
#define SQR(x)                  BML_Sqr(x)
#endif
#ifndef SQRT
#define SQRT(x)                 BML_f_Sqrt(x)
#endif
#define SQRT_(x)                BML_f_Sqrt(x)

#define GDBsqrt_75(x)           BML_f_Sqrt(x)

#undef iABS 
#define iABS(x)                 BML_u_Abs(x)

#define ABS(x)                  (((x)<(0))?(-(x)):(x))
#ifndef fABS
#define fABS(x)                 BML_f_Abs(x)
#endif
#ifndef fSIGN
#define fSIGN(x)                CML_Sign(x)
#endif
#ifndef ROUND
#define ROUND(x)                BML_Round(x)
#endif
#ifndef ROUND_TO_INT
#define ROUND_TO_INT(x)         BML_s_Round2Int(x)
#endif
#define ROUND_TO_UINT(x)        BML_u_Round2Uint(x)

#define GDB_fmod(x, y)          BML_f_ModTrig(x, y)

#define GET_BIT(source, bitmask) CML_GetBit(source, bitmask)

#define SET_BIT(source, bitmask) BML_SetBit(source, bitmask)

#define CLEAR_BIT(source, bitmask) CML_ClearBit(source, bitmask)

#define F32_IS_ZERO(value)      BML_f_IsZero(value)

#define F32_IS_NZERO(value)     BML_f_IsNonZero(value)
#ifndef MIN
#define MIN(x,y)                CML_Min(x,y)
#endif
#ifndef MAX
#define MAX(x,y)                BML_Max(x,y)
#endif
#ifndef MINMAX
#define MINMAX(min,max,value)   CML_MinMax(min,max,value)
#endif
#define MIN_FLOAT(x,y)          BML_f_Min(x,y)

#define MAX_FLOAT(x,y)          BML_f_Max(x,y)

#define MINMAX_FLOAT(min,max,value) BML_f_MinMax(min,max,value)

#define MUL_ADD_FLOAT(a,b,d)    BML_f_MultAdd(a,b,d)

#define A2_TO_RAD(x)            CML_f_Curvature2Radius(x)

#define GDB_FILTER(neu,alt,zeit_k) BML_f_LowPassFilter(neu,alt,zeit_k)

#define VECT_MAT_INDEX(index) BML_VectMatIndex(index)

#define GDB_LINEARE_VLC_INIT(MIN_Y,MAX_Y,MIN_X,MAX_X) BML_LinFuncInit(MIN_Y,MAX_Y,MIN_X,MAX_X)

#define ALGO_MTRX_CREATE(name, rows, cols) CML_CreateMatrix(name, rows, cols)

#define ALGO_MTRX_ELEM(name, row, col) BML_GetMatrixElEnvment(name, row, col)

#define ALGO_MTRX_SET_SIZE(name, rows, cols) BML_SetMatrixSize(name, rows, cols)

#define ALGO_MTRX_IS_EMPTY(name) BML_IsMatrixEnvmpty(name)

#define AlgoMathBayes2(ProbabilityA, ProbabilityB, CPT) CML_Bayes2(ProbabilityA, ProbabilityB, CPT)

#define AlgoMathBayes5(ProbabilityA, ProbabilityB, ProbabilityC, ProbabilityD, ProbabilityE, CPT) CML_Bayes5(ProbabilityA, ProbabilityB, ProbabilityC, ProbabilityD, ProbabilityE, CPT)

#define GDBmathTrafoPos2D(M, DistX, DistY) BML_v_TransformPosition2D(M, DistX, DistY)

#define GDBmathTrafoXPos2D(M , fDistX , fDistY) BML_f_TransformXPosition2D(M , fDistX , fDistY)

#define GDBmathTrafoYPos2D(M , fDistX , fDistY) BML_f_TransformYPosition2D(M , fDistX , fDistY)

#define GDBKalmanMeasUpdate(GDBKafi, KMat, FVec, CMat, fMeas, fMeasNoise) CML_v_KalmanMeasUpdate(GDBKafi, KMat, FVec, CMat, fMeas, fMeasNoise)

#define GDBKalmanUpdatePDiag(GDBKafi) CML_v_KalmanPDiagUpdate(GDBKafi)

#define GDBKalmanTimeUpdate(GDBKafi, WMat, DAMat, DAWVec) BML_v_KalmanTimeUpdate(GDBKafi, WMat, DAMat, DAWVec)

#define GDBKalmanCreateMat(pmatM, pData, MatType, uiRows, uiColumns) BML_v_KalmanCreatEnvmat(pmatM, pData, MatType, uiRows, uiColumns)

#define GDBKalmanGetMat(pmatM, uiRow, uiColumn) BML_f_KalmanGetMatElEnvment(pmatM, uiRow, uiColumn)

#define GDBKalmanSetMat(pmatM, uiRow, uiColumn, fValue) BML_v_KalmanSetMat(pmatM, uiRow, uiColumn, fValue)

#define GDBmathSimpleKafiInit(kafi) BML_u_SimpleKafiInit(kafi)

#define GDBmathSimpleKafiPrediction(kafi) BML_u_SimpleKafiPrediction(kafi)

#define GDBmathSimpleKafiUpdate(kafi) BML_u_SimpleKafiUpdate(kafi)

#define dGDBmathGewichtGerade(dX1, dY1, dX2, dY2, dXPos) BML_f_LinearInterpolation(dX1, dY1, dX2, dY2, dXPos)

#define dGDBmathLineareFunktion(dPara, dEingang) BML_f_BoundedLinInterpol(dPara, dEingang)

#define GDBGetPickupDist(fObjRelSpeed, fEgoSpeed, fEgoDeceleration, fGapTime, fLatencyTime) BML_f_GetPickupDist(fObjRelSpeed, fEgoSpeed, fEgoDeceleration, fGapTime, fLatencyTime)
#ifndef GDBmathLinFuncLimBounded
#define GDBmathLinFuncLimBounded(IVal, Imin, Imax, Omin, Omax) BML_f_BoundedLinInterpol2(IVal, Imin, Imax, Omin, Omax)
#endif
#ifndef GDB_Math_LowPassFilter
#define GDB_Math_LowPassFilter(Old, New, Alpha) BML_f_LowPassFilter2(Old, New, Alpha)
#endif
#define GDB_Math_LinInterpol(P1, P2, In) BML_f_BoundedLinInterpol3(P1, P2, In)

#define Math_LinInterpol_i32(P1, P2, In) BML_f_BoundedLinInterpol5(P1, P2, In)
#ifndef GDB_Math_CalculatePolygonValue
#define GDB_Math_CalculatePolygonValue(iNrOfTableRows, pTable, fInputValue) CML_f_CalculatePolygonValue(iNrOfTableRows, pTable, fInputValue)
#endif
#define fastlog10(x) BML_f_fastlog10(x)

#define GDBmathCalculateCOFEgomotionMatrices(pTrafoMatrix2DCOFForwardRaw, pTrafoMatrix2DCOFBackwardRaw, fSpeedCorrected, fEgoAcceleration, fYawRate_e, fCycleTime) BML_v_CalculateCOFEgomotionMatrices(pTrafoMatrix2DCOFForwardRaw, pTrafoMatrix2DCOFBackwardRaw, fSpeedCorrected, fEgoAcceleration, fYawRate_e, fCycleTime)

#define GDBmathCalcCOFEgomotionMatrices(pTrafoMatrix2DCOFForwardTgtSync, pTrafoMatrix2DCOFForJitTgtSync, fEgoSpeedXTgtSync, fEgoAccelXTgtSync, fYawRateTgtSync, fYawRateVarTgtSync, fYawRateMaxJitterTgtSync, fSensorXPosition, fSensorYPosition, fSlipAngleTgtSync, fSlipAngleVarTgtSync, fCycleTime) BML_v_CalcCOFEgomotionMatrices(pTrafoMatrix2DCOFForwardTgtSync, pTrafoMatrix2DCOFForJitTgtSync, fEgoSpeedXTgtSync, fEgoAccelXTgtSync, fYawRateTgtSync, fYawRateVarTgtSync, fYawRateMaxJitterTgtSync, fSensorXPosition, fSensorYPosition, fSlipAngleTgtSync, fSlipAngleVarTgtSync, fCycleTime)

#define GDBGetTrafoMatrixByAngle(fAngle) BML_GetTrafoMatrixByAngle(fAngle)

#define GDBGetTrafoMatrixByDisplacement(fX, fY) BML_GetTrafoMatrixByDisplacEnvment(fX, fY)

#define GDBRoadClothoid(fXpos, fC0, fC1, fAngle) BML_f_ComputeClothoidLateralDistance(fXpos, fC0, fC1, fAngle)

#define AlgoMtrxInit(A, RowNr, ColNr, fVal) CML_v_InitMatrix(A, RowNr, ColNr, fVal)

#define AlgoMtrxCopy(Res, A) BML_v_CopyMatrix(Res, A)

#define AlgoMtrxCopyFromSym(Res, RowNr, ColNr, pData) CML_v_CopyArrayToSymMatrix(Res, RowNr, ColNr, pData)

#define AlgoMtrxIdent(Res, Size) BML_v_CreateIdentityMatrix(Res, Size)

#define AlgoMtrxAdd(Res, A, B) CML_v_AddMatrices(Res, A, B)

#define AlgoMtrxSub(Res, A, B) CML_v_SubtractMatrices(Res, A, B)

#define AlgoMtrxMul(Res, A, B) CML_v_MultiplyMatrices(Res, A, B)

#define AlgoMtrxMulSym(Res, A, B) CML_v_MultiplyMatricesToSymResult(Res, A, B)

#define AlgoMtrxMulTrsp(Res, A, B) CML_v_MultiplyMatrixWithTranspose(Res, A, B)

#define AlgoMtrxInv(Res, A) CML_v_InvertMatrix(Res, A)

#define AlgoMtrxScale(A, fVal) CML_v_ScaleMatrix(A, fVal)

#define AlgoMtrxChol(Res, A) BML_v_CholeskyMatrix(Res, A)

#define AlgoMtrxLowTriaInv(Res, A) BML_v_LowTriaInvMatrix(Res,A)

#define AlgoMtrxLowTriaSqr(Res, A) BML_v_LowTriaSqrMatrix(Res,A) 

#define AlgoMtrxUppTriaSqr(Res, A) BML_v_UppTriaSqrMatrix(Res,A)

#define AlgoMtrxTrsp(Res, A) CML_v_TransposeMatrix(Res, A)

#define GDBmathMessIntervallInit(sample) BML_v_InitWeightedSample(sample)

#define GDBmathMessIntervallHinzufuegen(sample, value, weight) BML_v_AddSamplePoint(sample, value, weight)

#define GDBmathMessIntervallReduzieren(sample, factor) BML_v_MultiplySampleWithFactor(sample, factor)

#define GDBmathMittelStdAbwBerechnen(sample) BML_v_ComputEnvmeanStd(sample)

#define GDBCalcCumulatDistrFunction(fValue, fAver, fSigma) BML_f_CalcStdGaussianCDF(fValue, fAver, fSigma)

#endif 

