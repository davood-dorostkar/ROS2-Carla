         
#ifndef _BML_EXT_INCLUDED
  // #pragma message(__FILE__": Inclusion of BML_stat.h is discouraged. It exists only for compatibility with CR3xx and might be deleted without prior notice. Include BML_ext.h instead.")
#endif 

#ifndef _BML_STAT_INCLUDED
#define _BML_STAT_INCLUDED

#define CML_f_GaussErrFctMaxX 1.99f

#define CML_f_GaussErrFctConst0 0.002289f
#define BML_f_GaussErrFctConst1 1.146f
#define CML_f_GaussErrFctConst2 0.1092f
#define BML_f_GaussErrFctConst3 0.2841f
#define CML_f_GaussErrFctConst4 0.08869f

#define CML_f_GaussianCDFMinSigma 0.000001f

#define CML_f_GaussQuantile_a0  (  2.50662823884f)
#define CML_f_GaussQuantile_a1  (-18.61500062529f)
#define BML_f_GaussQuantile_a2  ( 41.39119773534f)
#define CML_f_GaussQuantile_a3  (-25.44106049637f)
#define BML_f_GaussQuantile_b0  (  1.f)
#define BML_f_GaussQuantile_b1  ( -8.47351093090f)
#define BML_f_GaussQuantile_b2  ( 23.08336743743f)
#define CML_f_GaussQuantile_b3  (-21.06224101826f)
#define BML_f_GaussQuantile_b4  (  3.13082909833f)

#define CML_f_GaussQuantile_lowerValue   (1.40507145f)
#define BML_f_GaussQuantile_lowerBound   (0.92f - 0.5f)
#define CML_f_GaussQuantile_sigma2Value  (1.99539331f)
#define CML_f_GaussQuantile_sigma2Bound  (0.977f - 0.5f)
#define CML_f_GaussQuantile_sigma3Value  (3.090232306f)
#define CML_f_GaussQuantile_sigma3Bound  (0.999f - 0.5f)
#define CML_f_GaussQuantile_highValue    (4.0f)

typedef struct {
  float32 dSumme;
  float32 dQuadSumme;
  float32 f_sumOfWeights; 
  float32 dMittel;
  float32 dStdAbw;
} BML_t_WeightedSample; 

typedef struct {
  sint32 iData_Counter;    
  float32 fsumX;              
  float32 fsumV;            
  float32 fsumXX;            
  float32 fsumXV;            
  float32 fsumVV;           
} BML_t_LeastSquareFit;

typedef struct {
  uint16  u_Data_Counter;    
  uint16  u_sumX;              
  sint16  s_sumV;            
  uint16  u_sumXX;            
  sint16  s_sumXV;            
  uint16  u_sumVV;           
} BML_t_LeastSquareFitQuant;

typedef struct {
  float32 fData_Counter;    
  float32 fsumX;              
  float32 fsumV;            
  float32 fsumXX;            
  float32 fsumXV;            
  float32 fsumVV;           
} t_LeastSquareFit_ForgetExponential;

typedef struct {
  float32 fSlope;           
  float32 fCorrelation;     
  float32 fYIntersection;   
  float32 fChi2_Error;      
  float32 fmse;             
} BML_t_LeastSquareFitResults;

extern void BML_v_InitWeightedSample(BML_t_WeightedSample *p_sample); 
extern void BML_v_AddSamplePoint(BML_t_WeightedSample *p_sample, float32 f_value, float32 f_weight); 
extern void BML_v_MultiplySampleWithFactor(BML_t_WeightedSample *p_sample, float32 f_factor); 
extern void BML_v_ComputEnvmeanStd(BML_t_WeightedSample *p_sample); 

extern float32 BML_f_CalcStdGaussianCDF ( float32 f_value, float32 f_avg, float32 f_sigma ); 
extern float32 BML_f_CalcGaussErrorFunction  ( float32 f_value ); 

extern void BML_v_Init_LSF(BML_t_LeastSquareFit *const p_LSF);
extern void CML_v_InitResults_LSF(BML_t_LeastSquareFitResults *const p_LSFRes);
extern void BML_v_AddData_LSF(BML_t_LeastSquareFit *const p_LSF, const float32 f_abscissae, const float32 f_ordinate);
extern float32 CML_f_CalculateSlope_LSF(BML_t_LeastSquareFit const *const p_LSF);
extern float32 CML_f_CalculateCorrelation_LSF(BML_t_LeastSquareFit const *const p_LSF);
extern float32 CML_f_CalculateYIntersection_LSF(BML_t_LeastSquareFit const *const p_LSF);
extern float32 CML_f_CalculateSquareError_LSF(BML_t_LeastSquareFit const *const p_LSF, const float32 f_LSFCorrelation);
extern float32 CML_f_CalculateMeanSquareError_LSF(BML_t_LeastSquareFit const *const p_LSF, const float32 f_LSFCorrelation);
extern void CML_v_CalculateQuality_LSF(BML_t_LeastSquareFit const *const p_LSF, float32 *const f_LSFCorrelation, float32 *const f_LSFmse);
extern void CML_v_CalculateAll_LSF(BML_t_LeastSquareFit const *const p_LSF, BML_t_LeastSquareFitResults *const p_LSFRes);
extern float32 BML_f_Predict_LSF(BML_t_LeastSquareFitResults const *const p_LSFResults, float32 f_xValue);

extern void BML_v_Init_LSF_ForgetExponential(t_LeastSquareFit_ForgetExponential *const p_LSF);
extern void BML_v_AddData_LSF_ForgetExponential(t_LeastSquareFit_ForgetExponential *const p_LSF, const float32 f_abscissa, const float32 f_ordinate, const float32 f_MemoryWeight);
extern void BML_v_CalculateAll_LSF_ForgetExponential(t_LeastSquareFit_ForgetExponential const *const p_LSF, BML_t_LeastSquareFitResults *const p_LSFRes);

extern float32 BML_f_CalcQuantile(float32 a_valuesDestroyed[], const uint32 u_NofValues, const uint32 u_NthSmallest);

extern float32 BML_f_StdGaussQuantile(float32 f_Prob);
extern float32 CML_f_GaussQuantile(float32 f_Mean, float32 f_Std, float32 f_Prob);

extern void BML_v_Init_LSF_Quant(BML_t_LeastSquareFitQuant *const p_LSFQuant);
extern void BML_v_DecodeData_LSF_Quant(BML_t_LeastSquareFitQuant const *const p_LSFQuant, const float32 f_decodeX, const float32 f_decodeV, BML_t_LeastSquareFit *const p_LSFNonQuant);
extern void CML_v_EncodeData_LSF_Quant(BML_t_LeastSquareFit const *const p_LSFNonQuant, const float32 f_encodeX, const float32 f_encodeV, BML_t_LeastSquareFitQuant *const p_LSFQuant);

#endif 

