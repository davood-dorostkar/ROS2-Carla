         
#ifdef PRQA_SIZE_T
  
  #pragma PRQA_NO_SIDE_EFFECTS BML_f_Sign
#endif 

#ifdef PRQA_SIZE_T
  
  #pragma PRQA_NO_SIDE_EFFECTS BML_f_ModTrig
  #pragma PRQA_NO_SIDE_EFFECTS BML_f_Mod
  #pragma PRQA_NO_SIDE_EFFECTS BML_f_REnvm
  #pragma PRQA_NO_SIDE_EFFECTS BML_s_CountNrOfBitsSet
  #pragma PRQA_NO_SIDE_EFFECTS BML_f_LinearInterpolation
  #pragma PRQA_NO_SIDE_EFFECTS BML_f_BoundedLinInterpol
  #pragma PRQA_NO_SIDE_EFFECTS BML_f_BoundedLinInterpol2
  #pragma PRQA_NO_SIDE_EFFECTS BML_f_fastlog10
  #pragma PRQA_NO_SIDE_EFFECTS BML_f_fastlog
  #pragma PRQA_NO_SIDE_EFFECTS BML_f_SqrtApprox
  #pragma PRQA_NO_SIDE_EFFECTS BML_f_PowerOfTwo
  #pragma PRQA_NO_SIDE_EFFECTS BML_f_XPowY
#endif 

#ifndef _BML_EXT_INCLUDED
  // #pragma message(__FILE__": Inclusion of BML_misc.h is discouraged. It exists only for compatibility with CR3xx and might be deleted without prior notice. Include BML_ext.h instead.")
#endif 

#ifndef _BML_MISC_INCLUDED
#define _BML_MISC_INCLUDED

#define BML_f_ModuloEps 0.0000001f

#define BML_f_MaxAngleSinApprox_1 0.03f
#define BML_f_MaxAngleSinApprox_2 0.003f

#define N_LUT 32
#define ONE_DIV_2POT23 1.192092895507813e-007f
#define C_LUT_MAP ((float32)N_LUT * ONE_DIV_2POT23)

#define BML_SqrtApprox_NumExpo        (23u)

#define BML_SqrtApprox_MantissaMask   (0x007fffffu)

#define BML_SqrtApprox_ExponentOffset (0x7f)

#define BML_SqrtApprox_AlmostZero (1e-20f)

#define CML_LN_10 2.3025850929940456840179914546844f

#define BML_DBPOW_FACTOR 10.0f     
#define CML_DBMAG_FACTOR 20.0f     


typedef struct {
  
  float32 dAmin;
  
  float32 dAmax;
  
  float32 dM;
  
  float32 dB;
}BML_t_LinFunctionArgs; 

typedef struct {
  float32 f_X;           
  float32 f_Y;           
  float32 f_DistToTraj;  
  float32 f_DistOnTraj;  
}BML_t_TrajRefPoint;

#define BML_LinFuncInit(MIN_Y,MAX_Y,MIN_X,MAX_X) {(MIN_Y),(MAX_Y),(((MAX_Y)-(MIN_Y))/((MAX_X)-(MIN_X))),(MIN_Y)-((((MAX_Y)-(MIN_Y))/((MAX_X)-(MIN_X)))*(MIN_X))}

#define BML_u_Abs(x)        (((x)<(0L))?(-(x)):(x))

#define CML_Min(x,y)        (((x)<(y))?(x):(y))

#define BML_Max(x,y)        (((x)>(y))?(x):(y))

#define CML_MinMax(min,max,value) (CML_Min(BML_Max(min,value),max))

#define CML_IsInRange(min,max,value)           (((min) <= (value)) && ((value) <= (max)))

#define CML_IsInRange_ExclLim(min,max,value)   (((min) <  (value)) && ((value) <  (max)))

#define CML_f_IsInRange(min,max,value)         (((min) <= (value)) && ((value) <= (max)))

#define CML_f_IsInRange_ExclLim(min,max,value) ((((min) + BML_f_AlmostZero) <  (value)) && ((value) < ((max) - BML_f_AlmostZero)))

#define BML_Sqr(x)          ((x)*(x))
    
#define CML_Sign(x)         (((x)==(0))?(0):(((x)>(0))?(1):(-1)))

BML_INLINE sint32 BML_f_Sign(float32 f_Input)
{
  sint32 s_Ret;

  s_Ret = (BML_f_Abs(f_Input) < BML_f_AlmostZero) ? 0L : ((f_Input < 0.0f) ? (-1L) : 1L);

  return s_Ret;
} 

#define BML_f_MinMax(min,max,value)  (BML_f_Min(BML_f_Max(min,value),max))
    
#define BML_Deg2Rad(deg_) ((deg_)*(BML_f_Pi/180.f))
    
#define BML_Rad2Deg(rad_)  ((rad_)*(180.f/BML_f_Pi))

#define CML_GetBit(source, bitmask)   ( ((source) & (bitmask)) == (bitmask) ) 

#define BML_SetBit(source, bitmask)   ((source) |=  (bitmask)) 

#define CML_ClearBit(source, bitmask) ((source) &= ~( bitmask) ) 

#define BML_ConditionalSetClearBit(flag, bitmask, destination) ( ((destination) & ~(bitmask)) | (-(flag) & (bitmask)) )

#define BML_f_IsZero(value) (BML_f_Abs(value) < BML_f_AlmostZero) 

#define BML_f_IsNonZero(value) (BML_f_Abs(value) >= BML_f_AlmostZero) 

#define BML_f_LowPassFilter(neu,alt,zeit_k) (((neu)+((alt)*(zeit_k)))/((zeit_k)+1.F)) 

#define CML_f_Curvature2Radius(x)   ( ( BML_f_Abs((x)) < 0.00002F ) ? (0.0F) : ( (1.0F)/(x) ) ) 

extern float32 BML_f_ModTrig(float32 f_dividend, float32 f_divisor); 
extern float32 BML_f_Mod(float32 f_value, float32 f_modulo);
extern float32 BML_f_REnvm(float32 f_dividend, float32 f_divisor);

extern sint32 BML_s_CountNrOfBitsSet (uint32 u_PruefBits); 

extern void BML_v_CalcCOFEgomotionMatrices(BML_t_TrafoMatrix2D * p_TrafoMatrix2DCOFForwardTgtSync,
                                           BML_t_TrafoMatrix2D * p_TrafoMatrix2DCOFForJitTgtSync,
                                           fVelocity_t f_EgoSpeedXTgtSync,
                                           fVelocity_t f_EgoAccelXTgtSync,
                                           fYawRate_t  f_YawRateTgtSync,
                                           fVariance_t f_YawRateVarTgtSync,
                                           fYawRate_t  f_YawRateMaxJitterTgtSync,
                                           fDistance_t f_SensorXPosition,
                                           fDistance_t f_SensorYPosition,
                                           fAngle_t    f_SlipAngleTgtSync,
                                           fVariance_t f_SlipAngleVarTgtSync,
                                           fTime_t     f_CycleTime);

extern void BML_v_CalculateCOFEgomotionMatrices(BML_t_TrafoMatrix2D * p_TrafoMatrix2DCOFForwardRaw,
                                                BML_t_TrafoMatrix2D * p_TrafoMatrix2DCOFBackwardRaw,
                                                fVelocity_t   f_SpeedCorrected,
                                                fAccel_t      f_EgoAcceleration,
                                                fYawRate_t    f_YawRate_e,
                                                fTime_t       f_CycleTime);

extern BML_t_TrafoMatrix2D BML_GetTrafoMatrixByAngle(float32 f_Angle);
extern BML_t_TrafoMatrix2D BML_GetTrafoMatrixByDisplacEnvment(float32 f_XDisplacement, float32 f_YDisplacement);

extern float32 BML_f_GetPickupDist(float32 f_ObjRelSpeed, float32 f_EgoSpeed, float32 f_EgoDeceleration, float32 f_GapTime, float32 f_LatencyTime); 
extern float32 BML_f_BoundedLinInterpol2(float32 f_IVal, float32 f_Imin, float32 f_Imax, float32 f_Omin, float32 f_Omax); 
extern float32 BML_f_ComputeClothoidLateralDistance (float32 f_Xpos, float32 f_C0, float32 f_C1, float32 f_Angle); 

extern void  BML_f_LowPassFilter2(float32 * f_Old, float32 f_New, float32 f_Alpha); 

extern float32 CML_f_log2(const float32 f_in);
extern float32 BML_f_log10(const float32 f_in);
extern float32 CML_f_ln(const float32 f_in);

extern float32 BML_f_fastlog10(float32 f_value);

extern float32 BML_f_fastlog(float32 f_value);

extern float32 BML_InvSqrt67(float32 f_x);

extern float32 BML_Sqrt67(float32 f_x);

typedef union 
{ 
  float f_d; 
  sint32 s_x; 
} BML_t_FloatAsSigned;

typedef union 
{
  float f_d; 
  uint32 u_x; 
} BML_t_FloatAsUnsigned;

BML_INLINE float32 CML_AlphaMaxBetaMin(float32 f_x, float32 f_y)
{
  const float32 f_Alpha0 = 0.96043387F;
  const float32 f_Beta0  = 0.39782473F;
  float32 f_Max;
  float32 f_Min;
  f_x = BML_f_Abs(f_x);
  f_y = BML_f_Abs(f_y);
  f_Max = BML_Max(f_x, f_y);
  f_Min = CML_Min(f_x, f_y);
  return (f_Alpha0 * f_Max) + (f_Beta0 * f_Min);
}

BML_INLINE float32 BML_InvSqrt53(float32 f_x);
BML_INLINE float32 BML_InvSqrt53(float32 f_x)
{
  const sint32 i_MagicNumber = 0x5f375a86;
  const float32 f_MagicNumber = 1.5F;
  sint32 s_i;
  float32 f_x2;
  float32 f_y;
  BML_t_FloatAsSigned u;
  u.f_d = f_x;

  f_x2 = f_x * 0.5F;
  f_y = f_x;
  
  s_i = u.s_x;
  s_i = i_MagicNumber - (s_i / 2);
  u.s_x = s_i;
  f_y = u.f_d;
  f_y = f_y * (f_MagicNumber - (f_x2 * f_y * f_y));
  f_y = f_y * (f_MagicNumber - (f_x2 * f_y * f_y));
  return f_y;
}

BML_INLINE float32 BML_InvSqrt27(float32 f_x);
BML_INLINE float32 BML_InvSqrt27(float32 f_x)
{
  const sint32 i_MagicNumber = 0x5f375a86;
  const float32 f_MagicNumber = 1.5F;
  sint32 s_i;
  float32 f_x2;
  float32 f_y;
  BML_t_FloatAsSigned u;
  u.f_d = f_x;

  f_x2 = f_x * 0.5F;
  f_y = f_x;
  
  s_i = u.s_x;
  s_i = i_MagicNumber - (s_i / 2);
  u.s_x = s_i;
  f_y = u.f_d;
  f_y = f_y * (f_MagicNumber - (f_x2 * f_y * f_y));
  return f_y;
}

BML_INLINE float32 BML_InvSqrt14(float32 f_x);
BML_INLINE float32 BML_InvSqrt14(float32 f_x)
{
  const sint32 i_MagicNumber = 0x5f375a86;
  sint32 s_i;
  BML_t_FloatAsSigned u;
  u.f_d = f_x;

  s_i = u.s_x;
  s_i = i_MagicNumber - (s_i / 2);
  u.s_x = s_i;
  return u.f_d;
}

BML_INLINE float32 BML_Sqrt_VeryFast(float32 f_x);
BML_INLINE float32 BML_Sqrt_VeryFast(float32 f_x)
{
  BML_t_FloatAsUnsigned u;
  uint32 u_HexRepresentation;
  u.f_d = f_x;
  u_HexRepresentation = u.u_x;
  u_HexRepresentation -= 1U << 23U;
  u_HexRepresentation /= 2U;
  u_HexRepresentation += 1U << 29U;
  u.u_x = u_HexRepresentation;
  return u.f_d;
}

BML_INLINE float32 CML_Pow_VeryFast(float32 f_x, float32 f_y);
BML_INLINE float32 CML_Pow_VeryFast(float32 f_x, float32 f_y)
{
  const sint32  i_MagicNumberTwo = 1065353216;              
  const sint32  i_MagicNumberTre = 1064866805;              
  const float32 f_LowerBound = 0.94F;
  const float32 f_UpperBound = 1.08F;
  sint32   i_MagicNumber = ((f_x > f_LowerBound) && (f_x < f_UpperBound)) ? i_MagicNumberTwo : i_MagicNumberTre;
  const float32 f_MagicNumber = 1064866805.0F;
  BML_t_FloatAsSigned u;
  u.f_d = f_x;
  u.s_x = (sint32)((f_y * (u.s_x - i_MagicNumber)) + f_MagicNumber);
  return u.f_d;
}

BML_INLINE float32 BML_Exp_VeryFast(float32 f_a);
BML_INLINE float32 BML_Exp_VeryFast(float32 f_a)
{
  const float32 f_MagicNumberOne = 12102203.0F;
  const float32 f_MagicNumberTwo = 1064866805.0F;
  BML_t_FloatAsSigned u;
  u.s_x = (sint32) ((f_MagicNumberOne * f_a) + f_MagicNumberTwo);
  return u.f_d;
}

BML_INLINE float32 BML_Ln_VeryFast(float32 f_a);
BML_INLINE float32 BML_Ln_VeryFast(float32 f_a)
{
  const float32 f_MagicNumberOne = 8.262958405176314e-8F;  
  const sint32  i_MagicNumberTwo = 1065353216;              
  const sint32  i_MagicNumberTre = 1064866805;              
  const float32 f_LowerBound = 0.94F;
  const float32 f_UpperBound = 1.08F;
  sint32 i_MagicNumber;
  BML_t_FloatAsSigned u;
  u.f_d = f_a;
  i_MagicNumber = ((f_a > f_LowerBound) && (f_a < f_UpperBound)) ? i_MagicNumberTwo : i_MagicNumberTre;
  return (float32)(u.s_x - i_MagicNumber) * f_MagicNumberOne;
}

BML_INLINE float32 BML_f_PowToDB(float32 f_ratio);
BML_INLINE float32 BML_f_PowToDB(float32 f_ratio)
{
  return BML_DBPOW_FACTOR * BML_f_log10(f_ratio);
} 

BML_INLINE float32 BML_f_MagToDB(float32 f_ratio);
BML_INLINE float32 BML_f_MagToDB(float32 f_ratio)
{
  return CML_DBMAG_FACTOR * BML_f_log10(f_ratio);
} 

extern float32 BML_f_SqrtApprox(float32 f_radicand);

extern sint32 CML_s_GetLutIndex(const float32 f_InputValue, const float32 f_LutMinInputValue, const float32 f_LutRes, const sint32 s_LutMinInd, const sint32 s_LutMaxInd);

extern sint32 BML_s_GetLutIndexBackwards(const float32 f_InputValue, const float32 a_LUT[], const sint32 s_LutMinInd, const sint32 s_LutMaxInd);

extern boolean CML_b_IsPointInsidePolygon(const float32 a_Xarray[], const float32 a_Yarray[], uint32 u_size, float32 f_Xpoint, float32 f_Ypoint);

extern float32 BML_f_PowerOfTwo(float32 f_value);

extern float32 BML_f_XPowY(const float32 f_Base, const float32 f_Exponent);

extern BML_t_TrajRefPoint CML_CalculateDistancePoint2Circle(float32 f_X, float32 f_Y, float32 f_C0);

extern BML_t_TrajRefPoint CML_CalculateDistancePoint2Clothoid(float32 f_X, float32 f_Y, float32 f_C0, float32 f_C1);

#endif 

