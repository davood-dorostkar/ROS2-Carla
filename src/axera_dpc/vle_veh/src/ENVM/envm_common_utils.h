#pragma once

#ifndef ENVM_COMMON_UTILS_H
#define ENVM_COMMON_UTILS_H
#ifdef __cplusplus
extern "C" {
#endif
#include "envm_consts.h"
#include "tue_common_libs.h"
#include "TM_Global_Types.h"

#ifndef LOCAL
#define LOCAL static
#endif

#ifndef FLT_ONE_QUARTER
#define FLT_ONE_QUARTER (0.25f)
#endif

#ifndef FLT_FOUR
#define FLT_FOUR (float32)(4.0f)
#endif

#ifndef FLT_ONE
#define FLT_ONE (float32)(1.0f)
#endif

#ifndef U32_MAX
#define U32_MAX (uint32)(4294967295u)
#endif

#define TUE_PRV_FUSION_MATH_FLOAT32_EXP_MASK_INF (0xFFu)
#define TUE_PRV_FUSION_MATH_FLOAT32_NAN_DEFAULT (0x7F800001u)
#define TUE_PRV_FUSION_MATH_FLOAT32_MANTISSA_NUM_BITS (23u)
#define TUE_PRV_FUSION_MATH_FLOAT32_EXP_MASK (0x7F800000u)
#define TUE_PRV_FUSION_MATH_FLOAT32_MANTISSA_MASK (0x007FFFFFu)
#define TUE_PRV_FUSION_MATH_FLOAT32_EXP_BIAS (127)
#define TUE_PRV_FUSION_MATH_FLOAT32_SIGN_BIT_MASK (0x80000000u)

/** Sine Constants */
#define TUE_PRV_FUSION_MATH_SIN_MAX_SCALING (32764.0f)
#define TUE_PRV_FUSION_MATH_SIN_SCALING_FACTOR (0.6366197723675813f)
#define TUE_PRV_FUSION_MATH_SIN_COEFFICIENTS_P0 (0.52930152776255e3f)
#define TUE_PRV_FUSION_MATH_SIN_COEFFICIENTS_P1 (-0.17389497132272e3f)
#define TUE_PRV_FUSION_MATH_SIN_COEFFICIENTS_P2 (0.1042302058487e2f)
#define TUE_PRV_FUSION_MATH_SIN_COEFFICIENTS_Q0 (0.33696381989527e3f)
#define TUE_PRV_FUSION_MATH_SIN_COEFFICIENTS_Q1 (0.2786575519992e2f)

/* used for global or static SWC specific constants */
#define ObjFusn_CONST
#define CONST(consttype, memclass) const consttype

// #ifndef CONSTP2VAR
// #define CONSTP2VAR(ptrtype, memclass, ptrclass) ptrtype* const
// #endif

typedef union TueObjFusn_f32UnionTypeTag {
    uint32 u32Val;
    float32 f32Val;
} TueObjFusn_f32UnionType;

#define CML_BAYES2_CPT_SIZE (4)

BML_t_TrajRefPoint Tue_CML_CalculateDistancePoint2Circle(float32 f_X,
                                                         float32 f_Y,
                                                         float32 f_C0);
uint8 Tue_CML_Bayes2(uint8 u_ProbabilityA,
                     uint8 u_ProbabilityB,
                     const uint8 a_CPT[CML_BAYES2_CPT_SIZE]);
float32 Tue_CML_BoundedLinInterpol(BML_t_LinFunctionArgs const* const p_Params,
                                   const float32 f_Value);

void Tue_CML_LowPassFilter2(float32* f_Old, float32 f_New, float32 f_Alpha);

float32 Tue_CML_BoundedLinInterpol2(float32 f_IVal,
                                    float32 f_Imin,
                                    float32 f_Imax,
                                    float32 f_Omin,
                                    float32 f_Omax);

float32 TUE_CML_EMPCalcVariance(const float32 afValues[], uint16 uiArraySize);

#define AlgoMathBayes2(ProbabilityA, ProbabilityB, CPT) \
    Tue_CML_Bayes2(ProbabilityA, ProbabilityB, CPT)
#define dGDBmathLinearFunction(dPara, dEingang) \
    Tue_CML_BoundedLinInterpol(dPara, dEingang)

// VLCSen add
/* floating point multiply and add: (a * b) + d */
#define BML_f_MultAdd(a, b, d) TUE_BML_f_MultAddGen(a, b, d)

/** cosine function for floating values **/
extern float32 tue_prv_em_cos(const float32 f32x);

/** sine function for floating values **/
extern float32 tue_prv_em_sin(const float32 f32x);

LOCAL float32 tue_prv_fusion_sin_impl(float32 f32x, sint32 s32Quad);

uint32 tue_prv_em_exp(const float32 f32x);

boolean tue_prv_em_isNaN(const float32 f32x);

LOCAL float32 tue_prv_fusion_modff(const float32 f32x,
                                   CONSTP2VAR(float32,
                                              AUTOMATIC,
                                              ObjFusn_VAR_NOINIT) pf32IntVal);

float32 tue_prv_em_copysign(const float32 f32x, const float32 f32Sign);

#ifdef __cplusplus
}
#endif
#endif
