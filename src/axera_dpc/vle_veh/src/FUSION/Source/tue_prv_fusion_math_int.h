/** \addtogroup tueFusion
 *  \{
 *  \file tue_prv_fusion_math_int.h
 *
 *  \brief Internal header file for fusion math AAU.
 *
 *
 *
 *
 *
 *   (C) Copyright Tuerme Inc. All rights reserved.
 *
 */

#ifndef TUE_PRV_FUSION_MATH_INT_H_
#define TUE_PRV_FUSION_MATH_INT_H_

#ifdef __cplusplus
extern "C" {
#endif

/*==================[inclusions]===========================================*/
#include "tue_prv_common_types.h"

/*==================[macros]===============================================*/
/*==================[type definitions]=====================================*/

typedef union TueObjFusn_f32UnionTypeTag {
    uint32 u32Val;
    float32 f32Val;
} TueObjFusn_f32UnionType;

/*==================[forward declarations]=================================*/
/*==================[symbolic constants]===================================*/

/** Constants for float data type */
#define TUE_PRV_FUSION_MATH_FLOAT32_SIGN_BIT_MASK (0x80000000u)
#define TUE_PRV_FUSION_MATH_FLOAT32_SIGN_BIT_POSITION (31u)
#define TUE_PRV_FUSION_MATH_FLOAT32_MANTISSA_NUM_BITS (23u)
#define TUE_PRV_FUSION_MATH_FLOAT32_MANTISSA_MASK (0x007FFFFFu)
#define TUE_PRV_FUSION_MATH_FLOAT32_EXP_MASK (0x7F800000u)
#define TUE_PRV_FUSION_MATH_FLOAT32_EXP_MASK_INF (0xFFu)
#define TUE_PRV_FUSION_MATH_FLOAT32_EXP_BIAS (127)
#define TUE_PRV_FUSION_MATH_FLOAT32_NAN_DEFAULT (0x7F800001u)
#define TUE_PRV_FUSION_MATH_FLOAT32_INF_DEFAULT (0x7F800000u)
#define TUE_PRV_FUSION_MATH_FLOAT32_MANT_ADD_IMPLICIT_ONE (0x00800000u)

/** Sqrt Constants */
#define TUE_PRV_FUSION_MATH_INV_SQRT2 (0.70710678118f)
#define TUE_PRV_FUSION_MATH_SQRT2 (1.41421356237f)

#define TUE_PRV_FUSION_MATH_SQRT_POLYNOM_1_CONSTANT_1 (-9.8843065718E-4f)
#define TUE_PRV_FUSION_MATH_SQRT_POLYNOM_1_CONSTANT_2 (7.9479950957E-4f)
#define TUE_PRV_FUSION_MATH_SQRT_POLYNOM_1_CONSTANT_3 (3.5890535377E-3f)
#define TUE_PRV_FUSION_MATH_SQRT_POLYNOM_1_CONSTANT_4 (1.1028809744E-2f)
#define TUE_PRV_FUSION_MATH_SQRT_POLYNOM_1_CONSTANT_5 (4.4195203560E-2f)
#define TUE_PRV_FUSION_MATH_SQRT_POLYNOM_1_CONSTANT_6 (3.5355338194E-1f)

#define TUE_PRV_FUSION_MATH_SQRT_POLYNOM_2_CONSTANT_1 (1.35199291026E-2f)
#define TUE_PRV_FUSION_MATH_SQRT_POLYNOM_2_CONSTANT_2 (2.26657767832E-2f)
#define TUE_PRV_FUSION_MATH_SQRT_POLYNOM_2_CONSTANT_3 (2.78720776889E-2f)
#define TUE_PRV_FUSION_MATH_SQRT_POLYNOM_2_CONSTANT_4 (3.89582788321E-2f)
#define TUE_PRV_FUSION_MATH_SQRT_POLYNOM_2_CONSTANT_5 (6.24811144548E-2f)
#define TUE_PRV_FUSION_MATH_SQRT_POLYNOM_2_CONSTANT_6 (1.25001503933E-1f)
#define TUE_PRV_FUSION_MATH_SQRT_POLYNOM_2_CONSTANT_7 (0.5f)

#define TUE_PRV_FUSION_MATH_SQRT_POLYNOM_3_CONSTANT_1 (-3.9495006054E-1f)
#define TUE_PRV_FUSION_MATH_SQRT_POLYNOM_3_CONSTANT_2 (5.1743034569E-1f)
#define TUE_PRV_FUSION_MATH_SQRT_POLYNOM_3_CONSTANT_3 (4.3214437330E-1f)
#define TUE_PRV_FUSION_MATH_SQRT_POLYNOM_3_CONSTANT_4 (3.5310730460E-1f)
#define TUE_PRV_FUSION_MATH_SQRT_POLYNOM_3_CONSTANT_5 (3.5354581892E-1f)
#define TUE_PRV_FUSION_MATH_SQRT_POLYNOM_3_CONSTANT_6 (7.0710676017E-1f)
#define TUE_PRV_FUSION_MATH_SQRT_POLYNOM_3_CONSTANT_7 (7.07106781187E-1f)

/** Log Constants */
#define TUE_PRV_FUSION_MATH_LOG2F (0.693147180559945f)

#define TUE_PRV_FUSION_MATH_LOG_COEFFICIENTS_P0 (-0.331355617479e1f)
#define TUE_PRV_FUSION_MATH_LOG_COEFFICIENTS_P1 (0.89554061525e0f)
#define TUE_PRV_FUSION_MATH_LOG_COEFFICIENTS_Q0 (-0.165677797691e1f)

/** Sine Constants */
#define TUE_PRV_FUSION_MATH_SIN_MAX_SCALING (32764.0f)
#define TUE_PRV_FUSION_MATH_SIN_SCALING_FACTOR (0.6366197723675813f)
#define TUE_PRV_FUSION_MATH_SIN_COEFFICIENTS_P0 (0.52930152776255e3f)
#define TUE_PRV_FUSION_MATH_SIN_COEFFICIENTS_P1 (-0.17389497132272e3f)
#define TUE_PRV_FUSION_MATH_SIN_COEFFICIENTS_P2 (0.1042302058487e2f)
#define TUE_PRV_FUSION_MATH_SIN_COEFFICIENTS_Q0 (0.33696381989527e3f)
#define TUE_PRV_FUSION_MATH_SIN_COEFFICIENTS_Q1 (0.2786575519992e2f)

/** atan and atan2 constants */
#define TUE_PRV_FUSION_MATH_ATAN_SQ2P1 (2.414213562373095f)
#define TUE_PRV_FUSION_MATH_ATAN_SQ2M1 (0.414213562373095f)

#define TUE_PRV_FUSION_MATH_ATAN_P0 (0.1265998609915e2f)
#define TUE_PRV_FUSION_MATH_ATAN_P1 (0.63691887127e1f)
#define TUE_PRV_FUSION_MATH_ATAN_Q0 (0.1265998646243e2f)
#define TUE_PRV_FUSION_MATH_ATAN_Q1 (0.105891113168e2f)

/*==================[return codes]=========================================*/
/*==================[functions]============================================*/
#define ObjFusn_START_SEC_CODE

uint32 tue_prv_fusion_exp(const float32 f32x);

boolean tue_prv_fusion_isInf(const float32 f32x);

boolean tue_prv_fusion_isNaN(const float32 f32x);

float32 tue_prv_fusion_copysign(const float32 f32x, const float32 f32Sign);

float32 tue_prv_fusion_frexp(const float32 f32x,
                             CONSTP2VAR(sint32, AUTOMATIC, ObjFusn_VAR_NOINIT)
                                 ps32Exp);

float32 tue_prv_fusion_set_exp(const float32 f32x, const uint32 u32Exp);

float32 tue_prv_fusion_pack(const boolean bSign,
                            const uint32 u32Exp,
                            const uint32 u32Mant);

boolean tue_prv_fusion_isZero(const float32 f32x);

LOCAL float32 tue_prv_fusion_modff(const float32 f32x,
                                   CONSTP2VAR(float32,
                                              AUTOMATIC,
                                              ObjFusn_VAR_NOINIT) pf32IntVal);

LOCAL float32 tue_prv_fusion_sin_impl(float32 f32x, sint32 s32Quad);

LOCAL float32 tue_prv_fusion_atan(float32 f32x);

LOCAL float32 tue_prv_fusion_fmod(const float32 f32x, const float32 f32y);

LOCAL float32 tue_prv_fusion_remainder(const float32 f32x, const float32 f32y);

boolean tue_prv_fusion_signbit(const float32 f32x);

uint32 tue_prv_fusion_mant(const float32);

#define ObjFusn_STOP_SEC_CODE

#ifdef __cplusplus
}
#endif
#endif /** \} TUE_PRV_FUSION_MATH_INT_H_ */
