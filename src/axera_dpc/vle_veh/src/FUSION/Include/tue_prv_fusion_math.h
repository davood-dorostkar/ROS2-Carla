/*
* Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
* wen mingshu <wenmingshu@senseauto.com>
*/
/** \addtogroup tueFusion
 *  \{
 *  \file tue_prv_fusion_math.h
 *
 *  \brief  Header file for Fusion Math Library. Defines all math functions that
 * shall be available
 *          to other AAU's
 *
 *   (C) Copyright Tuerme Inc. All rights reserved.
 *
 */

#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_INCLUDE_TUE_PRV_FUSION_MATH_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_INCLUDE_TUE_PRV_FUSION_MATH_H_

#ifdef __cplusplus
extern "C" {
#endif

/*==================[inclusions]===========================================*/
#include "tue_prv_common_types.h"

/*==================[macros]===============================================*/
/*==================[type definitions]=====================================*/

/*==================[forward declarations]=================================*/
/*==================[symbolic constants]===================================*/

/** Various versions of PI */
#define TUE_PRV_FUSION_MATH_PI (3.141592653589793f)
#define TUE_PRV_FUSION_MATH_PI_HALF (1.570796326794896f)
#define TUE_PRV_FUSION_MATH_PI_QUARTER (0.785398163397448f)
#define TUE_PRV_FUSION_MATH_PI_THREE_QUARTER (2.356194490192345f)
#define TUE_PRV_FUSION_MATH_TWO_PI (6.283185307179586f)

#define TUE_PRV_FUSION_MATH_DEGREES_PER_RAD (57.29577951308232f)

#define TUE_PRV_FUSION_MATH_RAD_PER_DEGREE (0.017453292519943295f)

#define TUE_PRV_FUSION_MATH_COMPARE_TO_ZERO (0.000001f)

/*==================[return codes]=========================================*/
/*==================[functions]============================================*/

/**
 * Defines and functions from math.h
 **/

#define ObjFusn_START_SEC_CODE

/** floating absolute value **/
extern float32 tue_prv_fusion_abs(const float32 f32x);

/** squared root function for floating values **/
float32 tue_prv_fusion_sqrt(float32 f32x);

/** log function for floating values **/
float32 tue_prv_fusion_log(float32 f32x);

/** function for conversion between radians and degrees **/
extern float32 tue_prv_fusion_rtod(const float32 f32x);

/** cosine function for floating values **/
extern float32 tue_prv_fusion_cos(const float32 f32x);

/** sine function for floating values **/
extern float32 tue_prv_fusion_sin(const float32 f32x);

/** angle normalization - angle is mapped between [-pi pi] **/
extern float32 tue_prv_fusion_norm_angle(const float32 f32x);

/** atan function for floating values **/
float32 tue_prv_fusion_atan2(const float32 f32y, const float32 f32x);

/** power function for floating values **/
extern float32 tue_prv_fusion_pow2(const float32 f32x);

/* interpolation function for segments between to points*/
extern float32 tue_prv_fusion_interp1(const float32 f32x0,
                                      const float32 f32y0,
                                      const float32 f32x1,
                                      const float32 f32y1,
                                      const float32 f32x);

/* -----------------------------------------------------------------------*/
/**
 * \typedef  MAX
 *
 * \brief    maximum of two numbers
 *
 *-----------------------------------------------------------------------*/
extern uint16 tue_prv_fusion_max_U16(const uint16 A, const uint16 B);
extern float32 tue_prv_fusion_max_F32(const float32 A, const float32 B);
extern float32 tue_prv_fusion_min_F32(const float32 A, const float32 B);

/* -----------------------------------------------------------------------*/
/**
 * \typedef  MIN
 *
 * \brief    minimum of two numbers
 *
 * -----------------------------------------------------------------------*/
extern uint8 tue_prv_fusion_min_U8(const uint8 A, const uint8 B);
extern uint16 tue_prv_fusion_min_U16(const uint16 A, const uint16 B);

/* -----------------------------------------------------------------------*/
/**
 * \typedef  TUE_CML_MinMax
 *
 * \brief    Limit input to min and max value
 *
 * -----------------------------------------------------------------------*/
extern sint16 tue_prv_fusion_min_max_S16(const sint16 A,
                                         const sint16 minVal,
                                         const sint16 maxVal);
extern float32 tue_prv_fusion_min_max_F32(const float32 A,
                                          const float32 minVal,
                                          const float32 maxVal);
extern uint32 tue_prv_fusion_min_max_U32(const uint32 A,
                                         const uint32 minVal,
                                         const uint32 maxVal);
#define ObjFusn_STOP_SEC_CODE

#ifdef __cplusplus
}
#endif
#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_INCLUDE_TUE_PRV_FUSION_MATH_H_
