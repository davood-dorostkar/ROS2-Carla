/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * shenzijian <shenzijian@senseauto.com>
 */
#pragma once
#ifndef VED_RT_MATRIXLIB_H
#define VED_RT_MATRIXLIB_H
#ifdef __cplusplus
extern "C" {
#endif
#include "ved_consts.h"  // NOLINT

void rt_BackwardSubstitutionRR_Sgl(real32_T* pU,
                                   const real32_T* pb,
                                   real32_T* x,
                                   int_T N,
                                   int_T P,
                                   boolean_T unit_upper);

void rt_ForwardSubstitutionRR_Sgl(real32_T* pL,
                                  const real32_T* pb,
                                  real32_T* x,
                                  int_T N,
                                  int_T P,
                                  const int32_T* piv,
                                  boolean_T unit_lower);

void rt_lu_real_sgl(real32_T* A, const int_T n, int32_T* piv);

void rt_MatDivRR_Sgl(real32_T* Out,
                     const real32_T* In1,
                     const real32_T* In2,
                     real32_T* lu,
                     int32_T* piv,
                     real32_T* x,
                     const int_T dims[3]);

void rt_MatMultRR_Sgl(real32_T* y,
                      const real32_T* A,
                      const real32_T* B,
                      const int_T dims[3]);

float32 VED_GDBexp(float32 f_power);

#ifdef __cplusplus
}
#endif
#endif
