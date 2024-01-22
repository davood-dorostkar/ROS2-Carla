/*
 * Copyright (C) 2022-2024 by SoftwareMotion Group Limited. All rights reserved.
 * 
 */
#ifndef _DIM_UTILS_H_
#define _DIM_UTILS_H_
#include "dim.h"
#include "TM_Global_Types.h"
void DIMGetInputValue_Float(const DIMInputValue_t inputValue,
                            const float32 fDefaultVal,
                            float32 *const pOutVal);
void DIMGetInputValue_Bool(const DIMInputValue_t inputValue,
                           const boolean bDefaultVal,
                           boolean *const pOutVal);
void DIMGetInputValue_UnsigedInt(const DIMInputValue_t inputValue,
                                 const uint8 uiDefaultVal,
                                 uint8 *const pOutVal);

#endif  // // DECISION_SRC_VLCVEH_DIM_DIM_UTILS_H_
