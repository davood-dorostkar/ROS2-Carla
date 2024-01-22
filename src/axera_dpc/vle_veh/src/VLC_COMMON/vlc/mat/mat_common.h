/*
 * Copyright (C) 2017-2021 by SenseTime Group Limited. All rights reserved.
 * He Qiushu <heqiushu@senseauto.com>
 * This is the head file of common math function.
 */

// INCLUDES
#include "TM_Global_Types.h"

#ifndef PROJECT_DECISION_SRC_VLC_COMMON_VLC_MAT_MAT_COMMON_
#define PROJECT_DECISION_SRC_VLC_COMMON_VLC_MAT_MAT_COMMON_

#ifdef __cplusplus
extern "C" {
#endif

// FUNCTIONS
float32 MatLowPassFilter(float32 old_value, float32 new_value, float32 alpha);

#ifdef __cplusplus
}
#endif
#endif  // PROJECT_DECISION_SRC_VLC_COMMON_VLC_MAT_MAT_COMMON_