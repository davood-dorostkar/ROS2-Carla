/*
 * Copyright (C) 2017-2021 by SenseTime Group Limited. All rights reserved.
 * He Qiushu <heqiushu@senseauto.com>
 * This is the head file of calibration lookup function, including 1D table and
 * 2D map.
 */

// INCLUDES
#include "TM_Global_Types.h"

#ifndef PROJECT_DECISION_SRC_VLC_COMMON_VLC_MAT_MAT_MAP_
#define PROJECT_DECISION_SRC_VLC_COMMON_VLC_MAT_MAT_MAP_

#ifdef __cplusplus
extern "C" {
#endif

// FUNCTIONS
float32 MatCalculateParamValue1D(const float32 table[][2],
                                 const uint8 num,
                                 const float32 x);

#ifdef __cplusplus
}
#endif
#endif  // PROJECT_DECISION_SRC_VLC_COMMON_VLC_MAT_MAT_MAP_