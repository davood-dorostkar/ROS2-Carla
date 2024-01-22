/*
 * Copyright (C) 2017-2021 by SenseTime Group Limited. All rights reserved.
 * He Qiushu <heqiushu@senseauto.com>
 * This is the implementation of common math function.
 */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
#include "TM_Global_Types.h"

float32 MatLowPassFilter(float32 old_value, float32 new_value, float32 alpha) {
    return alpha * new_value + (1 - alpha) * old_value;
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */