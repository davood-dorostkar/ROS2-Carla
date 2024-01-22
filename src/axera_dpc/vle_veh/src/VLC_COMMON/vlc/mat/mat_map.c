/*
 * Copyright (C) 2017-2021 by SenseTime Group Limited. All rights reserved.
 * He Qiushu <heqiushu@senseauto.com>
 * This is the implementation of calibration lookup function, including 1D table
 * and 2D map.
 */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

#include "TM_Global_Types.h"

float32 MatCalculateParamValue1D(const float32 table[][2],
                                 const uint8 num,
                                 const float32 x) {
    uint8 n;
    float32 y;
    float32 x2;
    float32 y2;
    float32 x1;
    float32 y1;

    /* find range on x-axle */
    n = 1;
    while (n < num && x >= table[n][0]) {
        n++;
    }

    /* calculate y */
    if (n < num) {
        /*take actual table position and previous table position for
         * interpolation*/
        x1 = table[n - 1][0];
        y1 = table[n - 1][1];

        x2 = table[n][0];
        y2 = table[n][1];

        if ((x >= x1) && (x1 < x2)) {
            /*interpolation*/
            y = (x - x1) * (y2 - y1) / (x2 - x1) + y1;
        } else {
            /*if x < actual (first) table position use first y value from
             * table*/
            y = y1;
        }
    } else {
        /*if x was greater than last value in table --> take last y value from
         * table*/
        y = table[num - 1][1];
    }

    return y;
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */