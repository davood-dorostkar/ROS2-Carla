#pragma once
#ifndef TUNNEL_RECOGNIZE_H
#define TUNNEL_RECOGNIZE_H
#include <string.h>
#include "envm_ext.h"
#include "envm_common_utils.h"
#include "ops.h"
#include "ops_par.h"
#include "TM_Global_Types.h"
#include "assert.h"

uint8 TUETunnelRecognize(const ExtObjectList_t* pRadarObjList,
                         float fEgoCurve,
                         float fEgoSpeed);
typedef struct {
    float pObjectsDistYArray[TUE_RADAR_RAW_OBJECT_NUM];
} TunnelInternalData_t;

#define TU_SC_TUNNEL_ZEROSCORE (0.0f)

/*! @brief       the minimum velocity value for valid tunnel recognize */
#define TU_TUNNEL_MIN_VELOCITY_THREASHOLD (2.0f)

/*! @brief       the minimum object number for a valid tunnel recognize input */
#define TU_TUNNEL_MIN_OBJ_SIZE (10)

/*! @brief       the maximum variance value for object list DistY variance value
 */
#define TU_TUNNEL_MAX_VARIANCE (10000.f)

/*! @brief       the maximum value for tunnel probability */
#define TU_TUNNEL_MAX_PROBABILITY_VAL (100)

/*! @brief       the maximum increase value for tunnel recognize probability */
#define TU_TUNNEL_MAX_PROB_INCREASE_THREASHOLD (10)

/*! @brief       the maximum decrease value for tunnel recognize probability */
#define TU_TUNNEL_MAX_PROB_DECREASE_THREASHOLD (-2)

/*! @brief       the minimum probability density value for DistY */
#define TU_TUNNEL_DISTY_PROB_DENSITY_THREASHOLD (0.89f)

/*! @brief       the probability density calculate input DistY threashold */
#define TU_TUNNEL_PROB_DENSITY_CAL_DISTY_THREASHOLD (15.0f)
#endif