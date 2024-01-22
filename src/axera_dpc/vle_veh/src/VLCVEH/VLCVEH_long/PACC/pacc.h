/*
 * Copyright (C) 2017-2021 by SoftwareMotion Group Limited. All rights reserved.
 * He Qiushu 
 * This is the head file of PACC function (Predict Adaptive Cruise Control),
 * which can decelerate in curve road.
 */

#ifndef PROJECT_DECISION_SRC_VLCVEH_LONG_PACC_PACC_
#define PROJECT_DECISION_SRC_VLCVEH_LONG_PACC_PACC_

// INCLUDES
#include "TM_Global_Types.h"
#include "../../vlcVeh_ext.h"

#ifdef __cplusplus
extern "C" {
#endif

// TYPEDEFS
// typedef struct {
//     float32 lateral_deviation;
//     float32 heading_angle;
//     float32 curvature;
//     float32 curvature_rate;
//     float32 view_range;
// } PathGeometry;

// typedef struct {
//     float32 lateral_acceleration_limit;
//     float32 longitudinal_acceleration_limit;
//     float32 longitudinal_velocity_limit;
//     float32 longitudinal_velocity_actual;
// } PathKinematic;

// typedef struct {
//     float32 pos_x;
//     float32 pos_y;
//     float32 distance_interval;
//     float32 distance_global;
// } PathPointPosition;

// typedef struct {
//     PathGeometry geometry;
//     PathKinematic kinematic;
//     PathPointPosition position;
// } PathPointInfo;

// typedef struct {
//     uint8 confidence;
//     boolean validity;
// } LaneProperty;

// typedef struct {
//     PathGeometry geometry;
//     PathGeometry filter_geometry;
//     LaneProperty property;
// } LaneMarkInfo;

// typedef struct {
//     boolean pacc_state;
//     float32 pacc_acceleration;
//     float32 pacc_raw_acceleration;
//     PathGeometry control_path;  // path used for PACC control
// } PACCInfo;

// FUNCTIONS
void PACCProcess(const uint16 cycle_time,
                 const uint8 speedometer_vehicle_speed,
                 const float32 ved_vehicle_speed,
                 const float32 ved_lateral_acceleration,
                 LaneMarkInfo* p_left_lane_mark,
                 LaneMarkInfo* p_right_lane_mark,
                 PACCInfo* p_pacc_info);

#ifdef __cplusplus
}
#endif
#endif  // PROJECT_DECISION_SRC_VLCVEH_LONG_PACC_PACC_
