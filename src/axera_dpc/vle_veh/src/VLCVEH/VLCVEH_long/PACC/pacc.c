/*
 * Copyright (C) 2017-2021 by SoftwareMotion Group Limited. All rights reserved.
 * He Qiushu 
 * This is the implementation of PACC function (Predict Adaptive Cruise
 * Control), which can decelerate in curve road.
 */
#include "string.h"
#include "math.h"
#include "mat_common.h"
#include "mat_map.h"
#include "pacc.h"

#define CAL_START_CODE
#include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
volatile uint8 PACC_ENABLE_CFG = TRUE;

volatile float32 kPACCZeroLimit = 0.0000001;

volatile float32 kPACCLongAccelMaxThreshold =
    5;  // Max longitudinal acceleration limitation in curve road, uint:m/s2
volatile float32 kPACCLongAccelMinThreshold =
    -1.5;  // Min longitudinal acceleration limitation in curve road, uint:m/s2
volatile uint8 kPACCMaxSpeed = 150;                     // unit:km/h
volatile uint8 kPACCMinSpeed = 60;                      // unit:km/h
volatile uint8 kPACCHysteresisSpeed = 5;                // unit:km/h
volatile float32 kLaneMarkViewRangeThreshold = 15;      // unit:m
volatile float32 kLaneMarkRadiusLowerThreshold = 800;   // unit:m
volatile float32 kLaneMarkRadiusUpperThreshold = 1200;  // unit:m
volatile float32 kLaneMarkConfidenceThreshold = 80;

volatile uint8 kLatAccelLimitFromCurvaturePointNum = 4;
volatile float32 kLatAccelLimitFromCurvatureMap[4][2] = {
    {0.001, 1.1},  // radius = 1000m  speed = 125kph
    {0.002, 1.2},  // radius = 500m   speed = 88kph
    {0.004, 1.6},  // radius = 250m   speed = 72kph
    {0.008, 2.2},  // radius = 125m   speed = 60kph
};                 // unit: m, m/s^2

volatile uint8 kLatAccelLimitFromSpeedPointNum = 6;
volatile float32 kLatAccelLimitFromSpeedMap[6][2] = {
    {5, 1.8},   //
    {10, 2.0},  //
    {20, 2.2},  //
    {25, 2.2},  //
    {30, 2.2},  //
    {35, 2.2}   //
};              // unit: m/s, m/s^2

volatile uint8 kLongAccelFromLatAccelOffsetPointNum = 6;
volatile float32 kLongAccelFromLatAccelOffsetMap[6][2] = {
    {-3, -1.5}, {-2, -1},   {-1, -0.5},
    {0, -0.2},  {2.5, 2.5}, {3, 5}};  // unit: m/s^2, m/s^2

volatile float32 kPACCLongAccelerationFilterCoefficient =
    0.2;                                                       // range:0 - 1.0
volatile float32 kPACCVEDLateralAccelCoefficient = 0.5;  // range:0 - 1.0

volatile float32 kPACCLaneViewRangeFiltCoef = 0.2;  // range:0 - 1.0
volatile float32 kPACCLaneLaterallDeviationFiltCoef =
    0.8;                                                      // range:0 - 1.0
volatile float32 kPACCLaneHeadingAngleFiltCoef = 0.8;   // range:0 - 1.0
volatile float32 kPACCLaneCurvatureFiltCoef = 0.2;      // range:0 - 1.0
volatile float32 kPACCLaneCurvatureRateFiltCoef = 0.2;  // range:0 - 1.0

volatile float32 kPACCPathLaterallDeviationFiltCoef =
    0.8;                                                      // range:0 - 1.0
volatile float32 kPACCPathHeadingAngleFiltCoef = 0.8;   // range:0 - 1.0
volatile float32 kPACCPathCurvatureFiltCoef = 0.2;      // range:0 - 1.0
volatile float32 kPACCPathCurvatureRateFiltCoef = 0.2;  // range:0 - 1.0
#define CAL_STOP_CODE
#include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */               /* MD_MSR_MemMap */
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
static float32 ved_lateral_acceleration_filter = 0;  // unit: m/s^2
static uint8 kControlPathSplitCount =
    21;  // split control path to some segements
static PathPointInfo control_path_points[21];
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"

// FUNCTIONS
// float32 fabsf(float32 input) { return input < 0 ? -input : input; }

float32 zero_limit(float32 input) {
    if (fabsf(input) > kPACCZeroLimit) {
        return input;
    } else if (input > 0) {
        return kPACCZeroLimit;
    } else {
        return -kPACCZeroLimit;
    }
}

void PACCActiveConditionCheck(const uint8 speedometer_vehicle_speed,
                              const LaneMarkInfo* p_left_lane_mark,
                              const LaneMarkInfo* p_right_lane_mark,
                              PACCInfo* p_pacc_info) {
    // ego vehicle speed condition
    if (speedometer_vehicle_speed >= kPACCMinSpeed &&
        speedometer_vehicle_speed < kPACCMaxSpeed - kPACCHysteresisSpeed) {
        p_pacc_info->pacc_state = TRUE;
    } else if (speedometer_vehicle_speed > kPACCMaxSpeed &&
               speedometer_vehicle_speed <
                   kPACCMinSpeed - kPACCHysteresisSpeed) {
        p_pacc_info->pacc_state = FALSE;
    }

    // start or end of control path should be bent enough
    if (p_pacc_info->pacc_state == TRUE) {
        if (p_left_lane_mark->property.validity == TRUE ||
            p_right_lane_mark->property.validity == TRUE) {
            if (fabsf(p_pacc_info->control_path.curvature) >
                    1.0 / kLaneMarkRadiusLowerThreshold ||
                fabsf(p_pacc_info->control_path.curvature +
                      p_pacc_info->control_path.curvature_rate *
                          kLaneMarkViewRangeThreshold) >
                    1.0 / kLaneMarkRadiusLowerThreshold) {
                p_pacc_info->pacc_state = TRUE;
            } else if (fabsf(p_pacc_info->control_path.curvature) <
                           1.0 / kLaneMarkRadiusUpperThreshold &&
                       fabsf(p_pacc_info->control_path.curvature +
                             p_pacc_info->control_path.curvature_rate *
                                 kLaneMarkViewRangeThreshold) <
                           1.0 / kLaneMarkRadiusUpperThreshold) {
                p_pacc_info->pacc_state = FALSE;
            }
        } else {
            p_pacc_info->pacc_state = FALSE;
        }
    }
}

void PACCControlPathCalculate(LaneMarkInfo* p_left_lane_mark,
                              LaneMarkInfo* p_right_lane_mark,
                              PACCInfo* p_pacc_info) {
    // left lane filter
    p_left_lane_mark->filter_geometry.view_range = MatLowPassFilter(
        p_left_lane_mark->filter_geometry.view_range,
        p_left_lane_mark->geometry.view_range, kPACCLaneViewRangeFiltCoef);
    p_left_lane_mark->filter_geometry.lateral_deviation =
        MatLowPassFilter(p_left_lane_mark->filter_geometry.lateral_deviation,
                         p_left_lane_mark->geometry.lateral_deviation,
                         kPACCLaneLaterallDeviationFiltCoef);
    p_left_lane_mark->filter_geometry.heading_angle =
        MatLowPassFilter(p_left_lane_mark->filter_geometry.heading_angle,
                         p_left_lane_mark->geometry.heading_angle,
                         kPACCLaneHeadingAngleFiltCoef);
    p_left_lane_mark->filter_geometry.curvature = MatLowPassFilter(
        p_left_lane_mark->filter_geometry.curvature,
        p_left_lane_mark->geometry.curvature, kPACCLaneCurvatureFiltCoef);
    p_left_lane_mark->filter_geometry.curvature_rate =
        MatLowPassFilter(p_left_lane_mark->filter_geometry.curvature_rate,
                         p_left_lane_mark->geometry.curvature_rate,
                         kPACCLaneCurvatureRateFiltCoef);

    // right lane filter
    p_right_lane_mark->filter_geometry.view_range = MatLowPassFilter(
        p_right_lane_mark->filter_geometry.view_range,
        p_right_lane_mark->geometry.view_range, kPACCLaneViewRangeFiltCoef);
    p_right_lane_mark->filter_geometry.lateral_deviation =
        MatLowPassFilter(p_right_lane_mark->filter_geometry.lateral_deviation,
                         p_right_lane_mark->geometry.lateral_deviation,
                         kPACCLaneLaterallDeviationFiltCoef);
    p_right_lane_mark->filter_geometry.heading_angle =
        MatLowPassFilter(p_right_lane_mark->filter_geometry.heading_angle,
                         p_right_lane_mark->geometry.heading_angle,
                         kPACCLaneHeadingAngleFiltCoef);
    p_right_lane_mark->filter_geometry.curvature = MatLowPassFilter(
        p_right_lane_mark->filter_geometry.curvature,
        p_right_lane_mark->geometry.curvature, kPACCLaneCurvatureFiltCoef);
    p_right_lane_mark->filter_geometry.curvature_rate =
        MatLowPassFilter(p_right_lane_mark->filter_geometry.curvature_rate,
                         p_right_lane_mark->geometry.curvature_rate,
                         kPACCLaneCurvatureRateFiltCoef);

    // left lane validity check
    if (p_left_lane_mark->filter_geometry.view_range >
            kLaneMarkViewRangeThreshold &&
        p_left_lane_mark->property.confidence > kLaneMarkConfidenceThreshold) {
        // delay is used to reduce state switch frequently.
        if (fabsf(p_left_lane_mark->filter_geometry.curvature) >
                1.0 / kLaneMarkRadiusLowerThreshold ||
            fabsf(p_left_lane_mark->filter_geometry.curvature +
                  p_left_lane_mark->filter_geometry.curvature_rate *
                      kLaneMarkViewRangeThreshold) >
                1.0 / kLaneMarkRadiusLowerThreshold) {
            p_left_lane_mark->property.validity = TRUE;
        } else if (fabsf(p_left_lane_mark->filter_geometry.curvature) <
                       1.0 / kLaneMarkRadiusUpperThreshold &&
                   fabsf(p_left_lane_mark->filter_geometry.curvature +
                         p_left_lane_mark->filter_geometry.curvature_rate *
                             kLaneMarkViewRangeThreshold) <
                       1.0 / kLaneMarkRadiusUpperThreshold) {
            p_left_lane_mark->property.validity = FALSE;
        }
    } else {
        p_left_lane_mark->property.validity = FALSE;
    }

    // right lane validity check
    if (p_right_lane_mark->filter_geometry.view_range >
            kLaneMarkViewRangeThreshold &&
        p_right_lane_mark->property.confidence > kLaneMarkConfidenceThreshold) {
        // delay is used to reduce state switch frequently.
        if (fabsf(p_right_lane_mark->filter_geometry.curvature) >
                1.0 / kLaneMarkRadiusLowerThreshold ||
            fabsf(p_right_lane_mark->filter_geometry.curvature +
                  p_right_lane_mark->filter_geometry.curvature_rate *
                      kLaneMarkViewRangeThreshold) >
                1.0 / kLaneMarkRadiusLowerThreshold) {
            p_right_lane_mark->property.validity = TRUE;
        } else if (fabsf(p_right_lane_mark->filter_geometry.curvature) <
                       1.0 / kLaneMarkRadiusUpperThreshold &&
                   fabsf(p_right_lane_mark->filter_geometry.curvature +
                         p_right_lane_mark->filter_geometry.curvature_rate *
                             kLaneMarkViewRangeThreshold) <
                       1.0 / kLaneMarkRadiusUpperThreshold) {
            p_right_lane_mark->property.validity = FALSE;
        }
    } else {
        p_right_lane_mark->property.validity = FALSE;
    }

    // calculate control path
    if (p_left_lane_mark->property.validity == TRUE &&
        p_right_lane_mark->property.validity == FALSE) {
        // PT1 filter is used for smooth lane mark coeficients change.
        p_pacc_info->control_path.lateral_deviation = MatLowPassFilter(
            p_pacc_info->control_path.lateral_deviation,
            p_left_lane_mark->filter_geometry.lateral_deviation,
            kPACCPathLaterallDeviationFiltCoef);

        p_pacc_info->control_path.heading_angle =
            MatLowPassFilter(p_pacc_info->control_path.heading_angle,
                             p_left_lane_mark->filter_geometry.heading_angle,
                             kPACCPathHeadingAngleFiltCoef);

        p_pacc_info->control_path.curvature =
            MatLowPassFilter(p_pacc_info->control_path.curvature,
                             p_left_lane_mark->filter_geometry.curvature,
                             kPACCPathCurvatureFiltCoef);

        p_pacc_info->control_path.curvature_rate =
            MatLowPassFilter(p_pacc_info->control_path.curvature_rate,
                             p_left_lane_mark->filter_geometry.curvature_rate,
                             kPACCPathCurvatureRateFiltCoef);

        p_pacc_info->control_path.view_range =
            p_left_lane_mark->filter_geometry.view_range;
    } else if (p_left_lane_mark->property.validity == FALSE &&
               p_right_lane_mark->property.validity == TRUE) {
        p_pacc_info->control_path.lateral_deviation = MatLowPassFilter(
            p_pacc_info->control_path.lateral_deviation,
            p_right_lane_mark->filter_geometry.lateral_deviation,
            kPACCPathLaterallDeviationFiltCoef);

        p_pacc_info->control_path.heading_angle =
            MatLowPassFilter(p_pacc_info->control_path.heading_angle,
                             p_right_lane_mark->filter_geometry.heading_angle,
                             kPACCPathHeadingAngleFiltCoef);

        p_pacc_info->control_path.curvature =
            MatLowPassFilter(p_pacc_info->control_path.curvature,
                             p_right_lane_mark->filter_geometry.curvature,
                             kPACCPathCurvatureFiltCoef);

        p_pacc_info->control_path.curvature_rate =
            MatLowPassFilter(p_pacc_info->control_path.curvature_rate,
                             p_right_lane_mark->filter_geometry.curvature_rate,
                             kPACCPathCurvatureRateFiltCoef);

        p_pacc_info->control_path.view_range =
            p_right_lane_mark->filter_geometry.view_range;
    } else {
        // mean value of both lane marks
        p_pacc_info->control_path.lateral_deviation = MatLowPassFilter(
            p_pacc_info->control_path.lateral_deviation,
            (p_left_lane_mark->filter_geometry.lateral_deviation +
             p_right_lane_mark->filter_geometry.lateral_deviation) /
                2,
            kPACCPathLaterallDeviationFiltCoef);

        p_pacc_info->control_path.heading_angle = MatLowPassFilter(
            p_pacc_info->control_path.heading_angle,
            (p_left_lane_mark->filter_geometry.heading_angle +
             p_right_lane_mark->filter_geometry.heading_angle) /
                2,
            kPACCPathHeadingAngleFiltCoef);

        p_pacc_info->control_path.curvature =
            MatLowPassFilter(p_pacc_info->control_path.curvature,
                             (p_left_lane_mark->filter_geometry.curvature +
                              p_right_lane_mark->filter_geometry.curvature) /
                                 2,
                             kPACCPathCurvatureFiltCoef);

        p_pacc_info->control_path.curvature_rate = MatLowPassFilter(
            p_pacc_info->control_path.curvature_rate,
            (p_left_lane_mark->filter_geometry.curvature_rate +
             p_right_lane_mark->filter_geometry.curvature_rate) /
                2,
            kPACCPathCurvatureRateFiltCoef);

        p_pacc_info->control_path.view_range =
            (p_left_lane_mark->filter_geometry.view_range +
             p_right_lane_mark->filter_geometry.view_range) /
            2;
    }
}

void PACCControlPathPointCalculate(const float32 control_path_interval,
                                   const float32 ved_vehicle_speed,
                                   const float32 ved_lateral_acceleration,
                                   PathPointInfo control_path_points[],
                                   PACCInfo* p_pacc_info) {
    // point 0 represents the state of ego vehicle at present.
    control_path_points[0].geometry.lateral_deviation =
        p_pacc_info->control_path.lateral_deviation;
    control_path_points[0].geometry.heading_angle =
        p_pacc_info->control_path.heading_angle;
    control_path_points[0].geometry.curvature =
        p_pacc_info->control_path.curvature;
    control_path_points[0].geometry.curvature_rate =
        p_pacc_info->control_path.curvature_rate;

    control_path_points[0].position.pos_x = 0;
    control_path_points[0].position.pos_y =
        p_pacc_info->control_path.lateral_deviation;
    control_path_points[0].position.distance_interval = 0;
    control_path_points[0].position.distance_global = 0;

    control_path_points[0].kinematic.lateral_acceleration_limit =
        MatCalculateParamValue1D(
            kLatAccelLimitFromCurvatureMap, kLatAccelLimitFromCurvaturePointNum,
            fabsf(control_path_points[0].geometry.curvature));

    control_path_points[0].kinematic.longitudinal_acceleration_limit =
        MatCalculateParamValue1D(
            kLongAccelFromLatAccelOffsetMap,
            kLongAccelFromLatAccelOffsetPointNum,
            control_path_points[0].kinematic.lateral_acceleration_limit -
                fabsf(ved_lateral_acceleration));  // error between lateral
                                                   // acceleration limit and
    // actual lateral acceleration.
    control_path_points[0].kinematic.longitudinal_velocity_limit = sqrtf(
        fabsf(control_path_points[0].kinematic.lateral_acceleration_limit /
              zero_limit(control_path_points[0].geometry.curvature)));

    control_path_points[0].kinematic.longitudinal_velocity_actual =
        ved_vehicle_speed;

    // initialize
    p_pacc_info->pacc_raw_acceleration =
        control_path_points[0].kinematic.longitudinal_acceleration_limit;

    for (uint8 i = 1; i <= kControlPathSplitCount - 1; i++) {
        // geometry calculation
        control_path_points[i].geometry.curvature =
            p_pacc_info->control_path.curvature +
            p_pacc_info->control_path.curvature_rate * control_path_interval *
                i;

        // position calculation
        float32 temp_x, temp_interval_x, temp_interval_y;
        temp_x = control_path_interval * i;

        control_path_points[i].position.pos_x = temp_x;
        control_path_points[i].position.pos_y =
            p_pacc_info->control_path.lateral_deviation +
            tanf(p_pacc_info->control_path.heading_angle) * temp_x +
            1.0 / 2.0 * p_pacc_info->control_path.curvature * temp_x * temp_x +
            1.0 / 6.0 * p_pacc_info->control_path.curvature_rate * temp_x *
                temp_x * temp_x;  // y=C0+C1*x+C2*x*x+C3*x*x*x;

        temp_interval_x = control_path_interval;
        temp_interval_y = control_path_points[i].position.pos_y -
                          control_path_points[i - 1].position.pos_y;

        // calculate the distance between this point and last point.
        control_path_points[i].position.distance_interval =
            sqrtf(temp_interval_x * temp_interval_x +
                  temp_interval_y * temp_interval_y);

        // calculate the distance between this point and point0.
        control_path_points[i].position.distance_global =
            control_path_points[i - 1].position.distance_global +
            control_path_points[i].position.distance_interval;

        // kinematic calculation
        float32 temp_velocity_0, temp_velocity_i, temp_distance;

        control_path_points[i].kinematic.lateral_acceleration_limit =
            MatCalculateParamValue1D(
                kLatAccelLimitFromCurvatureMap,
                kLatAccelLimitFromCurvaturePointNum,
                fabsf(control_path_points[i].geometry.curvature));

        // a_lat = v_long^2/r -> v_long=sqrt(a_lat*r)=sqrt(a_lat/curvature)
        // the curvature has checked in previous module,so it can be confirmed
        // more than 0.001.
        control_path_points[i].kinematic.longitudinal_velocity_limit = sqrtf(
            fabsf(control_path_points[i].kinematic.lateral_acceleration_limit /
                  zero_limit(control_path_points[i].geometry.curvature)));

        temp_velocity_i =
            control_path_points[i].kinematic.longitudinal_velocity_limit;
        temp_velocity_0 =
            control_path_points[0].kinematic.longitudinal_velocity_actual;
        temp_distance = control_path_points[i].position.distance_global;

        // a_long = (v_long_i^2 - v_long_0^2)/(2*s)
        control_path_points[i].kinematic.longitudinal_acceleration_limit =
            (temp_velocity_i * temp_velocity_i -
             temp_velocity_0 * temp_velocity_0) /
            zero_limit((2 * temp_distance));

        // find the max deceleration in all the points.
        if (control_path_points[i].kinematic.longitudinal_acceleration_limit <
            p_pacc_info->pacc_raw_acceleration) {
            p_pacc_info->pacc_raw_acceleration =
                control_path_points[i]
                    .kinematic.longitudinal_acceleration_limit;
        }
    }
}

void PACCProcess(const uint16 cycle_time,
                 const uint8 speedometer_vehicle_speed,
                 const float32 ved_vehicle_speed,
                 const float32 ved_lateral_acceleration,
                 LaneMarkInfo* p_left_lane_mark,
                 LaneMarkInfo* p_right_lane_mark,
                 PACCInfo* p_pacc_info) {
    // init
    p_pacc_info->pacc_raw_acceleration = kPACCLongAccelMaxThreshold;

    // PT1 filter is used for smooth lateral acceleration curve.
    ved_lateral_acceleration_filter = MatLowPassFilter(
        ved_lateral_acceleration_filter, ved_lateral_acceleration,
        kPACCVEDLateralAccelCoefficient);

    if (PACC_ENABLE_CFG) {
        PACCControlPathCalculate(p_left_lane_mark, p_right_lane_mark,
                                 p_pacc_info);

        PACCActiveConditionCheck(speedometer_vehicle_speed, p_left_lane_mark,
                                 p_right_lane_mark, p_pacc_info);

        if (p_pacc_info->pacc_state == TRUE) {
            float32 control_path_interval =
                kLaneMarkViewRangeThreshold / (kControlPathSplitCount - 1);

            // initialize control_path_points
            memset(control_path_points, 0,
                   sizeof(PathPointInfo) * (kControlPathSplitCount));

            PACCControlPathPointCalculate(control_path_interval,
                                          ved_vehicle_speed,
                                          ved_lateral_acceleration_filter,
                                          control_path_points, p_pacc_info);
        }
    }

    float32 lateral_acceleration_limit;
    float32 raw_long_accel_limit_from_speed;

    // lateral acceleration limit from ego vehicle speed.
    lateral_acceleration_limit = MatCalculateParamValue1D(
        kLatAccelLimitFromSpeedMap, kLatAccelLimitFromSpeedPointNum,
        ved_vehicle_speed);

    // error between lateral acceleration limit and actual lateral acceleration.
    raw_long_accel_limit_from_speed = MatCalculateParamValue1D(
        kLongAccelFromLatAccelOffsetMap, kLongAccelFromLatAccelOffsetPointNum,
        lateral_acceleration_limit - fabsf(ved_lateral_acceleration_filter));

    // limit output acceleration
    if (p_pacc_info->pacc_raw_acceleration > raw_long_accel_limit_from_speed) {
        p_pacc_info->pacc_raw_acceleration = raw_long_accel_limit_from_speed;
    }

    if (p_pacc_info->pacc_raw_acceleration > kPACCLongAccelMaxThreshold) {
        p_pacc_info->pacc_raw_acceleration = kPACCLongAccelMaxThreshold;
    } else if (p_pacc_info->pacc_raw_acceleration <
               kPACCLongAccelMinThreshold) {
        p_pacc_info->pacc_raw_acceleration = kPACCLongAccelMinThreshold;
    }

    // PT1 filter is used for smooth control.
    p_pacc_info->pacc_acceleration = MatLowPassFilter(
        p_pacc_info->pacc_acceleration, p_pacc_info->pacc_raw_acceleration,
        kPACCLongAccelerationFilterCoefficient);
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */