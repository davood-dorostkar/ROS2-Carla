/*
 * Copyright (C) 2017-2021 by SenseTime Group Limited. All rights reserved.
 * He Qiushu <heqiushu@senseauto.com>
 * This is the head file of Overtake Assist function
 */

#ifndef PROJECT_DECISION_SRC_VLCSEN_OVERTAKE_ASSIST_OVERTAKE_ASSIST_
#define PROJECT_DECISION_SRC_VLCSEN_OVERTAKE_ASSIST_OVERTAKE_ASSIST_

// INCLUDES
#include "TM_Global_Types.h"
#include "vlcSen_ext.h"

#ifdef __cplusplus
extern "C" {
#endif

// MACROS

// TYPEDEFS
typedef enum {
    OVERTAKE_ASSIST_DEACTIVE,
    OVERTAKE_ASSIST_ACTIVE,
} OvertakeAssistState;

typedef enum {
    NO_TURN,
    TURN_LEFT,
    TURN_RIGHT,
} TurnIndicatorState;

// typedef struct {
//     boolean overtake_assist_flag;
//     float32 target_time_gap;
//     float32 target_distance;
//     sint16 headway_setting;
// } OvertakeAssistInfo;

// FUNCTIONS
void OvertakeAssistProcess(const uint16 cycle_time,
                           const uint8 turn_indicator,
                           const uint8 speedometer_vehicle_speed,
                           const float32 ved_vehicle_speed,
                           const float32 ooi_vehicle_speed,
                           const sint8 ooi_id,
                           const sint16 headway_setting,
                           OvertakeAssistInfo *p_overtake_assist_info);

#ifdef __cplusplus
}
#endif
#endif  // PROJECT_DECISION_SRC_VLCSEN_OVERTAKE_ASSIST_OVERTAKE_ASSIST_
