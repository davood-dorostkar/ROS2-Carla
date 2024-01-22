/* **********************************************************************
Autosar Memory Map
**************************************************************************** */

/*
 * Copyright (C) 2017-2021 by SenseTime Group Limited. All rights reserved.
 * He Qiushu <heqiushu@senseauto.com>
 * This is the implementation of ISA function (Intelegent Speed Assist)
 */

#include "math.h"
#include "overtake_assist.h"
#include "TM_Global_Const.h"
// #include "vlc_sen.h"

#define CAL_START_CODE
#include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
const volatile uint8 kOvertakeAssistMaxSpeed = 150;           // unit:km/h
const volatile uint8 kOvertakeAssistMinSpeed = 60;            // unit:km/h
const volatile uint8 kOvertakeAssistHysteresisSpeed = 5;      // unit:km/h
const volatile float32 kEgoToOOIRelativeSpeedLimit = 15;      // unit:m/s
const volatile float32 kEgoToOOIRelativeSpeedHysteresis = 2;  // unit:m/s
const volatile uint16 kOvertakeAssistTimeThreshold =
    6000;  // Max Overtake Assist active time duration. uint: ms
const volatile uint8 exit_cycle_interval = 1;  // uint: cycle
#define CAL_STOP_CODE
#include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
static sint8 kOoiIDInvalid = -1;
static uint8 turn_indicator_last_cycle;
static boolean left_turn_indicator_active_flag = FALSE;
static uint16 left_turn_indicator_active_time = 0;  // uint: ms
static uint8 exit_cycle_count = 0;                  // uint: cycle
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"

void OvertakeAssistProcess(const uint16 cycle_time,
                           const uint8 turn_indicator,
                           const uint8 speedometer_vehicle_speed,
                           const float32 ved_vehicle_speed,
                           const float32 ooi_vehicle_speed,
                           const sint8 ooi_id,
                           const sint16 headway_setting,
                           OvertakeAssistInfo *p_overtake_assist_info) {
    p_overtake_assist_info->overtake_assist_flag_last_cycle =
        p_overtake_assist_info->overtake_assist_flag;

    // Overtake assist active only when left turn indicator active.
    if (turn_indicator == TURN_LEFT && turn_indicator_last_cycle == NO_TURN) {
        left_turn_indicator_active_flag = TRUE;
    } else if (turn_indicator != TURN_LEFT &&
               turn_indicator_last_cycle == TURN_LEFT) {
        left_turn_indicator_active_flag = FALSE;
    }

    if (left_turn_indicator_active_flag == TRUE) {
        if (left_turn_indicator_active_time < kOvertakeAssistTimeThreshold) {
            // prevent overflow, the if condition is necessary.
            left_turn_indicator_active_time += cycle_time;
        }
    } else {
        left_turn_indicator_active_time = 0;
    }

    if (left_turn_indicator_active_time < kOvertakeAssistTimeThreshold &&
        left_turn_indicator_active_flag == TRUE && ooi_id != kOoiIDInvalid) {
        // Hysteresis is applied to prevent frequently state change when speed
        // oscillates around threshold.
        if (speedometer_vehicle_speed >= kOvertakeAssistMinSpeed &&
            speedometer_vehicle_speed <
                kOvertakeAssistMaxSpeed - kOvertakeAssistHysteresisSpeed &&
            fabsf(ved_vehicle_speed - ooi_vehicle_speed) <
                kEgoToOOIRelativeSpeedLimit) {
            p_overtake_assist_info->overtake_assist_flag =
                OVERTAKE_ASSIST_ACTIVE;
        } else if (speedometer_vehicle_speed <
                       kOvertakeAssistMinSpeed -
                           kOvertakeAssistHysteresisSpeed ||
                   speedometer_vehicle_speed > kOvertakeAssistMaxSpeed ||
                   fabsf(ved_vehicle_speed - ooi_vehicle_speed) >
                       kEgoToOOIRelativeSpeedLimit +
                           kEgoToOOIRelativeSpeedHysteresis) {
            p_overtake_assist_info->overtake_assist_flag =
                OVERTAKE_ASSIST_DEACTIVE;
        }
    } else {
        p_overtake_assist_info->overtake_assist_flag = OVERTAKE_ASSIST_DEACTIVE;
    }

    if (p_overtake_assist_info->overtake_assist_flag ==
        OVERTAKE_ASSIST_ACTIVE) {
        p_overtake_assist_info->headway_setting = 0;
    } else if (p_overtake_assist_info->overtake_assist_flag ==
                   OVERTAKE_ASSIST_DEACTIVE &&
               p_overtake_assist_info->overtake_assist_flag_last_cycle ==
                   OVERTAKE_ASSIST_ACTIVE) {
        p_overtake_assist_info->release_flag = TRUE;
    } else if (p_overtake_assist_info->release_flag == TRUE) {
        exit_cycle_count++;
        if (p_overtake_assist_info->headway_setting < headway_setting) {
            if (exit_cycle_count % exit_cycle_interval == 0) {
                p_overtake_assist_info->headway_setting++;
            }
        } else if (p_overtake_assist_info->headway_setting > headway_setting) {
            if (exit_cycle_count % exit_cycle_interval == 0) {
                p_overtake_assist_info->headway_setting--;
            }
        } else {
            p_overtake_assist_info->release_flag = FALSE;
        }
    } else {
        p_overtake_assist_info->headway_setting = headway_setting;
    }

    turn_indicator_last_cycle = turn_indicator;
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */