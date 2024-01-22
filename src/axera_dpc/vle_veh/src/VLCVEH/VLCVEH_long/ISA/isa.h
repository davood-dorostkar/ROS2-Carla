/*
 * Copyright (C) 2017-2021 by SoftwareMotion Group Limited. All rights reserved.
 * He Qiushu 
 * This is the head file of ISA function (Intelegent Speed Assist)
 */

#ifndef PROJECT_DECISION_SRC_VLCVEH_LONG_ISA_ISA_
#define PROJECT_DECISION_SRC_VLCVEH_LONG_ISA_ISA_

// INCLUDES
#include "TM_Global_Types.h"
#include "../../vlcVeh_ext.h"

#ifdef __cplusplus
extern "C" {
#endif

// MACROS

// TYPEDEFS
typedef enum {
    ISA_SPEED_INVALID,
    ISA_SPEED_VALID,
    ISA_SPEED_ACCEPTED,
    ISA_SPEED_RELEASE,
} ISASpeedValidity;

typedef enum {
    ISA_FROM_TSR,
    ISA_FROM_NAV,
} ISASpeedSource;

// 0:No flag detected, 1:Speed limit flag detected, 2:Speed
// limit release flag detected
typedef enum {
    TSR_NO_FLAG_DETECTED,
    TSR_SPEED_LIMIT_SETUP_DETECTED,
    TSR_SPEED_LIMIT_RELEASE_DETECTED,
} TSRFlagDetectState;

// typedef struct {
//     uint8 tsr_speed_limit;
//     uint8 tsr_flag;  // 0:No flag detected, 1:Speed limit flag detected,
//     2:Speed
//                      // limit release flag detected
//     boolean tsr_active;
// } TSRInfo;

// typedef struct {
//     uint8 nav_speed_limit;
//     boolean nav_active;
// } NavInfo;

// typedef struct {
//     boolean isa_state;
//     // boolean set_speed_change_flag;  // If driver changes set speed after
//     isa
//     //                                 // speed is active and before it is
//     //                                 // released, this flag will be 1.
//     uint8 isa_speed_validity;  // 0: isa speed is invalid, 1:isa speed is
//     valid,
//                                // 2: isa speed is accept as set speed, 3: isa
//                                // speed is released, restore the previous set
//                                // speed.
//     uint8 isa_speed;
//     uint8 previous_set_speed;  // Store set speed before isa speed is
//     accepted.
// } ISAInfo;

// FUNCTIONS
void ISAProcess(const uint16 cycle_time,
                const sint16 speed_limit_source_cfg,  // speed limit from TSR(0)
                                                      // or Navigation(1)
                const NavInfo nav_info,
                const TSRInfo tsr_info,
                const boolean acc_state,
                const boolean nnp_state,
                const uint8 set_speed,
                const uint8 operation_mode,
                ISAInfo* p_isa_info);

#ifdef __cplusplus
}
#endif
#endif  // PROJECT_DECISION_SRC_VLCVEH_LONG_ISA_ISA_
