/*
 * Copyright (C) 2017-2021 by SoftwareMotion Group Limited. All rights reserved.
 * He Qiushu 
 * This is the implementation of ISA function (Intelegent Speed Assist)
 */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU1_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

#include "TM_Global_Const.h"
#include "vlcVeh_common_utils.h"
#include "isa.h"

const uint16 kSpeedLimitTimeThreshold =
    1000;  // TSR Active time duration, uint:ms
const uint16 kDriverConfirmWaitTimeThreshold =
    5000;  // Duration of waiting for driver confirmation, uint:ms
const uint8 kIsaMaxSpeedLimit = 120;
const uint8 kIsaMinSpeedLimit = 30;

void ISAProcess(const uint16 cycle_time,
                const sint16 speed_limit_source_cfg,  // speed limit from TSR(0)
                                                      // or Navigation(1)
                const NavInfo nav_info,
                const TSRInfo tsr_info,
                const boolean acc_state,
                const boolean nnp_state,
                const uint8 set_speed,
                const uint8 operation_mode,
                ISAInfo* p_isa_info) {
    static uint8 tsr_speed_limit_last_cycle = 0;
    static uint8 nav_speed_limit_last_cycle = 0;
    static uint8 isa_speed_validity_last_cycle = 0;
    static uint8 isa_speed_validity_last_state = 0;

    static uint16 tsr_setup_time_count = 0;
    static uint16 tsr_release_time_count = 0;
    static uint16 tsr_confirm_time_count = 0;
    uint16 speed_limit_thres = kSpeedLimitTimeThreshold / cycle_time;
    uint16 driver_confrim_thres = kDriverConfirmWaitTimeThreshold / cycle_time;

    if (acc_state == TRUE && nnp_state == FALSE &&
        p_isa_info->isa_state == TRUE) {
        if (speed_limit_source_cfg == ISA_FROM_TSR) {
            if (tsr_info.tsr_active == TRUE) {
                if (tsr_info.tsr_flag == TSR_SPEED_LIMIT_SETUP_DETECTED) {
                    if (tsr_info.tsr_speed_limit > 0 &&
                        tsr_speed_limit_last_cycle ==
                            tsr_info.tsr_speed_limit) {
                        if (tsr_setup_time_count < UINT16_MAX_LIMIT) {
                            tsr_setup_time_count++;
                        }
                    } else {
                        tsr_setup_time_count = 0;
                    }
                } else if (tsr_info.tsr_flag ==
                           TSR_SPEED_LIMIT_RELEASE_DETECTED) {
                    if (tsr_info.tsr_speed_limit > 0 &&
                        tsr_info.tsr_speed_limit == p_isa_info->isa_speed &&
                        tsr_speed_limit_last_cycle ==
                            tsr_info.tsr_speed_limit) {
                        if (tsr_release_time_count < UINT16_MAX_LIMIT) {
                            tsr_release_time_count++;
                        }
                    } else {
                        tsr_release_time_count = 0;
                    }
                } else {
                    tsr_setup_time_count = 0;
                    tsr_release_time_count = 0;
                }

                // when tsr is used, it needs to keep active for a while,
                // then the ISA can accept the TSR speed. Only when the
                // count is equal to threshold, the ISA speed will output.
                // so that one speed limit will only work once even it exist
                // for a long time.
                switch (p_isa_info->isa_speed_validity) {
                    case ISA_SPEED_INVALID:
                        tsr_confirm_time_count = 0;
                        p_isa_info->isa_speed = 0;
                        p_isa_info->previous_set_speed = 0;
                        p_isa_info->previous_isa_speed = 0;
                        if (tsr_setup_time_count == speed_limit_thres &&
                            tsr_info.tsr_speed_limit < set_speed) {
                            p_isa_info->isa_speed_validity = ISA_SPEED_VALID;
                            p_isa_info->isa_speed = tsr_info.tsr_speed_limit;
                            p_isa_info->previous_set_speed = set_speed;
                        }
                        break;

                    case ISA_SPEED_VALID:
                        if (operation_mode == Display_op_cc_recom_speed &&
                            set_speed == p_isa_info->isa_speed) {
                            p_isa_info->isa_speed_validity = ISA_SPEED_ACCEPTED;
                        }

                        if (tsr_confirm_time_count < driver_confrim_thres) {
                            tsr_confirm_time_count++;
                        } else {
                            if (isa_speed_validity_last_state ==
                                ISA_SPEED_INVALID) {
                                p_isa_info->isa_speed_validity =
                                    ISA_SPEED_INVALID;
                            } else if (isa_speed_validity_last_state ==
                                       ISA_SPEED_ACCEPTED) {
                                p_isa_info->isa_speed_validity =
                                    ISA_SPEED_ACCEPTED;
                                p_isa_info->isa_speed =
                                    p_isa_info->previous_isa_speed;
                            }
                        }
                        break;

                    case ISA_SPEED_ACCEPTED:
                        tsr_confirm_time_count = 0;
                        if (set_speed != p_isa_info->isa_speed) {
                            // driver changed the set speed, so the ISA speed
                            // will not release.
                            p_isa_info->isa_speed_validity = ISA_SPEED_INVALID;
                        } else if (tsr_setup_time_count == speed_limit_thres) {
                            if (tsr_info.tsr_speed_limit <
                                p_isa_info->previous_set_speed) {
                                p_isa_info->isa_speed_validity =
                                    ISA_SPEED_VALID;
                                p_isa_info->previous_isa_speed =
                                    p_isa_info->isa_speed;
                                p_isa_info->isa_speed =
                                    tsr_info.tsr_speed_limit;
                            } else {
                                p_isa_info->isa_speed_validity =
                                    ISA_SPEED_RELEASE;
                            }
                        }

                        if (tsr_release_time_count == speed_limit_thres) {
                            p_isa_info->isa_speed_validity = ISA_SPEED_RELEASE;
                        }

                        break;

                    case ISA_SPEED_RELEASE:
                        p_isa_info->isa_speed = p_isa_info->previous_set_speed;
                        if (operation_mode == Display_op_cc_recom_speed &&
                            set_speed == p_isa_info->isa_speed) {
                            p_isa_info->isa_speed_validity = ISA_SPEED_INVALID;
                        }

                        if (tsr_confirm_time_count < driver_confrim_thres) {
                            tsr_confirm_time_count++;
                        } else {
                            p_isa_info->isa_speed_validity = ISA_SPEED_INVALID;
                        }

                        break;

                    default:
                        break;
                }

                if (isa_speed_validity_last_cycle !=
                    p_isa_info->isa_speed_validity) {
                    isa_speed_validity_last_state =
                        isa_speed_validity_last_cycle;
                }

                isa_speed_validity_last_cycle = p_isa_info->isa_speed_validity;

            } else {
                tsr_setup_time_count = 0;
                tsr_release_time_count = 0;
                p_isa_info->isa_speed_validity = ISA_SPEED_INVALID;
            }
        } else if (speed_limit_source_cfg == ISA_FROM_NAV) {
            if (nav_info.nav_active == TRUE) {
                if (nav_speed_limit_last_cycle != nav_info.nav_speed_limit) {
                    p_isa_info->isa_speed = nav_info.nav_speed_limit;
                    p_isa_info->isa_speed_validity = ISA_SPEED_VALID;
                } else {
                    p_isa_info->isa_speed_validity = ISA_SPEED_INVALID;
                }
            } else {
                p_isa_info->isa_speed_validity = ISA_SPEED_INVALID;
            }
        }

    } else {
        p_isa_info->isa_speed_validity = ISA_SPEED_INVALID;
        p_isa_info->isa_speed = 0;
        p_isa_info->previous_set_speed = 0;
        p_isa_info->previous_isa_speed = 0;
        tsr_setup_time_count = 0;
        tsr_release_time_count = 0;
    }

    // ISA speed should be limited.
    if (p_isa_info->isa_speed_validity == ISA_SPEED_VALID ||
        p_isa_info->isa_speed_validity == ISA_SPEED_RELEASE) {
        if (p_isa_info->isa_speed > kIsaMaxSpeedLimit) {
            p_isa_info->isa_speed = kIsaMaxSpeedLimit;
        } else if (p_isa_info->isa_speed < kIsaMinSpeedLimit) {
            p_isa_info->isa_speed = kIsaMinSpeedLimit;
        }
    }

    tsr_speed_limit_last_cycle = tsr_info.tsr_speed_limit;
    nav_speed_limit_last_cycle = nav_info.nav_speed_limit;
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU1_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */