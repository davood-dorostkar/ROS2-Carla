

#include "ved_consts.h"
#include "ved_mot_st_types.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/* QAC Fixes */

/* Const memory section */
/* Definition for custom storage class: Const */
const uint8 ved__ALN_to_perc_p[15] = {70U, 30U, 33U, 10U, 2U,  0U, 0U, 34U,
                                      0U,  98U, 30U, 70U, 33U, 0U, 2U};

const uint8 ved__brake_torque_to_perc_p[21] = {49U, 40U, 15U, 20U, 4U, 0U, 30U,
                                               2U,  20U, 70U, 20U, 8U, 0U, 30U,
                                               49U, 40U, 15U, 20U, 4U, 0U, 30U};

const uint8 ved__cpt_pre_break_torque_p[4] = {0U, 100U, 80U, 100U};

const uint8 ved__cpt_pre_gear_parking_p[16] = {0U,  100U, 80U, 100U, 80U, 100U,
                                               80U, 100U, 50U, 100U, 90U, 100U,
                                               90U, 100U, 90U, 100U};

const uint8 ved__cpt_pulse_velo_dir_p[4] = {0U, 100U, 60U, 100U};

const uint8 ved__cpt_pulse_velo_p[4] = {0U, 100U, 85U, 100U};

const uint8 ved__cpt_whl_pulse_p[16] = {0U,   90U,  90U,  100U, 90U,  100U,
                                        100U, 100U, 90U,  100U, 100U, 100U,
                                        100U, 100U, 100U, 100U};

const uint8 ved__gear_shift_to_perc_p[18] = {7U,  35U, 5U,  30U, 10U, 33U,
                                             86U, 30U, 15U, 15U, 0U,  33U,
                                             7U,  35U, 80U, 5U,  0U,  33U};

const uint8 ved__parking_break_to_perc_p[9] = {25U, 15U, 0U,  10U, 70U,
                                               0U,  25U, 15U, 0U};

const uint8 ved__veh_velocity_to_perc_p[15] = {
    49U, 15U, 10U, 15U, 33U, 2U, 70U, 10U, 15U, 33U, 49U, 15U, 10U, 15U, 33U};

const uint8 ved__whl_direction_to_perc_p[12] = {95U, 5U, 33U, 0U,  0U,  0U,
                                                34U, 0U, 5U,  95U, 33U, 0U};

const uint8 ved__whl_puls_to_perc_p[15] = {
    49U, 2U, 25U, 25U, 0U, 2U, 98U, 25U, 25U, 0U, 49U, 2U, 25U, 25U, 0U};

const uint8 ved__yaw_rate_to_perc_p[6] = {40U, 10U, 0U, 0U, 0U, 10U};
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */