
#ifndef VLC_H
#define VLC_H

#include "veh_sim.h"
#include "pacc.h"
#include "vlcVeh_consts.h"
#include "vlcVeh_common_utils.h"

/* includes            */
#include "cc_ext.h"
//#include "vlc_long_veh.h"
#include "cart_ext.h"
#include "cc_cfg.h"

#ifdef __cplusplus
extern "C" {
#endif
/* SYMBOLIC CONSTANTS (KOMPONENT EXTERNAL) */

/* MACROS (KOMPONENT EXTERNAL) */

/* TYPEDEFS (KOMPONENT EXTERNAL) */

#define Cc_speed_default_value ((setspeed_t)255)

/* GLOBAL VARIABLES (KOMPONENT INTERNAL) */
#if VLC_LONG_VEH_DEBUG == 1
LaneMarkInfo left_lane_mark;
LaneMarkInfo right_lane_mark;
#endif

/* FUNKTION PROTOTYPES (KOMPONENT EXTERNAL) */
extern void VLC_LIMIT_LATERAL_ACCEL(const times_t cycle_time,
                                    const cc_input_data_t* input,
                                    const VLC_acc_output_data_t* acc_output,
                                    const cart_das_input_data_t* das_input,
                                    const t_CamLaneInputData* pCamLaneData,
                                    cc_status_t* cc_status,
                                    PACCInfo* p_pacc_info);
extern void VLC_COMMAND_ACCEL(const times_t cycle_time,
                              const cc_input_data_t* input,
                              const cart_das_input_data_t* das_input,
                              const VLC_acc_output_data_t* acc_output,
                              const t_CamLaneInputData* pCamLaneData,
                              cart_das_output_data_t* das_output,
                              cc_status_t* cc_status,
                              PACCInfo* p_pacc_info);
extern void VLC_DETERMINE_CONTROLSTATE_SETSPEED(
    const times_t cycle_time,
    const cart_das_input_data_t* das_input,
    const cc_driver_controls_t* driver_controls,
    const cc_input_data_t* input,
    cc_error_data_t* error_data,
    cart_das_output_data_t* das_output,
    cc_status_t* cc_status,
    const VLC_acc_output_data_t* acc_output);
extern void VLC_HMI_INIT(const cc_input_data_t* input_data,
                         cc_status_t* cc_status);

#ifdef __cplusplus
};
#endif
#endif
