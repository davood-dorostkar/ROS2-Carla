

#ifndef AVLC_H
#define AVLC_H

#include "TM_Global_Const.h"
#include "acc_ext.h"
#include "acc_lib_ext.h"
#include "acc_cfg.h"

/* Aliasas for new object OOI positions */
#define Obj_first_host_lane OBJ_NEXT_OOI
#define Obj_hidden_host_lane OBJ_HIDDEN_NEXT_OOI
#define Obj_first_left_lane OBJ_NEXT_LONG_LEFT_OOI
#define Obj_second_left_lane OBJ_NEXT_LAT_LEFT_OOI
#define Obj_first_right_lane OBJ_NEXT_LONG_RIGHT_OOI
#define Obj_second_right_lane OBJ_NEXT_LAT_RIGHT_OOI

typedef /*const*/ VLC_acc_object_t* acc_object_ptr_t;

/* GLOBAL VARIABLES (KOMPONENT EXTERNAL) */

extern VLC_acc_object_t AVLC_NO_OBJECT;

/* FUNKTION PROTOTYPES (KOMPONENT INTERNAL) */
extern void AVLC_ESTIMATE_TRAFFIC_SITUATION(
    const times_t cycle_time,
    acc_object_ptr_t object_list,
    const acc_input_data_t* input,
    VLC_acc_object_t* control_object,
    VLC_acc_output_data_t* output,
    acc_driver_intention_t* driver_intention);
extern void AVLC_ESTIMATE_DRIVER_INTENTION(const times_t cycle_time,
                                           acc_driver_intention_t* output,
                                           acc_object_ptr_t object_list,
                                           const acc_input_data_t* input);
extern void AVLC_ALERT_INIT(acc_alert_data_t* acc_alert_data);
extern void AVLC_ALERT_EXEC(const acc_input_data_t* input,
                            const VLC_acc_object_t* alert_object,
                            const VLC_DFV2SenInfo_t* pDFVLongOut,
                            VLC_acc_output_data_t* output,
                            acc_alert_data_t* acc_alert_data,
                            times_t cycle_time);
extern void AVLC_SELECT_OBJECTS_OF_INTEREST(
    const acc_input_data_t* acc_input,
    const VLCCustomInput_t* pVLCCustomInput,
    acc_driver_intention_t* driver_intention,
    acc_object_ptr_t object_list,
    acc_status_t* status,
    OvertakeAssistInfo* p_overtake_assist_info,
    times_t cycle_time);
extern void AVLC_SELECT_RELEVANT_OBJECT(const acc_input_data_t* input,
                                        const VLC_DFV2SenInfo_t* pDFVLongOut,
                                        const VLCCustomInput_t* pVLCCustomInput,
                                        acc_object_ptr_t object_list,
                                        VLC_acc_object_t* control_object,
                                        VLC_acc_object_t* alert_object,
                                        VLC_acc_object_t* display_object,
                                        VLC_acc_output_data_t* output,
                                        acc_status_t* status);
#endif
