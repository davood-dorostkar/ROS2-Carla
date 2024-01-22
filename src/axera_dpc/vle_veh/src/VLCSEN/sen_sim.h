#pragma once

#define VLC_LONG_SEN_DEBUG \
    0  // when this switch is on(1), the VLC_SEN internal variables can be
       // watched step by step. he qiushu 20211028

#if VLC_LONG_SEN_DEBUG == 1

typedef struct {
    float LatDisplRoadBordL;
    float LatDisplRoadBordR;
    float ObjDistY;
} ROVDataInput_pRelObj_t;

ROVDataInput_pRelObj_t ROVDataInput_pRelObj;
signed short global_v_own1;
signed short global_v_own2;
signed int fuzzy_rel_distance;
signed int fuzzy_speed_rel;
signed int fuzzy_a_obj;
signed int fuzzy_distance_set_error;
signed int fuzzy_distance_min_error;
signed int fuzzy_softness;
signed int fuzzy_v_own;
signed int fuzzy_v_obj;
signed int fuzzy_distance;
signed int global_host_velocity1;
signed int global_host_velocity2;
signed int global_host_acceleration;
#endif