

#ifndef RTW_HEADER_ved__mot_st_h_
#define RTW_HEADER_ved__mot_st_h_
#ifndef ved__mot_st_COMMON_INCLUDES_
#define ved__mot_st_COMMON_INCLUDES_
#include <math.h>
#include <stddef.h>
#include <string.h>
#include "ved_consts.h"
#include "ved.h"

#endif /* ved__mot_st_COMMON_INCLUDES_ */

#include "ved_mot_st_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm) ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val) ((rtm)->errorStatus = (val))
#endif

/* QAC Fixes */

/* Block signals for system '<S1>/Get_IO_State1' */
typedef struct {
    uint8_T IndexVector; /* '<S16>/Index Vector' */
} rtB_Get_IO_State1_ved__mot_st;

/* Block signals for system '<S59>/get_percentage' */
typedef struct {
    uint8 y1; /* '<S59>/get_percentage' */
    uint8 y2; /* '<S59>/get_percentage' */
    uint8 dT; /* '<S59>/get_percentage' */
} rtB_get_percentage_ved__mot_st;

/* Block signals for system '<S61>/get_percentage' */
typedef struct {
    uint8 y1; /* '<S61>/get_percentage' */
    uint8 y2; /* '<S61>/get_percentage' */
    uint8 dT; /* '<S61>/get_percentage' */
} rtB_get_percentage_ved__mot_st_n;

/* Block signals for system '<S63>/get_percentage' */
typedef struct {
    uint8 y1; /* '<S63>/get_percentage' */
    uint8 y2; /* '<S63>/get_percentage' */
    uint8 dT; /* '<S63>/get_percentage' */
} rtB_get_percentage_ved__mot_st_k;

/* Block signals for system '<S8>/whl_fl_motion_percentage' */
typedef struct {
    uint8_T fwd_percentage; /* '<S8>/whl_fl_motion_percentage' */
    uint8_T ss_percentage;  /* '<S8>/whl_fl_motion_percentage' */
    uint8_T rws_percentage; /* '<S8>/whl_fl_motion_percentage' */
} rtB_whl_fl_motion_percentage_vd;

/* Block signals for system '<S94>/get_percentage' */
typedef struct {
    uint8 y1; /* '<S94>/get_percentage' */
    uint8 y2; /* '<S94>/get_percentage' */
    uint8 dT; /* '<S94>/get_percentage' */
} rtB_get_percentage_ved__mot_st_m;

/* Block signals for system '<S95>/get_percentage' */
typedef struct {
    uint8 y1; /* '<S95>/get_percentage' */
    uint8 y2; /* '<S95>/get_percentage' */
    uint8 dT; /* '<S95>/get_percentage' */
} rtB_get_percentage_ved__mot_st_g;

/* Block signals for system '<S96>/get_percentage' */
typedef struct {
    uint8 y1; /* '<S96>/get_percentage' */
    uint8 y2; /* '<S96>/get_percentage' */
    uint8 dT; /* '<S96>/get_percentage' */
} rtB_get_percentage_ved__mot_st_p;

/* Block signals for system '<S90>/whl_puls_fl_percentage' */
typedef struct {
    uint8_T fwd_percentage; /* '<S90>/whl_puls_fl_percentage' */
    uint8_T ss_percentage;  /* '<S90>/whl_puls_fl_percentage' */
    uint8_T rws_percentage; /* '<S90>/whl_puls_fl_percentage' */
    uint8_T cnt_ramp_out;   /* '<S90>/whl_puls_fl_percentage' */
    uint8_T cnt_delay_out;  /* '<S90>/whl_puls_fl_percentage' */
} rtB_whl_puls_fl_percentage_ved__;

/* Block signals (auto storage) */
typedef struct {
    rtB_whl_puls_fl_percentage_ved__
        sf_whl_puls_fl_percentage_p; /* '<S93>/whl_puls_fl_percentage' */
    rtB_get_percentage_ved__mot_st_p
        sf_get_percentage_n; /* '<S117>/get_percentage' */
    rtB_get_percentage_ved__mot_st_g
        sf_get_percentage_g; /* '<S116>/get_percentage' */
    rtB_get_percentage_ved__mot_st_m
        sf_get_percentage_ln; /* '<S115>/get_percentage' */
    rtB_whl_puls_fl_percentage_ved__
        sf_whl_puls_fl_percentage_e; /* '<S92>/whl_puls_fl_percentage' */
    rtB_get_percentage_ved__mot_st_p
        sf_get_percentage_e; /* '<S110>/get_percentage' */
    rtB_get_percentage_ved__mot_st_g
        sf_get_percentage_d2; /* '<S109>/get_percentage' */
    rtB_get_percentage_ved__mot_st_m
        sf_get_percentage_l2; /* '<S108>/get_percentage' */
    rtB_whl_puls_fl_percentage_ved__
        sf_whl_puls_fl_percentage_o; /* '<S91>/whl_puls_fl_percentage' */
    rtB_get_percentage_ved__mot_st_p
        sf_get_percentage_f; /* '<S103>/get_percentage' */
    rtB_get_percentage_ved__mot_st_g
        sf_get_percentage_d; /* '<S102>/get_percentage' */
    rtB_get_percentage_ved__mot_st_m
        sf_get_percentage_l; /* '<S101>/get_percentage' */
    rtB_whl_puls_fl_percentage_ved__
        sf_whl_puls_fl_percentage; /* '<S90>/whl_puls_fl_percentage' */
    rtB_get_percentage_ved__mot_st_p
        sf_get_percentage_c; /* '<S96>/get_percentage' */
    rtB_get_percentage_ved__mot_st_g
        sf_get_percentage_k; /* '<S95>/get_percentage' */
    rtB_get_percentage_ved__mot_st_m
        sf_get_percentage;                       /* '<S94>/get_percentage' */
    rtB_Get_IO_State1_ved__mot_st Get_IO_State8; /* '<S9>/Get_IO_State8' */
    rtB_Get_IO_State1_ved__mot_st Get_IO_State3; /* '<S9>/Get_IO_State3' */
    rtB_Get_IO_State1_ved__mot_st Get_IO_State2; /* '<S9>/Get_IO_State2' */
    rtB_Get_IO_State1_ved__mot_st Get_IO_State1; /* '<S9>/Get_IO_State1' */
    rtB_whl_fl_motion_percentage_vd
        sf_whl_rr_motion_percentage; /* '<S8>/whl_rr_motion_percentage' */
    rtB_whl_fl_motion_percentage_vd
        sf_whl_rl_motion_percentage; /* '<S8>/whl_rl_motion_percentage' */
    rtB_whl_fl_motion_percentage_vd
        sf_whl_fr_motion_percentage; /* '<S8>/whl_fr_motion_percentage' */
    rtB_whl_fl_motion_percentage_vd
        sf_whl_fl_motion_percentage; /* '<S8>/whl_fl_motion_percentage' */
    rtB_Get_IO_State1_ved__mot_st Get_IO_State8_d; /* '<S8>/Get_IO_State8' */
    rtB_Get_IO_State1_ved__mot_st Get_IO_State7;   /* '<S8>/Get_IO_State7' */
    rtB_Get_IO_State1_ved__mot_st Get_IO_State6;   /* '<S8>/Get_IO_State6' */
    rtB_Get_IO_State1_ved__mot_st Get_IO_State5;   /* '<S8>/Get_IO_State5' */
    rtB_get_percentage_ved__mot_st_k
        sf_get_percentage_o; /* '<S64>/get_percentage' */
    rtB_get_percentage_ved__mot_st_k
        sf_get_percentage_b; /* '<S63>/get_percentage' */
    rtB_get_percentage_ved__mot_st_n
        sf_get_percentage_br; /* '<S62>/get_percentage' */
    rtB_get_percentage_ved__mot_st_n
        sf_get_percentage_h; /* '<S61>/get_percentage' */
    rtB_get_percentage_ved__mot_st
        sf_get_percentage_n0; /* '<S60>/get_percentage' */
    rtB_get_percentage_ved__mot_st
        sf_get_percentage_du;                       /* '<S59>/get_percentage' */
    rtB_Get_IO_State1_ved__mot_st Get_IO_State2_e;  /* '<S6>/Get_IO_State2' */
    rtB_Get_IO_State1_ved__mot_st Get_IO_State8_m;  /* '<S5>/Get_IO_State8' */
    rtB_Get_IO_State1_ved__mot_st Get_IO_State4_b;  /* '<S5>/Get_IO_State4' */
    rtB_Get_IO_State1_ved__mot_st Get_IO_State3_b;  /* '<S5>/Get_IO_State3' */
    rtB_Get_IO_State1_ved__mot_st Get_IO_State2_d0; /* '<S5>/Get_IO_State2' */
    rtB_Get_IO_State1_ved__mot_st Get_IO_State1_h;  /* '<S5>/Get_IO_State1' */
    rtB_Get_IO_State1_ved__mot_st Get_IO_State2_d;  /* '<S4>/Get_IO_State2' */
    rtB_Get_IO_State1_ved__mot_st Get_IO_State8_b;  /* '<S33>/Get_IO_State8' */
    rtB_Get_IO_State1_ved__mot_st Get_IO_State3_e;  /* '<S33>/Get_IO_State3' */
    rtB_Get_IO_State1_ved__mot_st Get_IO_State2_m1; /* '<S33>/Get_IO_State2' */
    rtB_Get_IO_State1_ved__mot_st Get_IO_State1_k;  /* '<S33>/Get_IO_State1' */
    rtB_Get_IO_State1_ved__mot_st Get_IO_State8_f;  /* '<S1>/Get_IO_State8' */
    rtB_Get_IO_State1_ved__mot_st Get_IO_State4;    /* '<S1>/Get_IO_State4' */
    rtB_Get_IO_State1_ved__mot_st Get_IO_State3_o;  /* '<S1>/Get_IO_State3' */
    rtB_Get_IO_State1_ved__mot_st Get_IO_State2_m;  /* '<S1>/Get_IO_State2' */
    rtB_Get_IO_State1_ved__mot_st Get_IO_State1_g;  /* '<S1>/Get_IO_State1' */
} BlockIO_ved__mot_st;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
    real32_T FWD_Init1_DSTATE;             /* '<S11>/FWD_Init1' */
    real32_T FWD_Init1_DSTATE_g;           /* '<S12>/FWD_Init1' */
    real32_T FWD_Init1_DSTATE_i;           /* '<S13>/FWD_Init1' */
    real32_T UD_DSTATE;                    /* '<S123>/UD' */
    real32_T UD_DSTATE_k;                  /* '<S122>/UD' */
    real32_T T3_DSTATE;                    /* '<S124>/T3' */
    real32_T T2_DSTATE;                    /* '<S124>/T2' */
    real32_T T1_DSTATE;                    /* '<S124>/T1' */
    real32_T T0_DSTATE;                    /* '<S124>/T0' */
    real32_T T4_DSTATE;                    /* '<S124>/T4' */
    real32_T T5_DSTATE;                    /* '<S124>/T5' */
    real32_T T6_DSTATE;                    /* '<S124>/T6' */
    int8_T mot_count_delay_DSTATE;         /* '<S1>/mot_count_delay' */
    uint8_T cnt_ramp_DSTATE;               /* '<S90>/cnt_ramp' */
    uint8_T cnt_delay_DSTATE;              /* '<S90>/cnt_delay' */
    uint8_T cnt_ramp_DSTATE_l;             /* '<S91>/cnt_ramp' */
    uint8_T cnt_delay_DSTATE_j;            /* '<S91>/cnt_delay' */
    uint8_T cnt_ramp_DSTATE_k;             /* '<S92>/cnt_ramp' */
    uint8_T cnt_delay_DSTATE_ju;           /* '<S92>/cnt_delay' */
    uint8_T cnt_ramp_DSTATE_d;             /* '<S93>/cnt_ramp' */
    uint8_T cnt_delay_DSTATE_f;            /* '<S93>/cnt_delay' */
    uint8_T cnt_delay_down_DSTATE;         /* '<S7>/cnt_delay_down' */
    uint8_T cnt_delay_up_DSTATE;           /* '<S7>/cnt_delay_up' */
    uint8_T Init2_DSTATE;                  /* '<S21>/Init2' */
    uint8_T ALN_e_Direction_Prev_DSTATE;   /* '<S21>/ALN_e_Direction_Prev' */
    uint8_T FWD_Init7_DSTATE;              /* '<S1>/FWD_Init7' */
    uint8_T SS_Init6_DSTATE;               /* '<S1>/SS_Init6' */
    uint8_T RWS_Init5_DSTATE;              /* '<S1>/RWS_Init5' */
    uint8_T FWD_Init1_DSTATE_l;            /* '<S1>/FWD_Init1' */
    uint8_T Init2_DSTATE_o;                /* '<S22>/Init2' */
    uint8_T FWD_Init_DSTATE;               /* '<S22>/FWD_Init' */
    uint8_T Init2_DSTATE_j;                /* '<S23>/Init2' */
    uint8_T ALN_e_Direction_Prev_DSTATE_p; /* '<S23>/ALN_e_Direction_Prev' */
    uint8_T FWD_Init3_DSTATE;              /* '<S1>/FWD_Init3' */
    uint8_T Init2_DSTATE_i;                /* '<S24>/Init2' */
    uint8_T FWD_Init_DSTATE_m;             /* '<S24>/FWD_Init' */
    uint8_T Init2_DSTATE_b;                /* '<S25>/Init2' */
    uint8_T ALN_e_Direction_Prev_DSTATE_h; /* '<S25>/ALN_e_Direction_Prev' */
    uint8_T FWD_Init4_DSTATE;              /* '<S1>/FWD_Init4' */
    uint8_T Init2_DSTATE_l;                /* '<S26>/Init2' */
    uint8_T FWD_Init_DSTATE_p;             /* '<S26>/FWD_Init' */
} D_Work_ved__mot_st;

/* Real-time Model Data Structure */
struct RT_MODEL_ved__mot_st {
    const char_T *volatile errorStatus;
};

/* Model entry point functions */
extern void ved__mot_st_initialize(boolean_T firstTime,
                                   RT_MODEL_ved__mot_st *const ved__mot_st_M,
                                   BlockIO_ved__mot_st *ved__mot_st_B,
                                   D_Work_ved__mot_st *ved__mot_st_DWork);
extern void ved__mot_st_step(
    BlockIO_ved__mot_st *ved__mot_st_B,
    D_Work_ved__mot_st *ved__mot_st_DWork,
    VED_InputData_t *ved__mot_st_U_VED_InputData,
    VED_InternalData_t *ved__mot_st_U_VED_InternalData_in,
    VEDALN_Monitoring_t *ved__mot_st_U_VED_ALNData,
    ved__bayes_mot_states_t *ved__mot_st_Y_ved__mot_st_out);

/* Const memory section */
/* Declaration for custom storage class: Const */
extern const uint8 ved__ALN_to_perc_p[15];
extern const uint8 ved__brake_torque_to_perc_p[21];
extern const uint8 ved__cpt_pre_break_torque_p[4];
extern const uint8 ved__cpt_pre_gear_parking_p[16];
extern const uint8 ved__cpt_pulse_velo_dir_p[4];
extern const uint8 ved__cpt_pulse_velo_p[4];
extern const uint8 ved__cpt_whl_pulse_p[16];
extern const uint8 ved__gear_shift_to_perc_p[18];
extern const uint8 ved__parking_break_to_perc_p[9];
extern const uint8 ved__veh_velocity_to_perc_p[15];
extern const uint8 ved__whl_direction_to_perc_p[12];
extern const uint8 ved__whl_puls_to_perc_p[15];
extern const uint8 ved__yaw_rate_to_perc_p[6];

#endif /* RTW_HEADER_ved__mot_st_h_ */
