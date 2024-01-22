

#ifndef RTW_HEADER_ved__sye_h_
#define RTW_HEADER_ved__sye_h_
#ifndef ved__sye_COMMON_INCLUDES_
#define ved__sye_COMMON_INCLUDES_
#include <math.h>
#include <stddef.h>
#include <string.h>
#include "ved_consts.h"
#endif /* ved__sye_COMMON_INCLUDES_ */
#include "ved.h"
#include "ved_sye_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm) ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val) ((rtm)->errorStatus = (val))
#endif

/* QAC Fixes */

/* Block signals for system '<S13>/make_A_matrix' */
typedef struct {
    real32_T A[4]; /* '<S13>/make_A_matrix' */
} rtB_make_A_matrix_ved__sye;

/* Block signals for system '<S2>/Time2Sec' */
typedef struct {
    real32_T Time2Sec; /* '<S15>/Time2Sec' */
} rtB_Time2Sec_ved__sye;

/* Block signals for system '<S17>/get_gain_bias' */
typedef struct {
    real32_T default_diff; /* '<S17>/get_gain_bias' */
    real32_T gain;         /* '<S17>/get_gain_bias' */
    real32_T bias;         /* '<S17>/get_gain_bias' */
} rtB_get_gain_bias_ved__sye;

/* Block signals for system '<S30>/Get_IO_State1' */
typedef struct {
    uint8_T IndexVector; /* '<S35>/Index Vector' */
} rtB_Get_IO_State1_ved__sye;

/* Block signals for system '<S43>/get_init_control_mode' */
typedef struct {
    boolean_T y; /* '<S43>/get_init_control_mode' */
} rtB_get_init_control_mode_ved__s;

/* Block signals for system '<S73>/Get_NVM_IO_State' */
typedef struct {
    uint32_T state; /* '<S73>/Get_NVM_IO_State' */
} rtB_Get_NVM_IO_State_ved__sye;

/* Block signals for system '<S11>/At' */
typedef struct {
    real32_T y[4]; /* '<S11>/At' */
} rtB_At_ved__sye;

/* Block states (auto storage) for system '<S94>/calculate the gain' */
typedef struct {
    real32_T PHt_HPHt_R_DWORK4; /* '<S101>/PHt_(HPHt_R)' */
} rtDW_calculatethegain_ved__sye;

/* Block signals for system '<S94>/calculate determinant' */
typedef struct {
    real32_T y; /* '<S94>/calculate determinant' */
} rtB_calculatedeterminant_ved__sy;

/* Block signals for system '<S11>/Ht' */
typedef struct {
    real32_T y[2]; /* '<S11>/Ht' */
} rtB_Ht_ved__sye;

/* Block signals for system '<S11>/Reset_P_pred' */
typedef struct {
    real32_T P_pred_out[4]; /* '<S11>/Reset_P_pred' */
} rtB_Reset_P_pred_ved__sye;

/* Block signals for system '<S11>/eye' */
typedef struct {
    real32_T y[4]; /* '<S11>/eye' */
} rtB_eye_ved__sye;

/* Block signals (auto storage) */
typedef struct {
    rtB_eye_ved__sye sf_eye_d;                   /* '<S12>/eye' */
    rtB_Reset_P_pred_ved__sye sf_Reset_P_pred_d; /* '<S12>/Reset_P_pred' */
    rtB_Ht_ved__sye sf_Ht_d;                     /* '<S12>/Ht' */
    rtB_calculatedeterminant_ved__sy
        sf_calculatedeterminant_e; /* '<S104>/calculate determinant' */
    rtB_At_ved__sye sf_At_d;       /* '<S12>/At' */
    rtB_eye_ved__sye sf_eye_g;     /* '<S11>/eye' */
    rtB_Reset_P_pred_ved__sye sf_Reset_P_pred_m; /* '<S11>/Reset_P_pred' */
    rtB_Ht_ved__sye sf_Ht_k;                     /* '<S11>/Ht' */
    rtB_calculatedeterminant_ved__sy
        sf_calculatedeterminant_k; /* '<S94>/calculate determinant' */
    rtB_At_ved__sye sf_At_dl;      /* '<S11>/At' */
    rtB_Get_NVM_IO_State_ved__sye
        sf_Get_NVM_IO_State; /* '<S91>/Get_NVM_IO_State' */
    rtB_get_init_control_mode_ved__s
        sf_get_init_control_mode;     /* '<S9>/get_init_control_mode' */
    rtB_Time2Sec_ved__sye Time2Sec_i; /* '<S9>/Time2Sec' */
    rtB_Get_NVM_IO_State_ved__sye
        sf_Get_NVM_IO_State_g;                   /* '<S73>/Get_NVM_IO_State' */
    rtB_Get_IO_State1_ved__sye Get_IO_State2_k;  /* '<S45>/Get_IO_State2' */
    rtB_get_gain_bias_ved__sye sf_get_gain_bias; /* '<S54>/get_gain_bias' */
    rtB_Time2Sec_ved__sye Time2Sec_b;            /* '<S44>/Time2Sec' */
    rtB_make_A_matrix_ved__sye sf_make_A_matrix_i; /* '<S51>/make_A_matrix' */
    rtB_get_init_control_mode_ved__s
        sf_get_init_control_mode_g; /* '<S43>/get_init_control_mode' */
    rtB_Get_IO_State1_ved__sye Get_IO_State2;      /* '<S28>/Get_IO_State2' */
    rtB_Time2Sec_ved__sye Time2Sec;                /* '<S26>/Time2Sec' */
    rtB_Get_IO_State1_ved__sye Get_IO_State1;      /* '<S30>/Get_IO_State1' */
    rtB_get_gain_bias_ved__sye sf_get_gain_bias_k; /* '<S17>/get_gain_bias' */
    rtB_Time2Sec_ved__sye Time2Sec_f;              /* '<S2>/Time2Sec' */
    rtB_make_A_matrix_ved__sye sf_make_A_matrix_d; /* '<S13>/make_A_matrix' */
} BlockIO_ved__sye;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
    real32_T x_delay_eg_DSTATE[3];       /* '<S10>/x_delay_eg' */
    real32_T T3_DSTATE;                  /* '<S37>/T3' */
    real32_T T2_DSTATE;                  /* '<S37>/T2' */
    real32_T T1_DSTATE;                  /* '<S37>/T1' */
    real32_T T0_DSTATE;                  /* '<S37>/T0' */
    real32_T T7_DSTATE;                  /* '<S37>/T7' */
    real32_T T6_DSTATE;                  /* '<S37>/T6' */
    real32_T T5_DSTATE;                  /* '<S37>/T5' */
    real32_T T4_DSTATE;                  /* '<S37>/T4' */
    real32_T P_delay_eg_DSTATE[9];       /* '<S10>/P_delay_eg' */
    real32_T FixPtUnitDelay1_DSTATE;     /* '<S75>/FixPt Unit Delay1' */
    real32_T FixPtUnitDelay1_DSTATE_b;   /* '<S76>/FixPt Unit Delay1' */
    real32_T FixPtUnitDelay1_DSTATE_g;   /* '<S70>/FixPt Unit Delay1' */
    real32_T last_SlfStGradDisc_DSTATE;  /* '<S9>/last_SlfStGradDisc' */
    real32_T x_delay_yaw_DSTATE[2];      /* '<S12>/x_delay_yaw' */
    real32_T T3_DSTATE_c;                /* '<S58>/T3' */
    real32_T T2_DSTATE_f;                /* '<S58>/T2' */
    real32_T T1_DSTATE_k;                /* '<S58>/T1' */
    real32_T T0_DSTATE_f;                /* '<S58>/T0' */
    real32_T P_delay_yaw_DSTATE[4];      /* '<S12>/P_delay_yaw' */
    real32_T last_not_zero_min_DSTATE;   /* '<S43>/last_not_zero_min' */
    real32_T last_steering_angle_DSTATE; /* '<S43>/last_steering_angle' */
    real32_T FixPtUnitDelay1_DSTATE_l;   /* '<S50>/FixPt Unit Delay1' */
    real32_T x_delay_curve_DSTATE[2];    /* '<S11>/x_delay_curve' */
    real32_T T3_DSTATE_f;                /* '<S23>/T3' */
    real32_T T2_DSTATE_e;                /* '<S23>/T2' */
    real32_T T1_DSTATE_h;                /* '<S23>/T1' */
    real32_T T0_DSTATE_l;                /* '<S23>/T0' */
    real32_T P_delay_curve_DSTATE[4];    /* '<S11>/P_delay_curve' */
    real32_T PHt_HPHt_R_DWORK4[4];       /* '<S89>/PHt_(HPHt_R)' */
    uint8_T init_nvm_eg_delay_DSTATE;    /* '<S10>/init_nvm_eg_delay' */
    uint8_T FixPtUnitDelay2_DSTATE;      /* '<S70>/FixPt Unit Delay2' */
    boolean_T trav_dist_reset_DSTATE;    /* '<S9>/trav_dist_reset' */
    rtDW_calculatethegain_ved__sye
        calculatethegain_b; /* '<S104>/calculate the gain' */
    rtDW_calculatethegain_ved__sye
        calculatethegain_bg; /* '<S94>/calculate the gain' */
} D_Work_ved__sye;

/* Constant parameters (auto storage) */
typedef struct {
    /* Computed Parameter: Constant_Value
     * Referenced by: '<S90>/Constant'
     */
    real32_T Constant_Value[6];

    /* Pooled Parameter (Expression: )
     * Referenced by:
     *   '<S102>/Constant'
     *   '<S113>/Constant'
     */
    real32_T pooled2[2];
} ConstParam_ved__sye;

/* Real-time Model Data Structure */
struct RT_MODEL_ved__sye {
    const char_T *volatile errorStatus;
};

/* Constant parameters (auto storage) */
extern const ConstParam_ved__sye ved__sye_ConstP;

/* Model entry point functions */
extern void ved__sye_initialize(boolean_T firstTime,
                                RT_MODEL_ved__sye *const ved__sye_M,
                                BlockIO_ved__sye *ved__sye_B,
                                D_Work_ved__sye *ved__sye_DWork);
extern void ved__sye_step(BlockIO_ved__sye *ved__sye_B,
                          D_Work_ved__sye *ved__sye_DWork,
                          VED_InputData_t *ved__sye_U_VED_InputData,
                          VED_InternalData_t *ved__sye_U_VED_InternalData_in,
                          VED_NvData_t *ved__sye_U_VED_NVData_in,
                          VED_InternalData_t *ved__sye_Y_VED_InternalData_out,
                          VED_NvData_t *ved__sye_Y_VED_NVData_out);

/* Const memory section */
/* Declaration for custom storage class: Const */
extern const real32_T ved__sye_P_SSG_init_p[9];
extern const real32_T ved__sye_P_correct_p[4];
extern const real32_T ved__sye_P_di_init_p[4];
extern const real32_T ved__sye_P_init_p[4];
extern const real32_T ved__sye_Q_SSG_range_p[5];
extern const real32_T ved__sye_Q_SSG_sigmas_p[2];
extern const real32_T ved__sye_Q_add_p[4];
extern const real32_T ved__sye_Q_di_add_p[4];
extern const real32_T ved__sye_Q_di_gain_p[3];
extern const real32_T ved__sye_Q_di_sigmas_p[2];
extern const real32_T ved__sye_Q_di_sigmas_velo_gain_p[4];
extern const real32_T ved__sye_Q_gain_p[3];
extern const real32_T ved__sye_Q_sigmas_p[2];
extern const real32_T ved__sye_R_SSG_p[4];
extern const real32_T ved__sye_R_p[2];
extern const real32_T ved__sye_x_SSG_init_p[3];
extern const real32_T ved__sye_x_di_init_p[2];
extern const real32_T ved__sye_x_init_p[2];

#endif /* RTW_HEADER_ved__sye_h_ */
