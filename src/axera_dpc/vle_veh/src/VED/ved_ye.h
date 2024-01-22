

#ifndef RTW_HEADER_ved__ye_h_
#define RTW_HEADER_ved__ye_h_
#ifndef ved__ye_COMMON_INCLUDES_
#define ved__ye_COMMON_INCLUDES_
#include <math.h>
#include <stddef.h>
#include <string.h>
#include "ved_consts.h"
#include "ved.h"
#include "stddef.h"
//#include "VS_Dev_Conf.h"
//#include "SensorSimulationDataConfig_60ms.h"

#include "assert.h"

#endif /* ved__ye_COMMON_INCLUDES_ */

#include "ved_ye_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm) ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val) ((rtm)->errorStatus = (val))
#endif

/* QAC Fixes */

/* Block signals for system '<S2>/diag_variance' */
typedef struct {
    real32_T y[4]; /* '<S2>/diag_variance' */
} rtB_diag_variance_ved__ye;

/* Block signals for system '<S6>/At' */
typedef struct {
    real32_T y[4]; /* '<S6>/At' */
} rtB_At_ved__ye;

/* Block signals for system '<S6>/eye' */
typedef struct {
    real32_T y[4]; /* '<S6>/eye' */
} rtB_eye_ved__ye;

/* Block signals for system '<Root>/diag_curve_variance' */
typedef struct {
    real32_T y[2]; /* '<Root>/diag_curve_variance' */
} rtB_diag_curve_variance_ved__ye;

/* Block signals for system '<S34>/make_A_matrix' */
typedef struct {
    real32_T A[4]; /* '<S34>/make_A_matrix' */
} rtB_make_A_matrix_ved__ye;

/* Block signals (auto storage) */
typedef struct {
    real32_T Divide;                              /* '<S36>/Divide' */
    rtB_diag_variance_ved__ye sf_diag_R_i;        /* '<S14>/diag_R' */
    rtB_eye_ved__ye sf_eye;                       /* '<S13>/eye' */
    rtB_At_ved__ye sf_At;                         /* '<S13>/At' */
    rtB_make_A_matrix_ved__ye sf_make_A_matrix;   /* '<S49>/make_A_matrix' */
    rtB_make_A_matrix_ved__ye sf_make_A_matrix_l; /* '<S34>/make_A_matrix' */
    rtB_diag_curve_variance_ved__ye
        sf_diag_yaw_variance; /* '<Root>/diag_yaw_variance' */
    rtB_diag_curve_variance_ved__ye
        sf_diag_curve_variance; /* '<Root>/diag_curve_variance' */
    rtB_eye_ved__ye sf_eye_j;   /* '<S6>/eye' */
    rtB_At_ved__ye sf_Ht_k;     /* '<S6>/Ht' */
    rtB_At_ved__ye sf_At_i;     /* '<S6>/At' */
    rtB_diag_variance_ved__ye sf_diag_variance; /* '<S2>/diag_variance' */
} BlockIO_ved__ye;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
    real32_T UD_DSTATE;                  /* '<S15>/UD' */
    real32_T x_delay_DSTATE[2];          /* '<S13>/x_delay' */
    real32_T UnitDelay_DSTATE;           /* '<S52>/Unit Delay' */
    real32_T T3_DSTATE;                  /* '<S56>/T3' */
    real32_T T2_DSTATE;                  /* '<S56>/T2' */
    real32_T T1_DSTATE;                  /* '<S56>/T1' */
    real32_T T0_DSTATE;                  /* '<S56>/T0' */
    real32_T P_delay_DSTATE[4];          /* '<S13>/P_delay' */
    real32_T UD_DSTATE_m;                /* '<S16>/UD' */
    real32_T UD_DSTATE_h;                /* '<S18>/UD' */
    real32_T UD_DSTATE_j;                /* '<S17>/UD' */
    real32_T UD_DSTATE_hz;               /* '<S62>/UD' */
    real32_T x_delay_DSTATE_k[2];        /* '<S6>/x_delay' */
    real32_T delay_DSTATE;               /* '<S36>/delay' */
    real32_T ramp_factor_DSTATE;         /* '<S11>/ramp_factor' */
    real32_T last_yaw_rate_curve_DSTATE; /* '<S11>/last_yaw_rate_curve' */
    real32_T UD_DSTATE_i;                /* '<S41>/UD' */
    real32_T WeightedMovingAverage_TapDelayU[7]; /* '<S36>/Weighted Moving
                                                    Average' */
    real32_T UnitDelay_DSTATE_j;                 /* '<S44>/Unit Delay' */
    real32_T FixPtUnitDelay1_DSTATE;             /* '<S48>/FixPt Unit Delay1' */
    real32_T hold_value_DSTATE;                  /* '<S42>/hold_value' */
    real32_T P_delay_DSTATE_l[4];                /* '<S6>/P_delay' */
    real32_T PHt_HPHt_R_DWORK4[16];              /* '<S73>/PHt_(HPHt_R)' */
    real32_T PHt_HPHt_R_DWORK4_m[4];             /* '<S32>/PHt_(HPHt_R)' */
    uint8_T hold_counter_DSTATE;                 /* '<S11>/hold_counter' */
    uint8_T ramp_counter_DSTATE;                 /* '<S11>/ramp_counter' */
    uint8_T FixPtUnitDelay2_DSTATE;              /* '<S48>/FixPt Unit Delay2' */
    uint8_T hold_counter_DSTATE_e;               /* '<S42>/hold_counter' */
} D_Work_ved__ye;

/* Constant parameters (auto storage) */
typedef struct {
    /* Computed Parameter: Constant_Value_k
     * Referenced by: '<S33>/Constant'
     */
    real32_T Constant_Value_k[4];

    /* Computed Parameter: Constant_Value_k1
     * Referenced by: '<S74>/Constant'
     */
    real32_T Constant_Value_k1[8];

    /* Expression: mgainval
     * Referenced by: '<S36>/Weighted Moving Average'
     */
    real32_T WeightedMovingAverage[8];
} ConstParam_ved__ye;

/* Real-time Model Data Structure */
struct RT_MODEL_ved__ye {
    const char_T *volatile errorStatus;
};

/* Constant parameters (auto storage) */
extern const ConstParam_ved__ye ved__ye_ConstP;

/* Model entry point functions */
extern void ved__ye_initialize(boolean_T firstTime,
                               RT_MODEL_ved__ye *const ved__ye_M,
                               BlockIO_ved__ye *ved__ye_B,
                               D_Work_ved__ye *ved__ye_DWork);
extern void ved__ye_step(BlockIO_ved__ye *ved__ye_B,
                         D_Work_ved__ye *ved__ye_DWork,
                         VED_InputData_t *ved__ye_U_VED_InputData,
                         VED_InternalData_t *ved__ye_U_VED_InternalData_in,
                         VED_InternalData_t *ved__ye_Y_VED_InternalData_out,
                         real32_T ved__ye_Y_K_yaw[8],
                         uint8_T *ved__ye_Y_K_yaw_fault,
                         real32_T ved__ye_Y_K_curve[4],
                         uint8_T *ved__ye_Y_K_curve_fault);

/* Const memory section */
/* Declaration for custom storage class: Const */
extern const real32_T ved__yaw_rate_var_tune_p;
extern const real32_T ved__ye_P_curve_init_p[4];
extern const real32_T ved__ye_P_init_p[4];
extern const real32_T ved__ye_Q_add_p[4];
extern const real32_T ved__ye_Q_curve_gain_p;
extern const real32_T ved__ye_Q_gain_p[4];
extern const real32_T ved__ye_Q_sigmas_curve_p[2];
extern const real32_T ved__ye_Q_sigmas_p[2];
extern const real32_T ved__ye_Q_sigmas_velo_gain_p[4];
extern const real32_T ved__ye_R_curve_p[3];
extern const real32_T ved__ye_mahala_para_p[9];
extern const real32_T ved__ye_x_curve_init_p[2];
extern const real32_T ved__ye_x_init_p[2];
extern const uint8_T ved__ye_yaw_curve_ramp_para_p[2];

#endif /* RTW_HEADER_ved__ye_h_ */
