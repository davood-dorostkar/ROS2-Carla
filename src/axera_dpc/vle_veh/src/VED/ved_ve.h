

#ifndef RTW_HEADER_ved__ve_h_
#define RTW_HEADER_ved__ve_h_
#ifndef ved__ve_COMMON_INCLUDES_
#define ved__ve_COMMON_INCLUDES_
#include <math.h>
#include <stddef.h>
#include <string.h>
#include "ved_consts.h"
#endif /* ved__ve_COMMON_INCLUDES_ */
#include "ved.h"
#include "ved_ve_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm) ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val) ((rtm)->errorStatus = (val))
#endif

/* QAC Fixes */

/* Block signals for system '<S3>/Get_IO_State8' */
typedef struct {
    uint8_T IndexVector; /* '<S16>/Index Vector' */
} rtB_Get_IO_State8_ved__ve;

/* Block signals for system '<S36>/check_threshold' */
typedef struct {
    uint8_T ext_reset; /* '<S36>/check_threshold' */
} rtB_check_threshold_ved__ve;

/* Block signals for system '<S42>/check_reset_condition' */
typedef struct {
    real32_T init_value; /* '<S42>/check_reset_condition' */
    uint8_T Reset;       /* '<S42>/check_reset_condition' */
} rtB_check_reset_condition_ved__v;

/* Block signals (auto storage) */
typedef struct {
    rtB_check_reset_condition_ved__v
        sf_check_reset_condition_g; /* '<S57>/check_reset_condition' */
    rtB_check_threshold_ved__ve
        sf_check_threshold_be; /* '<S39>/check_threshold' */
    rtB_check_reset_condition_ved__v
        sf_check_reset_condition_f; /* '<S52>/check_reset_condition' */
    rtB_check_threshold_ved__ve
        sf_check_threshold_b; /* '<S38>/check_threshold' */
    rtB_check_reset_condition_ved__v
        sf_check_reset_condition_m; /* '<S47>/check_reset_condition' */
    rtB_check_threshold_ved__ve
        sf_check_threshold_m; /* '<S37>/check_threshold' */
    rtB_check_reset_condition_ved__v
        sf_check_reset_condition; /* '<S42>/check_reset_condition' */
    rtB_check_threshold_ved__ve
        sf_check_threshold;                    /* '<S36>/check_threshold' */
    rtB_Get_IO_State8_ved__ve Get_IO_State8_i; /* '<S8>/Get_IO_State8' */
    rtB_Get_IO_State8_ved__ve Get_IO_State3;   /* '<S8>/Get_IO_State3' */
    rtB_Get_IO_State8_ved__ve Get_IO_State2;   /* '<S8>/Get_IO_State2' */
    rtB_Get_IO_State8_ved__ve Get_IO_State1;   /* '<S8>/Get_IO_State1' */
    rtB_Get_IO_State8_ved__ve Get_IO_State8;   /* '<S3>/Get_IO_State8' */
} BlockIO_ved__ve;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
    real32_T FixPtUnitDelay1_DSTATE[2]; /* '<S32>/FixPt Unit Delay1' */
    real32_T UD_DSTATE;                 /* '<S20>/UD' */
    real32_T UnitDelay_DSTATE;          /* '<S7>/Unit Delay' */
    real32_T P_delay_DSTATE[4];         /* '<S9>/P_delay' */
    real32_T UnitDelay_DSTATE_l;        /* '<S42>/Unit Delay' */
    real32_T FixPtUnitDelay1_DSTATE_o;  /* '<S45>/FixPt Unit Delay1' */
    real32_T UnitDelay_DSTATE_j;        /* '<S47>/Unit Delay' */
    real32_T FixPtUnitDelay1_DSTATE_n;  /* '<S50>/FixPt Unit Delay1' */
    real32_T UnitDelay_DSTATE_p;        /* '<S52>/Unit Delay' */
    real32_T FixPtUnitDelay1_DSTATE_f;  /* '<S55>/FixPt Unit Delay1' */
    real32_T UnitDelay_DSTATE_o;        /* '<S57>/Unit Delay' */
    real32_T FixPtUnitDelay1_DSTATE_ox; /* '<S60>/FixPt Unit Delay1' */
    real32_T last_v_var_DSTATE;         /* '<S12>/last_v_var' */
    real32_T last_a_var_DSTATE;         /* '<S5>/last_a_var' */
    real32_T PHt_HPHt_R_DWORK4[16];     /* '<S34>/PHt_(HPHt_R)' */
    uint8_T FixPtUnitDelay2_DSTATE;     /* '<S32>/FixPt Unit Delay2' */
    uint8_T UnitDelay_DSTATE_pd;        /* '<S36>/Unit Delay' */
    uint8_T FixPtUnitDelay2_DSTATE_l;   /* '<S45>/FixPt Unit Delay2' */
    uint8_T UnitDelay_DSTATE_n;         /* '<S37>/Unit Delay' */
    uint8_T FixPtUnitDelay2_DSTATE_n;   /* '<S50>/FixPt Unit Delay2' */
    uint8_T UnitDelay_DSTATE_m;         /* '<S38>/Unit Delay' */
    uint8_T FixPtUnitDelay2_DSTATE_h;   /* '<S55>/FixPt Unit Delay2' */
    uint8_T UnitDelay_DSTATE_k;         /* '<S39>/Unit Delay' */
    uint8_T FixPtUnitDelay2_DSTATE_ln;  /* '<S60>/FixPt Unit Delay2' */
} D_Work_ved__ve;

/* Constant parameters (auto storage) */
typedef struct {
    /* Computed Parameter: Constant_Value_a
     * Referenced by: '<S35>/Constant'
     */
    real32_T Constant_Value_a[8];
} ConstParam_ved__ve;

/* Real-time Model Data Structure */
struct RT_MODEL_ved__ve {
    const char_T *volatile errorStatus;
};

/* Constant parameters (auto storage) */
extern const ConstParam_ved__ve ved__ve_ConstP;

/* Model entry point functions */
extern void ved__ve_initialize(boolean_T firstTime,
                               RT_MODEL_ved__ve *const ved__ve_M,
                               BlockIO_ved__ve *ved__ve_B,
                               D_Work_ved__ve *ved__ve_DWork);
extern void ved__ve_step(BlockIO_ved__ve *ved__ve_B,
                         D_Work_ved__ve *ved__ve_DWork,
                         VED_InputData_t *ved__ve_U_VED_InputData,
                         VED_InternalData_t *ved__ve_U_VED_InternalData,
                         VED_InternalData_t *ved__ve_Y_VED_InternalData_out);

/* Const memory section */
/* Declaration for custom storage class: Const */
extern const real32_T ved__ve_P_init_p[4];
extern const real32_T ved__ve_Q_gain_p[3];
extern const real32_T ved__ve_Q_sigmas_p[2];
extern const real32_T ved__ve_a_v_zero_p[9];
extern const real32_T ved__ve_x_init_p;

#endif /* RTW_HEADER_ved__ve_h_ */
