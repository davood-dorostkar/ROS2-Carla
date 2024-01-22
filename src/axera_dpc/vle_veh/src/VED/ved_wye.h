

#ifndef RTW_HEADER_ved__wye_h_
#define RTW_HEADER_ved__wye_h_
#ifndef ved__wye_COMMON_INCLUDES_
#define ved__wye_COMMON_INCLUDES_
#include <math.h>
#include <stddef.h>
#include <string.h>
#include "ved_consts.h"
#endif /* ved__wye_COMMON_INCLUDES_ */
#include "ved.h"
#include "ved_wye_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm) ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val) ((rtm)->errorStatus = (val))
#endif

/* QAC Fixes */

/* Block signals for system '<S23>/calculate determinant' */
typedef struct {
    real32_T y; /* '<S23>/calculate determinant' */
} rtB_calculatedeterminant_ved__wy;

/* Block signals for system '<S45>/Get_NVM_IO_State' */
typedef struct {
    uint32_T state; /* '<S45>/Get_NVM_IO_State' */
} rtB_Get_NVM_IO_State_ved__wye;

/* Block signals for system '<S31>/Time2Sec' */
typedef struct {
    real32_T Time2Sec; /* '<S53>/Time2Sec' */
} rtB_Time2Sec_ved__wye;

/* Block signals for system '<S60>/make_A_matrix' */
typedef struct {
    real32_T A[9]; /* '<S60>/make_A_matrix' */
} rtB_make_A_matrix_ved__wye;

/* Block signals for system '<S64>/get_bit' */
typedef struct {
    uint16_T y; /* '<S64>/get_bit' */
} rtB_get_bit_ved__wye;

/* Block signals for system '<S11>/Get_IO_State' */
typedef struct {
    uint8_T IndexVector; /* '<S97>/Index Vector' */
} rtB_Get_IO_State_ved__wye;

/* Block signals (auto storage) */
typedef struct {
    rtB_Get_IO_State_ved__wye Get_IO_State1;       /* '<S11>/Get_IO_State1' */
    rtB_Get_IO_State_ved__wye Get_IO_State;        /* '<S11>/Get_IO_State' */
    rtB_Time2Sec_ved__wye Time2Sec_d;              /* '<S10>/Time2Sec' */
    rtB_get_bit_ved__wye sf_get_bit_i;             /* '<S78>/get_bit' */
    rtB_Time2Sec_ved__wye Time2Sec_l;              /* '<S9>/Time2Sec' */
    rtB_make_A_matrix_ved__wye sf_make_A_matrix_d; /* '<S74>/make_A_matrix' */
    rtB_get_bit_ved__wye sf_get_bit;               /* '<S64>/get_bit' */
    rtB_Time2Sec_ved__wye Time2Sec;                /* '<S33>/Time2Sec' */
    rtB_make_A_matrix_ved__wye sf_make_A_matrix;   /* '<S60>/make_A_matrix' */
    rtB_Time2Sec_ved__wye Time2Sec_n;              /* '<S31>/Time2Sec' */
    rtB_Get_NVM_IO_State_ved__wye
        sf_Get_NVM_IO_State_k; /* '<S58>/Get_NVM_IO_State' */
    rtB_Get_NVM_IO_State_ved__wye
        sf_Get_NVM_IO_State; /* '<S45>/Get_NVM_IO_State' */
    rtB_calculatedeterminant_ved__wy
        sf_calculatedeterminant; /* '<S34>/calculate determinant' */
    rtB_calculatedeterminant_ved__wy
        sf_calculatedeterminant_b; /* '<S23>/calculate determinant' */
} BlockIO_ved__wye;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
    real32_T UD_DSTATE;                  /* '<S49>/UD' */
    real32_T UD_DSTATE_a;                /* '<S50>/UD' */
    real32_T last_est_wld_DSTATE;        /* '<S8>/last_est_wld' */
    real32_T x_delay_wld_DSTATE[3];      /* '<S29>/x_delay_wld' */
    real32_T UD_DSTATE_b;                /* '<S70>/UD' */
    real32_T P_delay_wld_DSTATE[9];      /* '<S29>/P_delay_wld' */
    real32_T P_delay_dyn_off_DSTATE[9];  /* '<S2>/P_delay_dyn_off' */
    real32_T x_delay_dyn_off_DSTATE[3];  /* '<S2>/x_delay_dyn_off' */
    real32_T last_dyn_yaw_offset_DSTATE; /* '<S1>/last_dyn_yaw_offset' */
    real32_T
        dyn_yaw_off_overt_count_DSTATE;  /* '<S1>/dyn_yaw_off_overt_count' */
    real32_T whl_yaw_off_control_DSTATE; /* '<S87>/whl_yaw_off_control' */
    real32_T
        diff_raw_whl_est_whl_yaw_DSTATE; /* '<S10>/diff_raw_whl_est_whl_yaw' */
    real32_T T3_DSTATE;                  /* '<S90>/T3' */
    real32_T T2_DSTATE;                  /* '<S90>/T2' */
    real32_T T1_DSTATE;                  /* '<S90>/T1' */
    real32_T T0_DSTATE;                  /* '<S90>/T0' */
    real32_T last_est_whl_yaw_rate_DSTATE; /* '<S10>/last_est_whl_yaw_rate' */
    real32_T P_delay_whl_yaw_DSTATE[4];    /* '<S3>/P_delay_whl_yaw' */
    real32_T x_delay_whl_yaw_DSTATE[2];    /* '<S3>/x_delay_whl_yaw' */
    real32_T FixPtUnitDelay1_DSTATE;       /* '<S55>/FixPt Unit Delay1' */
    real32_T FixPtUnitDelay1_DSTATE_d;     /* '<S96>/FixPt Unit Delay1' */
    real32_T PHt_HPHt_R_DWORK4[4];         /* '<S43>/PHt_(HPHt_R)' */
    real32_T PHt_HPHt_R_DWORK4_i[4];       /* '<S27>/PHt_(HPHt_R)' */
    real32_T PHt_HPHt_R_DWORK4_b[9];       /* '<S21>/PHt_(HPHt_R)' */
    int8_T
        last_dyn_yaw_offset_sign_DSTATE; /* '<S1>/last_dyn_yaw_offset_sign' */
    uint8_T init_nvm_wld_delay_DSTATE;   /* '<S29>/init_nvm_wld_delay' */
    uint8_T hold_value_DSTATE;           /* '<S71>/hold_value' */
    uint8_T hold_counter_DSTATE;         /* '<S71>/hold_counter' */
    uint8_T FixPtUnitDelay2_DSTATE;      /* '<S55>/FixPt Unit Delay2' */
    uint8_T FixPtUnitDelay2_DSTATE_h;    /* '<S96>/FixPt Unit Delay2' */
    boolean_T trav_dist_reset_DSTATE;    /* '<S31>/trav_dist_reset' */
} D_Work_ved__wye;

/* Invariant block signals (auto storage) */
typedef struct {
    const real32_T Ht[9];   /* '<S2>/Ht' */
    const real32_T Ht_a[4]; /* '<S3>/Ht' */
} ConstBlockIO_ved__wye;

/* Constant parameters (auto storage) */
typedef struct {
    /* Computed Parameter: Constant_Value
     * Referenced by: '<S22>/Constant'
     */
    real32_T Constant_Value[9];

    /* Computed Parameter: Constant_Value_d
     * Referenced by: '<S28>/Constant'
     */
    real32_T Constant_Value_d[4];

    /* Computed Parameter: Constant_Value_a
     * Referenced by: '<S44>/Constant'
     */
    real32_T Constant_Value_a[6];

    /* Pooled Parameter (Expression: )
     * Referenced by:
     *   '<S2>/eye'
     *   '<S29>/eye'
     */
    real32_T pooled6[9];

    /* Computed Parameter: H_const_Value
     * Referenced by: '<S2>/H_const'
     */
    real32_T H_const_Value[9];

    /* Computed Parameter: eye_Value
     * Referenced by: '<S3>/eye'
     */
    real32_T eye_Value[4];

    /* Computed Parameter: H_const_Value_b
     * Referenced by: '<S3>/H_const'
     */
    real32_T H_const_Value_b[4];
} ConstParam_ved__wye;

/* Real-time Model Data Structure */
struct RT_MODEL_ved__wye {
    const char_T *volatile errorStatus;
};

extern const ConstBlockIO_ved__wye ved__wye_ConstB; /* constant block i/o */

/* Constant parameters (auto storage) */
extern const ConstParam_ved__wye ved__wye_ConstP;

/* Model entry point functions */
extern void ved__wye_initialize(boolean_T firstTime,
                                RT_MODEL_ved__wye *const ved__wye_M,
                                BlockIO_ved__wye *ved__wye_B,
                                D_Work_ved__wye *ved__wye_DWork);
extern void ved__wye_step(BlockIO_ved__wye *ved__wye_B,
                          D_Work_ved__wye *ved__wye_DWork,
                          VED_InputData_t *ved__wye_U_VED_InputData,
                          VED_InternalData_t *ved__wye_U_VED_InternalData_in,
                          VED_NvData_t *ved__wye_U_VED_NvData_in,
                          VED_InternalData_t *ved__wye_Y_VED_InternalData_out,
                          VED_NvData_t *ved__wye_Y_VED_NVData_out);

/* Const memory section */
/* Declaration for custom storage class: Const */
extern const real32_T ved__wye_P_correct_p[4];
extern const real32_T ved__wye_P_init_p[9];
extern const real32_T ved__wye_P_init_wld_p[9];
extern const real32_T ved__wye_Q_gain_p[3];
extern const real32_T ved__wye_Q_gain_wld_p[3];
extern const real32_T ved__wye_Q_sigmas_dyn_off_p[5];
extern const real32_T ved__wye_Q_sigmas_dyn_off_wld_p;
extern const real32_T ved__wye_Q_sigmas_p[3];
extern const real32_T ved__wye_Q_sigmas_wld_p[3];
extern const real32_T ved__wye_R_control_p;
extern const real32_T ved__wye_R_p[9];
extern const real32_T ved__wye_R_wld_p[4];
extern const real32_T ved__wye_x_init_p[3];
extern const real32_T ved__wye_x_init_wld_p[3];
extern const real32_T ved__wye_yaw_P_correct_p[5];
extern const real32_T ved__wye_yaw_P_init_p[4];
extern const real32_T ved__wye_yaw_Q_gain_p[3];
extern const real32_T ved__wye_yaw_Q_sigmas_p[2];
extern const real32_T ved__wye_yaw_R_p[4];
extern const real32_T ved__wye_yaw_offset_const_p[2];
extern const real32_T ved__wye_yaw_x_init_p[2];

#endif /* RTW_HEADER_ved__wye_h_ */
