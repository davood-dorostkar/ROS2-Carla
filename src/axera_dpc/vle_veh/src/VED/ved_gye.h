

#ifndef RTW_HEADER_ved__gye_h_
#define RTW_HEADER_ved__gye_h_
#ifndef ved__gye_COMMON_INCLUDES_
#define ved__gye_COMMON_INCLUDES_
#include <math.h>
#include <stddef.h>
#include <string.h>
#include "ved_consts.h"
#endif /* ved__gye_COMMON_INCLUDES_ */

#include "ved_gye_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm) ((rtm)->errorStatus)
#endif
#include <ved.h>

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val) ((rtm)->errorStatus = (val))
#endif

/* QAC Fixes */

/* Block signals for system '<S10>/Get_IO_State1' */
typedef struct {
    uint8_T IndexVector; /* '<S31>/Index Vector' */
} rtB_Get_IO_State1_ved__gye;

/* Block signals (auto storage) */
typedef struct {
    real32_T Divide;                          /* '<S7>/Divide' */
    rtB_Get_IO_State1_ved__gye Get_IO_State2; /* '<S10>/Get_IO_State2' */
    rtB_Get_IO_State1_ved__gye Get_IO_State1; /* '<S10>/Get_IO_State1' */
} BlockIO_ved__gye;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
    real32_T delay_DSTATE; /* '<S7>/delay' */
    real32_T UD_DSTATE;    /* '<S22>/UD' */
    real32_T
        WeightedMovingAverage_TapDelayU[7]; /* '<S7>/Weighted Moving Average' */
    real32_T UnitDelay_DSTATE;              /* '<S26>/Unit Delay' */
    real32_T FixPtUnitDelay1_DSTATE;        /* '<S30>/FixPt Unit Delay1' */
    real32_T UD_DSTATE_h;                   /* '<S23>/UD' */
    real32_T hold_value_DSTATE;             /* '<S24>/hold_value' */
    real32_T P_delay_DSTATE[4];             /* '<S2>/P_delay' */
    real32_T x_delay_DSTATE[2];             /* '<S2>/x_delay' */
    real32_T PHt_HPHt_R_DWORK4;             /* '<S15>/PHt_(HPHt_R)' */
    uint8_T FixPtUnitDelay2_DSTATE;         /* '<S30>/FixPt Unit Delay2' */
    uint8_T hold_counter_DSTATE;            /* '<S24>/hold_counter' */
} D_Work_ved__gye;

/* Invariant block signals (auto storage) */
typedef struct {
    const real32_T Ht[2]; /* '<S2>/Ht' */
} ConstBlockIO_ved__gye;

/* Constant parameters (auto storage) */
typedef struct {
    /* Computed Parameter: Constant_Value
     * Referenced by: '<S16>/Constant'
     */
    real32_T Constant_Value[2];

    /* Computed Parameter: H_const1_Value
     * Referenced by: '<S2>/H_const1'
     */
    real32_T H_const1_Value[4];

    /* Computed Parameter: H_const_Value
     * Referenced by: '<S2>/H_const'
     */
    real32_T H_const_Value[2];

    /* Expression: mgainval
     * Referenced by: '<S7>/Weighted Moving Average'
     */
    real32_T WeightedMovingAverage[8];
} ConstParam_ved__gye;

/* Real-time Model Data Structure */
struct RT_MODEL_ved__gye {
    const char_T *volatile errorStatus;
};

extern const ConstBlockIO_ved__gye ved__gye_ConstB; /* constant block i/o */

/* Constant parameters (auto storage) */
extern const ConstParam_ved__gye ved__gye_ConstP;

/* Model entry point functions */
extern void ved__gye_initialize(boolean_T firstTime,
                                RT_MODEL_ved__gye *const ved__gye_M,
                                BlockIO_ved__gye *ved__gye_B,
                                D_Work_ved__gye *ved__gye_DWork);
extern void ved__gye_step(BlockIO_ved__gye *ved__gye_B,
                          D_Work_ved__gye *ved__gye_DWork,
                          VED_InputData_t *ved__gye_U_VED_InputData,
                          VED_InternalData_t *ved__gye_U_VED_InternalData_in,
                          VED_InternalData_t *ved__gye_Y_VED_InternalData_out);

/* Const memory section */
/* Declaration for custom storage class: Const */
extern const real32_T ved__gye_P_correct_p[2];
extern const real32_T ved__gye_P_init_p[4];
extern const real32_T ved__gye_Q_gain_p;
extern const real32_T ved__gye_R_p[6];
extern const real32_T ved__gye_x_init_p[2];

#endif /* RTW_HEADER_ved__gye_h_ */
