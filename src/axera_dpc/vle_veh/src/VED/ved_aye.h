

#ifndef RTW_HEADER_ved__aye_h_
#define RTW_HEADER_ved__aye_h_
#ifndef ved__aye_COMMON_INCLUDES_
#define ved__aye_COMMON_INCLUDES_
#include <math.h>
#include <stddef.h>
#include <string.h>
#include "ved_consts.h"
#include "ved.h"
#include "stddef.h"
#include "assert.h"

#endif /* ved__aye_COMMON_INCLUDES_ */

#include "ved_aye_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm) ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val) ((rtm)->errorStatus = (val))
#endif

/* QAC Fixes */

/* Block states (auto storage) for system '<Root>' */
typedef struct {
    real32_T x_delay_DSTATE[4];    /* '<S2>/x_delay' */
    real32_T UD_DSTATE;            /* '<S25>/UD' */
    real32_T delay_DSTATE;         /* '<S10>/delay' */
    real32_T P_delay_DSTATE[16];   /* '<S2>/P_delay' */
    real32_T PHt_HPHt_R_DWORK4[4]; /* '<S17>/PHt_(HPHt_R)' */
} D_Work_ved__aye;

/* Constant parameters (auto storage) */
typedef struct {
    /* Computed Parameter: Constant_Value
     * Referenced by: '<S18>/Constant'
     */
    real32_T Constant_Value[8];

    /* Expression: single(eye(4))
     * Referenced by: '<S2>/eye'
     */
    real32_T eye_Value[16];
} ConstParam_ved__aye;

/* Real-time Model Data Structure */
struct RT_MODEL_ved__aye {
    const char_T *volatile errorStatus;
};

/* Constant parameters (auto storage) */
extern const ConstParam_ved__aye ved__aye_ConstP;

/* Model entry point functions */
extern void ved__aye_initialize(boolean_T firstTime,
                                RT_MODEL_ved__aye *const ved__aye_M,
                                D_Work_ved__aye *ved__aye_DWork);
extern void ved__aye_step(D_Work_ved__aye *ved__aye_DWork,
                          VED_InputData_t *ved__aye_U_VED_InputData,
                          VED_InternalData_t *ved__aye_U_VED_InternalData_in,
                          VED_InternalData_t *ved__aye_Y_ved__aye_out);

/* Const memory section */
/* Declaration for custom storage class: Const */
extern const real32_T ved__aye_P_correct_p[4];
extern const real32_T ved__aye_P_init_p[16];
extern const real32_T ved__aye_Q_add_p[16];
extern const real32_T ved__aye_Q_gain_p[2];
extern const real32_T ved__aye_Q_sigmas_p[4];
extern const real32_T ved__aye_R_ay_invalid_p;
extern const real32_T ved__aye_R_p[4];
extern const real32_T ved__aye_x_init_p[4];

#endif /* RTW_HEADER_ved__aye_h_ */
