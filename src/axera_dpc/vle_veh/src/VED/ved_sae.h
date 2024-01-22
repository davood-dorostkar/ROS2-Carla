

#ifndef RTW_HEADER_ved__sae_h_
#define RTW_HEADER_ved__sae_h_
#ifndef ved__sae_COMMON_INCLUDES_
#define ved__sae_COMMON_INCLUDES_
#include <math.h>
#include <stddef.h>
#include <string.h>
#include "ved_consts.h"
#endif /* ved__sae_COMMON_INCLUDES_ */

#include "ved_sae_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm) ((rtm)->errorStatus)
#endif
#include <ved.h>

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val) ((rtm)->errorStatus = (val))
#endif

/* QAC Fixes */

/* Block states (auto storage) for system '<Root>' */
typedef struct {
    real32_T x_delay_DSTATE[2]; /* '<S2>/x_delay' */
    real32_T T3_DSTATE;         /* '<S26>/T3' */
    real32_T T2_DSTATE;         /* '<S26>/T2' */
    real32_T T1_DSTATE;         /* '<S26>/T1' */
    real32_T T0_DSTATE;         /* '<S26>/T0' */
    real32_T P_delay_DSTATE[4]; /* '<S2>/P_delay' */
    real32_T PHt_HPHt_R_DWORK4; /* '<S19>/PHt_(HPHt_R)' */
} D_Work_ved__sae;

/* Constant parameters (auto storage) */
typedef struct {
    /* Computed Parameter: Constant_Value
     * Referenced by: '<S20>/Constant'
     */
    real32_T Constant_Value[2];
} ConstParam_ved__sae;

/* Real-time Model Data Structure */
struct RT_MODEL_ved__sae {
    const char_T *volatile errorStatus;
};

/* Constant parameters (auto storage) */
extern const ConstParam_ved__sae ved__sae_ConstP;

/* Model entry point functions */
extern void ved__sae_initialize(boolean_T firstTime,
                                RT_MODEL_ved__sae *const ved__sae_M,
                                D_Work_ved__sae *ved__sae_DWork);
extern void ved__sae_step(D_Work_ved__sae *ved__sae_DWork,
                          VED_InputData_t *ved__sae_U_VED_InputData,
                          VED_InternalData_t *ved__sae_U_VED_InternalData_in,
                          VED_InternalData_t *ved__sae_Y_VED_InternalData_out);

/* Const memory section */
/* Declaration for custom storage class: Const */
extern const real32_T ved__sae_Cr_p;
extern const real32_T ved__sae_P_init_p[4];
extern const real32_T ved__sae_Q_add_p[4];
extern const real32_T ved__sae_Q_gain_p[3];
extern const real32_T ved__sae_Q_sigmas_p[2];
extern const real32_T ved__sae_R_p;
extern const real32_T ved__sae_x_init_p[2];

#endif /* RTW_HEADER_ved__sae_h_ */
