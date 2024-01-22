

#ifndef RTW_HEADER_ved__wpp_h_
#define RTW_HEADER_ved__wpp_h_
#ifndef ved__wpp_COMMON_INCLUDES_
#define ved__wpp_COMMON_INCLUDES_
#include <math.h>
#include <stddef.h>
#include <string.h>
#include "ved_consts.h"
#include "ved.h"
#include "stddef.h"
//#include "VS_Dev_Conf.h"
//#include "SensorSimulationDataConfig_60ms.h"

#include "assert.h"
#endif /* ved__wpp_COMMON_INCLUDES_ */

#include "ved_wpp_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm) ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val) ((rtm)->errorStatus = (val))
#endif

/* QAC Fixes */

/* Block signals for system '<Root>/Get_IO_State10' */
typedef struct {
    uint8_T IndexVector; /* '<S1>/Index Vector' */
} rtB_Get_IO_State10_ved__wpp;

/* Block signals for system '<Root>/front_left_wheel_speed_fusion' */
typedef struct {
    real32_T y;                               /* '<S36>/velocity' */
    real32_T corrected_wheel_velocity_uncert; /* '<S35>/correct_estimated_ws_uncertainty'
                                               */
} rtB_wheel_speed_fusion;

/* Block states (auto storage) for system '<Root>/front_left_wheel_speed_fusion'
 */
typedef struct {
    real32_T FixPtUnitDelay1_DSTATE[2];       /* '<S69>/FixPt Unit Delay1' */
    real32_T th_Delay_DSTATE;                 /* '<S77>/1th_Delay' */
    real32_T th_Delay_DSTATE_b;               /* '<S77>/2th_Delay' */
    real32_T th_Delay_DSTATE_f;               /* '<S77>/3th_Delay' */
    real32_T th_Delay_DSTATE_o;               /* '<S77>/4th_Delay' */
    real32_T th_Delay1_DSTATE;                /* '<S77>/1th_Delay1' */
    real32_T th_Delay1_DSTATE_f;              /* '<S77>/2th_Delay1' */
    real32_T th_Delay1_DSTATE_j;              /* '<S77>/3th_Delay1' */
    real32_T puls_speed_delay_DSTATE;         /* '<S37>/puls_speed_delay' */
    real32_T counter_delay_DSTATE;            /* '<S37>/counter_delay' */
    real32_T m_delay_DSTATE;                  /* '<S37>/m_delay' */
    real32_T single_puls_velocity_delay_DSTA; /* '<S37>/single_puls_velocity_delay'
                                               */
    real32_T T3_DSTATE;                       /* '<S74>/T3' */
    real32_T T2_DSTATE;                       /* '<S74>/T2' */
    real32_T T1_DSTATE;                       /* '<S74>/T1' */
    real32_T T0_DSTATE;                       /* '<S74>/T0' */
    real32_T UnitDelay_DSTATE;                /* '<S58>/Unit Delay' */
    real32_T P_delay_DSTATE[4];               /* '<S56>/P_delay' */
    real32_T FixPtUnitDelay1_DSTATE_e;        /* '<S41>/FixPt Unit Delay1' */
    real32_T last_accel_corr_DSTATE;          /* '<S39>/last_accel_corr' */
    real32_T last_accel_corr_step_DSTATE;     /* '<S39>/last_accel_corr_step' */
    real32_T last_accel_corr_max_DSTATE;      /* '<S39>/last_accel_corr_max' */
    real32_T PHt_HPHt_R_DWORK4[4];            /* '<S71>/PHt_(HPHt_R)' */
    uint8_T FixPtUnitDelay2_DSTATE;           /* '<S69>/FixPt Unit Delay2' */
    uint8_T FixPtUnitDelay2_DSTATE_o;         /* '<S41>/FixPt Unit Delay2' */
    boolean_T init_delay_DSTATE;              /* '<S37>/init_delay' */
} rtDW_wheel_speed_fusion;

/* Block signals (auto storage) */
typedef struct {
    rtB_wheel_speed_fusion rear_right_wheel_speed_fusion; /* '<Root>/rear_right_wheel_speed_fusion'
                                                           */
    rtB_wheel_speed_fusion rear_left_wheel_speed_fusion; /* '<Root>/rear_left_wheel_speed_fusion'
                                                          */
    rtB_wheel_speed_fusion front_right_wheel_speed_fusion; /* '<Root>/front_right_wheel_speed_fusion'
                                                            */
    rtB_wheel_speed_fusion front_left_wheel_speed_fusion; /* '<Root>/front_left_wheel_speed_fusion'
                                                           */
    rtB_Get_IO_State10_ved__wpp Get_IO_State9;  /* '<Root>/Get_IO_State9' */
    rtB_Get_IO_State10_ved__wpp Get_IO_State8;  /* '<Root>/Get_IO_State8' */
    rtB_Get_IO_State10_ved__wpp Get_IO_State7;  /* '<Root>/Get_IO_State7' */
    rtB_Get_IO_State10_ved__wpp Get_IO_State6;  /* '<Root>/Get_IO_State6' */
    rtB_Get_IO_State10_ved__wpp Get_IO_State5;  /* '<Root>/Get_IO_State5' */
    rtB_Get_IO_State10_ved__wpp Get_IO_State12; /* '<Root>/Get_IO_State12' */
    rtB_Get_IO_State10_ved__wpp Get_IO_State11; /* '<Root>/Get_IO_State11' */
    rtB_Get_IO_State10_ved__wpp Get_IO_State10; /* '<Root>/Get_IO_State10' */
} BlockIO_ved__wpp;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
    real32_T UD_DSTATE;                                    /* '<S29>/UD' */
    real32_T UD_DSTATE_o;                                  /* '<S30>/UD' */
    real32_T UD_DSTATE_b;                                  /* '<S31>/UD' */
    real32_T UD_DSTATE_f;                                  /* '<S32>/UD' */
    rtDW_wheel_speed_fusion rear_right_wheel_speed_fusion; /* '<Root>/rear_right_wheel_speed_fusion'
                                                            */
    rtDW_wheel_speed_fusion rear_left_wheel_speed_fusion; /* '<Root>/rear_left_wheel_speed_fusion'
                                                           */
    rtDW_wheel_speed_fusion front_right_wheel_speed_fusion; /* '<Root>/front_right_wheel_speed_fusion'
                                                             */
    rtDW_wheel_speed_fusion front_left_wheel_speed_fusion; /* '<Root>/front_left_wheel_speed_fusion'
                                                            */
} D_Work_ved__wpp;

/* Constant parameters (auto storage) */
typedef struct {
    /* Pooled Parameter (Expression: )
     * Referenced by:
     *   '<S72>/Constant'
     *   '<S119>/Constant'
     *   '<S166>/Constant'
     *   '<S213>/Constant'
     */
    real32_T pooled1[4];
} ConstParam_ved__wpp;

/* Real-time Model Data Structure */
struct RT_MODEL_ved__wpp {
    const char_T *volatile errorStatus;
};

/* Constant parameters (auto storage) */
extern const ConstParam_ved__wpp ved__wpp_ConstP;

/* Model entry point functions */
extern void ved__wpp_initialize(boolean_T firstTime,
                                RT_MODEL_ved__wpp *const ved__wpp_M,
                                BlockIO_ved__wpp *ved__wpp_B,
                                D_Work_ved__wpp *ved__wpp_DWork);
extern void ved__wpp_step(BlockIO_ved__wpp *ved__wpp_B,
                          D_Work_ved__wpp *ved__wpp_DWork,
                          VED_InputData_t *ved__wpp_U_VED_InputData,
                          VED_InternalData_t *ved__wpp_U_VED_InternalData,
                          VED_InternalData_t *ved__wpp_Y_VED_InternalData_out);

/* Const memory section */
/* Declaration for custom storage class: Const */
extern const real32_T ved__wpp_P_correct_p[6];
extern const real32_T ved__wpp_P_init_p[4];
extern const real32_T ved__wpp_Q_gain_p[8];
extern const real32_T ved__wpp_Q_sigmas_p[2];
extern const real32_T ved__wpp_R_p[4];
extern const real32_T ved__wpp_accel_correct_p;
extern const real32_T ved__wpp_aqua_slip_correct_p[2];
extern const real32_T ved__wpp_puls_velocity_para_p[4];
extern const real32_T ved__wpp_x_init_p;

#endif /* RTW_HEADER_ved__wpp_h_ */
