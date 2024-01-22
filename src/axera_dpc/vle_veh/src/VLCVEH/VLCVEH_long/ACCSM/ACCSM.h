/**********************************Model Property********************************
 *
 * Company             : SoftwareMotion
 *
 * Version             : Ver1.9
 *
 * Model               : ACCSM
 *
 ************************************Auto Coder**********************************
 *
 * File                             : ACCSM.h
 *
 * FileType                         : Code Header File
 *
 * Real-Time Workshop file version  : 9.4 (R2020b) 29-Jul-2020
 *
 * TLC version                      : 9.4 (Aug 20 2020)
 *
 * C source code generated on       : Thu Jun 16 14:55:17 2022
 *******************************************************************************/

#ifndef RTW_HEADER_ACCSM_h_
#define RTW_HEADER_ACCSM_h_
#ifndef ACCSM_COMMON_INCLUDES_
#define ACCSM_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* ACCSM_COMMON_INCLUDES_ */

#include "ACCSM_types.h"

/* Macros for accessing real-time model data structure */

/* Block signals (default storage) */
typedef struct {
  uint8_T CONTROL_STATE;               /* '<S1>/VLC_DETERMINE_CONTROL_STATE' */
  uint8_T OPERATIONAL_MODE;            /* '<S1>/VLC_DETERMINE_CONTROL_STATE' */
  uint8_T DAS_ENGAGED;                 /* '<S1>/VLC_DETERMINE_CONTROL_STATE' */
  uint8_T DAS_OFF;                     /* '<S1>/VLC_DETERMINE_CONTROL_STATE' */
  uint8_T DAS_OVERRIDE;                /* '<S1>/VLC_DETERMINE_CONTROL_STATE' */
  uint8_T DAS_SHUTOFF_REQ;             /* '<S1>/VLC_DETERMINE_CONTROL_STATE' */
  uint8_T CANCEL_RAMP;                 /* '<S1>/VLC_DETERMINE_CONTROL_STATE' */
} B_ACCSM_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T override_count;               /* '<S1>/VLC_DETERMINE_CONTROL_STATE' */
  uint8_T UnitDelay_DSTATE;            /* '<S1>/Unit Delay' */
  uint8_T is_active_c3_ACCSM;          /* '<S1>/VLC_DETERMINE_CONTROL_STATE' */
  uint8_T is_control_state;            /* '<S1>/VLC_DETERMINE_CONTROL_STATE' */
  uint8_T is_on;                       /* '<S1>/VLC_DETERMINE_CONTROL_STATE' */
  uint8_T is_override;                 /* '<S1>/VLC_DETERMINE_CONTROL_STATE' */
  uint8_T is_engage;                   /* '<S1>/VLC_DETERMINE_CONTROL_STATE' */
  uint8_T is_active;                   /* '<S1>/VLC_DETERMINE_CONTROL_STATE' */
} DW_ACCSM_T;

/* Block signals (default storage) */
extern B_ACCSM_T ACCSM_B;

/* Block states (default storage) */
extern DW_ACCSM_T ACCSM_DW;

/*
 * Exported Global Signals
 *
 * Note: Exported global signals are block signals with an exported global
 * storage class designation.  Code generation will declare the memory for
 * these signals and export their symbols.
 *
 */
extern ACCSM_Input ACCSM_In;           /* '<Root>/Inport' */
extern ACCSM_Output ACCSM_Out;         /* '<S1>/Bus Creator' */

/* Model entry point functions */
extern void ACCSM_initialize(void);
extern void ACCSM_step(void);

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('ACCSM_Model/ACCSM')    - opens subsystem ACCSM_Model/ACCSM
 * hilite_system('ACCSM_Model/ACCSM/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'ACCSM_Model'
 * '<S1>'   : 'ACCSM_Model/ACCSM'
 * '<S3>'   : 'ACCSM_Model/ACCSM/VLC_DETERMINE_CONTROL_STATE'
 */
#endif                                 /* RTW_HEADER_ACCSM_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
