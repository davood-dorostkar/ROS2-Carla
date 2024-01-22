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
 * File                             : ACCSM_types.h
 *
 * FileType                         : Code Header File
 *
 * Real-Time Workshop file version  : 9.4 (R2020b) 29-Jul-2020
 *
 * TLC version                      : 9.4 (Aug 20 2020)
 *
 * C source code generated on       : Thu Jun 16 14:55:17 2022
 *******************************************************************************/

#ifndef RTW_HEADER_ACCSM_types_h_
#define RTW_HEADER_ACCSM_types_h_
#include "rtwtypes.h"

/* Model Code Variants */
#ifndef DEFINED_TYPEDEF_FOR_ACCSM_Input_
#define DEFINED_TYPEDEF_FOR_ACCSM_Input_

typedef struct {
  uint8_T SELECTED_FUNCTION;
  uint8_T CANCEL_FUNKTION;
  uint8_T VLC_INHIBIT;
  uint8_T DRIVER_OVERRIDE;
  uint8_T SWITCH_SPEED_UNIT;
  uint8_T PROPOSE_RECOMMENDED_SPEED;
  uint8_T RECOMMENDED_SPEED;
  uint8_T SETSPEED_STEP_LEVEL_1;
  uint8_T VLC_SETSPEED;
  uint8_T VLC_TAKE_ACTUAL_SPEED;
  uint8_T VLC_INCREASE_SET_SPEED;
  uint8_T VLC_DECREASE_SET_SPEED;
  uint8_T VLC_RESUME_SET_SPEED;
  uint8_T VLC_DECEL_ONLY;
  uint8_T VLC_END_DECEL_ONLY;
  uint8_T ACCEPT_VLC_ENGAGEMENT;
  uint8_T END_VLC_ENGAGEMENT;
  uint8_T VLC_DISENGAGEMENT;
  uint8_T VLC_DISENGAGEMENT_RAMP;
  uint8_T END_VLC_DISENGAGEMENT;
  uint8_T STANDSTILL;
  uint8_T OBJECT_EFFECTIVE;
} ACCSM_Input;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ACCSM_Output_
#define DEFINED_TYPEDEF_FOR_ACCSM_Output_

typedef struct {
  uint8_T CONTROL_STATE;
  uint8_T CONTROL_STATE_LAST_CYCLE;
  uint8_T OPERATIONAL_MODE;
  uint8_T DAS_ENGAGED;
  uint8_T DAS_OFF;
  uint8_T DAS_OVERRIDE;
  uint8_T DAS_SHUTOFF_REQ;
  uint8_T CANCEL_RAMP;
  uint8_T CONTROL_MODE;
} ACCSM_Output;

#endif
#endif                                 /* RTW_HEADER_ACCSM_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
