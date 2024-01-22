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
 * File                             : ACCSM.c
 *
 * FileType                         : Code Source File
 *
 * Real-Time Workshop file version  : 9.4 (R2020b) 29-Jul-2020
 *
 * TLC version                      : 9.4 (Aug 20 2020)
 *
 * C source code generated on       : Thu Jun 16 14:55:17 2022
 *******************************************************************************/
// #define DLMU1_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

#include "ACCSM.h"
#include "ACCSM_private.h"

/* Named constants for Chart: '<S1>/VLC_DETERMINE_CONTROL_STATE' */
#define ACCSM_Cc_cc_active             ((uint8_T)3U)
#define ACCSM_Cc_cc_disengage          ((uint8_T)5U)
#define ACCSM_Cc_cc_engage             ((uint8_T)2U)
#define ACCSM_Cc_cc_off                ((uint8_T)0U)
#define ACCSM_Cc_cc_override           ((uint8_T)4U)
#define ACCSM_Cc_cc_ready              ((uint8_T)1U)
#define ACCSM_Cc_follow_mode           ((uint8_T)1U)
#define ACCSM_Cc_free_mode             ((uint8_T)2U)
#define ACCSM_Cc_standstill_mode       ((uint8_T)0U)
#define ACCSM_Display_op_cc_active     ((uint8_T)3U)
#define ACCSM_Display_op_cc_disengage  ((uint8_T)5U)
#define ACCSM_Display_op_cc_invalid    ((uint8_T)4U)
#define ACCSM_Display_op_cc_none       ((uint8_T)1U)
#define ACCSM_Display_op_cc_override   ((uint8_T)6U)
#define ACCSM_Display_op_cc_recom_speed ((uint8_T)7U)
#define ACCSM_Display_op_cc_valid      ((uint8_T)2U)
#define ACCSM_Display_op_none          ((uint8_T)0U)
#define ACCSM_IN_Display_active        ((uint8_T)1U)
#define ACCSM_IN_Display_none          ((uint8_T)1U)
#define ACCSM_IN_Display_override      ((uint8_T)1U)
#define ACCSM_IN_Display_recom_speed   ((uint8_T)2U)
#define ACCSM_IN_Display_valid         ((uint8_T)3U)
#define ACCSM_IN_Display_valid_o       ((uint8_T)2U)
#define ACCSM_IN_NO_ACTIVE_CHILD       ((uint8_T)0U)
#define ACCSM_IN_active                ((uint8_T)1U)
#define ACCSM_IN_disengage             ((uint8_T)2U)
#define ACCSM_IN_engage                ((uint8_T)3U)
#define ACCSM_IN_off                   ((uint8_T)1U)
#define ACCSM_IN_on                    ((uint8_T)2U)
#define ACCSM_IN_override              ((uint8_T)4U)
#define ACCSM_IN_ready                 ((uint8_T)5U)

#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/* Exported block signals */
ACCSM_Input ACCSM_In;                  /* '<Root>/Inport' */
ACCSM_Output ACCSM_Out;                /* '<S1>/Bus Creator' */

/* Block signals (default storage) */
B_ACCSM_T ACCSM_B;

/* Block states (default storage) */
DW_ACCSM_T ACCSM_DW;
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"

/* Forward declaration for local functions */
static void ACCSM_on(void);

/* Function for Chart: '<S1>/VLC_DETERMINE_CONTROL_STATE' */
static void ACCSM_on(void)
{
  boolean_T guard1 = false;

  /* Inport: '<Root>/Inport' */
  guard1 = false;
  if (ACCSM_In.VLC_INHIBIT != 0) {
    guard1 = true;
  } else if ((ACCSM_In.SELECTED_FUNCTION != 1) || (ACCSM_In.CANCEL_FUNKTION != 0))
  {
    ACCSM_B.CANCEL_RAMP = 1U;
    guard1 = true;
  } else {
    switch (ACCSM_DW.is_on) {
     case ACCSM_IN_active:
      ACCSM_B.CONTROL_STATE = ACCSM_Cc_cc_active;
      ACCSM_B.DAS_ENGAGED = 1U;
      ACCSM_B.DAS_OVERRIDE = 0U;
      ACCSM_B.DAS_SHUTOFF_REQ = 0U;
      ACCSM_B.DAS_OFF = 0U;
      if ((ACCSM_In.VLC_DISENGAGEMENT_RAMP != 0) || (ACCSM_In.VLC_DISENGAGEMENT
           != 0) || (ACCSM_In.VLC_DECEL_ONLY != 0)) {
        ACCSM_DW.is_active = ACCSM_IN_NO_ACTIVE_CHILD;
        ACCSM_DW.is_on = ACCSM_IN_disengage;
        ACCSM_B.CONTROL_STATE = ACCSM_Cc_cc_disengage;
        ACCSM_B.OPERATIONAL_MODE = ACCSM_Display_op_cc_disengage;
        ACCSM_B.DAS_ENGAGED = 0U;
        ACCSM_B.DAS_OVERRIDE = 0U;
        ACCSM_B.DAS_SHUTOFF_REQ = 1U;
        ACCSM_B.DAS_OFF = 0U;
      } else if (ACCSM_In.DRIVER_OVERRIDE != 0) {
        ACCSM_DW.is_active = ACCSM_IN_NO_ACTIVE_CHILD;
        ACCSM_DW.is_on = ACCSM_IN_override;
        ACCSM_B.CONTROL_STATE = ACCSM_Cc_cc_override;
        ACCSM_B.DAS_ENGAGED = 1U;
        ACCSM_B.DAS_OVERRIDE = 1U;
        ACCSM_B.DAS_SHUTOFF_REQ = 0U;
        ACCSM_B.DAS_OFF = 0U;
        ACCSM_DW.override_count = 0.0;
        ACCSM_DW.is_override = ACCSM_IN_Display_override;
        ACCSM_B.OPERATIONAL_MODE = ACCSM_Display_op_cc_override;
      } else {
        switch (ACCSM_DW.is_active) {
         case ACCSM_IN_Display_active:
          if ((ACCSM_In.VLC_INCREASE_SET_SPEED != 0) ||
              (ACCSM_In.VLC_DECREASE_SET_SPEED != 0) ||
              (ACCSM_In.VLC_TAKE_ACTUAL_SPEED != 0) ||
              (ACCSM_In.VLC_RESUME_SET_SPEED != 0) ||
              (ACCSM_In.SWITCH_SPEED_UNIT != 0)) {
            ACCSM_DW.is_active = ACCSM_IN_Display_valid;
            ACCSM_B.OPERATIONAL_MODE = ACCSM_Display_op_cc_valid;
          } else if (ACCSM_In.PROPOSE_RECOMMENDED_SPEED != 0) {
            ACCSM_DW.is_active = ACCSM_IN_Display_recom_speed;
            ACCSM_B.OPERATIONAL_MODE = ACCSM_Display_op_cc_recom_speed;
          } else {
            ACCSM_B.OPERATIONAL_MODE = ACCSM_Display_op_cc_active;
          }
          break;

         case ACCSM_IN_Display_recom_speed:
          if (ACCSM_In.PROPOSE_RECOMMENDED_SPEED == 0) {
            ACCSM_DW.is_active = ACCSM_IN_Display_active;
            ACCSM_B.OPERATIONAL_MODE = ACCSM_Display_op_cc_active;
          } else {
            ACCSM_B.OPERATIONAL_MODE = ACCSM_Display_op_cc_recom_speed;
          }
          break;

         default:
          /* case IN_Display_valid: */
          if ((ACCSM_In.VLC_INCREASE_SET_SPEED == 0) &&
              (ACCSM_In.VLC_DECREASE_SET_SPEED == 0) &&
              (ACCSM_In.VLC_TAKE_ACTUAL_SPEED == 0) &&
              (ACCSM_In.VLC_RESUME_SET_SPEED == 0) &&
              (ACCSM_In.SWITCH_SPEED_UNIT == 0)) {
            ACCSM_DW.is_active = ACCSM_IN_Display_active;
            ACCSM_B.OPERATIONAL_MODE = ACCSM_Display_op_cc_active;
          } else {
            ACCSM_B.OPERATIONAL_MODE = ACCSM_Display_op_cc_valid;
          }
          break;
        }
      }
      break;

     case ACCSM_IN_disengage:
      ACCSM_B.CONTROL_STATE = ACCSM_Cc_cc_disengage;
      ACCSM_B.DAS_ENGAGED = 0U;
      ACCSM_B.DAS_OVERRIDE = 0U;
      ACCSM_B.DAS_SHUTOFF_REQ = 1U;
      ACCSM_B.DAS_OFF = 0U;
      if ((ACCSM_In.END_VLC_DISENGAGEMENT != 0) || (ACCSM_In.VLC_END_DECEL_ONLY
           != 0)) {
        ACCSM_DW.is_on = ACCSM_IN_ready;
        ACCSM_B.CONTROL_STATE = ACCSM_Cc_cc_ready;
        ACCSM_B.DAS_ENGAGED = 0U;
        ACCSM_B.DAS_OVERRIDE = 0U;
        ACCSM_B.DAS_SHUTOFF_REQ = 0U;
        ACCSM_B.DAS_OFF = 0U;
        ACCSM_B.OPERATIONAL_MODE = ACCSM_Display_op_cc_invalid;
      }
      break;

     case ACCSM_IN_engage:
      ACCSM_B.CONTROL_STATE = ACCSM_Cc_cc_engage;
      ACCSM_B.DAS_ENGAGED = 1U;
      ACCSM_B.DAS_OVERRIDE = 0U;
      ACCSM_B.DAS_SHUTOFF_REQ = 0U;
      ACCSM_B.DAS_OFF = 0U;
      if (ACCSM_In.END_VLC_ENGAGEMENT != 0) {
        ACCSM_DW.is_engage = ACCSM_IN_NO_ACTIVE_CHILD;
        ACCSM_DW.is_on = ACCSM_IN_active;
        ACCSM_B.CONTROL_STATE = ACCSM_Cc_cc_active;
        ACCSM_B.DAS_ENGAGED = 1U;
        ACCSM_B.DAS_OVERRIDE = 0U;
        ACCSM_B.DAS_SHUTOFF_REQ = 0U;
        ACCSM_B.DAS_OFF = 0U;
        ACCSM_DW.is_active = ACCSM_IN_Display_active;
        ACCSM_B.OPERATIONAL_MODE = ACCSM_Display_op_cc_active;
      } else if (ACCSM_DW.is_engage == ACCSM_IN_Display_none) {
        if (ACCSM_In.SWITCH_SPEED_UNIT != 0) {
          ACCSM_DW.is_engage = ACCSM_IN_Display_valid_o;
          ACCSM_B.OPERATIONAL_MODE = ACCSM_Display_op_cc_valid;
        } else {
          ACCSM_B.OPERATIONAL_MODE = ACCSM_Display_op_cc_none;
        }
      } else {
        /* case IN_Display_valid: */
        if (ACCSM_In.SWITCH_SPEED_UNIT == 0) {
          ACCSM_DW.is_engage = ACCSM_IN_Display_none;
          ACCSM_B.OPERATIONAL_MODE = ACCSM_Display_op_cc_none;
        } else {
          ACCSM_B.OPERATIONAL_MODE = ACCSM_Display_op_cc_valid;
        }
      }
      break;

     case ACCSM_IN_override:
      ACCSM_B.CONTROL_STATE = ACCSM_Cc_cc_override;
      ACCSM_B.DAS_ENGAGED = 1U;
      ACCSM_B.DAS_OVERRIDE = 1U;
      ACCSM_B.DAS_SHUTOFF_REQ = 0U;
      ACCSM_B.DAS_OFF = 0U;
      if ((ACCSM_In.VLC_DISENGAGEMENT_RAMP != 0) || (ACCSM_In.VLC_DISENGAGEMENT
           != 0) || (ACCSM_In.VLC_DECEL_ONLY != 0) || (ACCSM_DW.override_count >=
           45000.0)) {
        ACCSM_DW.is_override = ACCSM_IN_NO_ACTIVE_CHILD;
        ACCSM_DW.is_on = ACCSM_IN_disengage;
        ACCSM_B.CONTROL_STATE = ACCSM_Cc_cc_disengage;
        ACCSM_B.OPERATIONAL_MODE = ACCSM_Display_op_cc_disengage;
        ACCSM_B.DAS_ENGAGED = 0U;
        ACCSM_B.DAS_OVERRIDE = 0U;
        ACCSM_B.DAS_SHUTOFF_REQ = 1U;
        ACCSM_B.DAS_OFF = 0U;
      } else if (ACCSM_In.DRIVER_OVERRIDE == 0) {
        ACCSM_DW.is_override = ACCSM_IN_NO_ACTIVE_CHILD;
        ACCSM_DW.is_on = ACCSM_IN_active;
        ACCSM_B.CONTROL_STATE = ACCSM_Cc_cc_active;
        ACCSM_B.DAS_ENGAGED = 1U;
        ACCSM_B.DAS_OVERRIDE = 0U;
        ACCSM_B.DAS_SHUTOFF_REQ = 0U;
        ACCSM_B.DAS_OFF = 0U;
        ACCSM_DW.is_active = ACCSM_IN_Display_active;
        ACCSM_B.OPERATIONAL_MODE = ACCSM_Display_op_cc_active;
      } else {
        ACCSM_DW.override_count++;
        switch (ACCSM_DW.is_override) {
         case ACCSM_IN_Display_override:
          if ((ACCSM_In.VLC_INCREASE_SET_SPEED != 0) ||
              (ACCSM_In.VLC_DECREASE_SET_SPEED != 0) ||
              (ACCSM_In.VLC_TAKE_ACTUAL_SPEED != 0) ||
              (ACCSM_In.SWITCH_SPEED_UNIT != 0)) {
            ACCSM_DW.is_override = ACCSM_IN_Display_valid;
            ACCSM_B.OPERATIONAL_MODE = ACCSM_Display_op_cc_valid;
          } else if (((ACCSM_In.RECOMMENDED_SPEED < ACCSM_In.VLC_SETSPEED -
                       ACCSM_In.SETSPEED_STEP_LEVEL_1) &&
                      (ACCSM_In.PROPOSE_RECOMMENDED_SPEED == 1)) ||
                     (ACCSM_In.PROPOSE_RECOMMENDED_SPEED == 3)) {
            ACCSM_DW.is_override = ACCSM_IN_Display_recom_speed;
            ACCSM_B.OPERATIONAL_MODE = ACCSM_Display_op_cc_recom_speed;
          } else {
            ACCSM_B.OPERATIONAL_MODE = ACCSM_Display_op_cc_override;
          }
          break;

         case ACCSM_IN_Display_recom_speed:
          if (((ACCSM_In.RECOMMENDED_SPEED >= ACCSM_In.VLC_SETSPEED -
                ACCSM_In.SETSPEED_STEP_LEVEL_1) ||
               (ACCSM_In.PROPOSE_RECOMMENDED_SPEED != 1)) &&
              (ACCSM_In.PROPOSE_RECOMMENDED_SPEED != 3)) {
            ACCSM_DW.is_override = ACCSM_IN_Display_override;
            ACCSM_B.OPERATIONAL_MODE = ACCSM_Display_op_cc_override;
          } else {
            ACCSM_B.OPERATIONAL_MODE = ACCSM_Display_op_cc_recom_speed;
          }
          break;

         default:
          /* case IN_Display_valid: */
          if ((ACCSM_In.VLC_INCREASE_SET_SPEED == 0) &&
              (ACCSM_In.VLC_DECREASE_SET_SPEED == 0) &&
              (ACCSM_In.VLC_TAKE_ACTUAL_SPEED == 0) &&
              (ACCSM_In.SWITCH_SPEED_UNIT == 0)) {
            ACCSM_DW.is_override = ACCSM_IN_Display_override;
            ACCSM_B.OPERATIONAL_MODE = ACCSM_Display_op_cc_override;
          } else {
            ACCSM_B.OPERATIONAL_MODE = ACCSM_Display_op_cc_valid;
          }
          break;
        }
      }
      break;

     default:
      /* case IN_ready: */
      ACCSM_B.CONTROL_STATE = ACCSM_Cc_cc_ready;
      ACCSM_B.DAS_ENGAGED = 0U;
      ACCSM_B.DAS_OVERRIDE = 0U;
      ACCSM_B.DAS_SHUTOFF_REQ = 0U;
      ACCSM_B.DAS_OFF = 0U;
      if ((ACCSM_In.ACCEPT_VLC_ENGAGEMENT != 0) &&
          ((ACCSM_In.VLC_TAKE_ACTUAL_SPEED != 0) ||
           (ACCSM_In.VLC_RESUME_SET_SPEED != 0))) {
        ACCSM_DW.is_on = ACCSM_IN_engage;
        ACCSM_B.CONTROL_STATE = ACCSM_Cc_cc_engage;
        ACCSM_B.DAS_ENGAGED = 1U;
        ACCSM_B.DAS_OVERRIDE = 0U;
        ACCSM_B.DAS_SHUTOFF_REQ = 0U;
        ACCSM_B.DAS_OFF = 0U;
        ACCSM_DW.is_engage = ACCSM_IN_Display_valid_o;
        ACCSM_B.OPERATIONAL_MODE = ACCSM_Display_op_cc_valid;
      }
      break;
    }
  }

  if (guard1) {
    ACCSM_DW.is_active = ACCSM_IN_NO_ACTIVE_CHILD;
    ACCSM_DW.is_engage = ACCSM_IN_NO_ACTIVE_CHILD;
    ACCSM_DW.is_override = ACCSM_IN_NO_ACTIVE_CHILD;
    ACCSM_DW.is_on = ACCSM_IN_NO_ACTIVE_CHILD;
    ACCSM_DW.is_control_state = ACCSM_IN_off;
    ACCSM_B.CONTROL_STATE = ACCSM_Cc_cc_off;
    ACCSM_B.OPERATIONAL_MODE = ACCSM_Display_op_none;
    ACCSM_B.DAS_ENGAGED = 0U;
    ACCSM_B.DAS_OVERRIDE = 0U;
    ACCSM_B.DAS_SHUTOFF_REQ = 0U;
    ACCSM_B.DAS_OFF = 1U;
  }

  /* End of Inport: '<Root>/Inport' */
}

/* Model step function */
void ACCSM_step(void)
{
  /* Chart: '<S1>/VLC_DETERMINE_CONTROL_STATE' incorporates:
   *  Inport: '<Root>/Inport'
   */
  if (ACCSM_DW.is_active_c3_ACCSM == 0U) {
    ACCSM_DW.is_active_c3_ACCSM = 1U;
    ACCSM_DW.is_control_state = ACCSM_IN_off;
    ACCSM_B.CONTROL_STATE = ACCSM_Cc_cc_off;
    ACCSM_B.OPERATIONAL_MODE = ACCSM_Display_op_none;
    ACCSM_B.DAS_ENGAGED = 0U;
    ACCSM_B.DAS_OVERRIDE = 0U;
    ACCSM_B.DAS_SHUTOFF_REQ = 0U;
    ACCSM_B.DAS_OFF = 1U;
    if (ACCSM_In.STANDSTILL != 0) {
      /* BusCreator: '<S1>/Bus Creator' */
      ACCSM_Out.CONTROL_MODE = ACCSM_Cc_standstill_mode;
    } else if (ACCSM_In.OBJECT_EFFECTIVE != 0) {
      /* BusCreator: '<S1>/Bus Creator' */
      ACCSM_Out.CONTROL_MODE = ACCSM_Cc_follow_mode;
    } else {
      /* BusCreator: '<S1>/Bus Creator' */
      ACCSM_Out.CONTROL_MODE = ACCSM_Cc_free_mode;
    }
  } else {
    if (ACCSM_DW.is_control_state == ACCSM_IN_off) {
      ACCSM_B.CONTROL_STATE = ACCSM_Cc_cc_off;
      ACCSM_B.DAS_ENGAGED = 0U;
      ACCSM_B.DAS_OVERRIDE = 0U;
      ACCSM_B.DAS_SHUTOFF_REQ = 0U;
      ACCSM_B.DAS_OFF = 1U;
      if ((ACCSM_In.SELECTED_FUNCTION == 1) && (ACCSM_In.VLC_INHIBIT == 0)) {
        ACCSM_DW.is_control_state = ACCSM_IN_on;
        ACCSM_DW.is_on = ACCSM_IN_ready;
        ACCSM_B.CONTROL_STATE = ACCSM_Cc_cc_ready;
        ACCSM_B.DAS_ENGAGED = 0U;
        ACCSM_B.DAS_OVERRIDE = 0U;
        ACCSM_B.DAS_SHUTOFF_REQ = 0U;
        ACCSM_B.DAS_OFF = 0U;
        ACCSM_B.OPERATIONAL_MODE = ACCSM_Display_op_cc_invalid;
      }
    } else {
      /* case IN_on: */
      ACCSM_on();
    }

    if (ACCSM_In.STANDSTILL != 0) {
      /* BusCreator: '<S1>/Bus Creator' */
      ACCSM_Out.CONTROL_MODE = ACCSM_Cc_standstill_mode;
    } else if (ACCSM_In.OBJECT_EFFECTIVE != 0) {
      /* BusCreator: '<S1>/Bus Creator' */
      ACCSM_Out.CONTROL_MODE = ACCSM_Cc_follow_mode;
    } else {
      /* BusCreator: '<S1>/Bus Creator' */
      ACCSM_Out.CONTROL_MODE = ACCSM_Cc_free_mode;
    }
  }

  /* End of Chart: '<S1>/VLC_DETERMINE_CONTROL_STATE' */

  /* BusCreator: '<S1>/Bus Creator' incorporates:
   *  UnitDelay: '<S1>/Unit Delay'
   */
  ACCSM_Out.CONTROL_STATE = ACCSM_B.CONTROL_STATE;
  ACCSM_Out.CONTROL_STATE_LAST_CYCLE = ACCSM_DW.UnitDelay_DSTATE;
  ACCSM_Out.OPERATIONAL_MODE = ACCSM_B.OPERATIONAL_MODE;
  ACCSM_Out.DAS_ENGAGED = ACCSM_B.DAS_ENGAGED;
  ACCSM_Out.DAS_OFF = ACCSM_B.DAS_OFF;
  ACCSM_Out.DAS_OVERRIDE = ACCSM_B.DAS_OVERRIDE;
  ACCSM_Out.DAS_SHUTOFF_REQ = ACCSM_B.DAS_SHUTOFF_REQ;
  ACCSM_Out.CANCEL_RAMP = ACCSM_B.CANCEL_RAMP;

  /* Update for UnitDelay: '<S1>/Unit Delay' */
  ACCSM_DW.UnitDelay_DSTATE = ACCSM_B.CONTROL_STATE;
}

/* Model initialize function */
void ACCSM_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
// #define DLMU1_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */