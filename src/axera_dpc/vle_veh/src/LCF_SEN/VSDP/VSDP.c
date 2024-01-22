/**********************************Model
 Property********************************
 *
 * Company             : SENSETIME
 *
 * Tool Version        : Ver2.0
 *
 * Model Name          : VSDP
 *
 * Model Long Name     : Vehicle Signals Data Processing

 *

 * Model Advisor       : Not Check

 *

 * Model Version       : Ver_01

 *

 * Model Author        :

 *

 * Model Reviewer      :

 *

 * Model Review Data   :

 *

 * Model Cycle Time    : 60ms


 ************************************Auto
 Coder**********************************
 *
 * File                             : VSDP.c
 *
 * FileType                         : Code Source File
 *
 * Real-Time Workshop file version  : 9.4 (R2020b) 29-Jul-2020
 *
 * TLC version                      : 9.4 (Aug 20 2020)
 *
 * C source code generated on       : Wed Oct 26 12:20:40 2022
 *
 * Copyright (C) by SenseTime Group Limited. All rights reserved.
 *******************************************************************************/

#include "VSDP.h"
#include "VSDP_private.h"
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
#define CAL_START_CODE
#include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/* ConstVolatile memory section */
/* Definition for custom storage class: ConstVolatile */
const volatile uint8_T VSDP_DegTrig_C_St =
    3U; /* Referenced by: '<S5>/V_Parameter10' */

/* Degradation Trigger */
const volatile real32_T VSDP_FalDlyTiAccPedPstnRate_C_Sec =
    0.5F; /* Referenced by: '<S5>/V_Parameter7' */

/* Fall delay time of actual accerlator pedal rate is more than Minimum
 * threshold of accerlator pedal rate */
const volatile real32_T VSDP_FalDlyTiCtrlStEn_C_Sec =
    0.0F; /* Referenced by:
           * '<S4>/V_Parameter'
           * '<S4>/V_Parameter1'
           * '<S4>/V_Parameter10'
           * '<S4>/V_Parameter12'
           * '<S4>/V_Parameter2'
           * '<S4>/V_Parameter3'
           * '<S4>/V_Parameter4'
           * '<S4>/V_Parameter5'
           */

/* Fall delay time of ABS，ACC，ESC，TSC，VSM or EBA control function condition
 */
const volatile real32_T VSDP_FalDlyTiStErr_C_Sec = 0.0F; /* Referenced by:
                                                          * '<S6>/V_Parameter14'
                                                          * '<S6>/V_Parameter17'
                                                          * '<S6>/V_Parameter19'
                                                          * '<S6>/V_Parameter20'
                                                          * '<S6>/V_Parameter22'
                                                          * '<S6>/V_Parameter25'
                                                          * '<S6>/V_Parameter28'
                                                          * '<S6>/V_Parameter31'
                                                          * '<S6>/V_Parameter34'
                                                          * '<S6>/V_Parameter9'
                                                          */

/* Fall delay time value for error state  */
const volatile real32_T VSDP_FalDlyTiTrnSgl_C_Sec = 0.6F; /* Referenced by:
                                                           * '<S5>/V_Parameter8'
                                                           * '<S5>/V_Parameter9'
                                                           */

/* Fall delay time value of turn light is active */
const volatile real32_T VSDP_ManuActuTrqMx_C_Nm =
    3.5F; /* Referenced by: '<S5>/V_Parameter11' */

/*  Maximum threshold of manunal torque for EPS */
const volatile real32_T VSDP_NoDaytimeMn_C_Sec =
    0.5F; /* Referenced by: '<S84>/V_Parameter1' */

/* Minimum time value of no daytime state */
const volatile real32_T VSDP_NoDaytimeTrnOff_C_Sec =
    10.0F; /* Referenced by: '<S84>/V_Parameter2' */

/* Fall delay time value of no daytime state */
const volatile uint8_T VSDP_NoDaytime_C_St = 0U; /* Referenced by:
                                                  * '<S84>/V_Parameter10'
                                                  * '<S84>/V_Parameter11'
                                                  * '<S84>/V_Parameter3'
                                                  * '<S84>/V_Parameter4'
                                                  * '<S84>/V_Parameter6'
                                                  * '<S84>/V_Parameter8'
                                                  */

/* No daytime state value */
const volatile real32_T VSDP_RSDlyTimeABSAct_sec =
    0.3F; /* Referenced by: '<S4>/V_Parameter7' */

/* Fall delay time of ABS，ACC，ESC，TSC，VSM or EBA control function condition
 */
const volatile real32_T VSDP_RSDlyTimeAEBAct_sec =
    0.0F; /* Referenced by: '<S4>/V_Parameter9' */

/* Fall delay time of ABS，ACC，ESC，TSC，VSM or EBA control function condition
 */
const volatile real32_T VSDP_RSDlyTimeARPAct_sec =
    0.5F; /* Referenced by: '<S4>/V_Parameter11' */

/* Fall delay time of ABS，ACC，ESC，TSC，VSM or EBA control function condition
 */
const volatile real32_T VSDP_RSDlyTimeESCAct_sec =
    0.5F; /* Referenced by: '<S4>/V_Parameter6' */

/* Fall delay time of ABS，ACC，ESC，TSC，VSM or EBA control function condition
 */
const volatile real32_T VSDP_RSDlyTimeHDCAct_sec =
    0.0F; /* Referenced by: '<S4>/V_Parameter13' */

/* Fall delay time of ABS，ACC，ESC，TSC，VSM or EBA control function condition
 */
const volatile real32_T VSDP_RSDlyTimeTCSAct_sec =
    0.5F; /* Referenced by: '<S4>/V_Parameter8' */

/* Fall delay time of ABS，ACC，ESC，TSC，VSM or EBA control function condition
 */
const volatile real32_T VSDP_RisDlyTiManuActuTrq_C_Sec =
    0.15F; /* Referenced by: '<S5>/V_Parameter39' */

/* Fall delay time of manunal torque is more than Minimum threshold of
 * accerlator pedal rate */
const volatile real32_T VSDP_RisDlyTiStErr_C_Sec = 0.5F; /* Referenced by:
                                                          * '<S6>/V_Parameter12'
                                                          * '<S6>/V_Parameter15'
                                                          * '<S6>/V_Parameter16'
                                                          * '<S6>/V_Parameter18'
                                                          * '<S6>/V_Parameter23'
                                                          * '<S6>/V_Parameter26'
                                                          * '<S6>/V_Parameter29'
                                                          * '<S6>/V_Parameter32'
                                                          * '<S6>/V_Parameter40'
                                                          * '<S6>/V_Parameter8'
                                                          */

/* Rise delay time value for error state  */
const volatile boolean_T VSDP_StWiperNoErrEn_C_B =
    0; /* Referenced by: '<S88>/V_Parameter4' */

/* Debug value of wiper state is active */
const volatile uint8_T VSDP_StageWiperEn_C_St = 104U; /* Referenced by:
                                                       * '<S88>/V_Parameter10'
                                                       * '<S88>/V_Parameter11'
                                                       * '<S88>/V_Parameter13'
                                                       * '<S88>/V_Parameter15'
                                                       * '<S88>/V_Parameter17'
                                                       * '<S88>/V_Parameter6'
                                                       * '<S88>/V_Parameter8'
                                                       */

/* wiper stage value is valid */
const volatile uint8_T VSDP_StateWiperEn_C_St = 1U;
/* Referenced by: '<S88>/V_Parameter36' */

/* wiper state value is valid */
const volatile real32_T VSDP_TiTrigStErr_C_Sec = 10.0F; /* Referenced by:
                                                         * '<S6>/V_Parameter1'
                                                         * '<S6>/V_Parameter10'
                                                         * '<S6>/V_Parameter11'
                                                         * '<S6>/V_Parameter13'
                                                         * '<S6>/V_Parameter2'
                                                         * '<S6>/V_Parameter3'
                                                         * '<S6>/V_Parameter4'
                                                         * '<S6>/V_Parameter5'
                                                         * '<S6>/V_Parameter6'
                                                         * '<S6>/V_Parameter7'
                                                         */

/* Time trigger time value of  error condition  */
const volatile real32_T VSDP_TrsdSteAglDtct_C_Rad = 0.5235F;
/* Referenced by: '<S85>/V_Parameter37' */

/* Maximum steer angle limit value  */
const volatile real32_T VSDP_TrsdSteAglNoDtct_C_Rad = 0.4365F;
/* Referenced by: '<S85>/V_Parameter38' */

/* Maximum steer angle limit value for valid steer angle */
const volatile real32_T VSDP_UstpAccPedPstnMx_C_Per =
    10.0F; /* Referenced by: '<S5>/V_Parameter6' */

/*  Maximum threshold of accerlator pedal rate */
const volatile real32_T VSDP_VehSpdLmtMx_C_Mps = 5.6F;
/* Referenced by: '<S85>/V_Parameter36' */

/* Maximum vehicle speed limit value for steer angle */
const volatile real32_T VSDP_WiperContiTiMn_C_Sec =
    40.0F; /* Referenced by: '<S88>/V_Parameter3' */

/* continue time value for wiper state is active */
const volatile real32_T VSDP_WiperEnTiMn_C_Sec =
    0.4F; /* Referenced by: '<S88>/V_Parameter1' */

/* Rise  delay time value for wiper state is active */
const volatile real32_T VSDP_WiperEvtGapTiMx_C_Sec =
    10.0F; /* Referenced by: '<S88>/V_Parameter2' */

/* Fall delay time value of rise  delay time for wiper state is active */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
#define CAL_STOP_CODE
#include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
#define ASW_QM_CORE2_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/* Exported block signals */
uint16_T VSDP_VehStIvld_St;   /* '<S1>/60ms'
                               * Invalid state of vehicle system
                               */
uint8_T VSDP_CtrlStEn_St;     /* '<S1>/60ms'
                               * State of control feature is enable
                               */
uint8_T VSDP_CtrlStNoAvlb_St; /* '<S1>/60ms'
                               * State of control feature is not avaible
                               */
uint8_T VSDP_IvldStDrv_St;    /* '<S1>/60ms'
                               * Invalid state of driver
                               */
uint8_T VSDP_StError_St;      /* '<S1>/60ms'
                               * error state of system
                               */

/* Exported block states */
real32_T
    VSDP_RSDlyTiCtrlStEnABS_Sec; /* '<S21>/Unit Delay'
                                  * Condition of ABS in actively controlling
                                  */
real32_T
    VSDP_FalDlyTiCtrlStEnABS_Sec; /* '<S10>/Unit Delay'
                                   * Condition of ABS in actively controlling
                                   */
real32_T
    VSDP_FalDlyTiCtrlStEnACC_Sec; /* '<S11>/Unit Delay'
                                   * Condition of ACC in actively controlling
                                   */
real32_T
    VSDP_RSDlyTiCtrlStEnESC_Sec; /* '<S19>/Unit Delay'
                                  * Condition of ABS in actively controlling
                                  */
real32_T
    VSDP_FalDlyTiCtrlStEnESC_Sec; /* '<S12>/Unit Delay'
                                   * Condition of ESC in actively controlling
                                   */
real32_T
    VSDP_RSDlyTiCtrlStEnTCS_Sec; /* '<S20>/Unit Delay'
                                  * Condition of ABS in actively controlling
                                  */
real32_T
    VSDP_FalDlyTiCtrlStEnTCS_Sec; /* '<S13>/Unit Delay'
                                   * Condition of TSC in actively controlling
                                   */
real32_T
    VSDP_FalDlyTiCtrlStEnVSM_Sec; /* '<S14>/Unit Delay'
                                   * Condition of VSM in actively controlling
                                   */
real32_T
    VSDP_RSDlyTiCtrlStEnAEB_Sec; /* '<S22>/Unit Delay'
                                  * Condition of ABS in actively controlling
                                  */
real32_T
    VSDP_FalDlyTiCtrlStEnEBA_Sec; /* '<S15>/Unit Delay'
                                   * Condition of EBA in actively controlling
                                   */
real32_T
    VSDP_RSDlyTiCtrlStEnARP_Sec; /* '<S23>/Unit Delay'
                                  * Condition of ABS in actively controlling
                                  */
real32_T
    VSDP_FalDlyTiCtrlStEnAPR_Sec; /* '<S16>/Unit Delay'
                                   * Condition of EBA in actively controlling
                                   */
real32_T
    VSDP_RSDlyTiCtrlStEnHDC_Sec; /* '<S24>/Unit Delay'
                                  * Condition of ABS in actively controlling
                                  */
real32_T
    VSDP_FalDlyTiCtrlStEnHDC_Sec;         /* '<S17>/Unit Delay'
                                           * Condition of EBA in actively controlling
                                           */
real32_T VSDP_UstpAccPedPstn_Per;         /* '<S5>/Unit Delay'
                                           * Unit step of acceleration pedal position
                                           */
real32_T VSDP_FalDlyTiAccPedPstnRate_Sec; /* '<S26>/Unit Delay'
                                           * Fall delay time of acceleration
                                           * pedal position rate is out range
                                           */
real32_T VSDP_FalDlyTiTrnSglHarLigEn_Sec; /* '<S27>/Unit Delay'
                                           * Fall delay time of hazard light is
                                           * active
                                           */
real32_T VSDP_FalDlyTiTrnSglEn_Sec;       /* '<S28>/Unit Delay'
                                           * Fall delay time of turn light is active
                                           */
real32_T VSDP_RisDlyTiManuActuTrq_Sec;    /* '<S30>/Unit Delay'
                                           * Fall delay time of manunal torque for
                                           * EPS is out range
                                           */
real32_T VSDP_RisDlyTiStErrABS_Sec;       /* '<S63>/Unit Delay'
                                           * Rise delay time for ABS error condition
                                           */
real32_T VSDP_TiTrigStErrABS_Sec;         /* '<S73>/Unit Delay'
                                           * Time trigger of ABS error condition
                                           */
real32_T VSDP_FalDlyTiStErrABS_C_Sec;     /* '<S53>/Unit Delay'
                                           * Fall edge of rise delay time for ABS
                                           * error condition
                                           */
real32_T VSDP_RisDlyTiStErrESC_Sec;       /* '<S66>/Unit Delay'
                                           * Rise delay time for ESC error condition
                                           */
real32_T VSDP_TiTrigStErrESC_Sec;         /* '<S75>/Unit Delay'
                                           * Time trigger of ESC error condition
                                           */
real32_T VSDP_FalDlyTiStErrESC_C_Sec;     /* '<S54>/Unit Delay'
                                           * Fall edge of rise delay time for ESC
                                           * error condition
                                           */
real32_T VSDP_RisDlyTiStErrTSC_Sec;       /* '<S67>/Unit Delay'
                                           * Rise delay time for TSC error condition
                                           */
real32_T VSDP_TiTrigStErrTSC_Sec;         /* '<S76>/Unit Delay'
                                           * Time trigger of TSC error condition
                                           */
real32_T VSDP_FalDlyTiStErrTSC_C_Sec;     /* '<S55>/Unit Delay'
                                           * Fall edge of rise delay time for TSC
                                           * error condition
                                           */
real32_T VSDP_RisDlyTiStErrVSM_Sec;       /* '<S68>/Unit Delay'
                                           * Rise delay time for VSM error condition
                                           */
real32_T VSDP_TiTrigStErrVSM_Sec;         /* '<S77>/Unit Delay'
                                           * Time trigger of VSM error condition
                                           */
real32_T VSDP_FalDlyTiStErrVSM_C_Sec;     /* '<S56>/Unit Delay'
                                           * Fall edge of rise delay time for VSM
                                           * error condition
                                           */
real32_T VSDP_RisDlyTiStErrVDY_Sec;       /* '<S69>/Unit Delay'
                                           * Rise delay time for VDY error condition
                                           */
real32_T VSDP_TiTrigStErrVDY_Sec;         /* '<S78>/Unit Delay'
                                           * Time trigger of VDY error condition
                                           */
real32_T VSDP_FalDlyTiStErrVDY_C_Sec;     /* '<S57>/Unit Delay'
                                           * Fall edge of rise delay time for VDY
                                           * error condition
                                           */
real32_T
    VSDP_RisDlyTiStErrLatDMC_Sec;        /* '<S70>/Unit Delay'
                                          * Rise delay time for LatDMC error condition
                                          */
real32_T VSDP_TiTrigStErrLatDMC_Sec;     /* '<S79>/Unit Delay'
                                          * Time trigger of LatDMC error condition
                                          */
real32_T VSDP_FalDlyTiStErrLatDMC_C_Sec; /* '<S58>/Unit Delay'
                                          * Fall edge of rise delay time for
                                          * LatDMC error condition
                                          */
real32_T VSDP_RisDlyTiStErrACC_Sec;      /* '<S65>/Unit Delay'
                                          * Rise delay time for ACC error condition
                                          */
real32_T VSDP_TiTrigStErrACC_Sec;        /* '<S74>/Unit Delay'
                                          * Time trigger of ACC error condition
                                          */
real32_T VSDP_FalDlyTiStErrACC_C_Sec;    /* '<S61>/Unit Delay'
                                          * Fall edge of rise delay time for ACC
                                          * error condition
                                          */
real32_T VSDP_RisDlyTiStErrARP_Sec;      /* '<S72>/Unit Delay'
                                          * Rise delay time for EBA error condition
                                          */
real32_T VSDP_TiTrigStErrARP_Sec;        /* '<S81>/Unit Delay'
                                          * Time trigger of EBA error condition
                                          */
real32_T VSDP_FalDlyTiStErrARP_C_Sec;    /* '<S52>/Unit Delay'
                                          * Fall edge of rise delay time for EBA
                                          * error condition
                                          */
real32_T VSDP_RisDlyTiStErrEBA_Sec;      /* '<S71>/Unit Delay'
                                          * Rise delay time for EBA error condition
                                          */
real32_T VSDP_TiTrigStErrEBA_Sec;        /* '<S80>/Unit Delay'
                                          * Time trigger of EBA error condition
                                          */
real32_T VSDP_FalDlyTiStErrEBA_C_Sec;    /* '<S59>/Unit Delay'
                                          * Fall edge of rise delay time for EBA
                                          * error condition
                                          */
real32_T VSDP_RisDlyTiStErrHDC_Sec;      /* '<S64>/Unit Delay'
                                          * Rise delay time for EBA error condition
                                          */
real32_T VSDP_TiTrigStErrHDC_Sec;        /* '<S82>/Unit Delay'
                                          * Time trigger of EBA error condition
                                          */
real32_T VSDP_FalDlyTiStErrHDC_C_Sec;    /* '<S60>/Unit Delay'
                                          * Fall edge of rise delay time for EBA
                                          * error condition
                                          */
real32_T
    VSDP_RisDlyTiNoDaytimeMn_Sec;            /* '<S96>/Unit Delay'
                                              * Rise  delay time for no daytime is active
                                              */
real32_T VSDP_FalDlyTiNoDaytimeTrnOff_C_Sec; /* '<S95>/Unit Delay'
                                              * Fall edge of Rise  delay time
                                              * for no daytime is active
                                              */
real32_T
    VSDP_RisDlyTiWiperEnTiMn_Sec;            /* '<S108>/Unit Delay'
                                              * Rise  delay time for wiper state is active
                                              */
real32_T VSDP_FalDlyTiWiperEvtGapTiMx_C_Sec; /* '<S107>/Unit Delay'
                                              * Fall edge of rise  delay time
                                              * for wiper state is active
                                              */
real32_T
    VSDP_RisDlyTiWiperContiTiMn_Sec; /* '<S109>/Unit Delay'
                                      * continue time for wiper state is active
                                      */
boolean_T VSDP_RisEdgeStErrABS_B;    /* '<S32>/Unit Delay'
                                      * Rise edge of rise delay time for ABS error
                                      * condition
                                      */
boolean_T VSDP_FFStErrABS_B;         /* '<S42>/Unit Delay'
                                      * FF of ABS error condition
                                      */
boolean_T VSDP_RisEdgeStErrESC_B;    /* '<S34>/Unit Delay'
                                      * Rise edge of rise delay time for ESC error
                                      * condition
                                      */
boolean_T VSDP_FFStErrESC_B;         /* '<S44>/Unit Delay'
                                      * FF of ESC error condition
                                      */
boolean_T VSDP_RisEdgeStErrTSC_B;    /* '<S35>/Unit Delay'
                                      * Rise edge of rise delay time for TSC error
                                      * condition
                                      */
boolean_T VSDP_FFStErrTSC_B;         /* '<S45>/Unit Delay'
                                      * FF of TSC error condition
                                      */
boolean_T VSDP_RisEdgeStErrVSM_B;    /* '<S36>/Unit Delay'
                                      * Rise edge of rise delay time for VSM error
                                      * condition
                                      */
boolean_T VSDP_FFStErrVSM_B;         /* '<S46>/Unit Delay'
                                      * FF of VSM error condition
                                      */
boolean_T VSDP_RisEdgeStErrVDY_B;    /* '<S37>/Unit Delay'
                                      * Rise edge of rise delay time for VDY error
                                      * condition
                                      */
boolean_T VSDP_FFStErrVDY_B;         /* '<S47>/Unit Delay'
                                      * FF of VDY error condition
                                      */
boolean_T VSDP_RisEdgeStErrLatDMC_B; /* '<S38>/Unit Delay'
                                      * Rise edge of rise delay time for LatDMC
                                      * error condition
                                      */
boolean_T VSDP_FFStErrLatDMC_B;      /* '<S48>/Unit Delay'
                                      * FF of LatDMC error condition
                                      */
boolean_T VSDP_RisEdgeStErrACC_B;    /* '<S33>/Unit Delay'
                                      * Rise edge of rise delay time for ACC error
                                      * condition
                                      */
boolean_T VSDP_FFStErrACC_B;         /* '<S43>/Unit Delay'
                                      * FF of ACC error condition
                                      */
boolean_T VSDP_RisEdgeStErrARP_B;    /* '<S40>/Unit Delay'
                                      * Rise edge of rise delay time for EBA error
                                      * condition
                                      */
boolean_T VSDP_FFStErrARP_B;         /* '<S50>/Unit Delay'
                                      * FF of EBA error condition
                                      */
boolean_T VSDP_RisEdgeStErrEBA_B;    /* '<S39>/Unit Delay'
                                      * Rise edge of rise delay time for EBA error
                                      * condition
                                      */
boolean_T VSDP_FFStErrEBA_B;         /* '<S49>/Unit Delay'
                                      * FF of EBA error condition
                                      */
boolean_T VSDP_RisEdgeStErrHDC_B;    /* '<S41>/Unit Delay'
                                      * Rise edge of rise delay time for EBA error
                                      * condition
                                      */
boolean_T VSDP_FFStErrHDC_B;         /* '<S51>/Unit Delay'
                                      * FF of EBA error condition
                                      */
boolean_T VSDP_FFGrNoEnga_B;         /* '<S98>/Unit Delay'
                                      * FF of invalid engaged gear
                                      */
boolean_T VSDP_FFDtctSteAglStop_B;   /* '<S97>/Unit Delay'
                                      * FF of steer angle is out range
                                      */
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/* Exported data definition */
#define ASW_QM_CORE2_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/* Definition for custom storage class: Global */
uint8_T VSDP_SetBit_BS_Param_1[6] = {0U, 1U, 2U, 3U,
                                     4U, 5U}; /* Referenced by:
                                               * '<S9>/ex_sfun_set_bit'
                                               * '<S25>/ex_sfun_set_bit'
                                               */

uint8_T VSDP_SetBit_BS_Param_2[7] = {0U, 1U, 2U, 3U, 4U, 5U, 6U};
/* Referenced by: '<S31>/ex_sfun_set_bit' */

uint8_T VSDP_SetBit_BS_Param_3[8] = {0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U};
/* Referenced by: '<S83>/ex_sfun_set_bit' */

uint8_T VSDP_SetBit_BS_Param_4[14] = {
    0U, 1U, 2U,  3U,  4U,  5U, 6U, 7U,
    8U, 9U, 10U, 11U, 12U, 13U}; /* Referenced by: '<S99>/ex_sfun_set_bit' */
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/* Model step function */
void VSDP_step(void) {
    /* local block i/o variables */
    uint32_T rtb_ex_sfun_set_bit;
    boolean_T rtb_VectorConcatenate[6];
    boolean_T rtb_VectorConcatenate_mhyz[7];
    boolean_T rtb_VectorConcatenate_gjaw[14];
    boolean_T rtb_VectorConcatenate_kusl[8];
    real32_T rtb_Abs1;
    boolean_T rtb_AND_cvp0;
    boolean_T rtb_AND_g15l;
    boolean_T rtb_AND_hz4i;
    boolean_T rtb_AND_itq0;
    boolean_T rtb_AND_kbzm;
    boolean_T rtb_AND_ku13;
    boolean_T rtb_AND_mai0;
    boolean_T rtb_AND_med5;
    boolean_T rtb_AND_o5jf;
    boolean_T rtb_OR_b14z;
    boolean_T rtb_y;

    /* S-Function (fcgen): '<S1>/Function-Call Generator' incorporates:
     *  SubSystem: '<S1>/60ms'
     */
    /* Switch: '<S21>/Switch' incorporates:
     *  Constant: '<S4>/V_Parameter7'
     *  Inport: '<Root>/In'
     *  Inport: '<Root>/In42'
     *  MinMax: '<S21>/Max'
     *  Sum: '<S21>/Subtract'
     *  Switch: '<S21>/Switch1'
     *  UnaryMinus: '<S21>/Unary Minus'
     *  UnitDelay: '<S21>/Unit Delay'
     */
    if (VSDPI_CtrlStEnABS_B) {
        VSDP_RSDlyTiCtrlStEnABS_Sec =
            fmaxf(VSDP_RSDlyTiCtrlStEnABS_Sec, -VSDPI_CycleTime_Sec) -
            VSDPI_CycleTime_Sec;
    } else {
        VSDP_RSDlyTiCtrlStEnABS_Sec = VSDP_RSDlyTimeABSAct_sec;
    }

    /* End of Switch: '<S21>/Switch' */

    /* Logic: '<S21>/AND' incorporates:
     *  Inport: '<Root>/In'
     *  Inport: '<Root>/In42'
     *  RelationalOperator: '<S21>/LessThanOrEqual'
     *  UnaryMinus: '<S21>/Unary Minus1'
     *  UnitDelay: '<S21>/Unit Delay'
     */
    rtb_y = (VSDPI_CtrlStEnABS_B &&
             (VSDP_RSDlyTiCtrlStEnABS_Sec <= (-VSDPI_CycleTime_Sec)));

    /* Switch: '<S10>/Switch' incorporates:
     *  Constant: '<S4>/V_Parameter'
     *  Inport: '<Root>/In42'
     *  MinMax: '<S10>/Max'
     *  Sum: '<S10>/Subtract'
     *  Switch: '<S10>/Switch1'
     *  UnaryMinus: '<S10>/Unary Minus'
     *  UnitDelay: '<S10>/Unit Delay'
     */
    if (rtb_y) {
        VSDP_FalDlyTiCtrlStEnABS_Sec = VSDP_FalDlyTiCtrlStEn_C_Sec;
    } else {
        VSDP_FalDlyTiCtrlStEnABS_Sec =
            fmaxf(-VSDPI_CycleTime_Sec, VSDP_FalDlyTiCtrlStEnABS_Sec) -
            VSDPI_CycleTime_Sec;
    }

    /* End of Switch: '<S10>/Switch' */

    /* Logic: '<S10>/OR' incorporates:
     *  Inport: '<Root>/In42'
     *  RelationalOperator: '<S10>/GreaterThan'
     *  UnaryMinus: '<S10>/Unary Minus1'
     *  UnitDelay: '<S10>/Unit Delay'
     */
    rtb_VectorConcatenate_kusl[0] =
        (rtb_y || (VSDP_FalDlyTiCtrlStEnABS_Sec > (-VSDPI_CycleTime_Sec)));

    /* Switch: '<S11>/Switch' incorporates:
     *  Constant: '<S4>/V_Parameter1'
     *  Inport: '<Root>/In1'
     *  Inport: '<Root>/In42'
     *  MinMax: '<S11>/Max'
     *  Sum: '<S11>/Subtract'
     *  Switch: '<S11>/Switch1'
     *  UnaryMinus: '<S11>/Unary Minus'
     *  UnitDelay: '<S11>/Unit Delay'
     */
    if (VSDPI_CtrlStEnACC_B) {
        VSDP_FalDlyTiCtrlStEnACC_Sec = VSDP_FalDlyTiCtrlStEn_C_Sec;
    } else {
        VSDP_FalDlyTiCtrlStEnACC_Sec =
            fmaxf(-VSDPI_CycleTime_Sec, VSDP_FalDlyTiCtrlStEnACC_Sec) -
            VSDPI_CycleTime_Sec;
    }

    /* End of Switch: '<S11>/Switch' */

    /* Logic: '<S11>/OR' incorporates:
     *  Inport: '<Root>/In1'
     *  Inport: '<Root>/In42'
     *  RelationalOperator: '<S11>/GreaterThan'
     *  UnaryMinus: '<S11>/Unary Minus1'
     *  UnitDelay: '<S11>/Unit Delay'
     */
    rtb_VectorConcatenate_kusl[1] =
        (VSDPI_CtrlStEnACC_B ||
         (VSDP_FalDlyTiCtrlStEnACC_Sec > (-VSDPI_CycleTime_Sec)));

    /* Switch: '<S19>/Switch' incorporates:
     *  Constant: '<S4>/V_Parameter6'
     *  Inport: '<Root>/In2'
     *  Inport: '<Root>/In42'
     *  MinMax: '<S19>/Max'
     *  Sum: '<S19>/Subtract'
     *  Switch: '<S19>/Switch1'
     *  UnaryMinus: '<S19>/Unary Minus'
     *  UnitDelay: '<S19>/Unit Delay'
     */
    if (VSDPI_CtrlStEnESC_B) {
        VSDP_RSDlyTiCtrlStEnESC_Sec =
            fmaxf(VSDP_RSDlyTiCtrlStEnESC_Sec, -VSDPI_CycleTime_Sec) -
            VSDPI_CycleTime_Sec;
    } else {
        VSDP_RSDlyTiCtrlStEnESC_Sec = VSDP_RSDlyTimeESCAct_sec;
    }

    /* End of Switch: '<S19>/Switch' */

    /* Logic: '<S19>/AND' incorporates:
     *  Inport: '<Root>/In2'
     *  Inport: '<Root>/In42'
     *  RelationalOperator: '<S19>/LessThanOrEqual'
     *  UnaryMinus: '<S19>/Unary Minus1'
     *  UnitDelay: '<S19>/Unit Delay'
     */
    rtb_y = (VSDPI_CtrlStEnESC_B &&
             (VSDP_RSDlyTiCtrlStEnESC_Sec <= (-VSDPI_CycleTime_Sec)));

    /* Switch: '<S12>/Switch' incorporates:
     *  Constant: '<S4>/V_Parameter2'
     *  Inport: '<Root>/In42'
     *  MinMax: '<S12>/Max'
     *  Sum: '<S12>/Subtract'
     *  Switch: '<S12>/Switch1'
     *  UnaryMinus: '<S12>/Unary Minus'
     *  UnitDelay: '<S12>/Unit Delay'
     */
    if (rtb_y) {
        VSDP_FalDlyTiCtrlStEnESC_Sec = VSDP_FalDlyTiCtrlStEn_C_Sec;
    } else {
        VSDP_FalDlyTiCtrlStEnESC_Sec =
            fmaxf(-VSDPI_CycleTime_Sec, VSDP_FalDlyTiCtrlStEnESC_Sec) -
            VSDPI_CycleTime_Sec;
    }

    /* End of Switch: '<S12>/Switch' */

    /* Logic: '<S12>/OR' incorporates:
     *  Inport: '<Root>/In42'
     *  RelationalOperator: '<S12>/GreaterThan'
     *  UnaryMinus: '<S12>/Unary Minus1'
     *  UnitDelay: '<S12>/Unit Delay'
     */
    rtb_VectorConcatenate_kusl[2] =
        (rtb_y || (VSDP_FalDlyTiCtrlStEnESC_Sec > (-VSDPI_CycleTime_Sec)));

    /* Switch: '<S20>/Switch' incorporates:
     *  Constant: '<S4>/V_Parameter8'
     *  Inport: '<Root>/In3'
     *  Inport: '<Root>/In42'
     *  MinMax: '<S20>/Max'
     *  Sum: '<S20>/Subtract'
     *  Switch: '<S20>/Switch1'
     *  UnaryMinus: '<S20>/Unary Minus'
     *  UnitDelay: '<S20>/Unit Delay'
     */
    if (VSDPI_CtrlStEnTCS_B) {
        VSDP_RSDlyTiCtrlStEnTCS_Sec =
            fmaxf(VSDP_RSDlyTiCtrlStEnTCS_Sec, -VSDPI_CycleTime_Sec) -
            VSDPI_CycleTime_Sec;
    } else {
        VSDP_RSDlyTiCtrlStEnTCS_Sec = VSDP_RSDlyTimeTCSAct_sec;
    }

    /* End of Switch: '<S20>/Switch' */

    /* Logic: '<S20>/AND' incorporates:
     *  Inport: '<Root>/In3'
     *  Inport: '<Root>/In42'
     *  RelationalOperator: '<S20>/LessThanOrEqual'
     *  UnaryMinus: '<S20>/Unary Minus1'
     *  UnitDelay: '<S20>/Unit Delay'
     */
    rtb_y = (VSDPI_CtrlStEnTCS_B &&
             (VSDP_RSDlyTiCtrlStEnTCS_Sec <= (-VSDPI_CycleTime_Sec)));

    /* Switch: '<S13>/Switch' incorporates:
     *  Constant: '<S4>/V_Parameter3'
     *  Inport: '<Root>/In42'
     *  MinMax: '<S13>/Max'
     *  Sum: '<S13>/Subtract'
     *  Switch: '<S13>/Switch1'
     *  UnaryMinus: '<S13>/Unary Minus'
     *  UnitDelay: '<S13>/Unit Delay'
     */
    if (rtb_y) {
        VSDP_FalDlyTiCtrlStEnTCS_Sec = VSDP_FalDlyTiCtrlStEn_C_Sec;
    } else {
        VSDP_FalDlyTiCtrlStEnTCS_Sec =
            fmaxf(-VSDPI_CycleTime_Sec, VSDP_FalDlyTiCtrlStEnTCS_Sec) -
            VSDPI_CycleTime_Sec;
    }

    /* End of Switch: '<S13>/Switch' */

    /* Logic: '<S13>/OR' incorporates:
     *  Inport: '<Root>/In42'
     *  RelationalOperator: '<S13>/GreaterThan'
     *  UnaryMinus: '<S13>/Unary Minus1'
     *  UnitDelay: '<S13>/Unit Delay'
     */
    rtb_VectorConcatenate_kusl[3] =
        (rtb_y || (VSDP_FalDlyTiCtrlStEnTCS_Sec > (-VSDPI_CycleTime_Sec)));

    /* Switch: '<S14>/Switch' incorporates:
     *  Constant: '<S4>/V_Parameter4'
     *  Inport: '<Root>/In4'
     *  Inport: '<Root>/In42'
     *  MinMax: '<S14>/Max'
     *  Sum: '<S14>/Subtract'
     *  Switch: '<S14>/Switch1'
     *  UnaryMinus: '<S14>/Unary Minus'
     *  UnitDelay: '<S14>/Unit Delay'
     */
    if (VSDPI_CtrlStEnVSM_B) {
        VSDP_FalDlyTiCtrlStEnVSM_Sec = VSDP_FalDlyTiCtrlStEn_C_Sec;
    } else {
        VSDP_FalDlyTiCtrlStEnVSM_Sec =
            fmaxf(-VSDPI_CycleTime_Sec, VSDP_FalDlyTiCtrlStEnVSM_Sec) -
            VSDPI_CycleTime_Sec;
    }

    /* End of Switch: '<S14>/Switch' */

    /* Logic: '<S14>/OR' incorporates:
     *  Inport: '<Root>/In4'
     *  Inport: '<Root>/In42'
     *  RelationalOperator: '<S14>/GreaterThan'
     *  UnaryMinus: '<S14>/Unary Minus1'
     *  UnitDelay: '<S14>/Unit Delay'
     */
    rtb_VectorConcatenate_kusl[4] =
        (VSDPI_CtrlStEnVSM_B ||
         (VSDP_FalDlyTiCtrlStEnVSM_Sec > (-VSDPI_CycleTime_Sec)));

    /* Switch: '<S22>/Switch' incorporates:
     *  Constant: '<S4>/V_Parameter9'
     *  Inport: '<Root>/In42'
     *  Inport: '<Root>/In5'
     *  MinMax: '<S22>/Max'
     *  Sum: '<S22>/Subtract'
     *  Switch: '<S22>/Switch1'
     *  UnaryMinus: '<S22>/Unary Minus'
     *  UnitDelay: '<S22>/Unit Delay'
     */
    if (VSDPI_CtrlStEnEBA_B) {
        VSDP_RSDlyTiCtrlStEnAEB_Sec =
            fmaxf(VSDP_RSDlyTiCtrlStEnAEB_Sec, -VSDPI_CycleTime_Sec) -
            VSDPI_CycleTime_Sec;
    } else {
        VSDP_RSDlyTiCtrlStEnAEB_Sec = VSDP_RSDlyTimeAEBAct_sec;
    }

    /* End of Switch: '<S22>/Switch' */

    /* Logic: '<S22>/AND' incorporates:
     *  Inport: '<Root>/In42'
     *  Inport: '<Root>/In5'
     *  RelationalOperator: '<S22>/LessThanOrEqual'
     *  UnaryMinus: '<S22>/Unary Minus1'
     *  UnitDelay: '<S22>/Unit Delay'
     */
    rtb_y = (VSDPI_CtrlStEnEBA_B &&
             (VSDP_RSDlyTiCtrlStEnAEB_Sec <= (-VSDPI_CycleTime_Sec)));

    /* Switch: '<S15>/Switch' incorporates:
     *  Constant: '<S4>/V_Parameter5'
     *  Inport: '<Root>/In42'
     *  MinMax: '<S15>/Max'
     *  Sum: '<S15>/Subtract'
     *  Switch: '<S15>/Switch1'
     *  UnaryMinus: '<S15>/Unary Minus'
     *  UnitDelay: '<S15>/Unit Delay'
     */
    if (rtb_y) {
        VSDP_FalDlyTiCtrlStEnEBA_Sec = VSDP_FalDlyTiCtrlStEn_C_Sec;
    } else {
        VSDP_FalDlyTiCtrlStEnEBA_Sec =
            fmaxf(-VSDPI_CycleTime_Sec, VSDP_FalDlyTiCtrlStEnEBA_Sec) -
            VSDPI_CycleTime_Sec;
    }

    /* End of Switch: '<S15>/Switch' */

    /* Logic: '<S15>/OR' incorporates:
     *  Inport: '<Root>/In42'
     *  RelationalOperator: '<S15>/GreaterThan'
     *  UnaryMinus: '<S15>/Unary Minus1'
     *  UnitDelay: '<S15>/Unit Delay'
     */
    rtb_VectorConcatenate_kusl[5] =
        (rtb_y || (VSDP_FalDlyTiCtrlStEnEBA_Sec > (-VSDPI_CycleTime_Sec)));

    /* Switch: '<S23>/Switch' incorporates:
     *  Constant: '<S4>/V_Parameter11'
     *  Inport: '<Root>/In42'
     *  Inport: '<Root>/In44'
     *  MinMax: '<S23>/Max'
     *  Sum: '<S23>/Subtract'
     *  Switch: '<S23>/Switch1'
     *  UnaryMinus: '<S23>/Unary Minus'
     *  UnitDelay: '<S23>/Unit Delay'
     */
    if (VSDPI_CtrlStEnARP_B) {
        VSDP_RSDlyTiCtrlStEnARP_Sec =
            fmaxf(VSDP_RSDlyTiCtrlStEnARP_Sec, -VSDPI_CycleTime_Sec) -
            VSDPI_CycleTime_Sec;
    } else {
        VSDP_RSDlyTiCtrlStEnARP_Sec = VSDP_RSDlyTimeARPAct_sec;
    }

    /* End of Switch: '<S23>/Switch' */

    /* Logic: '<S23>/AND' incorporates:
     *  Inport: '<Root>/In42'
     *  Inport: '<Root>/In44'
     *  RelationalOperator: '<S23>/LessThanOrEqual'
     *  UnaryMinus: '<S23>/Unary Minus1'
     *  UnitDelay: '<S23>/Unit Delay'
     */
    rtb_y = (VSDPI_CtrlStEnARP_B &&
             (VSDP_RSDlyTiCtrlStEnARP_Sec <= (-VSDPI_CycleTime_Sec)));

    /* Switch: '<S16>/Switch' incorporates:
     *  Constant: '<S4>/V_Parameter10'
     *  Inport: '<Root>/In42'
     *  MinMax: '<S16>/Max'
     *  Sum: '<S16>/Subtract'
     *  Switch: '<S16>/Switch1'
     *  UnaryMinus: '<S16>/Unary Minus'
     *  UnitDelay: '<S16>/Unit Delay'
     */
    if (rtb_y) {
        VSDP_FalDlyTiCtrlStEnAPR_Sec = VSDP_FalDlyTiCtrlStEn_C_Sec;
    } else {
        VSDP_FalDlyTiCtrlStEnAPR_Sec =
            fmaxf(-VSDPI_CycleTime_Sec, VSDP_FalDlyTiCtrlStEnAPR_Sec) -
            VSDPI_CycleTime_Sec;
    }

    /* End of Switch: '<S16>/Switch' */

    /* Logic: '<S16>/OR' incorporates:
     *  Inport: '<Root>/In42'
     *  RelationalOperator: '<S16>/GreaterThan'
     *  UnaryMinus: '<S16>/Unary Minus1'
     *  UnitDelay: '<S16>/Unit Delay'
     */
    rtb_VectorConcatenate_kusl[6] =
        (rtb_y || (VSDP_FalDlyTiCtrlStEnAPR_Sec > (-VSDPI_CycleTime_Sec)));

    /* Switch: '<S24>/Switch' incorporates:
     *  Constant: '<S4>/V_Parameter13'
     *  Inport: '<Root>/In42'
     *  Inport: '<Root>/In45'
     *  MinMax: '<S24>/Max'
     *  Sum: '<S24>/Subtract'
     *  Switch: '<S24>/Switch1'
     *  UnaryMinus: '<S24>/Unary Minus'
     *  UnitDelay: '<S24>/Unit Delay'
     */
    if (VSDPI_CtrlStEnHDC_B) {
        VSDP_RSDlyTiCtrlStEnHDC_Sec =
            fmaxf(VSDP_RSDlyTiCtrlStEnHDC_Sec, -VSDPI_CycleTime_Sec) -
            VSDPI_CycleTime_Sec;
    } else {
        VSDP_RSDlyTiCtrlStEnHDC_Sec = VSDP_RSDlyTimeHDCAct_sec;
    }

    /* End of Switch: '<S24>/Switch' */

    /* Logic: '<S24>/AND' incorporates:
     *  Inport: '<Root>/In42'
     *  Inport: '<Root>/In45'
     *  RelationalOperator: '<S24>/LessThanOrEqual'
     *  UnaryMinus: '<S24>/Unary Minus1'
     *  UnitDelay: '<S24>/Unit Delay'
     */
    rtb_y = (VSDPI_CtrlStEnHDC_B &&
             (VSDP_RSDlyTiCtrlStEnHDC_Sec <= (-VSDPI_CycleTime_Sec)));

    /* Switch: '<S17>/Switch' incorporates:
     *  Constant: '<S4>/V_Parameter12'
     *  Inport: '<Root>/In42'
     *  MinMax: '<S17>/Max'
     *  Sum: '<S17>/Subtract'
     *  Switch: '<S17>/Switch1'
     *  UnaryMinus: '<S17>/Unary Minus'
     *  UnitDelay: '<S17>/Unit Delay'
     */
    if (rtb_y) {
        VSDP_FalDlyTiCtrlStEnHDC_Sec = VSDP_FalDlyTiCtrlStEn_C_Sec;
    } else {
        VSDP_FalDlyTiCtrlStEnHDC_Sec =
            fmaxf(-VSDPI_CycleTime_Sec, VSDP_FalDlyTiCtrlStEnHDC_Sec) -
            VSDPI_CycleTime_Sec;
    }

    /* End of Switch: '<S17>/Switch' */

    /* Logic: '<S17>/OR' incorporates:
     *  Inport: '<Root>/In42'
     *  RelationalOperator: '<S17>/GreaterThan'
     *  UnaryMinus: '<S17>/Unary Minus1'
     *  UnitDelay: '<S17>/Unit Delay'
     */
    rtb_VectorConcatenate_kusl[7] =
        (rtb_y || (VSDP_FalDlyTiCtrlStEnHDC_Sec > (-VSDPI_CycleTime_Sec)));

    /* S-Function (ex_sfun_set_bit): '<S25>/ex_sfun_set_bit' */
    set_bit(VSDP_ConstB.DataTypeConversion,
            (boolean_T*)&rtb_VectorConcatenate_kusl[0],
            (uint8_T*)(&(VSDP_SetBit_BS_Param_1[0])), ((uint8_T)6U),
            &rtb_ex_sfun_set_bit);

    /* SignalConversion generated from: '<S1>/60ms' incorporates:
     *  DataTypeConversion: '<S18>/Data Type Conversion1'
     *  DataTypeConversion: '<S25>/Data Type Conversion1'
     */
    VSDP_CtrlStEn_St = (uint8_T)rtb_ex_sfun_set_bit;

    /* Logic: '<S3>/NOT' incorporates:
     *  Inport: '<Root>/In6'
     */
    rtb_VectorConcatenate[0] = !VSDPI_CtrlStAvlbABS_B;

    /* Logic: '<S3>/NOT1' incorporates:
     *  Inport: '<Root>/In7'
     */
    rtb_VectorConcatenate[1] = !VSDPI_CtrlStAvlbACC_B;

    /* Logic: '<S3>/NOT2' incorporates:
     *  Inport: '<Root>/In8'
     */
    rtb_VectorConcatenate[2] = !VSDPI_CtrlStAvlbESC_B;

    /* Logic: '<S3>/NOT3' incorporates:
     *  Inport: '<Root>/In9'
     */
    rtb_VectorConcatenate[3] = !VSDPI_CtrlStAvlbTCS_B;

    /* Logic: '<S3>/NOT4' incorporates:
     *  Inport: '<Root>/In10'
     */
    rtb_VectorConcatenate[4] = !VSDPI_CtrlStAvlbVSM_B;

    /* Logic: '<S3>/NOT5' incorporates:
     *  Inport: '<Root>/In11'
     */
    rtb_VectorConcatenate[5] = !VSDPI_CtrlStAvlbEBA_B;

    /* S-Function (ex_sfun_set_bit): '<S9>/ex_sfun_set_bit' */
    set_bit(VSDP_ConstB.DataTypeConversion_doo1,
            (boolean_T*)&rtb_VectorConcatenate[0],
            (uint8_T*)(&(VSDP_SetBit_BS_Param_1[0])), ((uint8_T)6U),
            &rtb_ex_sfun_set_bit);

    /* SignalConversion generated from: '<S1>/60ms' incorporates:
     *  DataTypeConversion: '<S8>/Data Type Conversion1'
     *  DataTypeConversion: '<S9>/Data Type Conversion1'
     */
    VSDP_CtrlStNoAvlb_St = (uint8_T)rtb_ex_sfun_set_bit;

    /* RelationalOperator: '<S5>/Great Than' incorporates:
     *  Constant: '<S5>/V_Parameter6'
     *  Inport: '<Root>/In12'
     *  Sum: '<S5>/Subtract'
     *  UnitDelay: '<S5>/Unit Delay'
     */
    rtb_y = ((VSDPI_AccPedPstn_Per - VSDP_UstpAccPedPstn_Per) >
             VSDP_UstpAccPedPstnMx_C_Per);

    /* Switch: '<S26>/Switch' incorporates:
     *  Constant: '<S5>/V_Parameter7'
     *  Inport: '<Root>/In42'
     *  MinMax: '<S26>/Max'
     *  Sum: '<S26>/Subtract'
     *  Switch: '<S26>/Switch1'
     *  UnaryMinus: '<S26>/Unary Minus'
     *  UnitDelay: '<S26>/Unit Delay'
     */
    if (rtb_y) {
        VSDP_FalDlyTiAccPedPstnRate_Sec = VSDP_FalDlyTiAccPedPstnRate_C_Sec;
    } else {
        VSDP_FalDlyTiAccPedPstnRate_Sec =
            fmaxf(-VSDPI_CycleTime_Sec, VSDP_FalDlyTiAccPedPstnRate_Sec) -
            VSDPI_CycleTime_Sec;
    }

    /* End of Switch: '<S26>/Switch' */

    /* Logic: '<S26>/OR' incorporates:
     *  Inport: '<Root>/In42'
     *  RelationalOperator: '<S26>/GreaterThan'
     *  UnaryMinus: '<S26>/Unary Minus1'
     *  UnitDelay: '<S26>/Unit Delay'
     */
    rtb_VectorConcatenate_mhyz[0] =
        (rtb_y || (VSDP_FalDlyTiAccPedPstnRate_Sec > (-VSDPI_CycleTime_Sec)));

    /* Switch: '<S27>/Switch' incorporates:
     *  Constant: '<S5>/V_Parameter8'
     *  Inport: '<Root>/In14'
     *  Inport: '<Root>/In42'
     *  MinMax: '<S27>/Max'
     *  Sum: '<S27>/Subtract'
     *  Switch: '<S27>/Switch1'
     *  UnaryMinus: '<S27>/Unary Minus'
     *  UnitDelay: '<S27>/Unit Delay'
     */
    if (VSDPI_TrnSglHarLigEn_B) {
        VSDP_FalDlyTiTrnSglHarLigEn_Sec = VSDP_FalDlyTiTrnSgl_C_Sec;
    } else {
        VSDP_FalDlyTiTrnSglHarLigEn_Sec =
            fmaxf(-VSDPI_CycleTime_Sec, VSDP_FalDlyTiTrnSglHarLigEn_Sec) -
            VSDPI_CycleTime_Sec;
    }

    /* End of Switch: '<S27>/Switch' */

    /* Logic: '<S27>/OR' incorporates:
     *  Inport: '<Root>/In14'
     *  Inport: '<Root>/In42'
     *  RelationalOperator: '<S27>/GreaterThan'
     *  UnaryMinus: '<S27>/Unary Minus1'
     *  UnitDelay: '<S27>/Unit Delay'
     */
    rtb_VectorConcatenate_mhyz[2] =
        (VSDPI_TrnSglHarLigEn_B ||
         (VSDP_FalDlyTiTrnSglHarLigEn_Sec > (-VSDPI_CycleTime_Sec)));

    /* Logic: '<S5>/OR' incorporates:
     *  Inport: '<Root>/In15'
     *  Inport: '<Root>/In16'
     */
    rtb_y = (VSDPI_TrnSglEnLf_B || VSDPI_TrnSglEnRi_B);

    /* Switch: '<S28>/Switch' incorporates:
     *  Constant: '<S5>/V_Parameter9'
     *  Inport: '<Root>/In42'
     *  MinMax: '<S28>/Max'
     *  Sum: '<S28>/Subtract'
     *  Switch: '<S28>/Switch1'
     *  UnaryMinus: '<S28>/Unary Minus'
     *  UnitDelay: '<S28>/Unit Delay'
     */
    if (rtb_y) {
        VSDP_FalDlyTiTrnSglEn_Sec = VSDP_FalDlyTiTrnSgl_C_Sec;
    } else {
        VSDP_FalDlyTiTrnSglEn_Sec =
            fmaxf(-VSDPI_CycleTime_Sec, VSDP_FalDlyTiTrnSglEn_Sec) -
            VSDPI_CycleTime_Sec;
    }

    /* End of Switch: '<S28>/Switch' */

    /* Logic: '<S28>/OR' incorporates:
     *  Inport: '<Root>/In42'
     *  RelationalOperator: '<S28>/GreaterThan'
     *  UnaryMinus: '<S28>/Unary Minus1'
     *  UnitDelay: '<S28>/Unit Delay'
     */
    rtb_VectorConcatenate_mhyz[3] =
        (rtb_y || (VSDP_FalDlyTiTrnSglEn_Sec > (-VSDPI_CycleTime_Sec)));

    /* RelationalOperator: '<S5>/Great Than1' incorporates:
     *  Constant: '<S5>/V_Parameter10'
     *  Inport: '<Root>/In17'
     */
    rtb_VectorConcatenate_mhyz[4] = (VSDPI_SysWarn_St == VSDP_DegTrig_C_St);

    /* SignalConversion generated from: '<S29>/Vector Concatenate' incorporates:
     *  Inport: '<Root>/In13'
     */
    rtb_VectorConcatenate_mhyz[1] = VSDPI_DrvNoBuckledUp_B;

    /* SignalConversion generated from: '<S29>/Vector Concatenate' incorporates:
     *  Inport: '<Root>/In43'
     */
    rtb_VectorConcatenate_mhyz[6] = VSDPI_BrakePedalApplied_B;

    /* RelationalOperator: '<S5>/Great Than2' incorporates:
     *  Abs: '<S5>/Abs'
     *  Constant: '<S5>/V_Parameter11'
     *  Inport: '<Root>/In18'
     */
    rtb_y = (fabsf(VSDPI_ManuActuTrqEPS_Nm) >= VSDP_ManuActuTrqMx_C_Nm);

    /* Switch: '<S30>/Switch' incorporates:
     *  Constant: '<S5>/V_Parameter39'
     *  Inport: '<Root>/In42'
     *  MinMax: '<S30>/Max'
     *  Sum: '<S30>/Subtract'
     *  Switch: '<S30>/Switch1'
     *  UnaryMinus: '<S30>/Unary Minus'
     *  UnitDelay: '<S30>/Unit Delay'
     */
    if (rtb_y) {
        VSDP_RisDlyTiManuActuTrq_Sec =
            fmaxf(VSDP_RisDlyTiManuActuTrq_Sec, -VSDPI_CycleTime_Sec) -
            VSDPI_CycleTime_Sec;
    } else {
        VSDP_RisDlyTiManuActuTrq_Sec = VSDP_RisDlyTiManuActuTrq_C_Sec;
    }

    /* End of Switch: '<S30>/Switch' */

    /* Logic: '<S30>/AND' incorporates:
     *  Inport: '<Root>/In42'
     *  RelationalOperator: '<S30>/LessThanOrEqual'
     *  UnaryMinus: '<S30>/Unary Minus1'
     *  UnitDelay: '<S30>/Unit Delay'
     */
    rtb_VectorConcatenate_mhyz[5] =
        (rtb_y && (VSDP_RisDlyTiManuActuTrq_Sec <= (-VSDPI_CycleTime_Sec)));

    /* S-Function (ex_sfun_set_bit): '<S31>/ex_sfun_set_bit' */
    set_bit(VSDP_ConstB.DataTypeConversion_lzpg,
            (boolean_T*)&rtb_VectorConcatenate_mhyz[0],
            (uint8_T*)(&(VSDP_SetBit_BS_Param_2[0])), ((uint8_T)7U),
            &rtb_ex_sfun_set_bit);

    /* SignalConversion generated from: '<S1>/60ms' incorporates:
     *  DataTypeConversion: '<S29>/Data Type Conversion1'
     *  DataTypeConversion: '<S31>/Data Type Conversion1'
     */
    VSDP_IvldStDrv_St = (uint8_T)rtb_ex_sfun_set_bit;

    /* Switch: '<S63>/Switch' incorporates:
     *  Constant: '<S6>/V_Parameter40'
     *  Inport: '<Root>/In19'
     *  Inport: '<Root>/In42'
     *  MinMax: '<S63>/Max'
     *  Sum: '<S63>/Subtract'
     *  Switch: '<S63>/Switch1'
     *  UnaryMinus: '<S63>/Unary Minus'
     *  UnitDelay: '<S63>/Unit Delay'
     */
    if (VSDPI_StErrABS_B) {
        VSDP_RisDlyTiStErrABS_Sec =
            fmaxf(VSDP_RisDlyTiStErrABS_Sec, -VSDPI_CycleTime_Sec) -
            VSDPI_CycleTime_Sec;
    } else {
        VSDP_RisDlyTiStErrABS_Sec = VSDP_RisDlyTiStErr_C_Sec;
    }

    /* End of Switch: '<S63>/Switch' */

    /* Logic: '<S63>/AND' incorporates:
     *  Inport: '<Root>/In19'
     *  Inport: '<Root>/In42'
     *  RelationalOperator: '<S63>/LessThanOrEqual'
     *  UnaryMinus: '<S63>/Unary Minus1'
     *  UnitDelay: '<S63>/Unit Delay'
     */
    rtb_y = (VSDPI_StErrABS_B &&
             (VSDP_RisDlyTiStErrABS_Sec <= (-VSDPI_CycleTime_Sec)));

    /* Switch: '<S73>/Switch2' incorporates:
     *  Constant: '<S6>/V_Parameter13'
     *  Inport: '<Root>/In42'
     *  Logic: '<S32>/AND'
     *  Logic: '<S32>/NOT'
     *  RelationalOperator: '<S73>/GreaterThan'
     *  Switch: '<S73>/Switch'
     *  UnitDelay: '<S32>/Unit Delay'
     *  UnitDelay: '<S73>/Unit Delay'
     */
    if (rtb_y && (!VSDP_RisEdgeStErrABS_B)) {
        VSDP_TiTrigStErrABS_Sec = VSDP_TiTrigStErr_C_Sec;
    } else if (VSDP_TiTrigStErrABS_Sec > VSDPI_CycleTime_Sec) {
        /* Switch: '<S73>/Switch' incorporates:
         *  Inport: '<Root>/In42'
         *  Sum: '<S73>/Subtract'
         *  UnitDelay: '<S73>/Unit Delay'
         */
        VSDP_TiTrigStErrABS_Sec -= VSDPI_CycleTime_Sec;
    } else {
        /* UnitDelay: '<S73>/Unit Delay' incorporates:
         *  Constant: '<S73>/Constant1'
         *  Switch: '<S73>/Switch'
         */
        VSDP_TiTrigStErrABS_Sec = 0.0F;
    }

    /* End of Switch: '<S73>/Switch2' */

    /* Switch: '<S42>/Switch' incorporates:
     *  Constant: '<S42>/Constant2'
     *  Constant: '<S73>/Constant2'
     *  Inport: '<Root>/In19'
     *  Logic: '<S6>/NOT6'
     *  RelationalOperator: '<S73>/GreaterThan1'
     *  UnitDelay: '<S42>/Unit Delay'
     *  UnitDelay: '<S73>/Unit Delay'
     */
    if (!VSDPI_StErrABS_B) {
        VSDP_FFStErrABS_B = false;
    } else {
        VSDP_FFStErrABS_B =
            ((VSDP_TiTrigStErrABS_Sec > 0.0F) || VSDP_FFStErrABS_B);
    }

    /* End of Switch: '<S42>/Switch' */

    /* Switch: '<S53>/Switch' incorporates:
     *  Constant: '<S6>/V_Parameter14'
     *  Inport: '<Root>/In42'
     *  MinMax: '<S53>/Max'
     *  Sum: '<S53>/Subtract'
     *  Switch: '<S53>/Switch1'
     *  UnaryMinus: '<S53>/Unary Minus'
     *  UnitDelay: '<S42>/Unit Delay'
     *  UnitDelay: '<S53>/Unit Delay'
     */
    if (VSDP_FFStErrABS_B) {
        VSDP_FalDlyTiStErrABS_C_Sec = VSDP_FalDlyTiStErr_C_Sec;
    } else {
        VSDP_FalDlyTiStErrABS_C_Sec =
            fmaxf(-VSDPI_CycleTime_Sec, VSDP_FalDlyTiStErrABS_C_Sec) -
            VSDPI_CycleTime_Sec;
    }

    /* End of Switch: '<S53>/Switch' */

    /* Logic: '<S53>/OR' incorporates:
     *  Inport: '<Root>/In42'
     *  RelationalOperator: '<S53>/GreaterThan'
     *  UnaryMinus: '<S53>/Unary Minus1'
     *  UnitDelay: '<S42>/Unit Delay'
     *  UnitDelay: '<S53>/Unit Delay'
     */
    rtb_VectorConcatenate_kusl[0] =
        (VSDP_FFStErrABS_B ||
         (VSDP_FalDlyTiStErrABS_C_Sec > (-VSDPI_CycleTime_Sec)));

    /* Switch: '<S66>/Switch' incorporates:
     *  Constant: '<S6>/V_Parameter15'
     *  Inport: '<Root>/In21'
     *  Inport: '<Root>/In42'
     *  MinMax: '<S66>/Max'
     *  Sum: '<S66>/Subtract'
     *  Switch: '<S66>/Switch1'
     *  UnaryMinus: '<S66>/Unary Minus'
     *  UnitDelay: '<S66>/Unit Delay'
     */
    if (VSDPI_StErrESC_B) {
        VSDP_RisDlyTiStErrESC_Sec =
            fmaxf(VSDP_RisDlyTiStErrESC_Sec, -VSDPI_CycleTime_Sec) -
            VSDPI_CycleTime_Sec;
    } else {
        VSDP_RisDlyTiStErrESC_Sec = VSDP_RisDlyTiStErr_C_Sec;
    }

    /* End of Switch: '<S66>/Switch' */

    /* Logic: '<S66>/AND' incorporates:
     *  Inport: '<Root>/In21'
     *  Inport: '<Root>/In42'
     *  RelationalOperator: '<S66>/LessThanOrEqual'
     *  UnaryMinus: '<S66>/Unary Minus1'
     *  UnitDelay: '<S66>/Unit Delay'
     */
    rtb_AND_ku13 = (VSDPI_StErrESC_B &&
                    (VSDP_RisDlyTiStErrESC_Sec <= (-VSDPI_CycleTime_Sec)));

    /* Switch: '<S75>/Switch2' incorporates:
     *  Constant: '<S6>/V_Parameter2'
     *  Inport: '<Root>/In42'
     *  Logic: '<S34>/AND'
     *  Logic: '<S34>/NOT'
     *  RelationalOperator: '<S75>/GreaterThan'
     *  Switch: '<S75>/Switch'
     *  UnitDelay: '<S34>/Unit Delay'
     *  UnitDelay: '<S75>/Unit Delay'
     */
    if (rtb_AND_ku13 && (!VSDP_RisEdgeStErrESC_B)) {
        VSDP_TiTrigStErrESC_Sec = VSDP_TiTrigStErr_C_Sec;
    } else if (VSDP_TiTrigStErrESC_Sec > VSDPI_CycleTime_Sec) {
        /* Switch: '<S75>/Switch' incorporates:
         *  Inport: '<Root>/In42'
         *  Sum: '<S75>/Subtract'
         *  UnitDelay: '<S75>/Unit Delay'
         */
        VSDP_TiTrigStErrESC_Sec -= VSDPI_CycleTime_Sec;
    } else {
        /* UnitDelay: '<S75>/Unit Delay' incorporates:
         *  Constant: '<S75>/Constant1'
         *  Switch: '<S75>/Switch'
         */
        VSDP_TiTrigStErrESC_Sec = 0.0F;
    }

    /* End of Switch: '<S75>/Switch2' */

    /* Switch: '<S44>/Switch' incorporates:
     *  Constant: '<S44>/Constant2'
     *  Constant: '<S75>/Constant2'
     *  Inport: '<Root>/In21'
     *  Logic: '<S6>/NOT8'
     *  RelationalOperator: '<S75>/GreaterThan1'
     *  UnitDelay: '<S44>/Unit Delay'
     *  UnitDelay: '<S75>/Unit Delay'
     */
    if (!VSDPI_StErrESC_B) {
        VSDP_FFStErrESC_B = false;
    } else {
        VSDP_FFStErrESC_B =
            ((VSDP_TiTrigStErrESC_Sec > 0.0F) || VSDP_FFStErrESC_B);
    }

    /* End of Switch: '<S44>/Switch' */

    /* Switch: '<S54>/Switch' incorporates:
     *  Constant: '<S6>/V_Parameter20'
     *  Inport: '<Root>/In42'
     *  MinMax: '<S54>/Max'
     *  Sum: '<S54>/Subtract'
     *  Switch: '<S54>/Switch1'
     *  UnaryMinus: '<S54>/Unary Minus'
     *  UnitDelay: '<S44>/Unit Delay'
     *  UnitDelay: '<S54>/Unit Delay'
     */
    if (VSDP_FFStErrESC_B) {
        VSDP_FalDlyTiStErrESC_C_Sec = VSDP_FalDlyTiStErr_C_Sec;
    } else {
        VSDP_FalDlyTiStErrESC_C_Sec =
            fmaxf(-VSDPI_CycleTime_Sec, VSDP_FalDlyTiStErrESC_C_Sec) -
            VSDPI_CycleTime_Sec;
    }

    /* End of Switch: '<S54>/Switch' */

    /* Logic: '<S54>/OR' incorporates:
     *  Inport: '<Root>/In42'
     *  RelationalOperator: '<S54>/GreaterThan'
     *  UnaryMinus: '<S54>/Unary Minus1'
     *  UnitDelay: '<S44>/Unit Delay'
     *  UnitDelay: '<S54>/Unit Delay'
     */
    rtb_VectorConcatenate_kusl[2] =
        (VSDP_FFStErrESC_B ||
         (VSDP_FalDlyTiStErrESC_C_Sec > (-VSDPI_CycleTime_Sec)));

    /* Switch: '<S67>/Switch' incorporates:
     *  Constant: '<S6>/V_Parameter18'
     *  Inport: '<Root>/In22'
     *  Inport: '<Root>/In42'
     *  MinMax: '<S67>/Max'
     *  Sum: '<S67>/Subtract'
     *  Switch: '<S67>/Switch1'
     *  UnaryMinus: '<S67>/Unary Minus'
     *  UnitDelay: '<S67>/Unit Delay'
     */
    if (VSDPI_StErrTSC_B) {
        VSDP_RisDlyTiStErrTSC_Sec =
            fmaxf(VSDP_RisDlyTiStErrTSC_Sec, -VSDPI_CycleTime_Sec) -
            VSDPI_CycleTime_Sec;
    } else {
        VSDP_RisDlyTiStErrTSC_Sec = VSDP_RisDlyTiStErr_C_Sec;
    }

    /* End of Switch: '<S67>/Switch' */

    /* Logic: '<S67>/AND' incorporates:
     *  Inport: '<Root>/In22'
     *  Inport: '<Root>/In42'
     *  RelationalOperator: '<S67>/LessThanOrEqual'
     *  UnaryMinus: '<S67>/Unary Minus1'
     *  UnitDelay: '<S67>/Unit Delay'
     */
    rtb_AND_mai0 = (VSDPI_StErrTSC_B &&
                    (VSDP_RisDlyTiStErrTSC_Sec <= (-VSDPI_CycleTime_Sec)));

    /* Switch: '<S76>/Switch2' incorporates:
     *  Constant: '<S6>/V_Parameter3'
     *  Inport: '<Root>/In42'
     *  Logic: '<S35>/AND'
     *  Logic: '<S35>/NOT'
     *  RelationalOperator: '<S76>/GreaterThan'
     *  Switch: '<S76>/Switch'
     *  UnitDelay: '<S35>/Unit Delay'
     *  UnitDelay: '<S76>/Unit Delay'
     */
    if (rtb_AND_mai0 && (!VSDP_RisEdgeStErrTSC_B)) {
        VSDP_TiTrigStErrTSC_Sec = VSDP_TiTrigStErr_C_Sec;
    } else if (VSDP_TiTrigStErrTSC_Sec > VSDPI_CycleTime_Sec) {
        /* Switch: '<S76>/Switch' incorporates:
         *  Inport: '<Root>/In42'
         *  Sum: '<S76>/Subtract'
         *  UnitDelay: '<S76>/Unit Delay'
         */
        VSDP_TiTrigStErrTSC_Sec -= VSDPI_CycleTime_Sec;
    } else {
        /* UnitDelay: '<S76>/Unit Delay' incorporates:
         *  Constant: '<S76>/Constant1'
         *  Switch: '<S76>/Switch'
         */
        VSDP_TiTrigStErrTSC_Sec = 0.0F;
    }

    /* End of Switch: '<S76>/Switch2' */

    /* Switch: '<S45>/Switch' incorporates:
     *  Constant: '<S45>/Constant2'
     *  Constant: '<S76>/Constant2'
     *  Inport: '<Root>/In22'
     *  Logic: '<S6>/NOT9'
     *  RelationalOperator: '<S76>/GreaterThan1'
     *  UnitDelay: '<S45>/Unit Delay'
     *  UnitDelay: '<S76>/Unit Delay'
     */
    if (!VSDPI_StErrTSC_B) {
        VSDP_FFStErrTSC_B = false;
    } else {
        VSDP_FFStErrTSC_B =
            ((VSDP_TiTrigStErrTSC_Sec > 0.0F) || VSDP_FFStErrTSC_B);
    }

    /* End of Switch: '<S45>/Switch' */

    /* Switch: '<S55>/Switch' incorporates:
     *  Constant: '<S6>/V_Parameter22'
     *  Inport: '<Root>/In42'
     *  MinMax: '<S55>/Max'
     *  Sum: '<S55>/Subtract'
     *  Switch: '<S55>/Switch1'
     *  UnaryMinus: '<S55>/Unary Minus'
     *  UnitDelay: '<S45>/Unit Delay'
     *  UnitDelay: '<S55>/Unit Delay'
     */
    if (VSDP_FFStErrTSC_B) {
        VSDP_FalDlyTiStErrTSC_C_Sec = VSDP_FalDlyTiStErr_C_Sec;
    } else {
        VSDP_FalDlyTiStErrTSC_C_Sec =
            fmaxf(-VSDPI_CycleTime_Sec, VSDP_FalDlyTiStErrTSC_C_Sec) -
            VSDPI_CycleTime_Sec;
    }

    /* End of Switch: '<S55>/Switch' */

    /* Logic: '<S55>/OR' incorporates:
     *  Inport: '<Root>/In42'
     *  RelationalOperator: '<S55>/GreaterThan'
     *  UnaryMinus: '<S55>/Unary Minus1'
     *  UnitDelay: '<S45>/Unit Delay'
     *  UnitDelay: '<S55>/Unit Delay'
     */
    rtb_VectorConcatenate_kusl[3] =
        (VSDP_FFStErrTSC_B ||
         (VSDP_FalDlyTiStErrTSC_C_Sec > (-VSDPI_CycleTime_Sec)));

    /* Switch: '<S68>/Switch' incorporates:
     *  Constant: '<S6>/V_Parameter23'
     *  Inport: '<Root>/In23'
     *  Inport: '<Root>/In42'
     *  MinMax: '<S68>/Max'
     *  Sum: '<S68>/Subtract'
     *  Switch: '<S68>/Switch1'
     *  UnaryMinus: '<S68>/Unary Minus'
     *  UnitDelay: '<S68>/Unit Delay'
     */
    if (VSDPI_StErrVSM_B) {
        VSDP_RisDlyTiStErrVSM_Sec =
            fmaxf(VSDP_RisDlyTiStErrVSM_Sec, -VSDPI_CycleTime_Sec) -
            VSDPI_CycleTime_Sec;
    } else {
        VSDP_RisDlyTiStErrVSM_Sec = VSDP_RisDlyTiStErr_C_Sec;
    }

    /* End of Switch: '<S68>/Switch' */

    /* Logic: '<S68>/AND' incorporates:
     *  Inport: '<Root>/In23'
     *  Inport: '<Root>/In42'
     *  RelationalOperator: '<S68>/LessThanOrEqual'
     *  UnaryMinus: '<S68>/Unary Minus1'
     *  UnitDelay: '<S68>/Unit Delay'
     */
    rtb_AND_hz4i = (VSDPI_StErrVSM_B &&
                    (VSDP_RisDlyTiStErrVSM_Sec <= (-VSDPI_CycleTime_Sec)));

    /* Switch: '<S77>/Switch2' incorporates:
     *  Constant: '<S6>/V_Parameter4'
     *  Inport: '<Root>/In42'
     *  Logic: '<S36>/AND'
     *  Logic: '<S36>/NOT'
     *  RelationalOperator: '<S77>/GreaterThan'
     *  Switch: '<S77>/Switch'
     *  UnitDelay: '<S36>/Unit Delay'
     *  UnitDelay: '<S77>/Unit Delay'
     */
    if (rtb_AND_hz4i && (!VSDP_RisEdgeStErrVSM_B)) {
        VSDP_TiTrigStErrVSM_Sec = VSDP_TiTrigStErr_C_Sec;
    } else if (VSDP_TiTrigStErrVSM_Sec > VSDPI_CycleTime_Sec) {
        /* Switch: '<S77>/Switch' incorporates:
         *  Inport: '<Root>/In42'
         *  Sum: '<S77>/Subtract'
         *  UnitDelay: '<S77>/Unit Delay'
         */
        VSDP_TiTrigStErrVSM_Sec -= VSDPI_CycleTime_Sec;
    } else {
        /* UnitDelay: '<S77>/Unit Delay' incorporates:
         *  Constant: '<S77>/Constant1'
         *  Switch: '<S77>/Switch'
         */
        VSDP_TiTrigStErrVSM_Sec = 0.0F;
    }

    /* End of Switch: '<S77>/Switch2' */

    /* Switch: '<S46>/Switch' incorporates:
     *  Constant: '<S46>/Constant2'
     *  Constant: '<S77>/Constant2'
     *  Inport: '<Root>/In23'
     *  Logic: '<S6>/NOT10'
     *  RelationalOperator: '<S77>/GreaterThan1'
     *  UnitDelay: '<S46>/Unit Delay'
     *  UnitDelay: '<S77>/Unit Delay'
     */
    if (!VSDPI_StErrVSM_B) {
        VSDP_FFStErrVSM_B = false;
    } else {
        VSDP_FFStErrVSM_B =
            ((VSDP_TiTrigStErrVSM_Sec > 0.0F) || VSDP_FFStErrVSM_B);
    }

    /* End of Switch: '<S46>/Switch' */

    /* Switch: '<S56>/Switch' incorporates:
     *  Constant: '<S6>/V_Parameter25'
     *  Inport: '<Root>/In42'
     *  MinMax: '<S56>/Max'
     *  Sum: '<S56>/Subtract'
     *  Switch: '<S56>/Switch1'
     *  UnaryMinus: '<S56>/Unary Minus'
     *  UnitDelay: '<S46>/Unit Delay'
     *  UnitDelay: '<S56>/Unit Delay'
     */
    if (VSDP_FFStErrVSM_B) {
        VSDP_FalDlyTiStErrVSM_C_Sec = VSDP_FalDlyTiStErr_C_Sec;
    } else {
        VSDP_FalDlyTiStErrVSM_C_Sec =
            fmaxf(-VSDPI_CycleTime_Sec, VSDP_FalDlyTiStErrVSM_C_Sec) -
            VSDPI_CycleTime_Sec;
    }

    /* End of Switch: '<S56>/Switch' */

    /* Logic: '<S56>/OR' incorporates:
     *  Inport: '<Root>/In42'
     *  RelationalOperator: '<S56>/GreaterThan'
     *  UnaryMinus: '<S56>/Unary Minus1'
     *  UnitDelay: '<S46>/Unit Delay'
     *  UnitDelay: '<S56>/Unit Delay'
     */
    rtb_VectorConcatenate_kusl[4] =
        (VSDP_FFStErrVSM_B ||
         (VSDP_FalDlyTiStErrVSM_C_Sec > (-VSDPI_CycleTime_Sec)));

    /* Switch: '<S69>/Switch' incorporates:
     *  Constant: '<S6>/V_Parameter26'
     *  Inport: '<Root>/In24'
     *  Inport: '<Root>/In42'
     *  MinMax: '<S69>/Max'
     *  Sum: '<S69>/Subtract'
     *  Switch: '<S69>/Switch1'
     *  UnaryMinus: '<S69>/Unary Minus'
     *  UnitDelay: '<S69>/Unit Delay'
     */
    if (VSDPI_StErrVDY_B) {
        VSDP_RisDlyTiStErrVDY_Sec =
            fmaxf(VSDP_RisDlyTiStErrVDY_Sec, -VSDPI_CycleTime_Sec) -
            VSDPI_CycleTime_Sec;
    } else {
        VSDP_RisDlyTiStErrVDY_Sec = VSDP_RisDlyTiStErr_C_Sec;
    }

    /* End of Switch: '<S69>/Switch' */

    /* Logic: '<S69>/AND' incorporates:
     *  Inport: '<Root>/In24'
     *  Inport: '<Root>/In42'
     *  RelationalOperator: '<S69>/LessThanOrEqual'
     *  UnaryMinus: '<S69>/Unary Minus1'
     *  UnitDelay: '<S69>/Unit Delay'
     */
    rtb_AND_kbzm = (VSDPI_StErrVDY_B &&
                    (VSDP_RisDlyTiStErrVDY_Sec <= (-VSDPI_CycleTime_Sec)));

    /* Switch: '<S78>/Switch2' incorporates:
     *  Constant: '<S6>/V_Parameter5'
     *  Inport: '<Root>/In42'
     *  Logic: '<S37>/AND'
     *  Logic: '<S37>/NOT'
     *  RelationalOperator: '<S78>/GreaterThan'
     *  Switch: '<S78>/Switch'
     *  UnitDelay: '<S37>/Unit Delay'
     *  UnitDelay: '<S78>/Unit Delay'
     */
    if (rtb_AND_kbzm && (!VSDP_RisEdgeStErrVDY_B)) {
        VSDP_TiTrigStErrVDY_Sec = VSDP_TiTrigStErr_C_Sec;
    } else if (VSDP_TiTrigStErrVDY_Sec > VSDPI_CycleTime_Sec) {
        /* Switch: '<S78>/Switch' incorporates:
         *  Inport: '<Root>/In42'
         *  Sum: '<S78>/Subtract'
         *  UnitDelay: '<S78>/Unit Delay'
         */
        VSDP_TiTrigStErrVDY_Sec -= VSDPI_CycleTime_Sec;
    } else {
        /* UnitDelay: '<S78>/Unit Delay' incorporates:
         *  Constant: '<S78>/Constant1'
         *  Switch: '<S78>/Switch'
         */
        VSDP_TiTrigStErrVDY_Sec = 0.0F;
    }

    /* End of Switch: '<S78>/Switch2' */

    /* Switch: '<S47>/Switch' incorporates:
     *  Constant: '<S47>/Constant2'
     *  Constant: '<S78>/Constant2'
     *  Inport: '<Root>/In24'
     *  Logic: '<S6>/NOT11'
     *  RelationalOperator: '<S78>/GreaterThan1'
     *  UnitDelay: '<S47>/Unit Delay'
     *  UnitDelay: '<S78>/Unit Delay'
     */
    if (!VSDPI_StErrVDY_B) {
        VSDP_FFStErrVDY_B = false;
    } else {
        VSDP_FFStErrVDY_B =
            ((VSDP_TiTrigStErrVDY_Sec > 0.0F) || VSDP_FFStErrVDY_B);
    }

    /* End of Switch: '<S47>/Switch' */

    /* Switch: '<S57>/Switch' incorporates:
     *  Constant: '<S6>/V_Parameter28'
     *  Inport: '<Root>/In42'
     *  MinMax: '<S57>/Max'
     *  Sum: '<S57>/Subtract'
     *  Switch: '<S57>/Switch1'
     *  UnaryMinus: '<S57>/Unary Minus'
     *  UnitDelay: '<S47>/Unit Delay'
     *  UnitDelay: '<S57>/Unit Delay'
     */
    if (VSDP_FFStErrVDY_B) {
        VSDP_FalDlyTiStErrVDY_C_Sec = VSDP_FalDlyTiStErr_C_Sec;
    } else {
        VSDP_FalDlyTiStErrVDY_C_Sec =
            fmaxf(-VSDPI_CycleTime_Sec, VSDP_FalDlyTiStErrVDY_C_Sec) -
            VSDPI_CycleTime_Sec;
    }

    /* End of Switch: '<S57>/Switch' */

    /* Logic: '<S57>/OR' incorporates:
     *  Inport: '<Root>/In42'
     *  RelationalOperator: '<S57>/GreaterThan'
     *  UnaryMinus: '<S57>/Unary Minus1'
     *  UnitDelay: '<S47>/Unit Delay'
     *  UnitDelay: '<S57>/Unit Delay'
     */
    rtb_VectorConcatenate_kusl[5] =
        (VSDP_FFStErrVDY_B ||
         (VSDP_FalDlyTiStErrVDY_C_Sec > (-VSDPI_CycleTime_Sec)));

    /* Switch: '<S70>/Switch' incorporates:
     *  Constant: '<S6>/V_Parameter29'
     *  Inport: '<Root>/In25'
     *  Inport: '<Root>/In42'
     *  MinMax: '<S70>/Max'
     *  Sum: '<S70>/Subtract'
     *  Switch: '<S70>/Switch1'
     *  UnaryMinus: '<S70>/Unary Minus'
     *  UnitDelay: '<S70>/Unit Delay'
     */
    if (VSDPI_StErrLatDMC_B) {
        VSDP_RisDlyTiStErrLatDMC_Sec =
            fmaxf(VSDP_RisDlyTiStErrLatDMC_Sec, -VSDPI_CycleTime_Sec) -
            VSDPI_CycleTime_Sec;
    } else {
        VSDP_RisDlyTiStErrLatDMC_Sec = VSDP_RisDlyTiStErr_C_Sec;
    }

    /* End of Switch: '<S70>/Switch' */

    /* Logic: '<S70>/AND' incorporates:
     *  Inport: '<Root>/In25'
     *  Inport: '<Root>/In42'
     *  RelationalOperator: '<S70>/LessThanOrEqual'
     *  UnaryMinus: '<S70>/Unary Minus1'
     *  UnitDelay: '<S70>/Unit Delay'
     */
    rtb_AND_g15l = (VSDPI_StErrLatDMC_B &&
                    (VSDP_RisDlyTiStErrLatDMC_Sec <= (-VSDPI_CycleTime_Sec)));

    /* Switch: '<S79>/Switch2' incorporates:
     *  Constant: '<S6>/V_Parameter6'
     *  Inport: '<Root>/In42'
     *  Logic: '<S38>/AND'
     *  Logic: '<S38>/NOT'
     *  RelationalOperator: '<S79>/GreaterThan'
     *  Switch: '<S79>/Switch'
     *  UnitDelay: '<S38>/Unit Delay'
     *  UnitDelay: '<S79>/Unit Delay'
     */
    if (rtb_AND_g15l && (!VSDP_RisEdgeStErrLatDMC_B)) {
        VSDP_TiTrigStErrLatDMC_Sec = VSDP_TiTrigStErr_C_Sec;
    } else if (VSDP_TiTrigStErrLatDMC_Sec > VSDPI_CycleTime_Sec) {
        /* Switch: '<S79>/Switch' incorporates:
         *  Inport: '<Root>/In42'
         *  Sum: '<S79>/Subtract'
         *  UnitDelay: '<S79>/Unit Delay'
         */
        VSDP_TiTrigStErrLatDMC_Sec -= VSDPI_CycleTime_Sec;
    } else {
        /* UnitDelay: '<S79>/Unit Delay' incorporates:
         *  Constant: '<S79>/Constant1'
         *  Switch: '<S79>/Switch'
         */
        VSDP_TiTrigStErrLatDMC_Sec = 0.0F;
    }

    /* End of Switch: '<S79>/Switch2' */

    /* Switch: '<S48>/Switch' incorporates:
     *  Constant: '<S48>/Constant2'
     *  Constant: '<S79>/Constant2'
     *  Inport: '<Root>/In25'
     *  Logic: '<S6>/NOT12'
     *  RelationalOperator: '<S79>/GreaterThan1'
     *  UnitDelay: '<S48>/Unit Delay'
     *  UnitDelay: '<S79>/Unit Delay'
     */
    if (!VSDPI_StErrLatDMC_B) {
        VSDP_FFStErrLatDMC_B = false;
    } else {
        VSDP_FFStErrLatDMC_B =
            ((VSDP_TiTrigStErrLatDMC_Sec > 0.0F) || VSDP_FFStErrLatDMC_B);
    }

    /* End of Switch: '<S48>/Switch' */

    /* Switch: '<S58>/Switch' incorporates:
     *  Constant: '<S6>/V_Parameter31'
     *  Inport: '<Root>/In42'
     *  MinMax: '<S58>/Max'
     *  Sum: '<S58>/Subtract'
     *  Switch: '<S58>/Switch1'
     *  UnaryMinus: '<S58>/Unary Minus'
     *  UnitDelay: '<S48>/Unit Delay'
     *  UnitDelay: '<S58>/Unit Delay'
     */
    if (VSDP_FFStErrLatDMC_B) {
        VSDP_FalDlyTiStErrLatDMC_C_Sec = VSDP_FalDlyTiStErr_C_Sec;
    } else {
        VSDP_FalDlyTiStErrLatDMC_C_Sec =
            fmaxf(-VSDPI_CycleTime_Sec, VSDP_FalDlyTiStErrLatDMC_C_Sec) -
            VSDPI_CycleTime_Sec;
    }

    /* End of Switch: '<S58>/Switch' */

    /* Logic: '<S58>/OR' incorporates:
     *  Inport: '<Root>/In42'
     *  RelationalOperator: '<S58>/GreaterThan'
     *  UnaryMinus: '<S58>/Unary Minus1'
     *  UnitDelay: '<S48>/Unit Delay'
     *  UnitDelay: '<S58>/Unit Delay'
     */
    rtb_VectorConcatenate_kusl[6] =
        (VSDP_FFStErrLatDMC_B ||
         (VSDP_FalDlyTiStErrLatDMC_C_Sec > (-VSDPI_CycleTime_Sec)));

    /* Switch: '<S65>/Switch' incorporates:
     *  Constant: '<S6>/V_Parameter12'
     *  Inport: '<Root>/In20'
     *  Inport: '<Root>/In42'
     *  MinMax: '<S65>/Max'
     *  Sum: '<S65>/Subtract'
     *  Switch: '<S65>/Switch1'
     *  UnaryMinus: '<S65>/Unary Minus'
     *  UnitDelay: '<S65>/Unit Delay'
     */
    if (VSDPI_StErrACC_B) {
        VSDP_RisDlyTiStErrACC_Sec =
            fmaxf(VSDP_RisDlyTiStErrACC_Sec, -VSDPI_CycleTime_Sec) -
            VSDPI_CycleTime_Sec;
    } else {
        VSDP_RisDlyTiStErrACC_Sec = VSDP_RisDlyTiStErr_C_Sec;
    }

    /* End of Switch: '<S65>/Switch' */

    /* Logic: '<S65>/AND' incorporates:
     *  Inport: '<Root>/In20'
     *  Inport: '<Root>/In42'
     *  RelationalOperator: '<S65>/LessThanOrEqual'
     *  UnaryMinus: '<S65>/Unary Minus1'
     *  UnitDelay: '<S65>/Unit Delay'
     */
    rtb_AND_med5 = (VSDPI_StErrACC_B &&
                    (VSDP_RisDlyTiStErrACC_Sec <= (-VSDPI_CycleTime_Sec)));

    /* Switch: '<S74>/Switch2' incorporates:
     *  Constant: '<S6>/V_Parameter1'
     *  Inport: '<Root>/In42'
     *  Logic: '<S33>/AND'
     *  Logic: '<S33>/NOT'
     *  RelationalOperator: '<S74>/GreaterThan'
     *  Switch: '<S74>/Switch'
     *  UnitDelay: '<S33>/Unit Delay'
     *  UnitDelay: '<S74>/Unit Delay'
     */
    if (rtb_AND_med5 && (!VSDP_RisEdgeStErrACC_B)) {
        VSDP_TiTrigStErrACC_Sec = VSDP_TiTrigStErr_C_Sec;
    } else if (VSDP_TiTrigStErrACC_Sec > VSDPI_CycleTime_Sec) {
        /* Switch: '<S74>/Switch' incorporates:
         *  Inport: '<Root>/In42'
         *  Sum: '<S74>/Subtract'
         *  UnitDelay: '<S74>/Unit Delay'
         */
        VSDP_TiTrigStErrACC_Sec -= VSDPI_CycleTime_Sec;
    } else {
        /* UnitDelay: '<S74>/Unit Delay' incorporates:
         *  Constant: '<S74>/Constant1'
         *  Switch: '<S74>/Switch'
         */
        VSDP_TiTrigStErrACC_Sec = 0.0F;
    }

    /* End of Switch: '<S74>/Switch2' */

    /* Switch: '<S43>/Switch' incorporates:
     *  Constant: '<S43>/Constant2'
     *  Constant: '<S74>/Constant2'
     *  Inport: '<Root>/In20'
     *  Logic: '<S6>/NOT7'
     *  RelationalOperator: '<S74>/GreaterThan1'
     *  UnitDelay: '<S43>/Unit Delay'
     *  UnitDelay: '<S74>/Unit Delay'
     */
    if (!VSDPI_StErrACC_B) {
        VSDP_FFStErrACC_B = false;
    } else {
        VSDP_FFStErrACC_B =
            ((VSDP_TiTrigStErrACC_Sec > 0.0F) || VSDP_FFStErrACC_B);
    }

    /* End of Switch: '<S43>/Switch' */

    /* Switch: '<S61>/Switch' incorporates:
     *  Constant: '<S6>/V_Parameter17'
     *  Inport: '<Root>/In42'
     *  MinMax: '<S61>/Max'
     *  Sum: '<S61>/Subtract'
     *  Switch: '<S61>/Switch1'
     *  UnaryMinus: '<S61>/Unary Minus'
     *  UnitDelay: '<S43>/Unit Delay'
     *  UnitDelay: '<S61>/Unit Delay'
     */
    if (VSDP_FFStErrACC_B) {
        VSDP_FalDlyTiStErrACC_C_Sec = VSDP_FalDlyTiStErr_C_Sec;
    } else {
        VSDP_FalDlyTiStErrACC_C_Sec =
            fmaxf(-VSDPI_CycleTime_Sec, VSDP_FalDlyTiStErrACC_C_Sec) -
            VSDPI_CycleTime_Sec;
    }

    /* End of Switch: '<S61>/Switch' */

    /* Logic: '<S61>/OR' incorporates:
     *  Inport: '<Root>/In42'
     *  RelationalOperator: '<S61>/GreaterThan'
     *  UnaryMinus: '<S61>/Unary Minus1'
     *  UnitDelay: '<S43>/Unit Delay'
     *  UnitDelay: '<S61>/Unit Delay'
     */
    rtb_VectorConcatenate_kusl[1] =
        (VSDP_FFStErrACC_B ||
         (VSDP_FalDlyTiStErrACC_C_Sec > (-VSDPI_CycleTime_Sec)));

    /* Switch: '<S72>/Switch' incorporates:
     *  Constant: '<S6>/V_Parameter8'
     *  Inport: '<Root>/In42'
     *  Inport: '<Root>/In46'
     *  MinMax: '<S72>/Max'
     *  Sum: '<S72>/Subtract'
     *  Switch: '<S72>/Switch1'
     *  UnaryMinus: '<S72>/Unary Minus'
     *  UnitDelay: '<S72>/Unit Delay'
     */
    if (VSDPI_StErrARP_B) {
        VSDP_RisDlyTiStErrARP_Sec =
            fmaxf(VSDP_RisDlyTiStErrARP_Sec, -VSDPI_CycleTime_Sec) -
            VSDPI_CycleTime_Sec;
    } else {
        VSDP_RisDlyTiStErrARP_Sec = VSDP_RisDlyTiStErr_C_Sec;
    }

    /* End of Switch: '<S72>/Switch' */

    /* Logic: '<S72>/AND' incorporates:
     *  Inport: '<Root>/In42'
     *  Inport: '<Root>/In46'
     *  RelationalOperator: '<S72>/LessThanOrEqual'
     *  UnaryMinus: '<S72>/Unary Minus1'
     *  UnitDelay: '<S72>/Unit Delay'
     */
    rtb_AND_cvp0 = (VSDPI_StErrARP_B &&
                    (VSDP_RisDlyTiStErrARP_Sec <= (-VSDPI_CycleTime_Sec)));

    /* Switch: '<S81>/Switch2' incorporates:
     *  Constant: '<S6>/V_Parameter10'
     *  Inport: '<Root>/In42'
     *  Logic: '<S40>/AND'
     *  Logic: '<S40>/NOT'
     *  RelationalOperator: '<S81>/GreaterThan'
     *  Switch: '<S81>/Switch'
     *  UnitDelay: '<S40>/Unit Delay'
     *  UnitDelay: '<S81>/Unit Delay'
     */
    if (rtb_AND_cvp0 && (!VSDP_RisEdgeStErrARP_B)) {
        VSDP_TiTrigStErrARP_Sec = VSDP_TiTrigStErr_C_Sec;
    } else if (VSDP_TiTrigStErrARP_Sec > VSDPI_CycleTime_Sec) {
        /* Switch: '<S81>/Switch' incorporates:
         *  Inport: '<Root>/In42'
         *  Sum: '<S81>/Subtract'
         *  UnitDelay: '<S81>/Unit Delay'
         */
        VSDP_TiTrigStErrARP_Sec -= VSDPI_CycleTime_Sec;
    } else {
        /* UnitDelay: '<S81>/Unit Delay' incorporates:
         *  Constant: '<S81>/Constant1'
         *  Switch: '<S81>/Switch'
         */
        VSDP_TiTrigStErrARP_Sec = 0.0F;
    }

    /* End of Switch: '<S81>/Switch2' */

    /* Switch: '<S50>/Switch' incorporates:
     *  Constant: '<S50>/Constant2'
     *  Constant: '<S81>/Constant2'
     *  Inport: '<Root>/In46'
     *  Logic: '<S6>/NOT1'
     *  RelationalOperator: '<S81>/GreaterThan1'
     *  UnitDelay: '<S50>/Unit Delay'
     *  UnitDelay: '<S81>/Unit Delay'
     */
    if (!VSDPI_StErrARP_B) {
        VSDP_FFStErrARP_B = false;
    } else {
        VSDP_FFStErrARP_B =
            ((VSDP_TiTrigStErrARP_Sec > 0.0F) || VSDP_FFStErrARP_B);
    }

    /* End of Switch: '<S50>/Switch' */

    /* Switch: '<S52>/Switch' incorporates:
     *  Constant: '<S6>/V_Parameter9'
     *  Inport: '<Root>/In42'
     *  MinMax: '<S52>/Max'
     *  Sum: '<S52>/Subtract'
     *  Switch: '<S52>/Switch1'
     *  UnaryMinus: '<S52>/Unary Minus'
     *  UnitDelay: '<S50>/Unit Delay'
     *  UnitDelay: '<S52>/Unit Delay'
     */
    if (VSDP_FFStErrARP_B) {
        VSDP_FalDlyTiStErrARP_C_Sec = VSDP_FalDlyTiStErr_C_Sec;
    } else {
        VSDP_FalDlyTiStErrARP_C_Sec =
            fmaxf(-VSDPI_CycleTime_Sec, VSDP_FalDlyTiStErrARP_C_Sec) -
            VSDPI_CycleTime_Sec;
    }

    /* End of Switch: '<S52>/Switch' */

    /* Switch: '<S71>/Switch' incorporates:
     *  Constant: '<S6>/V_Parameter32'
     *  Inport: '<Root>/In26'
     *  Inport: '<Root>/In42'
     *  MinMax: '<S71>/Max'
     *  Sum: '<S71>/Subtract'
     *  Switch: '<S71>/Switch1'
     *  UnaryMinus: '<S71>/Unary Minus'
     *  UnitDelay: '<S71>/Unit Delay'
     */
    if (VSDPI_StErrEBA_B) {
        VSDP_RisDlyTiStErrEBA_Sec =
            fmaxf(VSDP_RisDlyTiStErrEBA_Sec, -VSDPI_CycleTime_Sec) -
            VSDPI_CycleTime_Sec;
    } else {
        VSDP_RisDlyTiStErrEBA_Sec = VSDP_RisDlyTiStErr_C_Sec;
    }

    /* End of Switch: '<S71>/Switch' */

    /* Logic: '<S71>/AND' incorporates:
     *  Inport: '<Root>/In26'
     *  Inport: '<Root>/In42'
     *  RelationalOperator: '<S71>/LessThanOrEqual'
     *  UnaryMinus: '<S71>/Unary Minus1'
     *  UnitDelay: '<S71>/Unit Delay'
     */
    rtb_AND_o5jf = (VSDPI_StErrEBA_B &&
                    (VSDP_RisDlyTiStErrEBA_Sec <= (-VSDPI_CycleTime_Sec)));

    /* Switch: '<S80>/Switch2' incorporates:
     *  Constant: '<S6>/V_Parameter7'
     *  Inport: '<Root>/In42'
     *  Logic: '<S39>/AND'
     *  Logic: '<S39>/NOT'
     *  RelationalOperator: '<S80>/GreaterThan'
     *  Switch: '<S80>/Switch'
     *  UnitDelay: '<S39>/Unit Delay'
     *  UnitDelay: '<S80>/Unit Delay'
     */
    if (rtb_AND_o5jf && (!VSDP_RisEdgeStErrEBA_B)) {
        VSDP_TiTrigStErrEBA_Sec = VSDP_TiTrigStErr_C_Sec;
    } else if (VSDP_TiTrigStErrEBA_Sec > VSDPI_CycleTime_Sec) {
        /* Switch: '<S80>/Switch' incorporates:
         *  Inport: '<Root>/In42'
         *  Sum: '<S80>/Subtract'
         *  UnitDelay: '<S80>/Unit Delay'
         */
        VSDP_TiTrigStErrEBA_Sec -= VSDPI_CycleTime_Sec;
    } else {
        /* UnitDelay: '<S80>/Unit Delay' incorporates:
         *  Constant: '<S80>/Constant1'
         *  Switch: '<S80>/Switch'
         */
        VSDP_TiTrigStErrEBA_Sec = 0.0F;
    }

    /* End of Switch: '<S80>/Switch2' */

    /* Switch: '<S49>/Switch' incorporates:
     *  Constant: '<S49>/Constant2'
     *  Constant: '<S80>/Constant2'
     *  Inport: '<Root>/In26'
     *  Logic: '<S6>/NOT13'
     *  RelationalOperator: '<S80>/GreaterThan1'
     *  UnitDelay: '<S49>/Unit Delay'
     *  UnitDelay: '<S80>/Unit Delay'
     */
    if (!VSDPI_StErrEBA_B) {
        VSDP_FFStErrEBA_B = false;
    } else {
        VSDP_FFStErrEBA_B =
            ((VSDP_TiTrigStErrEBA_Sec > 0.0F) || VSDP_FFStErrEBA_B);
    }

    /* End of Switch: '<S49>/Switch' */

    /* Switch: '<S59>/Switch' incorporates:
     *  Constant: '<S6>/V_Parameter34'
     *  Inport: '<Root>/In42'
     *  MinMax: '<S59>/Max'
     *  Sum: '<S59>/Subtract'
     *  Switch: '<S59>/Switch1'
     *  UnaryMinus: '<S59>/Unary Minus'
     *  UnitDelay: '<S49>/Unit Delay'
     *  UnitDelay: '<S59>/Unit Delay'
     */
    if (VSDP_FFStErrEBA_B) {
        VSDP_FalDlyTiStErrEBA_C_Sec = VSDP_FalDlyTiStErr_C_Sec;
    } else {
        VSDP_FalDlyTiStErrEBA_C_Sec =
            fmaxf(-VSDPI_CycleTime_Sec, VSDP_FalDlyTiStErrEBA_C_Sec) -
            VSDPI_CycleTime_Sec;
    }

    /* End of Switch: '<S59>/Switch' */

    /* Switch: '<S64>/Switch' incorporates:
     *  Constant: '<S6>/V_Parameter16'
     *  Inport: '<Root>/In42'
     *  Inport: '<Root>/In47'
     *  MinMax: '<S64>/Max'
     *  Sum: '<S64>/Subtract'
     *  Switch: '<S64>/Switch1'
     *  UnaryMinus: '<S64>/Unary Minus'
     *  UnitDelay: '<S64>/Unit Delay'
     */
    if (VSDPI_StErrHDC_B) {
        VSDP_RisDlyTiStErrHDC_Sec =
            fmaxf(VSDP_RisDlyTiStErrHDC_Sec, -VSDPI_CycleTime_Sec) -
            VSDPI_CycleTime_Sec;
    } else {
        VSDP_RisDlyTiStErrHDC_Sec = VSDP_RisDlyTiStErr_C_Sec;
    }

    /* End of Switch: '<S64>/Switch' */

    /* Logic: '<S64>/AND' incorporates:
     *  Inport: '<Root>/In42'
     *  Inport: '<Root>/In47'
     *  RelationalOperator: '<S64>/LessThanOrEqual'
     *  UnaryMinus: '<S64>/Unary Minus1'
     *  UnitDelay: '<S64>/Unit Delay'
     */
    rtb_AND_itq0 = (VSDPI_StErrHDC_B &&
                    (VSDP_RisDlyTiStErrHDC_Sec <= (-VSDPI_CycleTime_Sec)));

    /* Switch: '<S82>/Switch2' incorporates:
     *  Constant: '<S6>/V_Parameter11'
     *  Inport: '<Root>/In42'
     *  Logic: '<S41>/AND'
     *  Logic: '<S41>/NOT'
     *  RelationalOperator: '<S82>/GreaterThan'
     *  Switch: '<S82>/Switch'
     *  UnitDelay: '<S41>/Unit Delay'
     *  UnitDelay: '<S82>/Unit Delay'
     */
    if (rtb_AND_itq0 && (!VSDP_RisEdgeStErrHDC_B)) {
        VSDP_TiTrigStErrHDC_Sec = VSDP_TiTrigStErr_C_Sec;
    } else if (VSDP_TiTrigStErrHDC_Sec > VSDPI_CycleTime_Sec) {
        /* Switch: '<S82>/Switch' incorporates:
         *  Inport: '<Root>/In42'
         *  Sum: '<S82>/Subtract'
         *  UnitDelay: '<S82>/Unit Delay'
         */
        VSDP_TiTrigStErrHDC_Sec -= VSDPI_CycleTime_Sec;
    } else {
        /* UnitDelay: '<S82>/Unit Delay' incorporates:
         *  Constant: '<S82>/Constant1'
         *  Switch: '<S82>/Switch'
         */
        VSDP_TiTrigStErrHDC_Sec = 0.0F;
    }

    /* End of Switch: '<S82>/Switch2' */

    /* Switch: '<S51>/Switch' incorporates:
     *  Constant: '<S51>/Constant2'
     *  Constant: '<S82>/Constant2'
     *  Inport: '<Root>/In47'
     *  Logic: '<S6>/NOT2'
     *  RelationalOperator: '<S82>/GreaterThan1'
     *  UnitDelay: '<S51>/Unit Delay'
     *  UnitDelay: '<S82>/Unit Delay'
     */
    if (!VSDPI_StErrHDC_B) {
        VSDP_FFStErrHDC_B = false;
    } else {
        VSDP_FFStErrHDC_B =
            ((VSDP_TiTrigStErrHDC_Sec > 0.0F) || VSDP_FFStErrHDC_B);
    }

    /* End of Switch: '<S51>/Switch' */

    /* Switch: '<S60>/Switch' incorporates:
     *  Constant: '<S6>/V_Parameter19'
     *  Inport: '<Root>/In42'
     *  MinMax: '<S60>/Max'
     *  Sum: '<S60>/Subtract'
     *  Switch: '<S60>/Switch1'
     *  UnaryMinus: '<S60>/Unary Minus'
     *  UnitDelay: '<S51>/Unit Delay'
     *  UnitDelay: '<S60>/Unit Delay'
     */
    if (VSDP_FFStErrHDC_B) {
        VSDP_FalDlyTiStErrHDC_C_Sec = VSDP_FalDlyTiStErr_C_Sec;
    } else {
        VSDP_FalDlyTiStErrHDC_C_Sec =
            fmaxf(-VSDPI_CycleTime_Sec, VSDP_FalDlyTiStErrHDC_C_Sec) -
            VSDPI_CycleTime_Sec;
    }

    /* End of Switch: '<S60>/Switch' */

    /* Logic: '<S6>/OR' incorporates:
     *  Inport: '<Root>/In42'
     *  Logic: '<S52>/OR'
     *  Logic: '<S59>/OR'
     *  Logic: '<S60>/OR'
     *  RelationalOperator: '<S52>/GreaterThan'
     *  RelationalOperator: '<S59>/GreaterThan'
     *  RelationalOperator: '<S60>/GreaterThan'
     *  UnaryMinus: '<S52>/Unary Minus1'
     *  UnaryMinus: '<S59>/Unary Minus1'
     *  UnaryMinus: '<S60>/Unary Minus1'
     *  UnitDelay: '<S49>/Unit Delay'
     *  UnitDelay: '<S50>/Unit Delay'
     *  UnitDelay: '<S51>/Unit Delay'
     *  UnitDelay: '<S52>/Unit Delay'
     *  UnitDelay: '<S59>/Unit Delay'
     *  UnitDelay: '<S60>/Unit Delay'
     */
    rtb_VectorConcatenate_kusl[7] =
        (((VSDP_FFStErrEBA_B ||
           (VSDP_FalDlyTiStErrEBA_C_Sec > (-VSDPI_CycleTime_Sec))) ||
          (VSDP_FFStErrARP_B ||
           (VSDP_FalDlyTiStErrARP_C_Sec > (-VSDPI_CycleTime_Sec)))) ||
         (VSDP_FFStErrHDC_B ||
          (VSDP_FalDlyTiStErrHDC_C_Sec > (-VSDPI_CycleTime_Sec))));

    /* S-Function (ex_sfun_set_bit): '<S83>/ex_sfun_set_bit' */
    set_bit(VSDP_ConstB.DataTypeConversion_mvuh,
            (boolean_T*)&rtb_VectorConcatenate_kusl[0],
            (uint8_T*)(&(VSDP_SetBit_BS_Param_3[0])), ((uint8_T)8U),
            &rtb_ex_sfun_set_bit);

    /* SignalConversion generated from: '<S1>/60ms' incorporates:
     *  DataTypeConversion: '<S62>/Data Type Conversion1'
     *  DataTypeConversion: '<S83>/Data Type Conversion1'
     */
    VSDP_StError_St = (uint8_T)rtb_ex_sfun_set_bit;

    /* Logic: '<S84>/OR' incorporates:
     *  Constant: '<S84>/V_Parameter10'
     *  Constant: '<S84>/V_Parameter11'
     *  Constant: '<S84>/V_Parameter3'
     *  Constant: '<S84>/V_Parameter4'
     *  Constant: '<S84>/V_Parameter6'
     *  Constant: '<S84>/V_Parameter8'
     *  DataTypeConversion: '<S89>/Data Type Conversion'
     *  DataTypeConversion: '<S90>/Data Type Conversion'
     *  DataTypeConversion: '<S91>/Data Type Conversion'
     *  DataTypeConversion: '<S92>/Data Type Conversion'
     *  DataTypeConversion: '<S93>/Data Type Conversion'
     *  DataTypeConversion: '<S94>/Data Type Conversion'
     *  Inport: '<Root>/In41'
     *  Logic: '<S84>/OR2'
     *  Logic: '<S84>/OR4'
     *  Logic: '<S84>/OR5'
     *  Logic: '<S84>/OR6'
     *  Logic: '<S84>/OR7'
     *  Logic: '<S84>/OR8'
     *  RelationalOperator: '<S84>/Great Than1'
     *  RelationalOperator: '<S84>/Great Than2'
     *  RelationalOperator: '<S84>/Great Than3'
     *  RelationalOperator: '<S84>/Great Than4'
     *  RelationalOperator: '<S84>/Great Than5'
     *  RelationalOperator: '<S84>/Great Than6'
     *  RelationalOperator: '<S84>/Great Than8'
     *  S-Function (sfix_bitop): '<S89>/Bitwise AND'
     *  S-Function (sfix_bitop): '<S90>/Bitwise AND'
     *  S-Function (sfix_bitop): '<S91>/Bitwise AND'
     *  S-Function (sfix_bitop): '<S92>/Bitwise AND'
     *  S-Function (sfix_bitop): '<S93>/Bitwise AND'
     *  S-Function (sfix_bitop): '<S94>/Bitwise AND'
     */
    rtb_OR_b14z = ((((((((((int32_T)VSDPI_StBrightness_St) == 0) &&
                         ((((uint32_T)VSDP_NoDaytime_C_St) & 1U) != 0U)) ||
                        ((((int32_T)VSDPI_StBrightness_St) == 1) &&
                         ((((uint32_T)VSDP_NoDaytime_C_St) & 2U) != 0U))) ||
                       ((((int32_T)VSDPI_StBrightness_St) == 2) &&
                        ((((uint32_T)VSDP_NoDaytime_C_St) & 4U) != 0U))) ||
                      ((((int32_T)VSDPI_StBrightness_St) == 3) &&
                       ((((uint32_T)VSDP_NoDaytime_C_St) & 8U) != 0U))) ||
                     ((((int32_T)VSDPI_StBrightness_St) == 4) &&
                      ((((uint32_T)VSDP_NoDaytime_C_St) & 16U) != 0U))) ||
                    ((((int32_T)VSDPI_StBrightness_St) == 5) &&
                     ((((uint32_T)VSDP_NoDaytime_C_St) & 32U) != 0U))) ||
                   (((int32_T)VSDPI_StBrightness_St) == 255));

    /* Switch: '<S96>/Switch' incorporates:
     *  Constant: '<S84>/V_Parameter1'
     *  Inport: '<Root>/In42'
     *  MinMax: '<S96>/Max'
     *  Sum: '<S96>/Subtract'
     *  Switch: '<S96>/Switch1'
     *  UnaryMinus: '<S96>/Unary Minus'
     *  UnitDelay: '<S96>/Unit Delay'
     */
    if (rtb_OR_b14z) {
        VSDP_RisDlyTiNoDaytimeMn_Sec =
            fmaxf(VSDP_RisDlyTiNoDaytimeMn_Sec, -VSDPI_CycleTime_Sec) -
            VSDPI_CycleTime_Sec;
    } else {
        VSDP_RisDlyTiNoDaytimeMn_Sec = VSDP_NoDaytimeMn_C_Sec;
    }

    /* End of Switch: '<S96>/Switch' */

    /* Logic: '<S96>/AND' incorporates:
     *  Inport: '<Root>/In42'
     *  RelationalOperator: '<S96>/LessThanOrEqual'
     *  UnaryMinus: '<S96>/Unary Minus1'
     *  UnitDelay: '<S96>/Unit Delay'
     */
    rtb_OR_b14z = (rtb_OR_b14z &&
                   (VSDP_RisDlyTiNoDaytimeMn_Sec <= (-VSDPI_CycleTime_Sec)));

    /* Switch: '<S95>/Switch' incorporates:
     *  Constant: '<S84>/V_Parameter2'
     *  Inport: '<Root>/In42'
     *  MinMax: '<S95>/Max'
     *  Sum: '<S95>/Subtract'
     *  Switch: '<S95>/Switch1'
     *  UnaryMinus: '<S95>/Unary Minus'
     *  UnitDelay: '<S95>/Unit Delay'
     */
    if (rtb_OR_b14z) {
        VSDP_FalDlyTiNoDaytimeTrnOff_C_Sec = VSDP_NoDaytimeTrnOff_C_Sec;
    } else {
        VSDP_FalDlyTiNoDaytimeTrnOff_C_Sec =
            fmaxf(-VSDPI_CycleTime_Sec, VSDP_FalDlyTiNoDaytimeTrnOff_C_Sec) -
            VSDPI_CycleTime_Sec;
    }

    /* End of Switch: '<S95>/Switch' */

    /* Logic: '<S95>/OR' incorporates:
     *  Inport: '<Root>/In42'
     *  RelationalOperator: '<S95>/GreaterThan'
     *  UnaryMinus: '<S95>/Unary Minus1'
     *  UnitDelay: '<S95>/Unit Delay'
     */
    rtb_VectorConcatenate_gjaw[10] =
        (rtb_OR_b14z ||
         (VSDP_FalDlyTiNoDaytimeTrnOff_C_Sec > (-VSDPI_CycleTime_Sec)));

    /* Logic: '<S88>/OR10' incorporates:
     *  Constant: '<S88>/V_Parameter10'
     *  Constant: '<S88>/V_Parameter11'
     *  Constant: '<S88>/V_Parameter13'
     *  Constant: '<S88>/V_Parameter15'
     *  Constant: '<S88>/V_Parameter17'
     *  Constant: '<S88>/V_Parameter36'
     *  Constant: '<S88>/V_Parameter4'
     *  Constant: '<S88>/V_Parameter6'
     *  Constant: '<S88>/V_Parameter8'
     *  DataTypeConversion: '<S100>/Data Type Conversion'
     *  DataTypeConversion: '<S101>/Data Type Conversion'
     *  DataTypeConversion: '<S102>/Data Type Conversion'
     *  DataTypeConversion: '<S103>/Data Type Conversion'
     *  DataTypeConversion: '<S104>/Data Type Conversion'
     *  DataTypeConversion: '<S105>/Data Type Conversion'
     *  DataTypeConversion: '<S106>/Data Type Conversion'
     *  Inport: '<Root>/In39'
     *  Inport: '<Root>/In40'
     *  Logic: '<S88>/OR'
     *  Logic: '<S88>/OR1'
     *  Logic: '<S88>/OR2'
     *  Logic: '<S88>/OR4'
     *  Logic: '<S88>/OR5'
     *  Logic: '<S88>/OR6'
     *  Logic: '<S88>/OR7'
     *  Logic: '<S88>/OR8'
     *  Logic: '<S88>/OR9'
     *  RelationalOperator: '<S88>/Great Than1'
     *  RelationalOperator: '<S88>/Great Than11'
     *  RelationalOperator: '<S88>/Great Than2'
     *  RelationalOperator: '<S88>/Great Than3'
     *  RelationalOperator: '<S88>/Great Than4'
     *  RelationalOperator: '<S88>/Great Than5'
     *  RelationalOperator: '<S88>/Great Than6'
     *  RelationalOperator: '<S88>/Great Than7'
     *  RelationalOperator: '<S88>/Great Than8'
     *  S-Function (sfix_bitop): '<S100>/Bitwise AND'
     *  S-Function (sfix_bitop): '<S101>/Bitwise AND'
     *  S-Function (sfix_bitop): '<S102>/Bitwise AND'
     *  S-Function (sfix_bitop): '<S103>/Bitwise AND'
     *  S-Function (sfix_bitop): '<S104>/Bitwise AND'
     *  S-Function (sfix_bitop): '<S105>/Bitwise AND'
     *  S-Function (sfix_bitop): '<S106>/Bitwise AND'
     */
    rtb_OR_b14z =
        (((VSDP_StWiperNoErrEn_C_B) ||
          (VSDPI_StateWiper_St == VSDP_StateWiperEn_C_St)) &&
         (((((((((((int32_T)VSDPI_StageWiper_St) == 0) &&
                 ((((uint32_T)VSDP_StageWiperEn_C_St) & 1U) != 0U)) ||
                ((((int32_T)VSDPI_StageWiper_St) == 1) &&
                 ((((uint32_T)VSDP_StageWiperEn_C_St) & 2U) != 0U))) ||
               ((((int32_T)VSDPI_StageWiper_St) == 2) &&
                ((((uint32_T)VSDP_StageWiperEn_C_St) & 4U) != 0U))) ||
              ((((int32_T)VSDPI_StageWiper_St) == 3) &&
               ((((uint32_T)VSDP_StageWiperEn_C_St) & 8U) != 0U))) ||
             ((((int32_T)VSDPI_StageWiper_St) == 4) &&
              ((((uint32_T)VSDP_StageWiperEn_C_St) & 16U) != 0U))) ||
            ((((int32_T)VSDPI_StageWiper_St) == 5) &&
             ((((uint32_T)VSDP_StageWiperEn_C_St) & 32U) != 0U))) ||
           ((((int32_T)VSDPI_StageWiper_St) == 6) &&
            ((((uint32_T)VSDP_StageWiperEn_C_St) & 64U) != 0U))) ||
          (((int32_T)VSDPI_StageWiper_St) == 255)));

    /* Switch: '<S108>/Switch' incorporates:
     *  Constant: '<S88>/V_Parameter1'
     *  Inport: '<Root>/In42'
     *  MinMax: '<S108>/Max'
     *  MinMax: '<S88>/Max'
     *  Sum: '<S108>/Subtract'
     *  Switch: '<S108>/Switch1'
     *  UnaryMinus: '<S108>/Unary Minus'
     *  UnitDelay: '<S108>/Unit Delay'
     */
    if (rtb_OR_b14z) {
        VSDP_RisDlyTiWiperEnTiMn_Sec =
            fmaxf(VSDP_RisDlyTiWiperEnTiMn_Sec, -VSDPI_CycleTime_Sec) -
            VSDPI_CycleTime_Sec;
    } else {
        VSDP_RisDlyTiWiperEnTiMn_Sec =
            fmaxf(VSDP_WiperEnTiMn_C_Sec, VSDPI_CycleTime_Sec);
    }

    /* End of Switch: '<S108>/Switch' */

    /* Logic: '<S108>/AND' incorporates:
     *  Inport: '<Root>/In42'
     *  RelationalOperator: '<S108>/LessThanOrEqual'
     *  UnaryMinus: '<S108>/Unary Minus1'
     *  UnitDelay: '<S108>/Unit Delay'
     */
    rtb_OR_b14z = (rtb_OR_b14z &&
                   (VSDP_RisDlyTiWiperEnTiMn_Sec <= (-VSDPI_CycleTime_Sec)));

    /* Switch: '<S107>/Switch' incorporates:
     *  Constant: '<S88>/V_Parameter2'
     *  Inport: '<Root>/In42'
     *  MinMax: '<S107>/Max'
     *  MinMax: '<S88>/Max1'
     *  Sum: '<S107>/Subtract'
     *  Switch: '<S107>/Switch1'
     *  UnaryMinus: '<S107>/Unary Minus'
     *  UnitDelay: '<S107>/Unit Delay'
     */
    if (rtb_OR_b14z) {
        VSDP_FalDlyTiWiperEvtGapTiMx_C_Sec =
            fmaxf(VSDP_WiperEvtGapTiMx_C_Sec, VSDPI_CycleTime_Sec);
    } else {
        VSDP_FalDlyTiWiperEvtGapTiMx_C_Sec =
            fmaxf(-VSDPI_CycleTime_Sec, VSDP_FalDlyTiWiperEvtGapTiMx_C_Sec) -
            VSDPI_CycleTime_Sec;
    }

    /* End of Switch: '<S107>/Switch' */

    /* Logic: '<S107>/OR' incorporates:
     *  Inport: '<Root>/In42'
     *  RelationalOperator: '<S107>/GreaterThan'
     *  UnaryMinus: '<S107>/Unary Minus1'
     *  UnitDelay: '<S107>/Unit Delay'
     */
    rtb_OR_b14z = (rtb_OR_b14z || (VSDP_FalDlyTiWiperEvtGapTiMx_C_Sec >
                                   (-VSDPI_CycleTime_Sec)));

    /* Switch: '<S109>/Switch' incorporates:
     *  Constant: '<S88>/V_Parameter3'
     *  Inport: '<Root>/In42'
     *  MinMax: '<S109>/Max'
     *  MinMax: '<S88>/Max2'
     *  Sum: '<S109>/Subtract'
     *  Switch: '<S109>/Switch1'
     *  UnaryMinus: '<S109>/Unary Minus'
     *  UnitDelay: '<S109>/Unit Delay'
     */
    if (rtb_OR_b14z) {
        VSDP_RisDlyTiWiperContiTiMn_Sec =
            fmaxf(VSDP_RisDlyTiWiperContiTiMn_Sec, -VSDPI_CycleTime_Sec) -
            VSDPI_CycleTime_Sec;
    } else {
        VSDP_RisDlyTiWiperContiTiMn_Sec =
            fmaxf(VSDP_WiperContiTiMn_C_Sec, VSDPI_CycleTime_Sec);
    }

    /* End of Switch: '<S109>/Switch' */

    /* Logic: '<S109>/AND' incorporates:
     *  Inport: '<Root>/In42'
     *  RelationalOperator: '<S109>/LessThanOrEqual'
     *  UnaryMinus: '<S109>/Unary Minus1'
     *  UnitDelay: '<S109>/Unit Delay'
     */
    rtb_VectorConcatenate_gjaw[9] =
        (rtb_OR_b14z &&
         (VSDP_RisDlyTiWiperContiTiMn_Sec <= (-VSDPI_CycleTime_Sec)));

    /* SignalConversion generated from: '<S87>/Vector Concatenate' incorporates:
     *  Inport: '<Root>/In27'
     */
    rtb_VectorConcatenate_gjaw[0] = VSDPI_DoorOpen_B;

    /* SignalConversion generated from: '<S87>/Vector Concatenate' incorporates:
     *  Inport: '<Root>/In48'
     */
    rtb_VectorConcatenate_gjaw[11] = VSDPI_BrakeDiscTempSts_B;

    /* SignalConversion generated from: '<S87>/Vector Concatenate' incorporates:
     *  Inport: '<Root>/Inport3'
     */
    rtb_VectorConcatenate_gjaw[12] = VSDPI_SignalInvalidLongi_B;

    /* SignalConversion generated from: '<S87>/Vector Concatenate' incorporates:
     *  Inport: '<Root>/Inport1'
     */
    rtb_VectorConcatenate_gjaw[13] = VSDPI_SignalInvalidLat_B;

    /* SignalConversion generated from: '<S87>/Vector Concatenate' incorporates:
     *  Inport: '<Root>/In29'
     */
    rtb_VectorConcatenate_gjaw[2] = VSDPI_TrailerExs_B;

    /* Logic: '<S86>/OR1' incorporates:
     *  Inport: '<Root>/In30'
     *  Inport: '<Root>/In31'
     */
    rtb_OR_b14z = (VSDPI_GrRvs_B || VSDPI_GrPark_B);

    /* Switch: '<S98>/Switch' incorporates:
     *  Constant: '<S98>/Constant2'
     *  Inport: '<Root>/In32'
     *  Logic: '<S86>/NOT15'
     *  Logic: '<S86>/OR2'
     *  Switch: '<S98>/Switch1'
     *  UnitDelay: '<S98>/Unit Delay'
     */
    if ((!rtb_OR_b14z) && (!VSDPI_GrNeu_B)) {
        VSDP_FFGrNoEnga_B = false;
    } else {
        VSDP_FFGrNoEnga_B = (rtb_OR_b14z || VSDP_FFGrNoEnga_B);
    }

    /* End of Switch: '<S98>/Switch' */

    /* SignalConversion generated from: '<S87>/Vector Concatenate' incorporates:
     *  UnitDelay: '<S98>/Unit Delay'
     */
    rtb_VectorConcatenate_gjaw[3] = VSDP_FFGrNoEnga_B;

    /* SignalConversion generated from: '<S87>/Vector Concatenate' incorporates:
     *  Inport: '<Root>/In33'
     */
    rtb_VectorConcatenate_gjaw[4] = VSDPI_VehMoveBkwd_B;

    /* SignalConversion generated from: '<S87>/Vector Concatenate' incorporates:
     *  Inport: '<Root>/In34'
     */
    rtb_VectorConcatenate_gjaw[5] = VSDPI_DtctOverSte_B;

    /* SignalConversion generated from: '<S87>/Vector Concatenate' incorporates:
     *  Inport: '<Root>/In35'
     */
    rtb_VectorConcatenate_gjaw[6] = VSDPI_DtctUnderSte_B;

    /* SignalConversion generated from: '<S87>/Vector Concatenate' incorporates:
     *  Inport: '<Root>/In36'
     */
    rtb_VectorConcatenate_gjaw[7] = VSDPI_DtctRollerBench_B;

    /* Abs: '<S85>/Abs1' incorporates:
     *  Inport: '<Root>/In38'
     */
    rtb_Abs1 = fabsf(VSDPI_SteAglFrt_Rad);

    /* Switch: '<S97>/Switch' incorporates:
     *  Constant: '<S85>/V_Parameter36'
     *  Constant: '<S85>/V_Parameter37'
     *  Constant: '<S85>/V_Parameter38'
     *  Constant: '<S97>/Constant2'
     *  Inport: '<Root>/In37'
     *  Logic: '<S85>/OR3'
     *  RelationalOperator: '<S85>/Great Than11'
     *  RelationalOperator: '<S85>/Great Than12'
     *  RelationalOperator: '<S85>/Great Than13'
     *  UnitDelay: '<S97>/Unit Delay'
     */
    if (rtb_Abs1 < VSDP_TrsdSteAglNoDtct_C_Rad) {
        VSDP_FFDtctSteAglStop_B = false;
    } else {
        VSDP_FFDtctSteAglStop_B =
            (((VSDPI_VehSpdX_Mps < VSDP_VehSpdLmtMx_C_Mps) &&
              (rtb_Abs1 >= VSDP_TrsdSteAglDtct_C_Rad)) ||
             VSDP_FFDtctSteAglStop_B);
    }

    /* End of Switch: '<S97>/Switch' */

    /* SignalConversion generated from: '<S87>/Vector Concatenate' incorporates:
     *  UnitDelay: '<S97>/Unit Delay'
     */
    rtb_VectorConcatenate_gjaw[8] = VSDP_FFDtctSteAglStop_B;

    /* Logic: '<S7>/NOT14' incorporates:
     *  Inport: '<Root>/In28'
     */
    rtb_VectorConcatenate_gjaw[1] = !VSDPI_VehRdyToSta_B;

    /* S-Function (ex_sfun_set_bit): '<S99>/ex_sfun_set_bit' */
    set_bit(VSDP_ConstB.DataTypeConversion_gpte,
            (boolean_T*)&rtb_VectorConcatenate_gjaw[0],
            (uint8_T*)(&(VSDP_SetBit_BS_Param_4[0])), ((uint8_T)14U),
            &rtb_ex_sfun_set_bit);

    /* Update for UnitDelay: '<S5>/Unit Delay' incorporates:
     *  Inport: '<Root>/In12'
     */
    VSDP_UstpAccPedPstn_Per = VSDPI_AccPedPstn_Per;

    /* Update for UnitDelay: '<S32>/Unit Delay' */
    VSDP_RisEdgeStErrABS_B = rtb_y;

    /* Update for UnitDelay: '<S34>/Unit Delay' */
    VSDP_RisEdgeStErrESC_B = rtb_AND_ku13;

    /* Update for UnitDelay: '<S35>/Unit Delay' */
    VSDP_RisEdgeStErrTSC_B = rtb_AND_mai0;

    /* Update for UnitDelay: '<S36>/Unit Delay' */
    VSDP_RisEdgeStErrVSM_B = rtb_AND_hz4i;

    /* Update for UnitDelay: '<S37>/Unit Delay' */
    VSDP_RisEdgeStErrVDY_B = rtb_AND_kbzm;

    /* Update for UnitDelay: '<S38>/Unit Delay' */
    VSDP_RisEdgeStErrLatDMC_B = rtb_AND_g15l;

    /* Update for UnitDelay: '<S33>/Unit Delay' */
    VSDP_RisEdgeStErrACC_B = rtb_AND_med5;

    /* Update for UnitDelay: '<S40>/Unit Delay' */
    VSDP_RisEdgeStErrARP_B = rtb_AND_cvp0;

    /* Update for UnitDelay: '<S39>/Unit Delay' */
    VSDP_RisEdgeStErrEBA_B = rtb_AND_o5jf;

    /* Update for UnitDelay: '<S41>/Unit Delay' */
    VSDP_RisEdgeStErrHDC_B = rtb_AND_itq0;

    /* SignalConversion generated from: '<S1>/60ms' incorporates:
     *  DataTypeConversion: '<S99>/Data Type Conversion1'
     */
    VSDP_VehStIvld_St = (uint16_T)rtb_ex_sfun_set_bit;

    /* End of Outputs for S-Function (fcgen): '<S1>/Function-Call Generator' */
}

/* Model initialize function */
void VSDP_initialize(void) {
    /* Registration code */

    /* block I/O */

    /* exported global signals */
    VSDP_VehStIvld_St = ((uint16_T)0U);
    VSDP_CtrlStEn_St = ((uint8_T)0U);
    VSDP_CtrlStNoAvlb_St = ((uint8_T)0U);
    VSDP_IvldStDrv_St = ((uint8_T)0U);
    VSDP_StError_St = ((uint8_T)0U);

    /* states (dwork) */

    /* exported global states */
    VSDP_RSDlyTiCtrlStEnABS_Sec = 0.0F;
    VSDP_FalDlyTiCtrlStEnABS_Sec = 0.0F;
    VSDP_FalDlyTiCtrlStEnACC_Sec = 0.0F;
    VSDP_RSDlyTiCtrlStEnESC_Sec = 0.0F;
    VSDP_FalDlyTiCtrlStEnESC_Sec = 0.0F;
    VSDP_RSDlyTiCtrlStEnTCS_Sec = 0.0F;
    VSDP_FalDlyTiCtrlStEnTCS_Sec = 0.0F;
    VSDP_FalDlyTiCtrlStEnVSM_Sec = 0.0F;
    VSDP_RSDlyTiCtrlStEnAEB_Sec = 0.0F;
    VSDP_FalDlyTiCtrlStEnEBA_Sec = 0.0F;
    VSDP_RSDlyTiCtrlStEnARP_Sec = 0.0F;
    VSDP_FalDlyTiCtrlStEnAPR_Sec = 0.0F;
    VSDP_RSDlyTiCtrlStEnHDC_Sec = 0.0F;
    VSDP_FalDlyTiCtrlStEnHDC_Sec = 0.0F;
    VSDP_UstpAccPedPstn_Per = 0.0F;
    VSDP_FalDlyTiAccPedPstnRate_Sec = 0.0F;
    VSDP_FalDlyTiTrnSglHarLigEn_Sec = 0.0F;
    VSDP_FalDlyTiTrnSglEn_Sec = 0.0F;
    VSDP_RisDlyTiManuActuTrq_Sec = 0.0F;
    VSDP_RisDlyTiStErrABS_Sec = 0.0F;
    VSDP_TiTrigStErrABS_Sec = 0.0F;
    VSDP_FalDlyTiStErrABS_C_Sec = 0.0F;
    VSDP_RisDlyTiStErrESC_Sec = 0.0F;
    VSDP_TiTrigStErrESC_Sec = 0.0F;
    VSDP_FalDlyTiStErrESC_C_Sec = 0.0F;
    VSDP_RisDlyTiStErrTSC_Sec = 0.0F;
    VSDP_TiTrigStErrTSC_Sec = 0.0F;
    VSDP_FalDlyTiStErrTSC_C_Sec = 0.0F;
    VSDP_RisDlyTiStErrVSM_Sec = 0.0F;
    VSDP_TiTrigStErrVSM_Sec = 0.0F;
    VSDP_FalDlyTiStErrVSM_C_Sec = 0.0F;
    VSDP_RisDlyTiStErrVDY_Sec = 0.0F;
    VSDP_TiTrigStErrVDY_Sec = 0.0F;
    VSDP_FalDlyTiStErrVDY_C_Sec = 0.0F;
    VSDP_RisDlyTiStErrLatDMC_Sec = 0.0F;
    VSDP_TiTrigStErrLatDMC_Sec = 0.0F;
    VSDP_FalDlyTiStErrLatDMC_C_Sec = 0.0F;
    VSDP_RisDlyTiStErrACC_Sec = 0.0F;
    VSDP_TiTrigStErrACC_Sec = 0.0F;
    VSDP_FalDlyTiStErrACC_C_Sec = 0.0F;
    VSDP_RisDlyTiStErrARP_Sec = 0.0F;
    VSDP_TiTrigStErrARP_Sec = 0.0F;
    VSDP_FalDlyTiStErrARP_C_Sec = 0.0F;
    VSDP_RisDlyTiStErrEBA_Sec = 0.0F;
    VSDP_TiTrigStErrEBA_Sec = 0.0F;
    VSDP_FalDlyTiStErrEBA_C_Sec = 0.0F;
    VSDP_RisDlyTiStErrHDC_Sec = 0.0F;
    VSDP_TiTrigStErrHDC_Sec = 0.0F;
    VSDP_FalDlyTiStErrHDC_C_Sec = 0.0F;
    VSDP_RisDlyTiNoDaytimeMn_Sec = 0.0F;
    VSDP_FalDlyTiNoDaytimeTrnOff_C_Sec = 0.0F;
    VSDP_RisDlyTiWiperEnTiMn_Sec = 0.0F;
    VSDP_FalDlyTiWiperEvtGapTiMx_C_Sec = 0.0F;
    VSDP_RisDlyTiWiperContiTiMn_Sec = 0.0F;
    VSDP_RisEdgeStErrABS_B = false;
    VSDP_FFStErrABS_B = false;
    VSDP_RisEdgeStErrESC_B = false;
    VSDP_FFStErrESC_B = false;
    VSDP_RisEdgeStErrTSC_B = false;
    VSDP_FFStErrTSC_B = false;
    VSDP_RisEdgeStErrVSM_B = false;
    VSDP_FFStErrVSM_B = false;
    VSDP_RisEdgeStErrVDY_B = false;
    VSDP_FFStErrVDY_B = false;
    VSDP_RisEdgeStErrLatDMC_B = false;
    VSDP_FFStErrLatDMC_B = false;
    VSDP_RisEdgeStErrACC_B = false;
    VSDP_FFStErrACC_B = false;
    VSDP_RisEdgeStErrARP_B = false;
    VSDP_FFStErrARP_B = false;
    VSDP_RisEdgeStErrEBA_B = false;
    VSDP_FFStErrEBA_B = false;
    VSDP_RisEdgeStErrHDC_B = false;
    VSDP_FFStErrHDC_B = false;
    VSDP_FFGrNoEnga_B = false;
    VSDP_FFDtctSteAglStop_B = false;

    /* SystemInitialize for S-Function (fcgen): '<S1>/Function-Call Generator'
     * incorporates: SubSystem: '<S1>/60ms'
     */
    /* InitializeConditions for UnitDelay: '<S32>/Unit Delay' */
    VSDP_RisEdgeStErrABS_B = true;

    /* InitializeConditions for UnitDelay: '<S34>/Unit Delay' */
    VSDP_RisEdgeStErrESC_B = true;

    /* InitializeConditions for UnitDelay: '<S35>/Unit Delay' */
    VSDP_RisEdgeStErrTSC_B = true;

    /* InitializeConditions for UnitDelay: '<S36>/Unit Delay' */
    VSDP_RisEdgeStErrVSM_B = true;

    /* InitializeConditions for UnitDelay: '<S37>/Unit Delay' */
    VSDP_RisEdgeStErrVDY_B = true;

    /* InitializeConditions for UnitDelay: '<S38>/Unit Delay' */
    VSDP_RisEdgeStErrLatDMC_B = true;

    /* InitializeConditions for UnitDelay: '<S33>/Unit Delay' */
    VSDP_RisEdgeStErrACC_B = true;

    /* InitializeConditions for UnitDelay: '<S40>/Unit Delay' */
    VSDP_RisEdgeStErrARP_B = true;

    /* InitializeConditions for UnitDelay: '<S39>/Unit Delay' */
    VSDP_RisEdgeStErrEBA_B = true;

    /* InitializeConditions for UnitDelay: '<S41>/Unit Delay' */
    VSDP_RisEdgeStErrHDC_B = true;

    /* End of SystemInitialize for S-Function (fcgen): '<S1>/Function-Call
     * Generator' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */