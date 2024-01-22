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
 * File                             : VSDP.h
 *
 * FileType                         : Code Header File
 *
 * Real-Time Workshop file version  : 9.4 (R2020b) 29-Jul-2020
 *
 * TLC version                      : 9.4 (Aug 20 2020)
 *
 * C source code generated on       : Wed Oct 26 12:20:40 2022
 *
 * Copyright (C) by SenseTime Group Limited. All rights reserved.
 *******************************************************************************/

#ifndef RTW_HEADER_VSDP_h_
#define RTW_HEADER_VSDP_h_
#include <math.h>
#ifndef VSDP_COMMON_INCLUDES_
#define VSDP_COMMON_INCLUDES_
#include "Sfun_Set_Bit.h"
#include "rtwtypes.h"
#endif /* VSDP_COMMON_INCLUDES_ */

#include "VSDP_types.h"

/* Macros for accessing real-time model data structure */

/* Invariant block signals (default storage) */
typedef struct {
    const uint32_T DataTypeConversion;      /* '<S18>/Data Type Conversion' */
    const uint32_T DataTypeConversion_doo1; /* '<S8>/Data Type Conversion' */
    const uint32_T DataTypeConversion_lzpg; /* '<S29>/Data Type Conversion' */
    const uint32_T DataTypeConversion_mvuh; /* '<S62>/Data Type Conversion' */
    const uint32_T DataTypeConversion_gpte; /* '<S87>/Data Type Conversion' */
} ConstB_VSDP_T;

extern const ConstB_VSDP_T VSDP_ConstB; /* constant block i/o */

/*
 * Exported Global Signals
 *
 * Note: Exported global signals are block signals with an exported global
 * storage class designation.  Code generation will declare the memory for
 * these signals and export their symbols.
 *
 */
extern uint16_T VSDP_VehStIvld_St;   /* '<S1>/60ms'
                                      * Invalid state of vehicle system
                                      */
extern uint8_T VSDP_CtrlStEn_St;     /* '<S1>/60ms'
                                      * State of control feature is enable
                                      */
extern uint8_T VSDP_CtrlStNoAvlb_St; /* '<S1>/60ms'
                                      * State of control feature is not avaible
                                      */
extern uint8_T VSDP_IvldStDrv_St;    /* '<S1>/60ms'
                                      * Invalid state of driver
                                      */
extern uint8_T VSDP_StError_St;      /* '<S1>/60ms'
                                      * error state of system
                                      */

/*
 * Exported States
 *
 * Note: Exported states are block states with an exported global
 * storage class designation.  Code generation will declare the memory for these
 * states and exports their symbols.
 *
 */
extern real32_T
    VSDP_RSDlyTiCtrlStEnABS_Sec; /* '<S21>/Unit Delay'
                                  * Condition of ABS in actively controlling
                                  */
extern real32_T
    VSDP_FalDlyTiCtrlStEnABS_Sec; /* '<S10>/Unit Delay'
                                   * Condition of ABS in actively controlling
                                   */
extern real32_T
    VSDP_FalDlyTiCtrlStEnACC_Sec; /* '<S11>/Unit Delay'
                                   * Condition of ACC in actively controlling
                                   */
extern real32_T
    VSDP_RSDlyTiCtrlStEnESC_Sec; /* '<S19>/Unit Delay'
                                  * Condition of ABS in actively controlling
                                  */
extern real32_T
    VSDP_FalDlyTiCtrlStEnESC_Sec; /* '<S12>/Unit Delay'
                                   * Condition of ESC in actively controlling
                                   */
extern real32_T
    VSDP_RSDlyTiCtrlStEnTCS_Sec; /* '<S20>/Unit Delay'
                                  * Condition of ABS in actively controlling
                                  */
extern real32_T
    VSDP_FalDlyTiCtrlStEnTCS_Sec; /* '<S13>/Unit Delay'
                                   * Condition of TSC in actively controlling
                                   */
extern real32_T
    VSDP_FalDlyTiCtrlStEnVSM_Sec; /* '<S14>/Unit Delay'
                                   * Condition of VSM in actively controlling
                                   */
extern real32_T
    VSDP_RSDlyTiCtrlStEnAEB_Sec; /* '<S22>/Unit Delay'
                                  * Condition of ABS in actively controlling
                                  */
extern real32_T
    VSDP_FalDlyTiCtrlStEnEBA_Sec; /* '<S15>/Unit Delay'
                                   * Condition of EBA in actively controlling
                                   */
extern real32_T
    VSDP_RSDlyTiCtrlStEnARP_Sec; /* '<S23>/Unit Delay'
                                  * Condition of ABS in actively controlling
                                  */
extern real32_T
    VSDP_FalDlyTiCtrlStEnAPR_Sec; /* '<S16>/Unit Delay'
                                   * Condition of EBA in actively controlling
                                   */
extern real32_T
    VSDP_RSDlyTiCtrlStEnHDC_Sec; /* '<S24>/Unit Delay'
                                  * Condition of ABS in actively controlling
                                  */
extern real32_T
    VSDP_FalDlyTiCtrlStEnHDC_Sec; /* '<S17>/Unit Delay'
                                   * Condition of EBA in actively controlling
                                   */
extern real32_T
    VSDP_UstpAccPedPstn_Per; /* '<S5>/Unit Delay'
                              * Unit step of acceleration pedal position
                              */
extern real32_T
    VSDP_FalDlyTiAccPedPstnRate_Sec;             /* '<S26>/Unit Delay'
                                                  * Fall delay time of acceleration pedal
                                                  * position rate is out range
                                                  */
extern real32_T VSDP_FalDlyTiTrnSglHarLigEn_Sec; /* '<S27>/Unit Delay'
                                                  * Fall delay time of hazard
                                                  * light is active
                                                  */
extern real32_T
    VSDP_FalDlyTiTrnSglEn_Sec;                /* '<S28>/Unit Delay'
                                               * Fall delay time of turn light is active
                                               */
extern real32_T VSDP_RisDlyTiManuActuTrq_Sec; /* '<S30>/Unit Delay'
                                               * Fall delay time of manunal
                                               * torque for EPS is out range
                                               */
extern real32_T
    VSDP_RisDlyTiStErrABS_Sec;               /* '<S63>/Unit Delay'
                                              * Rise delay time for ABS error condition
                                              */
extern real32_T VSDP_TiTrigStErrABS_Sec;     /* '<S73>/Unit Delay'
                                              * Time trigger of ABS error condition
                                              */
extern real32_T VSDP_FalDlyTiStErrABS_C_Sec; /* '<S53>/Unit Delay'
                                              * Fall edge of rise delay time for
                                              * ABS error condition
                                              */
extern real32_T
    VSDP_RisDlyTiStErrESC_Sec;               /* '<S66>/Unit Delay'
                                              * Rise delay time for ESC error condition
                                              */
extern real32_T VSDP_TiTrigStErrESC_Sec;     /* '<S75>/Unit Delay'
                                              * Time trigger of ESC error condition
                                              */
extern real32_T VSDP_FalDlyTiStErrESC_C_Sec; /* '<S54>/Unit Delay'
                                              * Fall edge of rise delay time for
                                              * ESC error condition
                                              */
extern real32_T
    VSDP_RisDlyTiStErrTSC_Sec;               /* '<S67>/Unit Delay'
                                              * Rise delay time for TSC error condition
                                              */
extern real32_T VSDP_TiTrigStErrTSC_Sec;     /* '<S76>/Unit Delay'
                                              * Time trigger of TSC error condition
                                              */
extern real32_T VSDP_FalDlyTiStErrTSC_C_Sec; /* '<S55>/Unit Delay'
                                              * Fall edge of rise delay time for
                                              * TSC error condition
                                              */
extern real32_T
    VSDP_RisDlyTiStErrVSM_Sec;               /* '<S68>/Unit Delay'
                                              * Rise delay time for VSM error condition
                                              */
extern real32_T VSDP_TiTrigStErrVSM_Sec;     /* '<S77>/Unit Delay'
                                              * Time trigger of VSM error condition
                                              */
extern real32_T VSDP_FalDlyTiStErrVSM_C_Sec; /* '<S56>/Unit Delay'
                                              * Fall edge of rise delay time for
                                              * VSM error condition
                                              */
extern real32_T
    VSDP_RisDlyTiStErrVDY_Sec;               /* '<S69>/Unit Delay'
                                              * Rise delay time for VDY error condition
                                              */
extern real32_T VSDP_TiTrigStErrVDY_Sec;     /* '<S78>/Unit Delay'
                                              * Time trigger of VDY error condition
                                              */
extern real32_T VSDP_FalDlyTiStErrVDY_C_Sec; /* '<S57>/Unit Delay'
                                              * Fall edge of rise delay time for
                                              * VDY error condition
                                              */
extern real32_T
    VSDP_RisDlyTiStErrLatDMC_Sec; /* '<S70>/Unit Delay'
                                   * Rise delay time for LatDMC error condition
                                   */
extern real32_T
    VSDP_TiTrigStErrLatDMC_Sec;                 /* '<S79>/Unit Delay'
                                                 * Time trigger of LatDMC error condition
                                                 */
extern real32_T VSDP_FalDlyTiStErrLatDMC_C_Sec; /* '<S58>/Unit Delay'
                                                 * Fall edge of rise delay time
                                                 * for LatDMC error condition
                                                 */
extern real32_T
    VSDP_RisDlyTiStErrACC_Sec;               /* '<S65>/Unit Delay'
                                              * Rise delay time for ACC error condition
                                              */
extern real32_T VSDP_TiTrigStErrACC_Sec;     /* '<S74>/Unit Delay'
                                              * Time trigger of ACC error condition
                                              */
extern real32_T VSDP_FalDlyTiStErrACC_C_Sec; /* '<S61>/Unit Delay'
                                              * Fall edge of rise delay time for
                                              * ACC error condition
                                              */
extern real32_T
    VSDP_RisDlyTiStErrARP_Sec;               /* '<S72>/Unit Delay'
                                              * Rise delay time for EBA error condition
                                              */
extern real32_T VSDP_TiTrigStErrARP_Sec;     /* '<S81>/Unit Delay'
                                              * Time trigger of EBA error condition
                                              */
extern real32_T VSDP_FalDlyTiStErrARP_C_Sec; /* '<S52>/Unit Delay'
                                              * Fall edge of rise delay time for
                                              * EBA error condition
                                              */
extern real32_T
    VSDP_RisDlyTiStErrEBA_Sec;               /* '<S71>/Unit Delay'
                                              * Rise delay time for EBA error condition
                                              */
extern real32_T VSDP_TiTrigStErrEBA_Sec;     /* '<S80>/Unit Delay'
                                              * Time trigger of EBA error condition
                                              */
extern real32_T VSDP_FalDlyTiStErrEBA_C_Sec; /* '<S59>/Unit Delay'
                                              * Fall edge of rise delay time for
                                              * EBA error condition
                                              */
extern real32_T
    VSDP_RisDlyTiStErrHDC_Sec;               /* '<S64>/Unit Delay'
                                              * Rise delay time for EBA error condition
                                              */
extern real32_T VSDP_TiTrigStErrHDC_Sec;     /* '<S82>/Unit Delay'
                                              * Time trigger of EBA error condition
                                              */
extern real32_T VSDP_FalDlyTiStErrHDC_C_Sec; /* '<S60>/Unit Delay'
                                              * Fall edge of rise delay time for
                                              * EBA error condition
                                              */
extern real32_T
    VSDP_RisDlyTiNoDaytimeMn_Sec; /* '<S96>/Unit Delay'
                                   * Rise  delay time for no daytime is active
                                   */
extern real32_T
    VSDP_FalDlyTiNoDaytimeTrnOff_C_Sec; /* '<S95>/Unit Delay'
                                         * Fall edge of Rise  delay time for no
                                         * daytime is active
                                         */
extern real32_T
    VSDP_RisDlyTiWiperEnTiMn_Sec; /* '<S108>/Unit Delay'
                                   * Rise  delay time for wiper state is active
                                   */
extern real32_T
    VSDP_FalDlyTiWiperEvtGapTiMx_C_Sec; /* '<S107>/Unit Delay'
                                         * Fall edge of rise  delay time for
                                         * wiper state is active
                                         */
extern real32_T
    VSDP_RisDlyTiWiperContiTiMn_Sec;        /* '<S109>/Unit Delay'
                                             * continue time for wiper state is active
                                             */
extern boolean_T VSDP_RisEdgeStErrABS_B;    /* '<S32>/Unit Delay'
                                             * Rise edge of rise delay time for ABS
                                             * error condition
                                             */
extern boolean_T VSDP_FFStErrABS_B;         /* '<S42>/Unit Delay'
                                             * FF of ABS error condition
                                             */
extern boolean_T VSDP_RisEdgeStErrESC_B;    /* '<S34>/Unit Delay'
                                             * Rise edge of rise delay time for ESC
                                             * error condition
                                             */
extern boolean_T VSDP_FFStErrESC_B;         /* '<S44>/Unit Delay'
                                             * FF of ESC error condition
                                             */
extern boolean_T VSDP_RisEdgeStErrTSC_B;    /* '<S35>/Unit Delay'
                                             * Rise edge of rise delay time for TSC
                                             * error condition
                                             */
extern boolean_T VSDP_FFStErrTSC_B;         /* '<S45>/Unit Delay'
                                             * FF of TSC error condition
                                             */
extern boolean_T VSDP_RisEdgeStErrVSM_B;    /* '<S36>/Unit Delay'
                                             * Rise edge of rise delay time for VSM
                                             * error condition
                                             */
extern boolean_T VSDP_FFStErrVSM_B;         /* '<S46>/Unit Delay'
                                             * FF of VSM error condition
                                             */
extern boolean_T VSDP_RisEdgeStErrVDY_B;    /* '<S37>/Unit Delay'
                                             * Rise edge of rise delay time for VDY
                                             * error condition
                                             */
extern boolean_T VSDP_FFStErrVDY_B;         /* '<S47>/Unit Delay'
                                             * FF of VDY error condition
                                             */
extern boolean_T VSDP_RisEdgeStErrLatDMC_B; /* '<S38>/Unit Delay'
                                             * Rise edge of rise delay time for
                                             * LatDMC error condition
                                             */
extern boolean_T VSDP_FFStErrLatDMC_B;      /* '<S48>/Unit Delay'
                                             * FF of LatDMC error condition
                                             */
extern boolean_T VSDP_RisEdgeStErrACC_B;    /* '<S33>/Unit Delay'
                                             * Rise edge of rise delay time for ACC
                                             * error condition
                                             */
extern boolean_T VSDP_FFStErrACC_B;         /* '<S43>/Unit Delay'
                                             * FF of ACC error condition
                                             */
extern boolean_T VSDP_RisEdgeStErrARP_B;    /* '<S40>/Unit Delay'
                                             * Rise edge of rise delay time for EBA
                                             * error condition
                                             */
extern boolean_T VSDP_FFStErrARP_B;         /* '<S50>/Unit Delay'
                                             * FF of EBA error condition
                                             */
extern boolean_T VSDP_RisEdgeStErrEBA_B;    /* '<S39>/Unit Delay'
                                             * Rise edge of rise delay time for EBA
                                             * error condition
                                             */
extern boolean_T VSDP_FFStErrEBA_B;         /* '<S49>/Unit Delay'
                                             * FF of EBA error condition
                                             */
extern boolean_T VSDP_RisEdgeStErrHDC_B;    /* '<S41>/Unit Delay'
                                             * Rise edge of rise delay time for EBA
                                             * error condition
                                             */
extern boolean_T VSDP_FFStErrHDC_B;         /* '<S51>/Unit Delay'
                                             * FF of EBA error condition
                                             */
extern boolean_T VSDP_FFGrNoEnga_B;         /* '<S98>/Unit Delay'
                                             * FF of invalid engaged gear
                                             */
extern boolean_T VSDP_FFDtctSteAglStop_B;   /* '<S97>/Unit Delay'
                                             * FF of steer angle is out range
                                             */

/* Model entry point functions */
extern void VSDP_initialize(void);
extern void VSDP_step(void);

/* Exported data declaration */

/* Declaration for custom storage class: Global */
extern uint8_T VSDP_SetBit_BS_Param_1[6]; /* Referenced by:
                                           * '<S9>/ex_sfun_set_bit'
                                           * '<S25>/ex_sfun_set_bit'
                                           */
extern uint8_T VSDP_SetBit_BS_Param_2[7];
/* Referenced by: '<S31>/ex_sfun_set_bit' */
extern uint8_T VSDP_SetBit_BS_Param_3[8];
/* Referenced by: '<S83>/ex_sfun_set_bit' */
extern uint8_T VSDP_SetBit_BS_Param_4[14];
/* Referenced by: '<S99>/ex_sfun_set_bit' */

/* ConstVolatile memory section */
/* Declaration for custom storage class: ConstVolatile */
extern const volatile uint8_T
    VSDP_DegTrig_C_St; /* Referenced by: '<S5>/V_Parameter10' */

/* Degradation Trigger */
extern const volatile real32_T
    VSDP_FalDlyTiAccPedPstnRate_C_Sec; /* Referenced by: '<S5>/V_Parameter7' */

/* Fall delay time of actual accerlator pedal rate is more than Minimum
 * threshold of accerlator pedal rate */
extern const volatile real32_T
    VSDP_FalDlyTiCtrlStEn_C_Sec; /* Referenced by:
                                  * '<S4>/V_Parameter'
                                  * '<S4>/V_Parameter1'
                                  * '<S4>/V_Parameter10'
                                  * '<S4>/V_Parameter12'
                                  * '<S4>/V_Parameter2'
                                  * '<S4>/V_Parameter3'
                                  * '<S4>/V_Parameter4'
                                  * '<S4>/V_Parameter5'
                                  */

/* Fall delay time of ABS��ACC��ESC��TSC��VSM or EBA control function condition
 */
extern const volatile real32_T VSDP_FalDlyTiStErr_C_Sec; /* Referenced by:
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
extern const volatile real32_T VSDP_FalDlyTiTrnSgl_C_Sec; /* Referenced by:
                                                           * '<S5>/V_Parameter8'
                                                           * '<S5>/V_Parameter9'
                                                           */

/* Fall delay time value of turn light is active */
extern const volatile real32_T
    VSDP_ManuActuTrqMx_C_Nm; /* Referenced by: '<S5>/V_Parameter11' */

/*  Maximum threshold of manunal torque for EPS */
extern const volatile real32_T
    VSDP_NoDaytimeMn_C_Sec; /* Referenced by: '<S84>/V_Parameter1' */

/* Minimum time value of no daytime state */
extern const volatile real32_T
    VSDP_NoDaytimeTrnOff_C_Sec; /* Referenced by: '<S84>/V_Parameter2' */

/* Fall delay time value of no daytime state */
extern const volatile uint8_T VSDP_NoDaytime_C_St; /* Referenced by:
                                                    * '<S84>/V_Parameter10'
                                                    * '<S84>/V_Parameter11'
                                                    * '<S84>/V_Parameter3'
                                                    * '<S84>/V_Parameter4'
                                                    * '<S84>/V_Parameter6'
                                                    * '<S84>/V_Parameter8'
                                                    */

/* No daytime state value */
extern const volatile real32_T
    VSDP_RSDlyTimeABSAct_sec; /* Referenced by: '<S4>/V_Parameter7' */

/* Fall delay time of ABS��ACC��ESC��TSC��VSM or EBA control function condition
 */
extern const volatile real32_T
    VSDP_RSDlyTimeAEBAct_sec; /* Referenced by: '<S4>/V_Parameter9' */

/* Fall delay time of ABS��ACC��ESC��TSC��VSM or EBA control function condition
 */
extern const volatile real32_T
    VSDP_RSDlyTimeARPAct_sec; /* Referenced by: '<S4>/V_Parameter11' */

/* Fall delay time of ABS��ACC��ESC��TSC��VSM or EBA control function condition
 */
extern const volatile real32_T
    VSDP_RSDlyTimeESCAct_sec; /* Referenced by: '<S4>/V_Parameter6' */

/* Fall delay time of ABS��ACC��ESC��TSC��VSM or EBA control function condition
 */
extern const volatile real32_T
    VSDP_RSDlyTimeHDCAct_sec; /* Referenced by: '<S4>/V_Parameter13' */

/* Fall delay time of ABS��ACC��ESC��TSC��VSM or EBA control function condition
 */
extern const volatile real32_T
    VSDP_RSDlyTimeTCSAct_sec; /* Referenced by: '<S4>/V_Parameter8' */

/* Fall delay time of ABS��ACC��ESC��TSC��VSM or EBA control function condition
 */
extern const volatile real32_T
    VSDP_RisDlyTiManuActuTrq_C_Sec; /* Referenced by: '<S5>/V_Parameter39' */

/* Fall delay time of manunal torque is more than Minimum threshold of
 * accerlator pedal rate */
extern const volatile real32_T VSDP_RisDlyTiStErr_C_Sec; /* Referenced by:
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
extern const volatile boolean_T
    VSDP_StWiperNoErrEn_C_B; /* Referenced by: '<S88>/V_Parameter4' */

/* Debug value of wiper state is active */
extern const volatile uint8_T VSDP_StageWiperEn_C_St; /* Referenced by:
                                                       * '<S88>/V_Parameter10'
                                                       * '<S88>/V_Parameter11'
                                                       * '<S88>/V_Parameter13'
                                                       * '<S88>/V_Parameter15'
                                                       * '<S88>/V_Parameter17'
                                                       * '<S88>/V_Parameter6'
                                                       * '<S88>/V_Parameter8'
                                                       */

/* wiper stage value is valid */
extern const volatile uint8_T VSDP_StateWiperEn_C_St;
/* Referenced by: '<S88>/V_Parameter36' */

/* wiper state value is valid */
extern const volatile real32_T VSDP_TiTrigStErr_C_Sec; /* Referenced by:
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
extern const volatile real32_T VSDP_TrsdSteAglDtct_C_Rad;
/* Referenced by: '<S85>/V_Parameter37' */

/* Maximum steer angle limit value  */
extern const volatile real32_T VSDP_TrsdSteAglNoDtct_C_Rad;
/* Referenced by: '<S85>/V_Parameter38' */

/* Maximum steer angle limit value for valid steer angle */
extern const volatile real32_T
    VSDP_UstpAccPedPstnMx_C_Per; /* Referenced by: '<S5>/V_Parameter6' */

/*  Maximum threshold of accerlator pedal rate */
extern const volatile real32_T VSDP_VehSpdLmtMx_C_Mps;
/* Referenced by: '<S85>/V_Parameter36' */

/* Maximum vehicle speed limit value for steer angle */
extern const volatile real32_T
    VSDP_WiperContiTiMn_C_Sec; /* Referenced by: '<S88>/V_Parameter3' */

/* continue time value for wiper state is active */
extern const volatile real32_T
    VSDP_WiperEnTiMn_C_Sec; /* Referenced by: '<S88>/V_Parameter1' */

/* Rise  delay time value for wiper state is active */
extern const volatile real32_T
    VSDP_WiperEvtGapTiMx_C_Sec; /* Referenced by: '<S88>/V_Parameter2' */

/* Fall delay time value of rise  delay time for wiper state is active */

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S6>/V_Const' : Unused code path elimination
 * Block '<S6>/V_Const1' : Unused code path elimination
 * Block '<S6>/V_Const2' : Unused code path elimination
 * Block '<S6>/V_Const3' : Unused code path elimination
 * Block '<S6>/V_Const4' : Unused code path elimination
 * Block '<S6>/V_Const5' : Unused code path elimination
 * Block '<S6>/V_Const6' : Unused code path elimination
 * Block '<S6>/V_Const7' : Unused code path elimination
 * Block '<S6>/V_Const8' : Unused code path elimination
 * Block '<S6>/V_Const9' : Unused code path elimination
 */

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
 * hilite_system('VSDP_Model/VSDP')    - opens subsystem VSDP_Model/VSDP
 * hilite_system('VSDP_Model/VSDP/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'VSDP_Model'
 * '<S1>'   : 'VSDP_Model/VSDP'
 * '<S2>'   : 'VSDP_Model/VSDP/60ms'
 * '<S3>'   : 'VSDP_Model/VSDP/60ms/CtrlStAvlb'
 * '<S4>'   : 'VSDP_Model/VSDP/60ms/CtrlStEn'
 * '<S5>'   : 'VSDP_Model/VSDP/60ms/DrvSt'
 * '<S6>'   : 'VSDP_Model/VSDP/60ms/StError'
 * '<S7>'   : 'VSDP_Model/VSDP/60ms/VehStIvld'
 * '<S8>'   : 'VSDP_Model/VSDP/60ms/CtrlStAvlb/MappingUint8'
 * '<S9>'   : 'VSDP_Model/VSDP/60ms/CtrlStAvlb/MappingUint8/Set_bit'
 * '<S10>'  : 'VSDP_Model/VSDP/60ms/CtrlStEn/FallDelay'
 * '<S11>'  : 'VSDP_Model/VSDP/60ms/CtrlStEn/FallDelay1'
 * '<S12>'  : 'VSDP_Model/VSDP/60ms/CtrlStEn/FallDelay2'
 * '<S13>'  : 'VSDP_Model/VSDP/60ms/CtrlStEn/FallDelay3'
 * '<S14>'  : 'VSDP_Model/VSDP/60ms/CtrlStEn/FallDelay4'
 * '<S15>'  : 'VSDP_Model/VSDP/60ms/CtrlStEn/FallDelay5'
 * '<S16>'  : 'VSDP_Model/VSDP/60ms/CtrlStEn/FallDelay6'
 * '<S17>'  : 'VSDP_Model/VSDP/60ms/CtrlStEn/FallDelay7'
 * '<S18>'  : 'VSDP_Model/VSDP/60ms/CtrlStEn/MappingUint8'
 * '<S19>'  : 'VSDP_Model/VSDP/60ms/CtrlStEn/RiseDelay1'
 * '<S20>'  : 'VSDP_Model/VSDP/60ms/CtrlStEn/RiseDelay2'
 * '<S21>'  : 'VSDP_Model/VSDP/60ms/CtrlStEn/RiseDelay3'
 * '<S22>'  : 'VSDP_Model/VSDP/60ms/CtrlStEn/RiseDelay4'
 * '<S23>'  : 'VSDP_Model/VSDP/60ms/CtrlStEn/RiseDelay5'
 * '<S24>'  : 'VSDP_Model/VSDP/60ms/CtrlStEn/RiseDelay6'
 * '<S25>'  : 'VSDP_Model/VSDP/60ms/CtrlStEn/MappingUint8/Set_bit'
 * '<S26>'  : 'VSDP_Model/VSDP/60ms/DrvSt/FallDelay6'
 * '<S27>'  : 'VSDP_Model/VSDP/60ms/DrvSt/FallDelay7'
 * '<S28>'  : 'VSDP_Model/VSDP/60ms/DrvSt/FallDelay8'
 * '<S29>'  : 'VSDP_Model/VSDP/60ms/DrvSt/MappingUint8'
 * '<S30>'  : 'VSDP_Model/VSDP/60ms/DrvSt/RiseDelay'
 * '<S31>'  : 'VSDP_Model/VSDP/60ms/DrvSt/MappingUint8/Set_bit'
 * '<S32>'  : 'VSDP_Model/VSDP/60ms/StError/ Edge'
 * '<S33>'  : 'VSDP_Model/VSDP/60ms/StError/ Edge1'
 * '<S34>'  : 'VSDP_Model/VSDP/60ms/StError/ Edge2'
 * '<S35>'  : 'VSDP_Model/VSDP/60ms/StError/ Edge3'
 * '<S36>'  : 'VSDP_Model/VSDP/60ms/StError/ Edge4'
 * '<S37>'  : 'VSDP_Model/VSDP/60ms/StError/ Edge5'
 * '<S38>'  : 'VSDP_Model/VSDP/60ms/StError/ Edge6'
 * '<S39>'  : 'VSDP_Model/VSDP/60ms/StError/ Edge7'
 * '<S40>'  : 'VSDP_Model/VSDP/60ms/StError/ Edge8'
 * '<S41>'  : 'VSDP_Model/VSDP/60ms/StError/ Edge9'
 * '<S42>'  : 'VSDP_Model/VSDP/60ms/StError/FF'
 * '<S43>'  : 'VSDP_Model/VSDP/60ms/StError/FF1'
 * '<S44>'  : 'VSDP_Model/VSDP/60ms/StError/FF2'
 * '<S45>'  : 'VSDP_Model/VSDP/60ms/StError/FF3'
 * '<S46>'  : 'VSDP_Model/VSDP/60ms/StError/FF4'
 * '<S47>'  : 'VSDP_Model/VSDP/60ms/StError/FF5'
 * '<S48>'  : 'VSDP_Model/VSDP/60ms/StError/FF6'
 * '<S49>'  : 'VSDP_Model/VSDP/60ms/StError/FF7'
 * '<S50>'  : 'VSDP_Model/VSDP/60ms/StError/FF8'
 * '<S51>'  : 'VSDP_Model/VSDP/60ms/StError/FF9'
 * '<S52>'  : 'VSDP_Model/VSDP/60ms/StError/FallDelay1'
 * '<S53>'  : 'VSDP_Model/VSDP/60ms/StError/FallDelay10'
 * '<S54>'  : 'VSDP_Model/VSDP/60ms/StError/FallDelay11'
 * '<S55>'  : 'VSDP_Model/VSDP/60ms/StError/FallDelay12'
 * '<S56>'  : 'VSDP_Model/VSDP/60ms/StError/FallDelay13'
 * '<S57>'  : 'VSDP_Model/VSDP/60ms/StError/FallDelay14'
 * '<S58>'  : 'VSDP_Model/VSDP/60ms/StError/FallDelay15'
 * '<S59>'  : 'VSDP_Model/VSDP/60ms/StError/FallDelay16'
 * '<S60>'  : 'VSDP_Model/VSDP/60ms/StError/FallDelay2'
 * '<S61>'  : 'VSDP_Model/VSDP/60ms/StError/FallDelay9'
 * '<S62>'  : 'VSDP_Model/VSDP/60ms/StError/MappingUint8'
 * '<S63>'  : 'VSDP_Model/VSDP/60ms/StError/RiseDelay1'
 * '<S64>'  : 'VSDP_Model/VSDP/60ms/StError/RiseDelay10'
 * '<S65>'  : 'VSDP_Model/VSDP/60ms/StError/RiseDelay2'
 * '<S66>'  : 'VSDP_Model/VSDP/60ms/StError/RiseDelay3'
 * '<S67>'  : 'VSDP_Model/VSDP/60ms/StError/RiseDelay4'
 * '<S68>'  : 'VSDP_Model/VSDP/60ms/StError/RiseDelay5'
 * '<S69>'  : 'VSDP_Model/VSDP/60ms/StError/RiseDelay6'
 * '<S70>'  : 'VSDP_Model/VSDP/60ms/StError/RiseDelay7'
 * '<S71>'  : 'VSDP_Model/VSDP/60ms/StError/RiseDelay8'
 * '<S72>'  : 'VSDP_Model/VSDP/60ms/StError/RiseDelay9'
 * '<S73>'  : 'VSDP_Model/VSDP/60ms/StError/TimerRetrigger'
 * '<S74>'  : 'VSDP_Model/VSDP/60ms/StError/TimerRetrigger1'
 * '<S75>'  : 'VSDP_Model/VSDP/60ms/StError/TimerRetrigger2'
 * '<S76>'  : 'VSDP_Model/VSDP/60ms/StError/TimerRetrigger3'
 * '<S77>'  : 'VSDP_Model/VSDP/60ms/StError/TimerRetrigger4'
 * '<S78>'  : 'VSDP_Model/VSDP/60ms/StError/TimerRetrigger5'
 * '<S79>'  : 'VSDP_Model/VSDP/60ms/StError/TimerRetrigger6'
 * '<S80>'  : 'VSDP_Model/VSDP/60ms/StError/TimerRetrigger7'
 * '<S81>'  : 'VSDP_Model/VSDP/60ms/StError/TimerRetrigger8'
 * '<S82>'  : 'VSDP_Model/VSDP/60ms/StError/TimerRetrigger9'
 * '<S83>'  : 'VSDP_Model/VSDP/60ms/StError/MappingUint8/Set_bit'
 * '<S84>'  : 'VSDP_Model/VSDP/60ms/VehStIvld/DctcNoDayTime'
 * '<S85>'  : 'VSDP_Model/VSDP/60ms/VehStIvld/DtctSteAglStop'
 * '<S86>'  : 'VSDP_Model/VSDP/60ms/VehStIvld/GrIvldEnga'
 * '<S87>'  : 'VSDP_Model/VSDP/60ms/VehStIvld/MappingUint8'
 * '<S88>'  : 'VSDP_Model/VSDP/60ms/VehStIvld/WiperContiEn'
 * '<S89>'  : 'VSDP_Model/VSDP/60ms/VehStIvld/DctcNoDayTime/Bit Get'
 * '<S90>'  : 'VSDP_Model/VSDP/60ms/VehStIvld/DctcNoDayTime/Bit Get1'
 * '<S91>'  : 'VSDP_Model/VSDP/60ms/VehStIvld/DctcNoDayTime/Bit Get2'
 * '<S92>'  : 'VSDP_Model/VSDP/60ms/VehStIvld/DctcNoDayTime/Bit Get3'
 * '<S93>'  : 'VSDP_Model/VSDP/60ms/VehStIvld/DctcNoDayTime/Bit Get4'
 * '<S94>'  : 'VSDP_Model/VSDP/60ms/VehStIvld/DctcNoDayTime/Bit Get5'
 * '<S95>'  : 'VSDP_Model/VSDP/60ms/VehStIvld/DctcNoDayTime/FallDelay10'
 * '<S96>'  : 'VSDP_Model/VSDP/60ms/VehStIvld/DctcNoDayTime/RiseDelay1'
 * '<S97>'  : 'VSDP_Model/VSDP/60ms/VehStIvld/DtctSteAglStop/FF9'
 * '<S98>'  : 'VSDP_Model/VSDP/60ms/VehStIvld/GrIvldEnga/FF8'
 * '<S99>'  : 'VSDP_Model/VSDP/60ms/VehStIvld/MappingUint8/Set_bit'
 * '<S100>' : 'VSDP_Model/VSDP/60ms/VehStIvld/WiperContiEn/Bit Get'
 * '<S101>' : 'VSDP_Model/VSDP/60ms/VehStIvld/WiperContiEn/Bit Get1'
 * '<S102>' : 'VSDP_Model/VSDP/60ms/VehStIvld/WiperContiEn/Bit Get2'
 * '<S103>' : 'VSDP_Model/VSDP/60ms/VehStIvld/WiperContiEn/Bit Get3'
 * '<S104>' : 'VSDP_Model/VSDP/60ms/VehStIvld/WiperContiEn/Bit Get4'
 * '<S105>' : 'VSDP_Model/VSDP/60ms/VehStIvld/WiperContiEn/Bit Get5'
 * '<S106>' : 'VSDP_Model/VSDP/60ms/VehStIvld/WiperContiEn/Bit Get6'
 * '<S107>' : 'VSDP_Model/VSDP/60ms/VehStIvld/WiperContiEn/FallDelay10'
 * '<S108>' : 'VSDP_Model/VSDP/60ms/VehStIvld/WiperContiEn/RiseDelay1'
 * '<S109>' : 'VSDP_Model/VSDP/60ms/VehStIvld/WiperContiEn/RiseDelay2'
 */

/*-
 * Requirements for '<Root>': VSDP
 */
#endif /* RTW_HEADER_VSDP_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
