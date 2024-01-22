/**********************************Model Property********************************
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


 ************************************Auto Coder**********************************
 *
 * File                             : VSDP_private.h
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

#ifndef RTW_HEADER_VSDP_private_h_
#define RTW_HEADER_VSDP_private_h_
#include "rtwtypes.h"

/* Imported (extern) block signals */
extern boolean_T VSDPI_CtrlStEnABS_B;  /* '<Root>/In'
                                        * Condition of ABS in actively controlling
                                        */
extern boolean_T VSDPI_CtrlStEnACC_B;  /* '<Root>/In1'
                                        * Condition of ACC in actively controlling
                                        */
extern boolean_T VSDPI_CtrlStEnESC_B;  /* '<Root>/In2'
                                        * Condition of ESC in actively controlling
                                        */
extern boolean_T VSDPI_CtrlStEnTCS_B;  /* '<Root>/In3'
                                        * Condition of TSC in actively controlling
                                        */
extern boolean_T VSDPI_CtrlStEnVSM_B;  /* '<Root>/In4'
                                        * Condition of VSM in actively controlling
                                        */
extern boolean_T VSDPI_CtrlStEnEBA_B;  /* '<Root>/In5'
                                        * Condition of EBA in actively controlling
                                        */
extern boolean_T VSDPI_CtrlStAvlbABS_B;/* '<Root>/In6'
                                        * Condition of ABS is available
                                        */
extern boolean_T VSDPI_CtrlStAvlbACC_B;/* '<Root>/In7'
                                        * Condition of ACC is available
                                        */
extern boolean_T VSDPI_CtrlStAvlbESC_B;/* '<Root>/In8'
                                        * Condition of ESC is available
                                        */
extern boolean_T VSDPI_CtrlStAvlbTCS_B;/* '<Root>/In9'
                                        * Condition of TSC is available
                                        */
extern boolean_T VSDPI_CtrlStAvlbVSM_B;/* '<Root>/In10'
                                        * Condition of VSM is available
                                        */
extern boolean_T VSDPI_CtrlStAvlbEBA_B;/* '<Root>/In11'
                                        * Condition of EBA is available
                                        */
extern real32_T VSDPI_AccPedPstn_Per;  /* '<Root>/In12'
                                        * Acceleration pedal position
                                        */
extern boolean_T VSDPI_DrvNoBuckledUp_B;/* '<Root>/In13'
                                         * Condition of the driver is not buckled
                                         */
extern boolean_T VSDPI_TrnSglHarLigEn_B;/* '<Root>/In14'
                                         * Condition of turn hazard light is on
                                         */
extern boolean_T VSDPI_TrnSglEnLf_B;   /* '<Root>/In15'
                                        * Condition of left turn signal is on
                                        */
extern boolean_T VSDPI_TrnSglEnRi_B;   /* '<Root>/In16'
                                        * Condition of right turn signal is on
                                        */
extern uint8_T VSDPI_SysWarn_St;       /* '<Root>/In17'
                                        * State of lateral Adas system
                                        */
extern real32_T VSDPI_ManuActuTrqEPS_Nm;/* '<Root>/In18'
                                         * Actual manual torque of EPS
                                         */
extern boolean_T VSDPI_StErrABS_B;     /* '<Root>/In19'
                                        * Condition of ABS is in error state
                                        */
extern boolean_T VSDPI_StErrACC_B;     /* '<Root>/In20'
                                        * Condition of ACC is in error state
                                        */
extern boolean_T VSDPI_StErrESC_B;     /* '<Root>/In21'
                                        * Condition of ESC is in error state
                                        */
extern boolean_T VSDPI_StErrTSC_B;     /* '<Root>/In22'
                                        * Condition of TSC is in error state
                                        */
extern boolean_T VSDPI_StErrVSM_B;     /* '<Root>/In23'
                                        * Condition of VSM is in error state
                                        */
extern boolean_T VSDPI_StErrVDY_B;     /* '<Root>/In24'
                                        * Condition of VDY is in error state
                                        */
extern boolean_T VSDPI_StErrLatDMC_B;  /* '<Root>/In25'
                                        * Condition of LatDMC is in error state
                                        */
extern boolean_T VSDPI_StErrEBA_B;     /* '<Root>/In26'
                                        * Condition of EBA is in error state
                                        */
extern boolean_T VSDPI_DoorOpen_B;     /* '<Root>/In27'
                                        * Condition of the door is opened
                                        */
extern boolean_T VSDPI_VehRdyToSta_B;  /* '<Root>/In28'
                                        * Condition of vehicle is ready to start
                                        */
extern boolean_T VSDPI_TrailerExs_B;   /* '<Root>/In29'
                                        * Condition of trailer is attached
                                        */
extern boolean_T VSDPI_GrRvs_B;        /* '<Root>/In30'
                                        * Condition of the reverse gear is engaged
                                        */
extern boolean_T VSDPI_GrPark_B;       /* '<Root>/In31'
                                        * Condition of the parking gear is engaged
                                        */
extern boolean_T VSDPI_GrNeu_B;        /* '<Root>/In32'
                                        * Condition of the neutral gear is engaged
                                        */
extern boolean_T VSDPI_VehMoveBkwd_B;  /* '<Root>/In33'
                                        * Condition of vehicles moves backward
                                        */
extern boolean_T VSDPI_DtctOverSte_B;  /* '<Root>/In34'
                                        * Condition of oversteering has been detected
                                        */
extern boolean_T VSDPI_DtctUnderSte_B; /* '<Root>/In35'
                                        * Condition of understeering has been detected
                                        */
extern boolean_T VSDPI_DtctRollerBench_B;/* '<Root>/In36'
                                          * Condition of vehicle is on roller bench
                                          */
extern real32_T VSDPI_VehSpdX_Mps;     /* '<Root>/In37'
                                        * Vehicle Speed
                                        */
extern real32_T VSDPI_SteAglFrt_Rad;   /* '<Root>/In38'
                                        * Effective steering angle at front axle
                                        */
extern uint8_T VSDPI_StateWiper_St;    /* '<Root>/In39'
                                        * Wiper State
                                        */
extern uint8_T VSDPI_StageWiper_St;    /* '<Root>/In40'
                                        * Wiper state
                                        */
extern uint8_T VSDPI_StBrightness_St;  /* '<Root>/In41'
                                        * Brightmess state
                                        */
extern real32_T VSDPI_CycleTime_Sec;   /* '<Root>/In42'
                                        * VSDP cycle time
                                        */
extern boolean_T VSDPI_BrakePedalApplied_B;/* '<Root>/In43'
                                            * Condition of ABS is in error state
                                            */
extern boolean_T VSDPI_CtrlStEnARP_B;  /* '<Root>/In44'
                                        * Condition of ABS in actively controlling
                                        */
extern boolean_T VSDPI_CtrlStEnHDC_B;  /* '<Root>/In45'
                                        * Condition of ABS in actively controlling
                                        */
extern boolean_T VSDPI_StErrARP_B;     /* '<Root>/In46'
                                        * Condition of ABS in actively controlling
                                        */
extern boolean_T VSDPI_StErrHDC_B;     /* '<Root>/In47'
                                        * Condition of ABS in actively controlling
                                        */
extern boolean_T VSDPI_BrakeDiscTempSts_B;/* '<Root>/In48'
                                           * Condition of ABS in actively controlling
                                           */
extern boolean_T VSDPI_SignalInvalidLongi_B;/* '<Root>/Inport3'
                                             * Condition of ABS in actively controlling
                                             */
extern boolean_T VSDPI_SignalInvalidLat_B;/* '<Root>/Inport1'
                                           * Condition of ABS in actively controlling
                                           */

#endif                                 /* RTW_HEADER_VSDP_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
