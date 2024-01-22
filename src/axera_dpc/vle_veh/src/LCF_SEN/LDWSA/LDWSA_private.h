/**********************************Model Property********************************
 *
 * Company             : SENSETIME
 *
 * Tool Version        : Ver2.0
 *
 * Model Name          : LDWSA
 *
 * Model Long Name     : Lane Departure Warning

 *

 * Model Advisor       : Not Check

 *

 * Model Version       : Ver_02

 *

 * Model Author        : WJ

 *

 * Model Reviewer      :

 *

 * Model Review Data   :

 *

 * Model Cycle Time    : 50ms


 ************************************Auto Coder**********************************
 *
 * File                             : LDWSA_private.h
 *
 * FileType                         : Code Header File
 *
 * Real-Time Workshop file version  : 9.4 (R2020b) 29-Jul-2020
 *
 * TLC version                      : 9.4 (Aug 20 2020)
 *
 * C source code generated on       : Sun Jan 15 13:14:35 2023
 *
 * Copyright (C) by SenseTime Group Limited. All rights reserved.
 *******************************************************************************/

#ifndef RTW_HEADER_LDWSA_private_h_
#define RTW_HEADER_LDWSA_private_h_
#include "rtwtypes.h"

/* Imported (extern) block signals */
extern real32_T LDWSAI_VehWid_Mi;      /* '<Root>/Inport'
                                        * vehicle width
                                        */
extern real32_T LDWSAI_VehSpdActu_Mps; /* '<Root>/Inport1'
                                        * Vehicle speed
                                        */
extern real32_T LDWSAI_SpdVelShow_Kmph;/* '<Root>/Inport2'
                                        * Tachometer Vehicle Speed in kilometers/hour
                                        */
extern real32_T LDWSAI_VehAccSpdX_Npkg;/* '<Root>/Inport3'
                                        * Longitude acceleration spped
                                        */
extern real32_T LDWSAI_VehAccSpdY_Npkg;/* '<Root>/Inport4'
                                        * Lateral acceleration spped
                                        */
extern real32_T LDWSAI_VehCurv_ReMi;   /* '<Root>/Inport5'
                                        * Curvature of vehicle
                                        */
extern uint8_T LDWSAI_TrnSgl_St;       /* '<Root>/Inport6'
                                        * State of turn signal
                                        */
extern real32_T LDWSAI_WheSteAgl_Dgr;  /* '<Root>/Inport7'
                                        * Degrees of vehicle steer wheel angle
                                        */
extern real32_T LDWSAI_SteAglSpd_Dgpm; /* '<Root>/Inport8'
                                        * vehicle steer wheel  angle speed
                                        */
extern boolean_T LDWSAI_LDWSwitchEn_B; /* '<Root>/Inport9'
                                        * LDW switch of driver
                                        */
extern uint8_T LDWSAI_LDWMod_St;       /* '<Root>/Inport10'
                                        * Driver control mode of LDW
                                        */
extern boolean_T LDWSAI_LDWErrCdtn_B;  /* '<Root>/Inport11'
                                        * Error condition of LDW
                                        */
extern boolean_T LDWSAI_DtctLnChag_B;  /* '<Root>/Inport12'
                                        * Condition of change detected lane
                                        */
extern real32_T LDWSAI_LnWidCalc_Mi;   /* '<Root>/Inport13'
                                        * Lane width
                                        */
extern real32_T LDWSAI_PstnYLf_Mi;     /* '<Root>/Inport16'
                                        * Distance between left lane and Y0
                                        */
extern real32_T LDWSAI_PstnYSafeLf_Mi; /* '<Root>/Inport17'
                                        * Safe distance between left lane and Y0
                                        */
extern real32_T LDWSAI_PstnYRi_Mi;     /* '<Root>/Inport18'
                                        * Distance between right lane and Y0
                                        */
extern real32_T LDWSAI_PstnYSafeRi_Mi; /* '<Root>/Inport19'
                                        * Safe distance between right lane and Y0
                                        */
extern real32_T LDWSAI_HeadAglLf_Rad;  /* '<Root>/Inport20'
                                        * Heading angle of left lane clothoid
                                        */
extern real32_T LDWSAI_HeadAglSafeLf_Rad;/* '<Root>/Inport21'
                                          * Safe yaw angle of left lane
                                          */
extern real32_T LDWSAI_HeadAglRi_Rad;  /* '<Root>/Inport22'
                                        * Heading angle of right lane clothoid
                                        */
extern real32_T LDWSAI_HeadAglSafeRi_Rad;/* '<Root>/Inport23'
                                          * Safe yaw angle of right lane
                                          */
extern real32_T LDWSAI_CurvLf_ReMi;    /* '<Root>/Inport24'
                                        * Curvature of left lane clothoid
                                        */
extern real32_T LDWSAI_CurvSafeLf_ReMi;/* '<Root>/Inport25'
                                        * Safe curvature of left lane
                                        */
extern real32_T LDWSAI_CurvRi_ReMi;    /* '<Root>/Inport26'
                                        * Curvature of right lane clothoid
                                        */
extern real32_T LDWSAI_CurvSafeRi_ReMi;/* '<Root>/Inport27'
                                        * Safe curvature of rihgt lane
                                        */
extern uint8_T LDWSAI_IvldLnSafeLf_St; /* '<Root>/Inport34'
                                        * Invalid safe state of left lane
                                        */
extern uint16_T LDWSAI_LnIVldLf_St;    /* '<Root>/Inport35'
                                        * Invalid state of left lane
                                        */
extern uint8_T LDWSAI_IvldLnSafeRi_St; /* '<Root>/Inport36'
                                        * Invalid safe state of right lane
                                        */
extern uint16_T LDWSAI_LnIVldRi_St;    /* '<Root>/Inport37'
                                        * Invalid state of left lane
                                        */
extern uint16_T LDWSAI_VehStIvld_St;   /* '<Root>/Inport38'
                                        * Invalid state of vehicle
                                        */
extern uint8_T LDWSAI_IvldStDrv_St;    /* '<Root>/Inport39'
                                        * Invalid state of driver
                                        */
extern uint8_T LDWSAI_CtrlStEn_St;     /* '<Root>/Inport40'
                                        * Active state of driver control
                                        */
extern uint8_T LDWSAI_StError_St;      /* '<Root>/Inport41'
                                        * Error state of vehicle system
                                        */
extern uint8_T LDWSAI_CtrlStNoAvlb_St; /* '<Root>/Inport42'
                                        * Not Avaible state of vehicle system
                                        */
extern uint8_T LDWSAI_PrjSpecQu_St;    /* '<Root>/Inport43' */
extern boolean_T LDWSAI_DtctCstruSite_B;/* '<Root>/Inport44'
                                         * Condition of Construction site
                                         */
extern real32_T LDWSAI_CycleTime_Sec;  /* '<Root>/Inport45'
                                        * Vehicle speed
                                        */
extern real32_T LDWSAI_VehYawRate_rps; /* '<Root>/Inport31'
                                        * Vehicle yawrate
                                        */
extern boolean_T LDWSAI_AEBActive_B;   /* '<Root>/Inport47'
                                        * AEB active status
                                        */
extern boolean_T NVRAM_LDWSwitch_B;    /* '<Root>/Inport48'
                                        * NVRAM LDW switch
                                        */
extern real32_T NVRAM_LDWStartupSpd_Kmph;/* '<Root>/Inport49'
                                          * NVRAM vehicle startup speed
                                          */
extern real32_T LDWSAI_VehStartupSpdHMI_Kmph;/* '<Root>/Inport50'
                                              * Vehicle startup speed HMI setting
                                              */
extern boolean_T LDWSAI_LDPSwitchOn_B; /* '<Root>/Inport14'
                                        * Error condition of LDW
                                        */
extern real32_T LDWSAI_LnLengthLf_Mi;  /* '<Root>/Inport15' */
extern real32_T LDWSAI_LnLengthRi_Mi;  /* '<Root>/Inport28' */

#endif                                 /* RTW_HEADER_LDWSA_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
