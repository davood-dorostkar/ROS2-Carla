/**********************************Model Property********************************
 *
 * Company             : SENSETIME
 *
 * Tool Version        : Ver2.0
 *
 * Model Name          : LDPSA
 *
 * Model Long Name     : Lane Departure Prevention

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
 * File                             : LDPSA_private.h
 *
 * FileType                         : Code Header File
 *
 * Real-Time Workshop file version  : 9.4 (R2020b) 29-Jul-2020
 *
 * TLC version                      : 9.4 (Aug 20 2020)
 *
 * C source code generated on       : Sun Jan 15 16:16:18 2023
 *
 * Copyright (C) by SenseTime Group Limited. All rights reserved.
 *******************************************************************************/

#ifndef RTW_HEADER_LDPSA_private_h_
#define RTW_HEADER_LDPSA_private_h_
#include "rtwtypes.h"

/* Imported (extern) block signals */
extern real32_T LDPSAI_VehWid_Mi;      /* '<Root>/Inport'
                                        * vehicle width
                                        */
extern real32_T LDPSAI_VehSpdActu_Mps; /* '<Root>/Inport1'
                                        * Vehicle speed
                                        */
extern real32_T LDPSAI_SpdVelShow_Kmph;/* '<Root>/Inport2'
                                        * Tachometer Vehicle Speed in kilometers/hour
                                        */
extern real32_T LDPSAI_VehAccSpdX_Npkg;/* '<Root>/Inport3'
                                        * Longitude acceleration spped
                                        */
extern real32_T LDPSAI_VehAccSpdY_Npkg;/* '<Root>/Inport4'
                                        * Lateral acceleration spped
                                        */
extern real32_T LDPSAI_VehCurv_ReMi;   /* '<Root>/Inport5'
                                        * Curvature of vehicle
                                        */
extern uint8_T LDPSAI_TrnSgl_St;       /* '<Root>/Inport6'
                                        * State of turn signal
                                        */
extern real32_T LDPSAI_WheSteAgl_Dgr;  /* '<Root>/Inport7'
                                        * Degrees of vehicle steer wheel angle
                                        */
extern real32_T LDPSAI_SteAglSpd_Dgpm; /* '<Root>/Inport8'
                                        * vehicle steer wheel  angle speed
                                        */
extern boolean_T LDPSAI_LDPSwitchEn_B; /* '<Root>/Inport9'
                                        * LDP switch of driver
                                        */
extern uint8_T LDPSAI_LDPMod_St;       /* '<Root>/Inport10'
                                        * Driver control mode of LDP
                                        */
extern boolean_T LDPSAI_LDPErrCdtn_B;  /* '<Root>/Inport11'
                                        * LDP switch of driver
                                        */
extern boolean_T LDPSAI_DtctLnChag_B;  /* '<Root>/Inport12'
                                        * Condition of change detected lane
                                        */
extern real32_T LDPSAI_LnWidCalc_Mi;   /* '<Root>/Inport13'
                                        * Lane width
                                        */
extern real32_T LDPSAI_PstnXLf_Mi;     /* '<Root>/Inport14'
                                        * Filtered left lane clothoid X0 position
                                        */
extern real32_T LDPSAI_PstnXRi_Mi;     /* '<Root>/Inport15'
                                        * Filtered right lane clothoid X0 position
                                        */
extern real32_T LDPSAI_PstnYLf_Mi;     /* '<Root>/Inport16'
                                        * Distance between left lane and Y0
                                        */
extern real32_T LDPSAI_PstnYSafeLf_Mi; /* '<Root>/Inport17'
                                        * Safe distance between left lane and Y0
                                        */
extern real32_T LDPSAI_PstnYRi_Mi;     /* '<Root>/Inport18'
                                        * Distance between right lane and Y0
                                        */
extern real32_T LDPSAI_PstnYSafeRi_Mi; /* '<Root>/Inport19'
                                        * Safe distance between right lane and Y0
                                        */
extern real32_T LDPSAI_HeadAglLf_Rad;  /* '<Root>/Inport20'
                                        * Heading angle of left lane clothoid
                                        */
extern real32_T LDPSAI_HeadAglSafeLf_Rad;/* '<Root>/Inport21'
                                          * Safe yaw angle of left lane
                                          */
extern real32_T LDPSAI_HeadAglRi_Rad;  /* '<Root>/Inport22'
                                        * Heading angle of right lane clothoid
                                        */
extern real32_T LDPSAI_HeadAglSafeRi_Rad;/* '<Root>/Inport23'
                                          * Safe yaw angle of right lane
                                          */
extern real32_T LDPSAI_CurvLf_ReMi;    /* '<Root>/Inport24'
                                        * Curvature of left lane clothoid
                                        */
extern real32_T LDPSAI_CurvSafeLf_ReMi;/* '<Root>/Inport25'
                                        * Safe curvature of left lane
                                        */
extern real32_T LDPSAI_CurvRi_ReMi;    /* '<Root>/Inport26'
                                        * Curvature of right lane clothoid
                                        */
extern real32_T LDPSAI_CurvSafeRi_ReMi;/* '<Root>/Inport27'
                                        * Safe curvature of rihgt lane
                                        */
extern real32_T LDPSAI_CurvRateLf_ReMi2;/* '<Root>/Inport28'
                                         * Filtered left lane clothoid change of curvature
                                         */
extern real32_T LDPSAI_CurvRateRi_ReMi2;/* '<Root>/Inport29'
                                         * Filtered right lane clothoid change of curvature
                                         */
extern real32_T LDPSAI_VldLengLf_Mi;   /* '<Root>/Inport32'
                                        * Filtered left lane clothoid length
                                        */
extern real32_T LDPSAI_VldLengRi_Mi;   /* '<Root>/Inport33'
                                        * Filtered right lane clothoid length
                                        */
extern uint8_T LDPSAI_IvldLnSafeLf_St; /* '<Root>/Inport34'
                                        * Invalid safe state of left lane
                                        */
extern uint16_T LDPSAI_LnIVldLf_St;    /* '<Root>/Inport35'
                                        * Invalid state of left lane
                                        */
extern uint8_T LDPSAI_IvldLnSafeRi_St; /* '<Root>/Inport36'
                                        * Invalid safe state of right lane
                                        */
extern uint16_T LDPSAI_LnIVldRi_St;    /* '<Root>/Inport37'
                                        * Invalid state of left lane
                                        */
extern uint16_T LDPSAI_VehStIvld_St;   /* '<Root>/Inport38'
                                        * Invalid state of vehicle
                                        */
extern uint8_T LDPSAI_IvldStDrv_St;    /* '<Root>/Inport39'
                                        * Invalid state of driver
                                        */
extern uint8_T LDPSAI_CtrlStEn_St;     /* '<Root>/Inport40'
                                        * Active state of driver control
                                        */
extern uint8_T LDPSAI_StError_St;      /* '<Root>/Inport41'
                                        * Error state of vehicle system
                                        */
extern uint8_T LDPSAI_CtrlStNoAvlb_St; /* '<Root>/Inport42'
                                        * Not Avaible state of vehicle system
                                        */
extern uint8_T LDPSAI_PrjSpecQu_St;    /* '<Root>/Inport43' */
extern boolean_T LDPSAI_DtctCstruSite_B;/* '<Root>/Inport44'
                                         * Condition of Construction site
                                         */
extern real32_T LDPSAI_CycleTime_Sec;  /* '<Root>/Inport45'
                                        * Cycle time
                                        */
extern real32_T LDPSAI_ABDTimeStamp_Sec;/* '<Root>/Inport46'
                                         * Data timestamp in seconds
                                         */
extern real32_T LDPSAI_PstnYCent_Mi;   /* '<Root>/Inport30' */
extern real32_T LDPSAI_VehYawRate_rps; /* '<Root>/Inport31'
                                        * Vehicle YawRate
                                        */
extern boolean_T LDPSAI_AEBActive_B;   /* '<Root>/Inport47'
                                        * AEB Active
                                        */
extern boolean_T NVRAM_LDPSwitch_B;    /* '<Root>/Inport48'
                                        * NVRAM LDP Switch
                                        */
extern real32_T NVRAM_LDPStartupSpd_Kmph;/* '<Root>/Inport49'
                                          * Vehicle VehStartupSpd
                                          */
extern real32_T LDPSAI_VehStartupSpdHMI_Kmph;/* '<Root>/Inport50'
                                              * Vehicle VehStartupSpd setting
                                              */

#endif                                 /* RTW_HEADER_LDPSA_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
