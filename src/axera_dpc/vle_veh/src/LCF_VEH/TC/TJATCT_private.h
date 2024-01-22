/**********************************Model Property********************************
 *
 * Company             : PHIGENT
 *
 * Tool Version        : Ver2.0
 *
 * Model Name          : TJATCT
 *
 * Model Long Name     : Trajectoty control

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

 * Model Cycle Time    : 20ms


 ************************************Auto Coder**********************************
 *
 * File                             : TJATCT_private.h
 *
 * FileType                         : Code Header File
 *
 * Real-Time Workshop file version  : 9.4 (R2020b) 29-Jul-2020
 *
 * TLC version                      : 9.4 (Aug 20 2020)
 *
 * C source code generated on       : Tue May  9 16:33:31 2023
 *
 * Copyright (C) by PhiGent Group Limited. All rights reserved.
 *******************************************************************************/

#ifndef RTW_HEADER_TJATCT_private_h_
#define RTW_HEADER_TJATCT_private_h_
#include "rtwtypes.h"

/* Imported (extern) block signals */
extern real32_T TCTI_TimeSysCycle;     /* '<Root>/TCTI_TimeSysCycle'
                                        * Cycle Time for VEH task

                                        */
extern real32_T TCTI_VehicleVelX;      /* '<Root>/TCTI_VehicleVelX'
                                        * Vehicle Speed in miles/hour, computed based on the wheel speeds
                                          Negative values when vehicle moves backward

                                        */
extern real32_T TCTI_VehYawRate;       /* '<Root>/TCTI_VehYawRate' */
extern real32_T TCTI_WhlSteerAngleVdy; /* '<Root>/TCTI_WhlSteerAngleVdy'
                                        * Offset compensated steering wheel angle

                                        */
extern real32_T TCTI_SteerAngleLaDmc;  /* '<Root>/TCTI_SteerAngleLaDmc'
                                        * Offset compensated steer angle at the front wheels provided by the LatDMC

                                        */
extern boolean_T TCTI_EnaReplanCurValues;/* '<Root>/TCTI_EnaReplanCurValues' */
extern uint8_T TCTI_BtfTrajGuiQualifier;/* '<Root>/TCTI_BtfTrajGuiQualifier'
                                         * Trajectory Guidance Qualifier send by TPLFBT
                                         */
extern real32_T TCTI_ReqTrajDistYTpl;  /* '<Root>/TCTI_ReqTrajDistYTpl' */
extern real32_T TCTI_CurTrajDistYTpl;  /* '<Root>/TCTI_CurTrajDistYTpl'
                                        * Current lateral distance for trajectory plan
                                        */
extern real32_T TCTI_ReqTrajCrvTpl;    /* '<Root>/TCTI_ReqTrajCrvTpl'
                                        * Trajectory planning setting curvature

                                        */
extern uint8_T TCTI_StCntrlFcn;        /* '<Root>/TCTI_StCntrlFcn'
                                        * Carries which function is allowed to control

                                        */
extern uint8_T TCTI_StLatCtrlMode;     /* '<Root>/TCTI _StLatCtrlMode' */
extern real32_T TCTI_EstSelfSteerGrdnt;/* '<Root>/TCTI_EstSelfSteerGrdnt'
                                        * Estimated self steering gradient of the vehicle



                                        */
extern real32_T TCTI_ReqTrajCrvCsc;    /* '<Root>/TCTI_ReqTrajCrvCsc' */
extern real32_T TCTI_ReqTrajHeadTpl;   /* '<Root>/TCTI_ReqTrajHeadTpl' */
extern real32_T TCTI_InclPrevTrajHeadTpl;/* '<Root>/TCTI_InclPrevTrajHeadTpl' */
extern real32_T TCTI_MaxCrvGrdBuildup; /* '<Root>/TCTI_MaxCrvGrdBuildup' */
extern real32_T TCTI_MaxCrvGrdRed;     /* '<Root>/TCTI_MaxCrvGrdRed' */
extern real32_T TCTI_LmtReqTrajCrvGrd; /* '<Root>/TCTI_LmtReqTrajCrvGrd' */
extern real32_T TCTI_MaxTrajCrv;       /* '<Root>/TCTI_MaxTrajCrv' */
extern uint8_T TCTI_StVehOdo;          /* '<Root>/TCTI_StVehOdo' */
extern real32_T TCTI_VehCurvature;     /* '<Root>/TCTI_VehCrv'
                                        * Vehicle curvature



                                        */
extern uint8_T TCTI_EnaLmtActCsc;      /* '<Root>/TCTI_EnaLmtActCsc' */
extern real32_T TCTI_TimeLmtDur;       /* '<Root>/TCTI_TimeLmtDur'
                                        * CSCLTA_OUTPUT

                                        */
extern uint8_T TCTI_StLaneLaKmc;       /* '<Root>/TCTI_StLaneLaKmc'
                                        * Side of xDP Intervention



                                        */
extern uint8_T TCTI_StLcfSys;          /* '<Root>/TCTI_StLcfSys' */
extern uint8_T TestFlag_EnableFF;      /* '<Root>/TCTI_ActualTrqEPS1' */
extern uint8_T TestFlag_EnableTP;      /* '<Root>/TCTI_ActualTrqEPS2' */
extern uint8_T TestFlag_EnableOL;      /* '<Root>/TCTI_ActualTrqEPS3' */
extern real32_T Test_CoeffMainPGainLdc;/* '<Root>/Outport32' */
extern real32_T Test_CoeffPGainLdc;    /* '<Root>/Outport33' */
extern real32_T Test_CoeffIGainLdc;    /* '<Root>/Outport34' */
extern real32_T Test_CoeffDGainLdc;    /* '<Root>/Outport35' */
extern real32_T Test_TimeDT1Ldc;       /* '<Root>/Outport36' */
extern real32_T Test_CoeffPT1GainLdc;  /* '<Root>/Outport37' */
extern real32_T Test_TimePT1Ldc;       /* '<Root>/Outport38' */
extern real32_T Test_CoeffMainPGainCac;/* '<Root>/Outport39' */
extern real32_T Test_CoeffPGainCac;    /* '<Root>/Outport40' */
extern real32_T Test_CoeffIGainCac;    /* '<Root>/Outport41' */
extern real32_T Test_CoeffDGainCac;    /* '<Root>/Outport42' */
extern real32_T Test_TimeDT1Cac;       /* '<Root>/Outport43' */
extern real32_T Test_CoeffPT1GainCac;  /* '<Root>/Outport44' */
extern real32_T Test_TimePT1Cac;       /* '<Root>/Outport45' */
extern real32_T Test_CDCTimeFltCurHeading;/* '<Root>/Outport46' */
extern real32_T LQR_e1_gains[9];       /* '<Root>/Inport' */
extern real32_T LQR_e1dot_gains[9];    /* '<Root>/Inport1' */
extern real32_T LQR_e2_gains[9];       /* '<Root>/Inport2' */
extern real32_T LQR_e2dot_gains[9];    /* '<Root>/Inport3' */
extern real32_T LQR_Feedforward_gains[9];/* '<Root>/Inport4' */

#endif                                 /* RTW_HEADER_TJATCT_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
