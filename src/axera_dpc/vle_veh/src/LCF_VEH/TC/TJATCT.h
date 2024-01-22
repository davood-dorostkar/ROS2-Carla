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
 * File                             : TJATCT.h
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

#ifndef RTW_HEADER_TJATCT_h_
#define RTW_HEADER_TJATCT_h_
#include <math.h>
#include <string.h>
#ifndef TJATCT_COMMON_INCLUDES_
#define TJATCT_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* TJATCT_COMMON_INCLUDES_ */

#include "TJATCT_types.h"

/* Macros for accessing real-time model data structure */

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real32_T UnitDelay_DSTATE[16];       /* '<S608>/Unit Delay' */
  real32_T UnitDelay3_DSTATE[4];       /* '<S308>/Unit Delay3' */
  real32_T Unit_Delay_DSTATE;          /* '<S305>/Unit_Delay' */
  real32_T Unit_Delay1_DSTATE;         /* '<S305>/Unit_Delay1' */
  real32_T Unit_Delay2_DSTATE;         /* '<S305>/Unit_Delay2' */
  real32_T Unit_Delay3_DSTATE;         /* '<S305>/Unit_Delay3' */
  real32_T Unit_Delay4_DSTATE;         /* '<S305>/Unit_Delay4' */
  real32_T FixPtUnitDelay1_DSTATE;     /* '<S381>/FixPt Unit Delay1' */
  real32_T UnitDelay_DSTATE_m;         /* '<S396>/Unit Delay' */
  real32_T UnitDelay_DSTATE_p;         /* '<S400>/Unit Delay' */
  real32_T UnitDelay_DSTATE_p0;        /* '<S398>/Unit Delay' */
  real32_T Unit_Delay_DSTATE_h[2];     /* '<S385>/Unit_Delay' */
  real32_T Unit_Delay_DSTATE_m;        /* '<S386>/Unit_Delay' */
  real32_T Unit_Delay1_DSTATE_j;       /* '<S386>/Unit_Delay1' */
  real32_T Unit_Delay2_DSTATE_g;       /* '<S386>/Unit_Delay2' */
  real32_T Unit_Delay3_DSTATE_j;       /* '<S386>/Unit_Delay3' */
  real32_T Unit_Delay4_DSTATE_j;       /* '<S386>/Unit_Delay4' */
  real32_T FixPtUnitDelay1_DSTATE_f;   /* '<S384>/FixPt Unit Delay1' */
  real32_T FixPtUnitDelay1_DSTATE_g;   /* '<S329>/FixPt Unit Delay1' */
  real32_T UnitDelay2_DSTATE_b;        /* '<S366>/Unit Delay2' */
  real32_T UnitDelay2_DSTATE_k;        /* '<S396>/Unit Delay2' */
  real32_T FixPtUnitDelay1_DSTATE_e;   /* '<S327>/FixPt Unit Delay1' */
  real32_T UnitDelay2_DSTATE_c;        /* '<S355>/Unit Delay2' */
  real32_T UnitDelay2_DSTATE_a;        /* '<S356>/Unit Delay2' */
  real32_T UnitDelay3_DSTATE_p;        /* '<S357>/Unit Delay3' */
  real32_T UnitDelay_DSTATE_f;         /* '<S363>/Unit Delay' */
  real32_T UnitDelay_DSTATE_b1;        /* '<S361>/Unit Delay' */
  real32_T Unit_Delay_DSTATE_n;        /* '<S92>/Unit_Delay' */
  real32_T UnitDelay_DSTATE_n;         /* '<S92>/Unit Delay' */
  real32_T UnitDelay1_DSTATE_a;        /* '<S92>/Unit Delay1' */
  real32_T UnitDelay2_DSTATE_ja;       /* '<S92>/Unit Delay2' */
  real32_T UnitDelay3_DSTATE_k;        /* '<S92>/Unit Delay3' */
  real32_T UnitDelay6_DSTATE;          /* '<S92>/Unit Delay6' */
  real32_T UnitDelay5_DSTATE;          /* '<S92>/Unit Delay5' */
  real32_T UnitDelay7_DSTATE;          /* '<S92>/Unit Delay7' */
  real32_T Unit_Delay1_DSTATE_d;       /* '<S609>/Unit_Delay1' */
  real32_T UnitDelay_DSTATE_e5;        /* '<S610>/Unit Delay' */
  real32_T FixPtUnitDelay1_DSTATE_o;   /* '<S616>/FixPt Unit Delay1' */
  real32_T Delay_DSTATE;               /* '<S603>/Delay' */
  real32_T Delay1_DSTATE;              /* '<S603>/Delay1' */
  real32_T Delay3_DSTATE;              /* '<S603>/Delay3' */
  real32_T UnitDelay_DSTATE_g;         /* '<S601>/Unit Delay' */
  real32_T UnitDelay1_DSTATE_i;        /* '<S627>/Unit Delay1' */
  real32_T UnitDelay_DSTATE_pa;        /* '<S628>/Unit Delay' */
  real32_T UnitDelay_DSTATE_bb;        /* '<S115>/Unit Delay' */
  real32_T UnitDelay_DSTATE_g4;        /* '<S571>/Unit Delay' */
  real32_T UnitDelay_DSTATE_ou;        /* '<S125>/Unit Delay' */
  real32_T FixPtUnitDelay1_DSTATE_c;   /* '<S556>/FixPt Unit Delay1' */
  real32_T UnitDelay1_DSTATE_c;        /* '<S539>/Unit Delay1' */
  real32_T FixPtUnitDelay1_DSTATE_h;   /* '<S534>/FixPt Unit Delay1' */
  real32_T Unit_Delay1_DSTATE_e;       /* '<S526>/Unit_Delay1' */
  real32_T UnitDelay_DSTATE_jb;        /* '<S529>/Unit Delay' */
  real32_T UnitDelay_DSTATE_lex;       /* '<S535>/Unit Delay' */
  real32_T UnitDelay_DSTATE_g5;        /* '<S536>/Unit Delay' */
  real32_T FixPtUnitDelay1_DSTATE_cy;  /* '<S474>/FixPt Unit Delay1' */
  real32_T UnitDelay1_DSTATE_n;        /* '<S457>/Unit Delay1' */
  real32_T FixPtUnitDelay1_DSTATE_m;   /* '<S454>/FixPt Unit Delay1' */
  real32_T Unit_Delay1_DSTATE_k;       /* '<S446>/Unit_Delay1' */
  real32_T UnitDelay_DSTATE_o0;        /* '<S449>/Unit Delay' */
  real32_T UnitDelay_DSTATE_c;         /* '<S455>/Unit Delay' */
  real32_T FixPtUnitDelay1_DSTATE_d;   /* '<S514>/FixPt Unit Delay1' */
  real32_T UnitDelay1_DSTATE_o;        /* '<S497>/Unit Delay1' */
  real32_T FixPtUnitDelay1_DSTATE_i;   /* '<S494>/FixPt Unit Delay1' */
  real32_T Unit_Delay1_DSTATE_m;       /* '<S486>/Unit_Delay1' */
  real32_T UnitDelay_DSTATE_bw;        /* '<S489>/Unit Delay' */
  real32_T UnitDelay_DSTATE_iu;        /* '<S495>/Unit Delay' */
  real32_T Unit_Delay_DSTATE_l;        /* '<S277>/Unit_Delay' */
  real32_T Unit_Delay1_DSTATE_d4;      /* '<S277>/Unit_Delay1' */
  real32_T Unit_Delay2_DSTATE_h;       /* '<S277>/Unit_Delay2' */
  real32_T Unit_Delay3_DSTATE_a;       /* '<S277>/Unit_Delay3' */
  real32_T Unit_Delay4_DSTATE_c;       /* '<S277>/Unit_Delay4' */
  real32_T FixPtUnitDelay1_DSTATE_ov;  /* '<S290>/FixPt Unit Delay1' */
  real32_T FixPtUnitDelay1_DSTATE_cn;  /* '<S234>/FixPt Unit Delay1' */
  real32_T Unit_Delay_DSTATE_n0;       /* '<S238>/Unit_Delay' */
  real32_T Unit_Delay1_DSTATE_ea;      /* '<S238>/Unit_Delay1' */
  real32_T Unit_Delay2_DSTATE_ge;      /* '<S238>/Unit_Delay2' */
  real32_T Unit_Delay3_DSTATE_b;       /* '<S238>/Unit_Delay3' */
  real32_T Unit_Delay4_DSTATE_i;       /* '<S238>/Unit_Delay4' */
  real32_T UnitDelay_DSTATE_h2;        /* '<S572>/Unit Delay' */
  real32_T UnitDelay_DSTATE_dt;        /* '<S120>/Unit Delay' */
  real32_T UnitDelay_DSTATE_on;        /* '<S109>/Unit Delay' */
  real32_T UnitDelay_DSTATE_a3;        /* '<S108>/Unit Delay' */
  real32_T UnitDelay_DSTATE_ep;        /* '<S585>/Unit Delay' */
  real32_T FixPtUnitDelay1_DSTATE_l;   /* '<S265>/FixPt Unit Delay1' */
  real32_T Unit_Delay_DSTATE_nr;       /* '<S266>/Unit_Delay' */
  real32_T Unit_Delay1_DSTATE_a;       /* '<S266>/Unit_Delay1' */
  real32_T Unit_Delay2_DSTATE_i;       /* '<S266>/Unit_Delay2' */
  real32_T Unit_Delay3_DSTATE_h;       /* '<S266>/Unit_Delay3' */
  real32_T Unit_Delay4_DSTATE_ib;      /* '<S266>/Unit_Delay4' */
  real32_T FixPtUnitDelay1_DSTATE_ht;  /* '<S259>/FixPt Unit Delay1' */
  real32_T FixPtUnitDelay1_DSTATE_dm;  /* '<S262>/FixPt Unit Delay1' */
  real32_T UnitDelay1_DSTATE_f;        /* '<S249>/Unit Delay1' */
  real32_T UnitDelay1_DSTATE_i1;       /* '<S247>/Unit Delay1' */
  real32_T UnitDelay1_DSTATE_g2;       /* '<S248>/Unit Delay1' */
  real32_T UnitDelay1_DSTATE_gy;       /* '<S252>/Unit Delay1' */
  real32_T UnitDelay2_DSTATE_o;        /* '<S252>/Unit Delay2' */
  real32_T UnitDelay_DSTATE_k;         /* '<S252>/Unit Delay' */
  real32_T UnitDelay1_DSTATE_e;        /* '<S157>/Unit Delay1' */
  real32_T UnitDelay1_DSTATE_if;       /* '<S153>/Unit Delay1' */
  real32_T UnitDelay1_DSTATE_fo;       /* '<S151>/Unit Delay1' */
  real32_T UnitDelay1_DSTATE_ip;       /* '<S152>/Unit Delay1' */
  real32_T UnitDelay_DSTATE_mg;        /* '<S157>/Unit Delay' */
  real32_T FixPtUnitDelay1_DSTATE_de;  /* '<S167>/FixPt Unit Delay1' */
  real32_T FixPtUnitDelay1_DSTATE_of;  /* '<S164>/FixPt Unit Delay1' */
  real32_T FixPtUnitDelay1_DSTATE_ee;  /* '<S170>/FixPt Unit Delay1' */
  real32_T FixPtUnitDelay1_DSTATE_i4;  /* '<S193>/FixPt Unit Delay1' */
  real32_T UnitDelay_DSTATE_mw;        /* '<S144>/Unit Delay' */
  real32_T FixPtUnitDelay1_DSTATE_im;  /* '<S187>/FixPt Unit Delay1' */
  real32_T FixPtUnitDelay1_DSTATE_es;  /* '<S190>/FixPt Unit Delay1' */
  real32_T UnitDelay1_DSTATE_ei;       /* '<S176>/Unit Delay1' */
  real32_T UnitDelay1_DSTATE_d;        /* '<S174>/Unit Delay1' */
  real32_T UnitDelay1_DSTATE_cz;       /* '<S175>/Unit Delay1' */
  real32_T UnitDelay1_DSTATE_c3;       /* '<S180>/Unit Delay1' */
  real32_T UnitDelay2_DSTATE_cu;       /* '<S180>/Unit Delay2' */
  real32_T UnitDelay_DSTATE_pzg;       /* '<S180>/Unit Delay' */
  real32_T UnitDelay_DSTATE_hr;        /* '<S280>/Unit Delay' */
  real32_T UnitDelay1_DSTATE_j;        /* '<S279>/Unit Delay1' */
  real32_T UnitDelay1_DSTATE_fq;       /* '<S285>/Unit Delay1' */
  real32_T UnitDelay_DSTATE_c2;        /* '<S285>/Unit Delay' */
  real32_T FixPtUnitDelay1_DSTATE_ox;  /* '<S293>/FixPt Unit Delay1' */
  real32_T FixPtUnitDelay1_DSTATE_oi;  /* '<S237>/FixPt Unit Delay1' */
  real32_T UnitDelay_DSTATE_hn;        /* '<S228>/Unit Delay' */
  real32_T UnitDelay1_DSTATE_k;        /* '<S227>/Unit Delay1' */
  real32_T UnitDelay_DSTATE_h4;        /* '<S225>/Unit Delay' */
  real32_T UnitDelay1_DSTATE_ay;       /* '<S225>/Unit Delay1' */
  real32_T FixPtUnitDelay1_DSTATE_os;  /* '<S517>/FixPt Unit Delay1' */
  real32_T UnitDelay_DSTATE_nu;        /* '<S502>/Unit Delay' */
  real32_T UnitDelay1_DSTATE_fc;       /* '<S502>/Unit Delay1' */
  real32_T Unit_Delay3_DSTATE_k;       /* '<S504>/Unit_Delay3' */
  real32_T Unit_Delay1_DSTATE_f;       /* '<S505>/Unit_Delay1' */
  real32_T Unit_Delay3_DSTATE_kl;      /* '<S546>/Unit_Delay3' */
  real32_T Unit_Delay1_DSTATE_a2;      /* '<S547>/Unit_Delay1' */
  real32_T FixPtUnitDelay1_DSTATE_cj;  /* '<S559>/FixPt Unit Delay1' */
  real32_T Unit_Delay3_DSTATE_jb;      /* '<S464>/Unit_Delay3' */
  real32_T Unit_Delay1_DSTATE_fj;      /* '<S465>/Unit_Delay1' */
  real32_T UnitDelay_DSTATE_mz;        /* '<S462>/Unit Delay' */
  real32_T FixPtUnitDelay1_DSTATE_d2;  /* '<S477>/FixPt Unit Delay1' */
  real32_T FixPtUnitDelay1_DSTATE_h1;  /* '<S323>/FixPt Unit Delay1' */
  real32_T FixPtUnitDelay1_DSTATE_j;   /* '<S325>/FixPt Unit Delay1' */
  uint8_T FixPtUnitDelay2_DSTATE;      /* '<S381>/FixPt Unit Delay2' */
  uint8_T FixPtUnitDelay2_DSTATE_m;    /* '<S384>/FixPt Unit Delay2' */
  uint8_T FixPtUnitDelay2_DSTATE_b;    /* '<S329>/FixPt Unit Delay2' */
  uint8_T FixPtUnitDelay2_DSTATE_o;    /* '<S327>/FixPt Unit Delay2' */
  uint8_T Unit_Delay_DSTATE_c;         /* '<S86>/Unit_Delay' */
  uint8_T FixPtUnitDelay2_DSTATE_c;    /* '<S616>/FixPt Unit Delay2' */
  uint8_T FixPtUnitDelay2_DSTATE_f;    /* '<S556>/FixPt Unit Delay2' */
  uint8_T FixPtUnitDelay2_DSTATE_a;    /* '<S534>/FixPt Unit Delay2' */
  uint8_T FixPtUnitDelay2_DSTATE_k;    /* '<S474>/FixPt Unit Delay2' */
  uint8_T FixPtUnitDelay2_DSTATE_b1;   /* '<S454>/FixPt Unit Delay2' */
  uint8_T FixPtUnitDelay2_DSTATE_b3;   /* '<S514>/FixPt Unit Delay2' */
  uint8_T FixPtUnitDelay2_DSTATE_ms;   /* '<S494>/FixPt Unit Delay2' */
  uint8_T FixPtUnitDelay2_DSTATE_e;    /* '<S290>/FixPt Unit Delay2' */
  uint8_T FixPtUnitDelay2_DSTATE_j;    /* '<S234>/FixPt Unit Delay2' */
  uint8_T DelayInput1_DSTATE;          /* '<S198>/Delay Input1' */
  uint8_T FixPtUnitDelay2_DSTATE_ej;   /* '<S265>/FixPt Unit Delay2' */
  uint8_T DelayInput1_DSTATE_g;        /* '<S199>/Delay Input1' */
  uint8_T FixPtUnitDelay2_DSTATE_kd;   /* '<S259>/FixPt Unit Delay2' */
  uint8_T FixPtUnitDelay2_DSTATE_ck;   /* '<S262>/FixPt Unit Delay2' */
  uint8_T FixPtUnitDelay2_DSTATE_el;   /* '<S167>/FixPt Unit Delay2' */
  uint8_T FixPtUnitDelay2_DSTATE_kf;   /* '<S164>/FixPt Unit Delay2' */
  uint8_T FixPtUnitDelay2_DSTATE_oe;   /* '<S170>/FixPt Unit Delay2' */
  uint8_T FixPtUnitDelay2_DSTATE_i;    /* '<S193>/FixPt Unit Delay2' */
  uint8_T FixPtUnitDelay2_DSTATE_on;   /* '<S187>/FixPt Unit Delay2' */
  uint8_T FixPtUnitDelay2_DSTATE_ct;   /* '<S190>/FixPt Unit Delay2' */
  uint8_T FixPtUnitDelay2_DSTATE_n;    /* '<S293>/FixPt Unit Delay2' */
  uint8_T FixPtUnitDelay2_DSTATE_d;    /* '<S237>/FixPt Unit Delay2' */
  uint8_T FixPtUnitDelay2_DSTATE_fb;   /* '<S517>/FixPt Unit Delay2' */
  uint8_T FixPtUnitDelay2_DSTATE_h;    /* '<S559>/FixPt Unit Delay2' */
  uint8_T FixPtUnitDelay2_DSTATE_of;   /* '<S477>/FixPt Unit Delay2' */
  uint8_T FixPtUnitDelay2_DSTATE_dm;   /* '<S323>/FixPt Unit Delay2' */
  uint8_T FixPtUnitDelay2_DSTATE_g;    /* '<S325>/FixPt Unit Delay2' */
  boolean_T DelayInput1_DSTATE_i;      /* '<S414>/Delay Input1' */
  boolean_T DelayInput1_DSTATE_a;      /* '<S96>/Delay Input1' */
  boolean_T DelayInput1_DSTATE_m;      /* '<S623>/Delay Input1' */
  boolean_T DelayInput1_DSTATE_k;      /* '<S624>/Delay Input1' */
  boolean_T DelayInput1_DSTATE_p;      /* '<S562>/Delay Input1' */
  boolean_T UnitDelay_DSTATE_hg;       /* '<S539>/Unit Delay' */
  boolean_T UnitDelay_DSTATE_jw;       /* '<S457>/Unit Delay' */
  boolean_T UnitDelay_DSTATE_bp;       /* '<S497>/Unit Delay' */
  boolean_T DelayInput1_DSTATE_g0;     /* '<S216>/Delay Input1' */
  boolean_T DelayInput1_DSTATE_l;      /* '<S135>/Delay Input1' */
  boolean_T DelayInput1_DSTATE_n;      /* '<S147>/Delay Input1' */
} DW_TJATCT_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  uint16_T S_TCTEST_QualifierService_nu;
                                     /* '<Root>/S_TCTEST_QualifierService_nu' */
  real32_T DEV_ReqDeltaFPar;           /* '<Root>/DEV_ReqDeltaFPar' */
  uint8_T CLM_BtfQualifier;            /* '<Root>/CLM_BtfQualifier' */
} ExtY_TJATCT_T;

/* Block states (default storage) */
extern DW_TJATCT_T TJATCT_DW;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_TJATCT_T TJATCT_Y;

/*
 * Exported Global Signals
 *
 * Note: Exported global signals are block signals with an exported global
 * storage class designation.  Code generation will declare the memory for
 * these signals and export their symbols.
 *
 */
extern real32_T data_log_5;            /* '<S439>/Gain' */
extern real32_T CLM_ReqFfcCrv;         /* '<S103>/Multiport Switch1'
                                        * Control signal of the Feedforward Controller after safety checks
                                        */
extern real32_T CLM_ReqFbcDcCrv;       /* '<S103>/Multiport Switch2'
                                        * Control signal of the Feedback Controller + Disturbance Compensator after safety checks
                                        */
extern real32_T CLM_LmtReqFfcCrv;      /* '<S103>/Add'
                                        * UNDEFINED
                                        */
extern real32_T CLM_DeltaFCmd;         /* '<S103>/Multiport Switch'
                                        * UNDEFINED
                                        */
extern real32_T CLM_SumCtrlCrv;        /* '<S134>/Add1'
                                        * UNDEFINED
                                        */
extern real32_T CLM_TimerPlauCheck;    /* '<S131>/Multiport Switch'
                                        * UNDEFINED
                                        */
extern real32_T CLM_HldVehCrv;         /* '<S131>/Multiport Switch1'
                                        * UNDEFINED
                                        */
extern real32_T CLM_ThdCrvPlauChkUp;   /* '<S136>/Multiport Switch'
                                        * UNDEFINED
                                        */
extern real32_T CLM_ThdCrvPlauChkLow;  /* '<S136>/Multiport Switch1'
                                        * UNDEFINED
                                        */
extern real32_T data_log_0;            /* '<S81>/Gain' */
extern real32_T data_log_4;            /* '<S439>/Gain1' */
extern real32_T data_log_3;            /* '<S416>/Gain' */
extern real32_T data_log_6;            /* '<S73>/Data Type Conversion8' */
extern real32_T data_log_7;            /* '<S73>/Data Type Conversion9' */
extern real32_T data_log_8;            /* '<S73>/Data Type Conversion10' */
extern real32_T data_log_1;            /* '<S73>/Data Type Conversion17' */
extern real32_T data_log_2;            /* '<S73>/Data Type Conversion18' */
extern real32_T data_log_9;            /* '<S73>/Data Type Conversion5' */
extern uint8_T CLM_BtfQulifierTrajCtrl;/* '<S105>/Add'
                                        * Trajectory controller qualifier. Includes relevant information for the Situation Assessment components.
                                        */
extern boolean_T CLM_EnaUnplauRequest; /* '<S102>/Unit Delay'
                                        * UNDEFINED
                                        */
extern boolean_T CLM_EnaRstDegrReq;    /* '<S106>/OR'
                                        * Reset condition for Flip-Flop which holds degradation request
                                        */
extern boolean_T CLM_EnaSetDegrReq;    /* '<S110>/Switch'
                                        * Set condition for Flip-Flop which holds degradation request
                                        */
extern boolean_T CLM_EnaPlausibilityCheck;/* '<S132>/AND1'
                                           * UNDEFINED
                                           */
extern boolean_T CLM_EnaHldVehCrv;     /* '<S135>/FixPt Relational Operator'
                                        * UNDEFINED
                                        */
extern boolean_T CLM_EnaLmtWarn;       /* '<S137>/AND'
                                        * UNDEFINED
                                        */
extern boolean_T CLM_EnaPlauCheck;     /* '<S131>/AND'
                                        * UNDEFINED
                                        */

/* Model entry point functions */
extern void TJATCT_initialize(void);
extern void TJATCT_step(void);

/* Exported data declaration */

/* ConstVolatile memory section */
/* Declaration for custom storage class: ConstVolatile */
extern const volatile uint8_T DTE_DelayCyclesLaDMC_P;/* Referenced by:
                                                      * '<S238>/Parameter1'
                                                      * '<S266>/Parameter1'
                                                      */

/* Number of cycle delays of the LaDMC */
extern const volatile real32_T DTE_KappaAngleLaDmc_M[12];
                                  /* Referenced by: '<S229>/1-D Lookup Table' */

/* Grid points of the y-Axis of the LaDMC Look Up Table "Kappa To Angle". */
extern const volatile real32_T DTE_KappaAngleLaDmc_X[12];
                                  /* Referenced by: '<S229>/1-D Lookup Table' */

/* Grid points for the x-Axis of the LaDMC Look Up Table "Kappa To Angle" */
extern const volatile real32_T DTE_SetCrvGainLaDmc_M[13];
                                  /* Referenced by: '<S238>/1-D Lookup Table' */

/* Gain of the calculated curvature needed to compensate the disturbances in dependence on the vehicle's velocity */
extern const volatile real32_T DTE_SetCrvGainLaDmc_X[13];
                                  /* Referenced by: '<S238>/1-D Lookup Table' */

/* X axis for gain of the calculated curvature needed to compensate the disturbances in dependence on the vehicle's velocity */
extern const volatile real32_T DTE_SetDeltaGainLaDmc_M[13];
                                  /* Referenced by: '<S266>/1-D Lookup Table' */

/* Gain of the Disturbance Compensator set value */
extern const volatile real32_T DTE_SetDeltaGainLaDmc_X[13];
                                  /* Referenced by: '<S266>/1-D Lookup Table' */

/* X axis for gain of the Disturbance Compensator set value */
extern const volatile real32_T DTE_Time1FltLaDmc_P;/* Referenced by:
                                                    * '<S224>/Parameter2'
                                                    * '<S224>/Parameter4'
                                                    * '<S253>/Parameter4'
                                                    * '<S253>/Parameter6'
                                                    * '<S253>/Parameter7'
                                                    * '<S253>/Parameter8'
                                                    */

/* First Time Constant of the of second order low pass needed for the inversion of the DMC's trasfer function */
extern const volatile real32_T DTE_Time1LaDmc_P;/* Referenced by:
                                                 * '<S226>/Parameter6'
                                                 * '<S226>/Parameter8'
                                                 */

/* First Time Constant of the DMC's transfer function (command steering angle -> driven curvature) */
extern const volatile real32_T DTE_Time2FltLaDmc_P;/* Referenced by:
                                                    * '<S224>/Parameter3'
                                                    * '<S224>/Parameter5'
                                                    * '<S253>/Parameter1'
                                                    * '<S253>/Parameter2'
                                                    * '<S253>/Parameter3'
                                                    * '<S253>/Parameter5'
                                                    */

/* Second Time Constant of the second order low pass needed for the inversion of the DMC's transfer function */
extern const volatile real32_T DTE_Time2LaDmc_P;/* Referenced by:
                                                 * '<S226>/Parameter7'
                                                 * '<S226>/Parameter9'
                                                 */

/* Second Time Constant of the DMC's transfer function (command steering angle -> driven curvature) */
extern const volatile uint8_T LGC_CswDynPT1Rst_P;/* Referenced by: '<S434>/Parameter5' */
extern const volatile uint8_T P_TCTLGC_ActivateDynBacGain_nu;/* Referenced by:
                                                              * '<S565>/Constant'
                                                              * '<S583>/Constant'
                                                              */
extern const volatile uint8_T P_TCTLGC_ActivateOverride_nu;/* Referenced by: '<S419>/Constant2' */
extern const volatile uint8_T P_TCTLGC_CssCrv_nu;/* Referenced by:
                                                  * '<S419>/Constant'
                                                  * '<S419>/Constant3'
                                                  */
extern const volatile real32_T P_VEH_DistCogToFrontAxle_m;/* Referenced by: '<S564>/Constant1' */
extern const volatile real32_T P_VEH_DistCogToRearAxle_m;/* Referenced by: '<S564>/Constant2' */
extern const volatile real32_T P_VEH_SelfSteeringGrd_nu;/* Referenced by: '<S564>/Constant4' */
extern const volatile real32_T X_TCTLGC_CtrlErrDistY_met[6];/* Referenced by:
                                                             * '<S565>/Y_TCTLGC_DynBacGain_nu'
                                                             * '<S583>/Y_TCTLGC_DynBacGain_nu'
                                                             */
extern const volatile real32_T Y_TCTLGC_DynBacGain_nu[6];/* Referenced by:
                                                          * '<S565>/Y_TCTLGC_DynBacGain_nu'
                                                          * '<S583>/Y_TCTLGC_DynBacGain_nu'
                                                          */

/* Declaration for custom storage class: Global */
extern real_T LQR_EnaButterVel;        /* Referenced by: '<S603>/Parameter8' */

/* ConstVolatile memory section */
/* Declaration for custom storage class: ConstVolatile */
extern const volatile real32_T CDC_CoeffDeltaPsiKmc_P;/* Referenced by: '<S92>/Constant' */

/* UNDEFINED */
extern const volatile real32_T CDC_CoeffDeltaPsiObsKmc_P;
                       /* Referenced by: '<S92>/Kmc_delta_psi_obs_gain_const' */

/* UNDEFINED */
extern const volatile uint8_T CDC_CswSelDeltaTheta_P;/* Referenced by: '<S91>/Parameter' */

/* Output Source Selection of DeltaTheta
   ------------------------------------------
   0: Raw unfiltered
   1: PT1 Filter
   2: Plant Observer
   3: Old Course Angle Observer */
extern const volatile uint8_T CDC_CswSelDistY_P;
                              /* Referenced by: '<S87>/P_TCTCDC_OssDeltaY_nu' */

/* Output Source Selection of DeltaY
   ------------------------------------------
   1: Raw unfiltered
   2: PT1 Filter
   3: Plant Observer */
extern const volatile uint8_T CDC_CswTrajCrv_P;/* Referenced by: '<S92>/Constant4' */

/* UNDEFINED */
extern const volatile uint8_T CDC_CswWatchdogAct;
                           /* Referenced by: '<S86>/P_TCTCDC_WtchdgActive_nu' */

/* Configuration switch for watchdog activation
   0: Watchdog is deactivated
   1: Watchdog is activated */
extern const volatile uint8_T CDC_DelayCycleNum_P;/* Referenced by: '<S92>/Constant1' */

/* Cylce numer for the delay in the heading angle signal */
extern const volatile uint8_T CDC_EnaFreezeByTgq_P;/* Referenced by:
                                                    * '<S85>/Parameter1'
                                                    * '<S85>/Parameter4'
                                                    */

/* Freeze parameter for Trajectory Guidance Qualifier Output */
extern const volatile uint8_T CDC_EnaOffByTgq_P;/* Referenced by: '<S85>/Parameter2' */

/* Off parameter for Trajectory Guidance Qualifier Output */
extern const volatile real32_T CDC_FltDistYFc;
                             /* Referenced by: '<S88>/P_TCTCDC_OssDeltaY_nu2' */
extern const volatile real32_T CDC_FltYawFc;
        /* Referenced by: '<S93>/P_TRJCTR_DE_TSteeringAngleFrontAxleEff_sec1' */
extern const volatile real32_T CDC_HeadingOffset;
        /* Referenced by: '<S93>/P_TRJCTR_DE_TSteeringAngleFrontAxleEff_sec2' */
extern const volatile uint8_T CDC_StLcfOff_P;
                              /* Referenced by: '<S86>/from_data_definition1' */

/* Lcf off state parameter */
extern const volatile real32_T CDC_ThdErrCourseAngle_P;
                      /* Referenced by: '<S86>/P_TCTCDC_WtchdgLimCoAnErr_rad' */

/* Absolute value of the threshold used to classify the Course Angle Error as too high */
extern const volatile real32_T CDC_ThdErrDistY_P;
                           /* Referenced by: '<S86>/P_TCTCDC_WtchdgLimYErr_m' */

/* Absolute value of the threshold used to classify the Y-Coordinate Error as too high */
extern const volatile real32_T CDC_TimeSysCycle_P;/* Referenced by:
                                                   * '<S88>/P_TCTCDC_OssDeltaY_nu1'
                                                   * '<S92>/Parameter'
                                                   * '<S93>/Parameter'
                                                   */
extern const volatile uint8_T CLM_BtfQualifierCdc_P;/* Referenced by: '<S106>/Parameter' */

/* Bitmask for selecting the bits to be checked in the signal S_TCTCDC_QualifierService_nu */
extern const volatile uint8_T CLM_BtfQualifierClm_P;/* Referenced by: '<S106>/Parameter1' */

/* Bitmask for selecting the bits to be checked in the signal D_TCTCLM_QualifierService_nu */
extern const volatile uint8_T CLM_CswForceDegrReq_P;/* Referenced by: '<S106>/Parameter2' */

/* Force degradation request via signal S_TCTCLM_QuServTrajCtr
   0: Use conventional degradation request conditions
   1: Force degradation request */
extern const volatile uint8_T CLM_CswHldDegrReq_P;/* Referenced by: '<S106>/Parameter6' */

/* Activate/Deactivate hold functionality of degradation request
   0: Deactivate
   1: Activate */
extern const volatile uint8_T CLM_CswSelFbcCrv_P;/* Referenced by:
                                                  * '<S134>/Parameter'
                                                  * '<S134>/Parameter1'
                                                  */

/* Select interface for the feedback controller
   0: Steer Angle Interface
   1: Curvature Interface */
extern const volatile real32_T CLM_DistCogToFrontAxle_P;/* Referenced by: '<S134>/Parameter3' */

/* Distance between front axle and the vehicle's center of gravity */
extern const volatile real32_T CLM_DistCogToRearAxle_P;/* Referenced by: '<S134>/Parameter4' */

/* Distance between rear axle and the vehicle's center of gravity */
extern const volatile real32_T CLM_MaxHldTiDegrReq_P;/* Referenced by: '<S106>/Parameter8' */

/* Maximum hold time of degradation request */
extern const volatile real32_T CLM_MinHldTiDegrReq_P;/* Referenced by: '<S106>/Parameter4' */

/* Minimum hold time of degradation request */
extern const volatile real32_T CLM_SelfSteerGrd_P;/* Referenced by: '<S134>/Parameter2' */

/* Self Steering Gradient of the Vehicle */
extern const volatile uint8_T CLM_StControlCsc_P;/* Referenced by:
                                                  * '<S132>/Parameter1'
                                                  * '<S106>/Parameter3'
                                                  */

/* Cycle Time for VEH task */
extern const volatile uint8_T CLM_StDepartLeftLaKmc_P;/* Referenced by: '<S136>/Parameter' */

/* Cycle Time for VEH task */
extern const volatile uint8_T CLM_StDepartRightLaKmc_P;/* Referenced by: '<S136>/Parameter1' */

/* Cycle Time for VEH task */
extern const volatile uint8_T CLM_StRequestCsc_P;/* Referenced by: '<S132>/Parameter' */

/* Cycle Time for VEH task */
extern const volatile real32_T CLM_ThdDeltaFCmdGrd_LDP_M[10];
                                 /* Referenced by: '<S115>/1-D Lookup Table1' */

/* Absolut value of the gradient limitation of the set value send to the LaDMC */
extern const volatile real32_T CLM_ThdDeltaFCmdGrd_M[10];
                                  /* Referenced by: '<S115>/1-D Lookup Table' */

/* Absolut value of the gradient limitation of the set value send to the LaDMC */
extern const volatile real32_T CLM_ThdDeltaFCmdGrd_X[10];/* Referenced by:
                                                          * '<S115>/1-D Lookup Table'
                                                          * '<S115>/1-D Lookup Table1'
                                                          */

/* Velocity vector along the vehicle's longitudinal axis / kph */
extern const volatile real32_T CLM_ThdDeltaFCmdSat_LDP_M[10];
                                 /* Referenced by: '<S116>/1-D Lookup Table1' */

/* Absolute value of the saturation of the set value send to the LaDMC */
extern const volatile real32_T CLM_ThdDeltaFCmdSat_M[10];
                                 /* Referenced by: '<S116>/1-D Lookup Table2' */

/* Absolute value of the saturation of the set value send to the LaDMC */
extern const volatile real32_T CLM_ThdDeltaFCmdSat_X[10];/* Referenced by:
                                                          * '<S116>/1-D Lookup Table1'
                                                          * '<S116>/1-D Lookup Table2'
                                                          */

/* Velocity vector along the vehicle's longitudinal axis / kph */
extern const volatile real32_T CLM_ThdFbcDcSat_M[15];
                                 /* Referenced by: '<S121>/1-D Lookup Table2' */

/* Absolute value / (1/m) of the maximal allowed control signal "Feedback Controller + Disturbance Compensator" send to the LaDMC */
extern const volatile real32_T CLM_ThdFbcDcSat_X[15];
                                 /* Referenced by: '<S121>/1-D Lookup Table2' */

/* Velocity vector along the vehicle's longitudinal axis / (m/s) */
extern const volatile real32_T CLM_ThdFfcCrvGrd_M[15];
                                  /* Referenced by: '<S125>/1-D Lookup Table' */

/* Absolute value / (1/(m*s)) of the maximal allowed gradient of the control signal Feedforward Controller" send to the LaDMC */
extern const volatile real32_T CLM_ThdFfcCrvGrd_X[15];
                                  /* Referenced by: '<S125>/1-D Lookup Table' */

/* Velocity vector along the vehicle's longitudinal axis / (m/s) */
extern const volatile real32_T CLM_ThdFfcCrvSat_M[15];
                                 /* Referenced by: '<S126>/1-D Lookup Table2' */

/* Absolute value / (1/m) of the maximal allowed control signal "Feedforward Controller" send to the LaDMC */
extern const volatile real32_T CLM_ThdFfcCrvSat_X[15];
                                 /* Referenced by: '<S126>/1-D Lookup Table2' */

/* Velocity vector along the vehicle's longitudinal axis / (m/s) */
extern const volatile real32_T CLM_ThdFfcFbcDcGrd_M[15];
                                  /* Referenced by: '<S120>/1-D Lookup Table' */

/* Absolute value / (1/(m*s)) of the maximal allowed gradient of the control signal "Feedback Controller + Disturbance Compensator" send to the LaDMC */
extern const volatile real32_T CLM_ThdFfcFbcDcGrd_X[15];
                                  /* Referenced by: '<S120>/1-D Lookup Table' */

/* Velocity vector along the vehicle's longitudinal axis / (m/s) */
extern const volatile real32_T CLM_TimeSysCycle_P;/* Referenced by:
                                                   * '<S131>/Parameter'
                                                   * '<S131>/Parameter1'
                                                   * '<S106>/Parameter7'
                                                   * '<S106>/Parameter9'
                                                   * '<S115>/Parameter'
                                                   * '<S115>/Parameter1'
                                                   * '<S120>/Parameter'
                                                   * '<S120>/Parameter1'
                                                   * '<S125>/Parameter'
                                                   * '<S125>/Parameter1'
                                                   */

/* Cycle Time for VEH task */
extern const volatile real32_T DEV_CoeffCrvAmp_P;/* Referenced by: '<S194>/Parameter5' */

/* Amplitude (1/m) of the chirp signal used to stimulate the curvature interface of the LaKMC */
extern const volatile real32_T DEV_CoeffCrvFrqGain_P;/* Referenced by: '<S194>/Parameter3' */

/* Start frequency / (1/s) of the chirp signal used to stimulate curvature interface of LaKMC */
extern const volatile real32_T DEV_CoeffDeltaFAmp_P;/* Referenced by: '<S196>/Parameter5' */

/* Amplitude /deg of the chirp signal used to stimulate S_TCTLGC_DeltaFCmd_deg */
extern const volatile real32_T DEV_CoeffDeltaFFrqGain_P;/* Referenced by: '<S196>/Parameter3' */

/* Gain of the frequency increase of the chirp signal used to stimulate S_TCTLGC_DeltaFCmd_deg */
extern const volatile real32_T DEV_CoeffDeltaGainFfc_M[3];/* Referenced by:
                                                           * '<S143>/1-D Lookup Table1'
                                                           * '<S144>/Y_TCTFFC_GainFFC_nu'
                                                           */

/* Gain of the control signal part of the feedforward controller */
extern const volatile real32_T DEV_CoeffDeltaGainFfc_X[3];/* Referenced by:
                                                           * '<S143>/1-D Lookup Table1'
                                                           * '<S144>/Y_TCTFFC_GainFFC_nu'
                                                           */

/* X axis for gain of the control signal part of the feedforward controller */
extern const volatile uint8_T DEV_CswCrvTestSignal_P;/* Referenced by: '<S195>/Parameter' */

/* Mode of the TCTDEV Test Signal generator to stimulate curvature interface of LaKMC:
   0000 0000: Signal generator off
   0000 xxx1: Chirp active */
extern const volatile uint8_T DEV_CswDeltaFTestSignal_P;/* Referenced by: '<S197>/Parameter' */

/* Mode of the TCTDEV Test Signal generator to stimulate S_TCTLGC_DeltaFCmd_deg:
   0000 0000: Signal generator off
   0000 xxx1: Chirp active */
extern const volatile real32_T DEV_DistCogToFrontAxle_P;/* Referenced by:
                                                         * '<S141>/Constant1'
                                                         * '<S142>/Constant1'
                                                         */

/* Distance between front axle and the vehicle's center of gravity */
extern const volatile real32_T DEV_DistCogToRearAxle_P;/* Referenced by:
                                                        * '<S141>/Constant2'
                                                        * '<S142>/Constant2'
                                                        */

/* Distance between rear axle and the vehicle's center of gravity */
extern const volatile uint8_T DEV_EnaFreezeByTgq_P;/* Referenced by:
                                                    * '<S140>/Parameter1'
                                                    * '<S140>/Parameter4'
                                                    */

/* Freeze parameter for Trajectory Guidance Qualifier Output */
extern const volatile uint8_T DEV_EnaOffByTgq_P;/* Referenced by: '<S140>/Parameter2' */

/* Off parameter for Trajectory Guidance Qualifier Output */
extern const volatile real32_T DEV_SelfSteeringGrd_nu;/* Referenced by: '<S142>/Constant3' */
extern const volatile real32_T DEV_StrtFrqCrvF_P;/* Referenced by: '<S194>/Parameter4' */

/* Gain of the frequency increase of the chirp signal used to stimulate the curvature interface of the LaKMC */
extern const volatile real32_T DEV_StrtFrqDeltaF_P;/* Referenced by: '<S196>/Parameter4' */

/* Start frequency (1/s) of the chirp signal used to stimulate S_TCTLGC_DeltaFCmd_deg */
extern const volatile real32_T DEV_Time1FltLaDmc_P;/* Referenced by:
                                                    * '<S156>/Parameter4'
                                                    * '<S156>/Parameter6'
                                                    * '<S156>/Parameter7'
                                                    * '<S156>/Parameter8'
                                                    * '<S179>/Parameter4'
                                                    * '<S179>/Parameter6'
                                                    * '<S179>/Parameter7'
                                                    * '<S179>/Parameter8'
                                                    */

/* First Time Constant of the of second order low pass needed for the inversion of the DMC's trasfer function */
extern const volatile real32_T DEV_Time2FltLaDmc_P;/* Referenced by:
                                                    * '<S156>/Parameter1'
                                                    * '<S156>/Parameter2'
                                                    * '<S156>/Parameter3'
                                                    * '<S156>/Parameter5'
                                                    * '<S179>/Parameter1'
                                                    * '<S179>/Parameter2'
                                                    * '<S179>/Parameter3'
                                                    * '<S179>/Parameter5'
                                                    */

/* Second Time Constant of the of second order low pass needed for the inversion of the DMC's trasfer function */
extern const volatile real32_T DEV_TimeCrvFWait_P;/* Referenced by: '<S194>/Parameter2' */

/* Time of the command value being set to 0 before the chirp signal output starts (for curvature interface of LaKMC) */
extern const volatile real32_T DEV_TimeDeltaFWait_P;/* Referenced by: '<S196>/Parameter2' */

/* Time of the command value being set to 0 before the chirp signal output starts */
extern const volatile real32_T DEV_TimeSysCycle_P;/* Referenced by:
                                                   * '<S195>/Parameter1'
                                                   * '<S197>/Parameter1'
                                                   * '<S151>/Parameter1'
                                                   * '<S152>/Parameter1'
                                                   * '<S153>/Parameter1'
                                                   * '<S153>/Parameter3'
                                                   * '<S157>/Parameter'
                                                   * '<S157>/Parameter1'
                                                   * '<S157>/Parameter2'
                                                   * '<S174>/Parameter1'
                                                   * '<S175>/Parameter1'
                                                   * '<S176>/Parameter1'
                                                   * '<S176>/Parameter3'
                                                   * '<S180>/Parameter'
                                                   * '<S180>/Parameter1'
                                                   * '<S180>/Parameter2'
                                                   */
extern const volatile real32_T DTE_CoeffDenS0LaDmc_M[13];
                                  /* Referenced by: '<S241>/1-D Lookup Table' */

/* s^0 coefficient of the approximated LaDMC transfer function's denominator */
extern const volatile real32_T DTE_CoeffDenS0LaDmc_X[13];
                                  /* Referenced by: '<S241>/1-D Lookup Table' */

/* x axis for s^0 coefficient of the approximated LaDMC transfer function's denominator */
extern const volatile real32_T DTE_CoeffDenS1LaDmc_M[13];
                                 /* Referenced by: '<S241>/1-D Lookup Table1' */

/* s^1 coefficient of the approximated LaDMC transfer function's denominator */
extern const volatile real32_T DTE_CoeffDenS1LaDmc_X[13];
                                 /* Referenced by: '<S241>/1-D Lookup Table1' */

/* X axis for s^1 coefficient of the approximated LaDMC transfer function's denominator */
extern const volatile real32_T DTE_CoeffDenS2LaDmc_M[13];
                                 /* Referenced by: '<S241>/1-D Lookup Table2' */

/* s^2 coefficient of the approximated LaDMC transfer function's denominator */
extern const volatile real32_T DTE_CoeffDenS2LaDmc_X[13];
                                 /* Referenced by: '<S241>/1-D Lookup Table2' */

/* X axis for s^2 coefficient of the approximated LaDMC transfer function's denominator */
extern const volatile real32_T DTE_CoeffDenS3LaDmc_M[13];
                                 /* Referenced by: '<S241>/1-D Lookup Table3' */

/* s^3 coefficient of the approximated LaDMC transfer function's denominator */
extern const volatile real32_T DTE_CoeffDenS3LaDmc_X[13];
                                 /* Referenced by: '<S241>/1-D Lookup Table3' */

/* X axis for s^3 coefficient of the approximated LaDMC transfer function's denominator */
extern const volatile real32_T DTE_CoeffNumS0LaDmc_M[13];
                                 /* Referenced by: '<S241>/1-D Lookup Table4' */

/* s^0 coefficient of the approximated LaDMC transfer function's numerator */
extern const volatile real32_T DTE_CoeffNumS0LaDmc_X[13];
                                 /* Referenced by: '<S241>/1-D Lookup Table4' */

/* X axis for s^0 coefficient of the approximated LaDMC transfer function's numerator */
extern const volatile real32_T DTE_CoeffNumS1LaDmc_M[13];
                                 /* Referenced by: '<S241>/1-D Lookup Table5' */

/* s^1 coefficient of the approximated LaDMC transfer function's numerator */
extern const volatile real32_T DTE_CoeffNumS1LaDmc_X[13];
                                 /* Referenced by: '<S241>/1-D Lookup Table5' */

/* X axis for s^1 coefficient of the approximated LaDMC transfer function's numerator */
extern const volatile real32_T DTE_CoeffReqCrvGain_P;/* Referenced by: '<S207>/Parameter4' */

/* Gain Coefficient for Required vehicle curvature by road bank angle compensation */
extern const volatile real32_T DTE_CoeffReqDeltaGain_P;/* Referenced by: '<S205>/Parameter3' */

/* Gain Coefficient for Required Delta by road bank angle compensation */
extern const volatile real32_T DTE_CorStiffFrontAxle_P;/* Referenced by:
                                                        * '<S271>/Parameter9'
                                                        * '<S272>/Parameter8'
                                                        * '<S273>/Parameter13'
                                                        * '<S273>/Parameter2'
                                                        * '<S274>/Parameter10'
                                                        */

/* Cornering stiffness coefficient of the tires at the front axle */
extern const volatile real32_T DTE_CorStiffRearAxle_P;/* Referenced by:
                                                       * '<S271>/Parameter5'
                                                       * '<S273>/Parameter1'
                                                       * '<S273>/Parameter10'
                                                       * '<S274>/Parameter5'
                                                       */
extern const volatile uint8_T DTE_CswBnkAglCpmn_P;/* Referenced by:
                                                   * '<S212>/Parameter'
                                                   * '<S213>/Parameter'
                                                   */

/* Configuration switch for road bank angle compensation activation function */
extern const volatile uint8_T DTE_CswDstrbCmpn_P;/* Referenced by:
                                                  * '<S239>/Parameter'
                                                  * '<S267>/Parameter'
                                                  */

/* Configuration switch for disturbance compensator activation function */
extern const volatile real32_T DTE_DistCogToFrontAxle_P;/* Referenced by:
                                                         * '<S205>/Parameter1'
                                                         * '<S271>/Parameter7'
                                                         * '<S272>/Parameter9'
                                                         * '<S273>/Parameter12'
                                                         * '<S273>/Parameter3'
                                                         * '<S274>/Parameter2'
                                                         * '<S274>/Parameter4'
                                                         */

/* Distance between front axle and the vehicle's center of gravity */
extern const volatile real32_T DTE_DistCogToRearAxle_P;/* Referenced by:
                                                        * '<S205>/Parameter2'
                                                        * '<S271>/Parameter8'
                                                        * '<S273>/Parameter11'
                                                        * '<S273>/Parameter4'
                                                        * '<S274>/Parameter3'
                                                        * '<S274>/Parameter8'
                                                        */

/* Distance between rear axle and the vehicle's center of gravity */
extern const volatile uint8_T DTE_EnaFreezeByTgq_P;/* Referenced by:
                                                    * '<S201>/Parameter1'
                                                    * '<S201>/Parameter4'
                                                    */

/* Freeze parameter for Trajectory Guidance Qualifier Output */
extern const volatile uint8_T DTE_EnaOffByTgq_P;/* Referenced by: '<S201>/Parameter2' */

/* Off parameter for Trajectory Guidance Qualifier Output */
extern const volatile real32_T DTE_InertiaVehicle_P;/* Referenced by:
                                                     * '<S274>/Parameter6'
                                                     * '<S275>/Parameter6'
                                                     */

/* Inertia of the Vehicle */
extern const volatile real32_T DTE_MassVehicle_P;/* Referenced by:
                                                  * '<S272>/Parameter2'
                                                  * '<S273>/Parameter6'
                                                  * '<S274>/Parameter1'
                                                  * '<S275>/Parameter2'
                                                  */

/* Mass of the vehicle */
extern const volatile real32_T DTE_MaxCrvByBnkAgl_M[7];
                                  /* Referenced by: '<S212>/1-D Lookup Table' */

/* Max required vehicle curvature by road bank angle compensation */
extern const volatile real32_T DTE_MaxCrvByBnkAgl_X[7];
                                  /* Referenced by: '<S212>/1-D Lookup Table' */

/* X axis for max required vehicle curvature by road bank angle compensation */
extern const volatile real32_T DTE_MaxDeltaByBnkAgl_M[7];
                                  /* Referenced by: '<S213>/1-D Lookup Table' */

/* Max required Delta by road bank angle compensation */
extern const volatile real32_T DTE_MaxDeltaByBnkAgl_X[7];
                                  /* Referenced by: '<S213>/1-D Lookup Table' */

/* X axis for max required Delta by road bank angle compensation */
extern const volatile real32_T DTE_MaxReqCrvByDstrb_M[7];
                                  /* Referenced by: '<S239>/1-D Lookup Table' */

/* Max required curvature by disturbance compensator */
extern const volatile real32_T DTE_MaxReqCrvByDstrb_X[7];
                                  /* Referenced by: '<S239>/1-D Lookup Table' */

/* X axis for max required curvature by disturbance compensator */
extern const volatile real32_T DTE_MaxReqDeltaByDstrb_M[7];
                                  /* Referenced by: '<S267>/1-D Lookup Table' */

/* Max required Delta by disturbance compensator */
extern const volatile real32_T DTE_MaxReqDeltaByDstrb_X[7];
                                  /* Referenced by: '<S267>/1-D Lookup Table' */

/* X axis for max required Delta by disturbance compensator */
extern const volatile real32_T DTE_SelfSteerGrdnt_P;/* Referenced by: '<S205>/Parameter' */

/* Self Steering Gradient of the Vehicle */
extern const volatile real32_T DTE_ThdLmtReqCrv_P;/* Referenced by: '<S207>/Parameter2' */

/* Gradient Limit threshold of the curvature request needed to compensate the bank angle */
extern const volatile real32_T DTE_ThdVehVelX_P;/* Referenced by:
                                                 * '<S201>/Parameter'
                                                 * '<S201>/Parameter3'
                                                 */

/* Threhold of the absolute value of the velocity for avoiding a potential division by zero */
extern const volatile real32_T DTE_TiFltSteerAngle_P;/* Referenced by:
                                                      * '<S284>/Parameter1'
                                                      * '<S284>/Parameter6'
                                                      */

/* Time constant of the low pass filter used to filter the steering angle */
extern const volatile real32_T DTE_TimeFiterReqCrv_P;/* Referenced by: '<S206>/Parameter1' */

/* Low pass filtered time for required curvature */
extern const volatile real32_T DTE_TimeFltDeltaFPar_P;/* Referenced by: '<S176>/Parameter2' */
extern const volatile real32_T DTE_TimeFltDeltaFRte_P;/* Referenced by: '<S153>/Parameter2' */
extern const volatile real32_T DTE_TimeSysCycle_P;/* Referenced by:
                                                   * '<S206>/Parameter'
                                                   * '<S207>/Parameter3'
                                                   * '<S277>/Parameter1'
                                                   * '<S277>/Parameter2'
                                                   * '<S225>/Parameter'
                                                   * '<S225>/Parameter1'
                                                   * '<S227>/Parameter1'
                                                   * '<S228>/Parameter'
                                                   * '<S279>/Parameter1'
                                                   * '<S280>/Parameter'
                                                   * '<S285>/Parameter'
                                                   * '<S285>/Parameter1'
                                                   * '<S247>/Parameter1'
                                                   * '<S248>/Parameter1'
                                                   * '<S249>/Parameter1'
                                                   * '<S249>/Parameter2'
                                                   * '<S249>/Parameter3'
                                                   * '<S252>/Parameter'
                                                   * '<S252>/Parameter1'
                                                   * '<S252>/Parameter2'
                                                   */

/* DTE system cycle time */
extern const volatile real32_T EST_AnglePObsDThetaThd0_M[13];
                                  /* Referenced by: '<S355>/1-D Lookup Table' */
extern const volatile real32_T EST_AnglePObsDThetaThd0_X[13];
                                  /* Referenced by: '<S355>/1-D Lookup Table' */
extern const volatile real32_T EST_AnglePObsDThetaThd_M[13];
                                  /* Referenced by: '<S354>/1-D Lookup Table' */
extern const volatile real32_T EST_AnglePObsDThetaThd_X[13];
                                  /* Referenced by: '<S354>/1-D Lookup Table' */
extern const volatile real32_T EST_AnglePobsDThetaLmt1_P;/* Referenced by:
                                                          * '<S356>/Parameter2'
                                                          * '<S356>/Parameter8'
                                                          * '<S357>/Parameter7'
                                                          */
extern const volatile real32_T EST_AnglePobsDThetaLmt2_P;/* Referenced by: '<S357>/Parameter5' */
extern const volatile real32_T EST_AnglePobsDThetaThd1_P;/* Referenced by: '<S356>/Parameter1' */
extern const volatile real32_T EST_AnglePobsDThetaThd2_P;/* Referenced by: '<S357>/Parameter4' */
extern const volatile real32_T EST_CoeffL11Pobs_M[13];
                                  /* Referenced by: '<S311>/1-D Lookup Table' */

/* First element of first column of gain matrix of the Luenberger observer to estimate the course angle */
extern const volatile real32_T EST_CoeffL11Pobs_X[13];
                                  /* Referenced by: '<S311>/1-D Lookup Table' */

/* X axis for first element of first column of gain matrix of the Luenberger observer to estimate the course angle */
extern const volatile real32_T EST_CoeffL11Sobs_M[13];
                                  /* Referenced by: '<S387>/1-D Lookup Table' */
extern const volatile real32_T EST_CoeffL11Sobs_X[13];
                                  /* Referenced by: '<S387>/1-D Lookup Table' */
extern const volatile real32_T EST_CoeffL12Pobs_M[13];
                                 /* Referenced by: '<S311>/1-D Lookup Table4' */

/* First element of second column of gain matrix of the Luenberger observer to estimate the course angle */
extern const volatile real32_T EST_CoeffL12Pobs_X[13];
                                 /* Referenced by: '<S311>/1-D Lookup Table4' */

/* X axis for first element of second column of gain matrix of the Luenberger observer to estimate the course angle */
extern const volatile real32_T EST_CoeffL13Pobs_M[13];
                                 /* Referenced by: '<S311>/1-D Lookup Table9' */

/* First element of third column of gain matrix of the Luenberger observer to estimate the course angle */
extern const volatile real32_T EST_CoeffL13Pobs_X[13];
                                 /* Referenced by: '<S311>/1-D Lookup Table9' */

/* X axis for first element of third column of gain matrix of the Luenberger observer to estimate the course angle */
extern const volatile real32_T EST_CoeffL21Pobs_M[13];
                                 /* Referenced by: '<S311>/1-D Lookup Table1' */

/* Second element of first column of gain matrix of the Luenberger observer to estimate the course angle */
extern const volatile real32_T EST_CoeffL21Pobs_X[13];
                                 /* Referenced by: '<S311>/1-D Lookup Table1' */

/* X axis for second element of first column of gain matrix of the Luenberger observer to estimate the course angle */
extern const volatile real32_T EST_CoeffL21Sobs_M[13];
                                 /* Referenced by: '<S387>/1-D Lookup Table1' */
extern const volatile real32_T EST_CoeffL21Sobs_X[13];
                                 /* Referenced by: '<S387>/1-D Lookup Table1' */
extern const volatile real32_T EST_CoeffL22Pobs_M[13];
                                 /* Referenced by: '<S311>/1-D Lookup Table5' */

/* Second element of second column of gain matrix of the Luenberger observer to estimate the course angle */
extern const volatile real32_T EST_CoeffL22Pobs_X[13];
                                 /* Referenced by: '<S311>/1-D Lookup Table5' */

/* X axis for second element of second column of gain matrix of the Luenberger observer to estimate the course angle */
extern const volatile real32_T EST_CoeffL23Pobs_M[13];
                                /* Referenced by: '<S311>/1-D Lookup Table10' */

/* Second element of third column of gain matrix of the Luenberger observer to estimate the course angle */
extern const volatile real32_T EST_CoeffL23Pobs_X[13];
                                /* Referenced by: '<S311>/1-D Lookup Table10' */

/* X axis for second element of third column of gain matrix of the Luenberger observer to estimate the course angle */
extern const volatile real32_T EST_CoeffL31Pobs_M[13];
                                 /* Referenced by: '<S311>/1-D Lookup Table2' */

/* Third element of first column of gain matrix of the Luenberger observer to estimate the course angle */
extern const volatile real32_T EST_CoeffL31Pobs_X[13];
                                 /* Referenced by: '<S311>/1-D Lookup Table2' */

/* X axis for third element of first column of gain matrix of the Luenberger observer to estimate the course angle */
extern const volatile real32_T EST_CoeffL32Pobs_M[13];
                                 /* Referenced by: '<S311>/1-D Lookup Table6' */

/* Third element of second column of gain matrix of the Luenberger observer to estimate the course angle */
extern const volatile real32_T EST_CoeffL32Pobs_X[13];
                                 /* Referenced by: '<S311>/1-D Lookup Table6' */

/* X axis for third element of second column of gain matrix of the Luenberger observer to estimate the course angle */
extern const volatile real32_T EST_CoeffL33Pobs_M[13];
                                /* Referenced by: '<S311>/1-D Lookup Table11' */

/* Third element of third column of gain matrix of the Luenberger observer to estimate the course angle */
extern const volatile real32_T EST_CoeffL33Pobs_X[13];
                                /* Referenced by: '<S311>/1-D Lookup Table11' */

/* X axis for third element of third column of gain matrix of the Luenberger observer to estimate the course angle */
extern const volatile real32_T EST_CoeffL41Pobs_M[13];
                                 /* Referenced by: '<S311>/1-D Lookup Table3' */

/* Fourth element of first column of gain matrix of the Luenberger observer to estimate the course angle */
extern const volatile real32_T EST_CoeffL41Pobs_X[13];/* Referenced by:
                                                       * '<S311>/1-D Lookup Table12'
                                                       * '<S311>/1-D Lookup Table3'
                                                       */

/* X axis for fourth element of first column of gain matrix of the Luenberger observer to estimate the course angle */
extern const volatile real32_T EST_CoeffL42Pobs_M[13];
                                 /* Referenced by: '<S311>/1-D Lookup Table7' */

/* Fourth element of second column of gain matrix of the Luenberger observer to estimate the course angle */
extern const volatile real32_T EST_CoeffL42Pobs_X[13];
                                 /* Referenced by: '<S311>/1-D Lookup Table7' */

/* X axis for fourth element of second column of gain matrix of the Luenberger observer to estimate the course angle */
extern const volatile real32_T EST_CoeffL43Pobs_M[13];
                                /* Referenced by: '<S311>/1-D Lookup Table12' */

/* Fourth element of third column of gain matrix of the Luenberger observer to estimate the course angle */
extern const volatile real32_T EST_CorStiffFrontAxle_P;/* Referenced by:
                                                        * '<S596>/Parameter5'
                                                        * '<S608>/Parameter5'
                                                        * '<S407>/Parameter4'
                                                        * '<S408>/Parameter4'
                                                        * '<S409>/Parameter4'
                                                        * '<S410>/Parameter4'
                                                        * '<S411>/Parameter4'
                                                        * '<S412>/Parameter1'
                                                        */

/* Cornering stiffness coefficient of the tires at the front axle */
extern const volatile real32_T EST_CorStiffRearAxle_P;/* Referenced by:
                                                       * '<S596>/Parameter6'
                                                       * '<S608>/Parameter6'
                                                       * '<S407>/Parameter5'
                                                       * '<S408>/Parameter5'
                                                       * '<S409>/Parameter5'
                                                       * '<S410>/Parameter5'
                                                       */
extern const volatile uint8_T EST_CswEnaBetaSObs_P;/* Referenced by: '<S374>/Parameter' */

/* Configuration switch for activating beta estimation of Sobs  */
extern const volatile uint8_T EST_CswPObsDThetaLmt_P;/* Referenced by: '<S358>/Parameter' */
extern const volatile uint8_T EST_CswPObsDYSel_P;/* Referenced by: '<S366>/Parameter2' */
extern const volatile uint8_T EST_CswPlObsCrv_P;/* Referenced by:
                                                 * '<S389>/Parameter1'
                                                 * '<S389>/Parameter2'
                                                 */

/* Configuration switch for curvature of plant observer */
extern const volatile uint8_T EST_CswPlObsDeltaY_P;/* Referenced by: '<S390>/Parameter' */

/* Configuration switch for lateral distance of plant observer */
extern const volatile uint8_T EST_CswSteerAngle_P;/* Referenced by: '<S391>/Parameter2' */

/* Configuration switch for steer angle input
   0: Use Steer Angle signal provided by VDY 1: Use Steer Angle signal */
extern const volatile uint8_T EST_CswStrAngleDly_P;/* Referenced by:
                                                    * '<S386>/Parameter'
                                                    * '<S305>/Parameter1'
                                                    */

/* Configuration switch for steer angle delay cycles */
extern const volatile real32_T EST_DistCogToFrontAxle_P;/* Referenced by:
                                                         * '<S596>/Parameter'
                                                         * '<S608>/Parameter'
                                                         * '<S407>/Parameter'
                                                         * '<S408>/Parameter'
                                                         * '<S408>/Parameter7'
                                                         * '<S409>/Parameter'
                                                         * '<S410>/Parameter'
                                                         * '<S411>/Parameter'
                                                         * '<S412>/Parameter2'
                                                         * '<S413>/Parameter'
                                                         */

/* Distance between front axle and the vehicle's center of gravity */
extern const volatile real32_T EST_DistCogToRearAxle_P;/* Referenced by:
                                                        * '<S596>/Parameter1'
                                                        * '<S608>/Parameter1'
                                                        * '<S407>/Parameter1'
                                                        * '<S408>/Parameter1'
                                                        * '<S408>/Parameter8'
                                                        * '<S409>/Parameter1'
                                                        * '<S410>/Parameter1'
                                                        */

/* Distance between rear axle and the vehicle's center of gravity */
extern const volatile real32_T EST_DistPObsDYGrdntThd_M[13];
                                 /* Referenced by: '<S366>/1-D Lookup Table1' */
extern const volatile real32_T EST_DistPObsDYGrdntThd_X[13];
                                 /* Referenced by: '<S366>/1-D Lookup Table1' */
extern const volatile real32_T EST_EST_DistPObsDYThd_M[13];
                                  /* Referenced by: '<S368>/1-D Lookup Table' */
extern const volatile real32_T EST_EST_DistPObsDYThd_X[13];
                                  /* Referenced by: '<S368>/1-D Lookup Table' */
extern const volatile uint8_T EST_EnaFreezeByTgq_P;/* Referenced by:
                                                    * '<S403>/Parameter1'
                                                    * '<S403>/Parameter4'
                                                    */

/* Freeze parameter for Trajectory Guidance Qualifier Output */
extern const volatile uint8_T EST_EnaOffByTgq_P;/* Referenced by: '<S403>/Parameter2' */

/* Off parameter for Trajectory Guidance Qualifier Output */
extern const volatile real32_T EST_FacDThetaWghtHdrOf_P;/* Referenced by: '<S349>/Parameter2' */
extern const volatile real32_T EST_FacDThetaWghtHdrSf_P;/* Referenced by: '<S349>/Parameter3' */
extern const volatile real32_T EST_FacDThetaWghtHdr_P;/* Referenced by: '<S349>/Parameter1' */
extern const volatile real32_T EST_FacDYWghtHdrOf_P;/* Referenced by: '<S349>/Parameter5' */
extern const volatile real32_T EST_FacDYWghtHdrSf_P;/* Referenced by: '<S349>/Parameter6' */
extern const volatile real32_T EST_FacDYWghtHdr_P;/* Referenced by: '<S349>/Parameter4' */
extern const volatile real32_T EST_InertiaVehicle_P;/* Referenced by:
                                                     * '<S608>/Parameter4'
                                                     * '<S407>/Parameter3'
                                                     * '<S407>/Parameter6'
                                                     * '<S408>/Parameter3'
                                                     * '<S408>/Parameter6'
                                                     * '<S409>/Parameter3'
                                                     * '<S410>/Parameter3'
                                                     * '<S411>/Parameter3'
                                                     * '<S412>/Parameter3'
                                                     */

/* Inertia of the Vehicle */
extern const volatile real32_T EST_MassVehicle_P;/* Referenced by:
                                                  * '<S596>/Parameter2'
                                                  * '<S608>/Parameter2'
                                                  * '<S407>/Parameter2'
                                                  * '<S407>/Parameter7'
                                                  * '<S408>/Parameter2'
                                                  * '<S408>/Parameter9'
                                                  * '<S411>/Parameter2'
                                                  */

/* Mass of the vehicle */
extern const volatile real32_T EST_MaxDThetaDotHdr_P;/* Referenced by: '<S340>/Parameter2' */
extern const volatile real32_T EST_MaxDYDotHdr_P;/* Referenced by: '<S335>/Parameter2' */
extern const volatile real32_T EST_MinDThetaDotHdr_P;/* Referenced by:
                                                      * '<S340>/Parameter1'
                                                      * '<S340>/Parameter3'
                                                      */
extern const volatile real32_T EST_MinDYDotHdr_P;/* Referenced by:
                                                  * '<S335>/Parameter1'
                                                  * '<S335>/Parameter3'
                                                  */
extern const volatile uint8_T EST_ModeTJALatCtrlOf_P;/* Referenced by: '<S345>/Parameter2' */
extern const volatile real32_T EST_PlObsInDYTolBndGL1;/* Referenced by:
                                                       * '<S395>/Parameter2'
                                                       * '<S395>/Parameter8'
                                                       */
extern const volatile real32_T EST_PlObsInDYTolBndGL2_P;/* Referenced by:
                                                         * '<S396>/Parameter2'
                                                         * '<S396>/Parameter8'
                                                         */
extern const volatile real32_T EST_PlObsInDYTolBndThr1;/* Referenced by: '<S395>/Parameter1' */
extern const volatile real32_T EST_PlObsInDYTolBndThr2_P;/* Referenced by: '<S396>/Parameter1' */
extern const volatile real32_T EST_PreviewDistX_M[13];
                                 /* Referenced by: '<S413>/1-D Lookup Table1' */

/* Preview lateral distance for single track model */
extern const volatile real32_T EST_PreviewDistX_X[13];
                                 /* Referenced by: '<S413>/1-D Lookup Table1' */

/* Vehicle Speed after limit for avoiding a potential division by zero */
extern const volatile real32_T EST_RatioSteerGear_M[6];
                                  /* Referenced by: '<S391>/1-D Lookup Table' */

/* Steering Gear Ratio of the vehicle */
extern const volatile real32_T EST_RatioSteerGear_X[6];
                                  /* Referenced by: '<S391>/1-D Lookup Table' */

/* X axis for steering Gear Ratio of the vehicle */
extern const volatile uint8_T EST_StMainLcfOff_P;/* Referenced by: '<S345>/Parameter' */
extern const volatile uint8_T EST_StMainLcfTJA_P;/* Referenced by: '<S345>/Parameter1' */
extern const volatile real32_T EST_ThdBetaSatSObs_M[7];
                                  /* Referenced by: '<S374>/1-D Lookup Table' */

/* Threshold of saturation limit for beta of Sobs */
extern const volatile real32_T EST_ThdBetaSatSObs_X[7];
                                  /* Referenced by: '<S374>/1-D Lookup Table' */

/* X axis for threshold of saturation limit for beta of Sobs */
extern const volatile real32_T EST_ThdMeanHdrOf_M[13];
                                 /* Referenced by: '<S348>/1-D Lookup Table1' */
extern const volatile real32_T EST_ThdMeanHdrOf_X[13];
                                 /* Referenced by: '<S348>/1-D Lookup Table1' */
extern const volatile real32_T EST_ThdMeanHdrSf_M[13];
                                 /* Referenced by: '<S348>/1-D Lookup Table2' */
extern const volatile real32_T EST_ThdMeanHdrSf_X[13];
                                 /* Referenced by: '<S348>/1-D Lookup Table2' */
extern const volatile real32_T EST_ThdMeanHdr_M[13];
                                  /* Referenced by: '<S348>/1-D Lookup Table' */
extern const volatile real32_T EST_ThdMeanHdr_X[13];
                                  /* Referenced by: '<S348>/1-D Lookup Table' */
extern const volatile real32_T EST_ThdMulHdrOf_M[13];
                                 /* Referenced by: '<S348>/1-D Lookup Table4' */
extern const volatile real32_T EST_ThdMulHdrOf_X[13];
                                 /* Referenced by: '<S348>/1-D Lookup Table4' */
extern const volatile real32_T EST_ThdMulHdrSf_M[13];
                                 /* Referenced by: '<S348>/1-D Lookup Table5' */
extern const volatile real32_T EST_ThdMulHdrSf_X[13];
                                 /* Referenced by: '<S348>/1-D Lookup Table5' */
extern const volatile real32_T EST_ThdMulHdr_M[13];
                                 /* Referenced by: '<S348>/1-D Lookup Table3' */
extern const volatile real32_T EST_ThdMulHdr_X[13];
                                 /* Referenced by: '<S348>/1-D Lookup Table3' */
extern const volatile real32_T EST_ThdVehVel_P;/* Referenced by:
                                                * '<S406>/Parameter1'
                                                * '<S406>/Parameter2'
                                                */

/* Limit the absolute value of the velocity for avoiding a potential division by zero */
extern const volatile real32_T EST_TiHdrDThetaDotFlt_P;/* Referenced by: '<S341>/Parameter7' */
extern const volatile real32_T EST_TiHdrDYDotFlt_P;/* Referenced by: '<S336>/Parameter7' */
extern const volatile real32_T EST_TiPlObsInCrvFlt_P;/* Referenced by: '<S389>/Parameter3' */
extern const volatile real32_T EST_TiSysCycle_P;/* Referenced by:
                                                 * '<S389>/Parameter9'
                                                 * '<S376>/Parameter1'
                                                 * '<S376>/Parameter9'
                                                 * '<S395>/Parameter3'
                                                 * '<S395>/Parameter9'
                                                 * '<S396>/Parameter3'
                                                 * '<S396>/Parameter9'
                                                 * '<S313>/Parameter1'
                                                 * '<S313>/Parameter2'
                                                 * '<S313>/Parameter3'
                                                 * '<S313>/Parameter4'
                                                 * '<S336>/Parameter6'
                                                 * '<S341>/Parameter6'
                                                 * '<S366>/Parameter3'
                                                 * '<S366>/Parameter9'
                                                 * '<S355>/Parameter3'
                                                 * '<S355>/Parameter9'
                                                 * '<S356>/Parameter3'
                                                 * '<S356>/Parameter9'
                                                 * '<S357>/Parameter10'
                                                 * '<S357>/Parameter6'
                                                 */
extern const volatile uint8_T FFC_CswFfcCrv_P;/* Referenced by: '<S80>/Parameter' */

/* Configuration switch for feedforward curvature activation */
extern const volatile uint8_T FFC_StTgqReqFreeze_P;/* Referenced by: '<S80>/Parameter1' */

/* Freeze parameter for Trajectory Guidance Qualifier Output */
extern const volatile uint8_T FFC_StTgqReqOff_P;/* Referenced by: '<S80>/Parameter2' */

/* Off parameter for Trajectory Guidance Qualifier Output */
extern const volatile uint8_T LGC_BtmDynIntRst_P;/* Referenced by:
                                                  * '<S434>/Parameter12'
                                                  * '<S434>/Parameter13'
                                                  * '<S434>/Parameter14'
                                                  * '<S434>/Parameter15'
                                                  * '<S434>/Parameter16'
                                                  * '<S434>/Parameter17'
                                                  */

/* UNDEFINED */
extern const volatile uint8_T LGC_BtmDynPT1Rst_P;/* Referenced by:
                                                  * '<S434>/Parameter'
                                                  * '<S434>/Parameter1'
                                                  * '<S434>/Parameter2'
                                                  * '<S434>/Parameter3'
                                                  * '<S434>/Parameter4'
                                                  */

/* 0000 0000 : Dynamic PT1 reset deactivated
   0000 xxx1 : Reset based on lateral deviation active
   0000 xx1x : Reset based on driver torque active
   0000 x1xx : Reset based on standstill active */
extern const volatile real32_T LGC_CoeffDGainLcCac_M[15];
                         /* Referenced by: '<S426>/Y_TCTLGC_LdcDGain_radspm1' */

/* Gain of the Course Angle Controller's differential part in dependence on the vehicle's velocity */
extern const volatile real32_T LGC_CoeffDGainLcCac_X[15];
                         /* Referenced by: '<S426>/Y_TCTLGC_LdcDGain_radspm1' */
extern const volatile real32_T LGC_CoeffDGainLcLdc_M[15];
                          /* Referenced by: '<S430>/Y_TCTLGC_LdcDGain_radspm' */

/* Gain of the Y-Coorindate Controller's differential part in dependence on the vehicle's velocity */
extern const volatile real32_T LGC_CoeffDGainLcLdc_X[15];
                          /* Referenced by: '<S430>/Y_TCTLGC_LdcDGain_radspm' */
extern const volatile real32_T LGC_CoeffDGainOfCac_M[15];
                        /* Referenced by: '<S426>/Y_TCTLGC_LdcOfDGain_radspm' */

/* Gain of the Course Angle Controller's differential part in dependence on the vehicle's velocity */
extern const volatile real32_T LGC_CoeffDGainOfCac_X[15];
                        /* Referenced by: '<S426>/Y_TCTLGC_LdcOfDGain_radspm' */
extern const volatile real32_T LGC_CoeffDGainOfLdc_M[15];
                        /* Referenced by: '<S430>/Y_TCTLGC_LdcOfDGain_radspm' */

/* Gain of the Y-Coorindate Controller's differential part in dependence on the vehicle's velocity */
extern const volatile real32_T LGC_CoeffDGainOfLdc_X[15];
                        /* Referenced by: '<S430>/Y_TCTLGC_LdcOfDGain_radspm' */
extern const volatile real32_T LGC_CoeffDGainSfCac_M[15];
                        /* Referenced by: '<S426>/Y_TCTLGC_LdcSfDGain_radspm' */

/* Gain of the Course Angle Controller's differential part in dependence on the vehicle's velocity */
extern const volatile real32_T LGC_CoeffDGainSfCac_X[15];
                        /* Referenced by: '<S426>/Y_TCTLGC_LdcSfDGain_radspm' */
extern const volatile real32_T LGC_CoeffDGainSfLdc_M[15];
                        /* Referenced by: '<S430>/Y_TCTLGC_LdcSfDGain_radspm' */

/* Gain of the Y-Coorindate Controller's differential part in dependence on the vehicle's velocity */
extern const volatile real32_T LGC_CoeffDGainSfLdc_X[15];
                        /* Referenced by: '<S430>/Y_TCTLGC_LdcSfDGain_radspm' */
extern const volatile real32_T LGC_CoeffIGainLcCac_M[15];
                          /* Referenced by: '<S427>/Y_TCTLGC_LdcIGain_radpsm' */

/* Gain of the Course Angle Controller's integral part in dependence on the vehicle's velocity */
extern const volatile real32_T LGC_CoeffIGainLcCac_X[15];
                          /* Referenced by: '<S427>/Y_TCTLGC_LdcIGain_radpsm' */
extern const volatile real32_T LGC_CoeffIGainLcLdc_M[15];
                          /* Referenced by: '<S431>/Y_TCTLGC_LdcIGain_radpsm' */

/* Gain of the Y-Coordinate Controller's integral part in dependence on the vehicle's velocity */
extern const volatile real32_T LGC_CoeffIGainLcLdc_X[15];
                          /* Referenced by: '<S431>/Y_TCTLGC_LdcIGain_radpsm' */
extern const volatile real32_T LGC_CoeffIGainOfCac_M[15];
                        /* Referenced by: '<S427>/Y_TCTLGC_LdcOfIGain_radpsm' */

/* Gain of the Course Angle Controller's integral part in dependence on the vehicle's velocity */
extern const volatile real32_T LGC_CoeffIGainOfCac_X[15];
                        /* Referenced by: '<S427>/Y_TCTLGC_LdcOfIGain_radpsm' */
extern const volatile real32_T LGC_CoeffIGainOfLdc_M[15];
                        /* Referenced by: '<S431>/Y_TCTLGC_LdcOfIGain_radpsm' */

/* Gain of the Y-Coordinate Controller's integral part in dependence on the vehicle's velocity */
extern const volatile real32_T LGC_CoeffIGainOfLdc_X[15];
                        /* Referenced by: '<S431>/Y_TCTLGC_LdcOfIGain_radpsm' */
extern const volatile real32_T LGC_CoeffIGainSfCac_M[15];
                        /* Referenced by: '<S427>/Y_TCTLGC_LdcSfIGain_radpsm' */

/* Gain of the Course Angle Controller's integral part in dependence on the vehicle's velocity */
extern const volatile real32_T LGC_CoeffIGainSfCac_X[15];
                        /* Referenced by: '<S427>/Y_TCTLGC_LdcSfIGain_radpsm' */
extern const volatile real32_T LGC_CoeffIGainSfLdc_M[15];
                        /* Referenced by: '<S431>/Y_TCTLGC_LdcSfIGain_radpsm' */

/* Gain of the Y-Coordinate Controller's integral part in dependence on the vehicle's velocity */
extern const volatile real32_T LGC_CoeffIGainSfLdc_X[15];
                        /* Referenced by: '<S431>/Y_TCTLGC_LdcSfIGain_radpsm' */
extern const volatile real32_T LGC_CoeffMainPGainLcCac_P;
                               /* Referenced by: '<S428>/P_TCTLGC_LdcP_radpm' */

/* Proportional master gain of the Course Angle Controller */
extern const volatile real32_T LGC_CoeffMainPGainLcLdc_P;
                               /* Referenced by: '<S432>/P_TCTLGC_LdcP_radpm' */

/* Proportional master gain of the Lateral Deviation Controller(Lane Centering)
 */
extern const volatile real32_T LGC_CoeffMainPGainOfCac_P;/* Referenced by: '<S428>/Constant' */

/* Proportional master gain of the Course Angle Controller */
extern const volatile real32_T LGC_CoeffMainPGainOfLdc_P;/* Referenced by: '<S432>/Constant' */

/* Proportional master gain of the Lateral Deviation Controller */
extern const volatile real32_T LGC_CoeffMainPGainSfCac_P;
                             /* Referenced by: '<S428>/P_TCTLGC_LdcSfP_radpm' */

/* Proportional master gain of the Course Angle Controller */
extern const volatile real32_T LGC_CoeffMainPGainSfLdc_P;
                             /* Referenced by: '<S432>/P_TCTLGC_LdcSfP_radpm' */

/* Proportional master gain of the Lateral Deviation Controller */
extern const volatile real32_T LGC_CoeffNumS0LaDmc_M[15];
                                  /* Referenced by: '<S425>/1-D Lookup Table' */

/* s^0 coefficient of the approximated LaDMC transfer function's numerator */
extern const volatile real32_T LGC_CoeffNumS0LaDmc_X[15];
                                  /* Referenced by: '<S425>/1-D Lookup Table' */

/* X axis for s^0 coefficient of the approximated LaDMC transfer function's numerator */
extern const volatile real32_T LGC_CoeffNumS1LaDmc_M[15];
                                 /* Referenced by: '<S425>/1-D Lookup Table1' */

/* s^1 coefficient of the approximated LaDMC transfer function's numerator */
extern const volatile real32_T LGC_CoeffNumS1LaDmc_X[15];
                                 /* Referenced by: '<S425>/1-D Lookup Table1' */

/* X aixs for s^1 coefficient of the approximated LaDMC transfer function's numerator */
extern const volatile real32_T LGC_CoeffPGainByCrvCac_M[9];
                       /* Referenced by: '<S428>/Y_TCTLGC_LdcPGainCrv_radpm1' */
extern const volatile real32_T LGC_CoeffPGainByCrvCac_X[9];
                       /* Referenced by: '<S428>/Y_TCTLGC_LdcPGainCrv_radpm1' */
extern const volatile real32_T LGC_CoeffPGainByCrvLdc_M[9];
                       /* Referenced by: '<S432>/Y_TCTLGC_LdcPGainCrv_radpm1' */
extern const volatile real32_T LGC_CoeffPGainByCrvLdc_X[9];
                       /* Referenced by: '<S432>/Y_TCTLGC_LdcPGainCrv_radpm1' */
extern const volatile real32_T LGC_CoeffPGainLcCac_M[15];
                              /* Referenced by: '<S428>/Y_TCTLGC_LdcP_radpm1' */

/* Proportional gain of the Course Angle Controller in dependence on the vehicle's velocity */
extern const volatile real32_T LGC_CoeffPGainLcCac_X[15];
                              /* Referenced by: '<S428>/Y_TCTLGC_LdcP_radpm1' */
extern const volatile real32_T LGC_CoeffPGainLcLdc_M[15];
                              /* Referenced by: '<S432>/Y_TCTLGC_LdcP_radpm1' */

/* Proportional gain of the Y-Coordinate Controller in dependence on the vehicle's velocity */
extern const volatile real32_T LGC_CoeffPGainLcLdc_X[15];
                              /* Referenced by: '<S432>/Y_TCTLGC_LdcP_radpm1' */
extern const volatile real32_T LGC_CoeffPGainOfCac_M[15];
                            /* Referenced by: '<S428>/Y_TCTLGC_LdcOfP_radpm1' */

/* Course Angle Controller P-Gain for object following mode */
extern const volatile real32_T LGC_CoeffPGainOfCac_X[15];
                            /* Referenced by: '<S428>/Y_TCTLGC_LdcOfP_radpm1' */
extern const volatile real32_T LGC_CoeffPGainOfLdc_M[15];
                            /* Referenced by: '<S432>/Y_TCTLGC_LdcOfP_radpm1' */

/* Lateral Deviation Controller P-Gain for object following mode */
extern const volatile real32_T LGC_CoeffPGainOfLdc_X[15];
                            /* Referenced by: '<S432>/Y_TCTLGC_LdcOfP_radpm1' */
extern const volatile real32_T LGC_CoeffPGainSfCac_M[15];
                            /* Referenced by: '<S428>/Y_TCTLGC_LdcSfP_radpm1' */

/* Proportional gain of the Course Angle Controller in dependence on the vehicle's velocity */
extern const volatile real32_T LGC_CoeffPGainSfCac_X[15];
                            /* Referenced by: '<S428>/Y_TCTLGC_LdcSfP_radpm1' */
extern const volatile real32_T LGC_CoeffPGainSfLdc_M[15];
                            /* Referenced by: '<S432>/Y_TCTLGC_LdcSfP_radpm1' */

/* Proportional gain of the Y-Coordinate Controller in dependence on the vehicle's velocity */
extern const volatile real32_T LGC_CoeffPGainSfLdc_X[15];
                            /* Referenced by: '<S432>/Y_TCTLGC_LdcSfP_radpm1' */
extern const volatile real32_T LGC_CoeffPT1GainLcCac_M[15];
                          /* Referenced by: '<S429>/Y_TCTLGC_LdcDGain_radspm' */

/* Gain of the Course Angle Deviation Controller's PT1-Branch */
extern const volatile real32_T LGC_CoeffPT1GainLcCac_X[15];
                          /* Referenced by: '<S429>/Y_TCTLGC_LdcDGain_radspm' */
extern const volatile real32_T LGC_CoeffPT1GainLcLdc_M[15];
                          /* Referenced by: '<S433>/Y_TCTLGC_LdcDGain_radspm' */

/* Gain of the Lateral Deviation Controller's PT1-Branch */
extern const volatile real32_T LGC_CoeffPT1GainLcLdc_X[15];
                          /* Referenced by: '<S433>/Y_TCTLGC_LdcDGain_radspm' */
extern const volatile real32_T LGC_CoeffPT1GainOfCac_M[15];
                        /* Referenced by: '<S429>/Y_TCTLGC_LdcOfDGain_radspm' */

/* Gain of the Course Angle Deviation Controller's PT1-Branch */
extern const volatile real32_T LGC_CoeffPT1GainOfCac_X[15];
                        /* Referenced by: '<S429>/Y_TCTLGC_LdcOfDGain_radspm' */
extern const volatile real32_T LGC_CoeffPT1GainOfLdc_M[15];
                        /* Referenced by: '<S433>/Y_TCTLGC_LdcOfDGain_radspm' */

/* Gain of the Lateral Deviation Controller's PT1-Branch */
extern const volatile real32_T LGC_CoeffPT1GainOfLdc_X[15];
                        /* Referenced by: '<S433>/Y_TCTLGC_LdcOfDGain_radspm' */
extern const volatile real32_T LGC_CoeffPT1GainSfCac_M[15];
                        /* Referenced by: '<S429>/Y_TCTLGC_LdcSfDGain_radspm' */

/* Gain of the Course Angle Deviation Controller's PT1-Branch */
extern const volatile real32_T LGC_CoeffPT1GainSfCac_X[15];
                        /* Referenced by: '<S429>/Y_TCTLGC_LdcSfDGain_radspm' */
extern const volatile real32_T LGC_CoeffPT1GainSfLdc_M[15];
                        /* Referenced by: '<S433>/Y_TCTLGC_LdcSfDGain_radspm' */

/* Gain of the Lateral Deviation Controller's PT1-Branch */
extern const volatile real32_T LGC_CoeffPT1GainSfLdc_X[15];
                        /* Referenced by: '<S433>/Y_TCTLGC_LdcSfDGain_radspm' */
extern const volatile real32_T LGC_CoeffPole1LaDmc_M[15];
                                 /* Referenced by: '<S425>/1-D Lookup Table2' */

/* Pole 1 of the approximated LaDMC transfer function */
extern const volatile real32_T LGC_CoeffPole1LaDmc_X[15];
                                 /* Referenced by: '<S425>/1-D Lookup Table2' */

/* X axis for pole 1 of the approximated LaDMC transfer function */
extern const volatile real32_T LGC_CoeffPole2LaDmc_M[15];
                                 /* Referenced by: '<S425>/1-D Lookup Table3' */

/* Pole 2 of the approximated LaDMC transfer function */
extern const volatile real32_T LGC_CoeffPole2LaDmc_X[15];
                                 /* Referenced by: '<S425>/1-D Lookup Table3' */

/* X axis for pole 2 of the approximated LaDMC transfer function */
extern const volatile uint8_T LGC_CswActOverride_P;/* Referenced by:
                                                    * '<S628>/Constant18'
                                                    * '<S590>/Constant18'
                                                    */

/* Configuration switch for DMC command signal
   1: Force ENABLE OUTPUT = 1
   0: Normal ENABLE OUTPUT behavior */
extern const volatile uint8_T LGC_CswCacMode_P;/* Referenced by:
                                                * '<S444>/P_TCTLGC_LdcMode_nu'
                                                * '<S444>/P_TCTLGC_LdcMode_nu1'
                                                * '<S444>/P_TCTLGC_LdcMode_nu2'
                                                * '<S444>/P_TCTLGC_LdcMode_nu3'
                                                * '<S484>/P_TCTLGC_LdcMode_nu'
                                                * '<S484>/P_TCTLGC_LdcMode_nu1'
                                                * '<S484>/P_TCTLGC_LdcMode_nu2'
                                                * '<S484>/P_TCTLGC_LdcMode_nu3'
                                                */

/* Y-Coordinate Controller Mode:
   xxxx xxx0	Controller is deactivated
   xxxx xxx1	P-Controller
   xxxx xx11	PDT1-Controller, DT1-Controller parallel to the P-Controller
   xxxx x1x1	PI-Controller, I-Part parallel to the P-Controller
   xxxx 1xx1	PPT1-Controller, PT1-Part parallel to the P-Controller */
extern const volatile uint8_T LGC_CswCssDeltaF_P;/* Referenced by:
                                                  * '<S574>/Constant10'
                                                  * '<S574>/Constant15'
                                                  * '<S574>/Constant17'
                                                  * '<S574>/Constant18'
                                                  * '<S574>/Constant8'
                                                  */

/* Mode of TCTLGC (Steer Angle interface):
   Source  |  Controller
   0000 0000: Off
   0000 xxx1: FBC active
   0000 xx1x: FFC active
   0000 x1xx: DC active
   0000 1xxx: BAC active
   0001 0000: Chirp active */
extern const volatile uint8_T LGC_CswFltErrCourse_P;/* Referenced by:
                                                     * '<S463>/Constant1'
                                                     * '<S503>/Constant1'
                                                     */

/* 1: Enable low pass first order for filtering the control error regarding the Course Angle
   0: Disable low pass first order for filtering the control error regarding the Course Angle */
extern const volatile uint8_T LGC_CswFltErrDistY_P;/* Referenced by: '<S545>/Constant1' */

/* 1: Enable low pass first order for filtering the control error regarding the Y-Coordinate
   0: Disable low pass first order for filtering the control error regarding the Y-Coordinate */
extern const volatile uint8_T LGC_CswLaDmcCmpnCac_P;/* Referenced by:
                                                     * '<S461>/Constant'
                                                     * '<S501>/Constant'
                                                     */

/* 1: Enable filter to compensate the LaDMC's transmission behavior
   0: Disable filter to compensate the LaDMC's transmission behavior */
extern const volatile uint8_T LGC_CswLaDmcCmpnLdc_P;/* Referenced by: '<S543>/Constant' */

/* 1: Enable filter to compensate the LaDMC's transmission behavior
   0: Disable filter to compensate the LaDMC's transmission behavior */
extern const volatile uint8_T LGC_CswLdcMode_P;/* Referenced by:
                                                * '<S524>/P_TCTLGC_LdcMode_nu'
                                                * '<S524>/P_TCTLGC_LdcMode_nu1'
                                                * '<S524>/P_TCTLGC_LdcMode_nu2'
                                                * '<S524>/P_TCTLGC_LdcMode_nu3'
                                                */

/* Y-Coordinate Controller Mode:
   xxxx xxx0	Controller is deactivated
   xxxx xxx1	P-Controller
   xxxx xx11	PDT1-Controller, DT1-Controller parallel to the P-Controller
   xxxx x1x1	PI-Controller, I-Part parallel to the P-Controller
   xxxx 1xx1	PPT1-Controller, PT1-Part parallel to the P-Controller */
extern const volatile uint8_T LGC_CswPT1DeltaFCmd_P;/* Referenced by: '<S574>/Constant13' */

/* 1: Enable low pass first order for filtering the control signal DeltaFCmd
   0: Disable low pass first order for filtering the control signal DeltaFCmd */
extern const volatile uint8_T LGC_EnaFreezeByTgq_P;/* Referenced by:
                                                    * '<S417>/Parameter1'
                                                    * '<S417>/Parameter4'
                                                    * '<S597>/Parameter1'
                                                    * '<S597>/Parameter3'
                                                    * '<S597>/Parameter4'
                                                    * '<S597>/Parameter5'
                                                    */

/* Freeze parameter for Trajectory Guidance Qualifier Output */
extern const volatile real32_T LGC_EnaFreezeByVel_P;/* Referenced by: '<S597>/Parameter8' */
extern const volatile uint8_T LGC_EnaOffByTgq_P;/* Referenced by:
                                                 * '<S417>/Parameter2'
                                                 * '<S597>/Parameter2'
                                                 */

/* Off parameter for Trajectory Guidance Qualifier Output */
extern const volatile real32_T LGC_IntResMaxCrv_P;/* Referenced by: '<S435>/Parameter1' */

/* UNDEFINED */
extern const volatile real32_T LGC_IntResMaxLatErr_P;/* Referenced by: '<S435>/Parameter' */

/* UNDEFINED */
extern const volatile real32_T LGC_IntResMaxManTrq_P;/* Referenced by: '<S435>/Parameter2' */

/* UNDEFINED */
extern const volatile real32_T LGC_MaxGrdPT1Ldc_P;/* Referenced by: '<S528>/Constant12' */

/* Absolute value of the maximum allowed gradient of the PT1 control signal part of the lateral deviation controller */
extern const volatile real32_T LGC_MaxReqDeltaFGrd_P;/* Referenced by:
                                                      * '<S627>/Constant'
                                                      * '<S589>/Constant'
                                                      */

/* Maximum allowed gradient of the steer angle command signal which is send to the LaDMC */
extern const volatile real32_T LGC_MaxReqDeltaF_P;/* Referenced by:
                                                   * '<S628>/Constant2'
                                                   * '<S628>/Constant3'
                                                   * '<S590>/Constant2'
                                                   * '<S590>/Constant3'
                                                   */

/* Maximum allowed steer angle command signal which is send to the LaDMC */
extern const volatile uint8_T LGC_OfLatCtrlMode_P;/* Referenced by: '<S422>/Parameter' */

/* Of mode for lateral control by TJASTM */
extern const volatile uint8_T LGC_OffLcfCtrlFcn_P;/* Referenced by: '<S422>/Parameter1' */

/* Off function for lcf control funtion by MCTLFC */
extern const volatile uint8_T LGC_TJALcfCtrlFcn_P;/* Referenced by: '<S422>/Parameter2' */

/* TJA function for lcf control funtion by MCTLFC */
extern const volatile real32_T LGC_ThdPGainGrdCac_P;/* Referenced by:
                                                     * '<S497>/Constant6'
                                                     * '<S497>/Constant8'
                                                     */

/* Gradient Limit for P-Gain Transition */
extern const volatile real32_T LGC_ThdPGainGrdCas_P;/* Referenced by:
                                                     * '<S457>/Constant6'
                                                     * '<S457>/Constant8'
                                                     * '<S458>/Constant1'
                                                     * '<S498>/Constant1'
                                                     */

/* Gradient Limit for P-Gain Transition */
extern const volatile real32_T LGC_ThdPGainGrdLdc_P;/* Referenced by:
                                                     * '<S539>/Constant6'
                                                     * '<S539>/Constant8'
                                                     * '<S540>/Constant1'
                                                     * '<S446>/Constant18'
                                                     * '<S486>/Constant18'
                                                     * '<S526>/Constant18'
                                                     */

/* Gradient Limit for P-Gain Transition */
extern const volatile real32_T LGC_ThdVelStandStill_P;/* Referenced by: '<S435>/Parameter3' */

/* Definition of standstill for TCTLGC */
extern const volatile real32_T LGC_TimeDT1LcCac_M[15];
                               /* Referenced by: '<S426>/Y_TCTLGC_LdcDT1_sec' */

/* Time constant of the first order low pass needed to filter the Course Angle Controller's D-part in dependence on the vehicle's velocity */
extern const volatile real32_T LGC_TimeDT1LcCac_X[15];
                               /* Referenced by: '<S426>/Y_TCTLGC_LdcDT1_sec' */
extern const volatile real32_T LGC_TimeDT1LcLdc_M[15];
                               /* Referenced by: '<S430>/Y_TCTLGC_LdcDT1_sec' */

/* Time constant of the first order low pass needed to filter the Y-Coorindate Controller's D-part in dependence on the vehicle's velocity */
extern const volatile real32_T LGC_TimeDT1LcLdc_X[15];
                               /* Referenced by: '<S430>/Y_TCTLGC_LdcDT1_sec' */
extern const volatile real32_T LGC_TimeDT1OfCac_M[15];
                             /* Referenced by: '<S426>/Y_TCTLGC_LdcOfDT1_sec' */

/* Time constant of the first order low pass needed to filter the Course Angle Controller's D-part in dependence on the vehicle's velocity */
extern const volatile real32_T LGC_TimeDT1OfCac_X[15];
                             /* Referenced by: '<S426>/Y_TCTLGC_LdcOfDT1_sec' */
extern const volatile real32_T LGC_TimeDT1OfLdc_M[15];
                             /* Referenced by: '<S430>/Y_TCTLGC_LdcOfDT1_sec' */

/* Lateral Deviation Controller P-Gain for object following mode */
extern const volatile real32_T LGC_TimeDT1OfLdc_X[15];
                             /* Referenced by: '<S430>/Y_TCTLGC_LdcOfDT1_sec' */
extern const volatile real32_T LGC_TimeDT1SfCac_M[15];
                             /* Referenced by: '<S426>/Y_TCTLGC_LdcSfDT1_sec' */

/* Time constant of the first order low pass needed to filter the Course Angle Controller's D-part in dependence on the vehicle's velocity */
extern const volatile real32_T LGC_TimeDT1SfCac_X[15];
                             /* Referenced by: '<S426>/Y_TCTLGC_LdcSfDT1_sec' */
extern const volatile real32_T LGC_TimeDT1SfLdc_M[15];
                             /* Referenced by: '<S430>/Y_TCTLGC_LdcSfDT1_sec' */

/* Proportional gain of the Y-Coordinate Controller in dependence on the vehicle's velocity */
extern const volatile real32_T LGC_TimeDT1SfLdc_X[15];
                             /* Referenced by: '<S430>/Y_TCTLGC_LdcSfDT1_sec' */
extern const volatile real32_T LGC_TimeFltErrCourse_M[15];/* Referenced by:
                                                           * '<S463>/Y_TCTLGC_PT1YErrTime_sec'
                                                           * '<S503>/Y_TCTLGC_PT1YErrTime_sec'
                                                           */

/* Time constant of the low pass filter used to filter the control error regarding the Course Angle */
extern const volatile real32_T LGC_TimeFltErrCourse_X[15];/* Referenced by:
                                                           * '<S463>/Y_TCTLGC_PT1YErrTime_sec'
                                                           * '<S503>/Y_TCTLGC_PT1YErrTime_sec'
                                                           */
extern const volatile real32_T LGC_TimeFltErrDistY_M[15];
                          /* Referenced by: '<S545>/Y_TCTLGC_PT1YErrTime_sec' */

/* Time constant of the low pass filter used to filter the control error regarding the Y-Coordinate */
extern const volatile real32_T LGC_TimeFltErrDistY_X[15];
                          /* Referenced by: '<S545>/Y_TCTLGC_PT1YErrTime_sec' */
extern const volatile real32_T LGC_TimePT1DeltaFCmd_M[15];
                     /* Referenced by: '<S574>/Y_TCTLGC_PT1DeltaFCmdTime_sec' */

/* Velocity dependend value of the low pass' time constant to filter DeltaFCmd */
extern const volatile real32_T LGC_TimePT1DeltaFCmd_X[15];
                     /* Referenced by: '<S574>/Y_TCTLGC_PT1DeltaFCmdTime_sec' */
extern const volatile real32_T LGC_TimePT1LcCac_M[15];
                               /* Referenced by: '<S429>/Y_TCTLGC_LdcDT1_sec' */

/* Time Constant of the Course Angle Deviation Controller's PT1-Branch */
extern const volatile real32_T LGC_TimePT1LcCac_X[15];
                               /* Referenced by: '<S429>/Y_TCTLGC_LdcDT1_sec' */
extern const volatile real32_T LGC_TimePT1LcLdc_M[15];
                               /* Referenced by: '<S433>/Y_TCTLGC_LdcDT1_sec' */

/* Time Constant of the Lateral Deviation Controller's PT1-Branch */
extern const volatile real32_T LGC_TimePT1LcLdc_X[15];
                               /* Referenced by: '<S433>/Y_TCTLGC_LdcDT1_sec' */
extern const volatile real32_T LGC_TimePT1OfCac_M[15];
                             /* Referenced by: '<S429>/Y_TCTLGC_LdcOfDT1_sec' */

/* Time Constant of the Course Angle Deviation Controller's PT1-Branch */
extern const volatile real32_T LGC_TimePT1OfCac_X[15];
                             /* Referenced by: '<S429>/Y_TCTLGC_LdcOfDT1_sec' */
extern const volatile real32_T LGC_TimePT1OfLdc_M[15];
                             /* Referenced by: '<S433>/Y_TCTLGC_LdcOfDT1_sec' */

/* Time Constant of the Lateral Deviation Controller's PT1-Branch */
extern const volatile real32_T LGC_TimePT1OfLdc_X[15];
                             /* Referenced by: '<S433>/Y_TCTLGC_LdcOfDT1_sec' */
extern const volatile real32_T LGC_TimePT1SfCac_M[15];
                             /* Referenced by: '<S429>/Y_TCTLGC_LdcSfDT1_sec' */

/* Time Constant of the Course Angle Deviation Controller's PT1-Branch */
extern const volatile real32_T LGC_TimePT1SfCac_X[15];
                             /* Referenced by: '<S429>/Y_TCTLGC_LdcSfDT1_sec' */
extern const volatile real32_T LGC_TimePT1SfLdc_M[15];
                             /* Referenced by: '<S433>/Y_TCTLGC_LdcSfDT1_sec' */

/* Time Constant of the Lateral Deviation Controller's PT1-Branch */
extern const volatile real32_T LGC_TimePT1SfLdc_X[15];
                             /* Referenced by: '<S433>/Y_TCTLGC_LdcSfDT1_sec' */
extern const volatile real32_T LGC_TimeSysCycle_P;/* Referenced by:
                                                   * '<S82>/Parameter1'
                                                   * '<S82>/Parameter2'
                                                   * '<S82>/Parameter4'
                                                   * '<S419>/Parameter3'
                                                   * '<S601>/Parameter1'
                                                   * '<S602>/Parameter1'
                                                   * '<S574>/Parameter'
                                                   * '<S604>/Parameter3'
                                                   * '<S605>/Parameter4'
                                                   * '<S627>/Constant4'
                                                   * '<S463>/Parameter'
                                                   * '<S503>/Parameter'
                                                   * '<S545>/Parameter'
                                                   * '<S589>/Constant4'
                                                   * '<S457>/Constant7'
                                                   * '<S457>/Constant9'
                                                   * '<S458>/Constant5'
                                                   * '<S464>/Parameter'
                                                   * '<S464>/Parameter1'
                                                   * '<S465>/Parameter1'
                                                   * '<S467>/Parameter1'
                                                   * '<S467>/Parameter9'
                                                   * '<S497>/Constant7'
                                                   * '<S497>/Constant9'
                                                   * '<S498>/Constant5'
                                                   * '<S504>/Parameter'
                                                   * '<S504>/Parameter1'
                                                   * '<S505>/Parameter1'
                                                   * '<S507>/Parameter1'
                                                   * '<S507>/Parameter9'
                                                   * '<S539>/Constant7'
                                                   * '<S539>/Constant9'
                                                   * '<S540>/Constant5'
                                                   * '<S546>/Parameter'
                                                   * '<S546>/Parameter1'
                                                   * '<S547>/Parameter1'
                                                   * '<S549>/Parameter1'
                                                   * '<S549>/Parameter9'
                                                   * '<S446>/Parameter2'
                                                   * '<S446>/Parameter3'
                                                   * '<S447>/Parameter4'
                                                   * '<S448>/Parameter'
                                                   * '<S486>/Parameter2'
                                                   * '<S486>/Parameter3'
                                                   * '<S487>/Parameter4'
                                                   * '<S488>/Parameter'
                                                   * '<S526>/Parameter2'
                                                   * '<S526>/Parameter3'
                                                   * '<S527>/Parameter4'
                                                   * '<S528>/Parameter'
                                                   * '<S528>/Parameter1'
                                                   */
extern const volatile real32_T LQR_ButterCutFreq;/* Referenced by: '<S82>/Parameter3' */
extern const volatile real32_T LQR_ButterQ;/* Referenced by: '<S82>/Parameter5' */
extern const volatile real32_T LQR_FltDeltaDistYFc;/* Referenced by: '<S604>/Parameter1' */
extern const volatile real32_T LQR_FltKappaFc;/* Referenced by: '<S82>/Parameter6' */
extern const volatile real32_T LQR_FltYawRateFc;/* Referenced by: '<S602>/Parameter2' */
extern const volatile real32_T LQR_LeadLagCutFreq;/* Referenced by: '<S601>/Parameter2' */
extern const volatile real32_T LQR_LeadLagGain;/* Referenced by: '<S601>/Parameter3' */
extern const volatile real32_T LQR_VelX[9];/* Referenced by:
                                            * '<S598>/Constant1'
                                            * '<S607>/Constant1'
                                            */
extern const volatile real32_T P_TCTLGC_CacILimit_rad;/* Referenced by:
                                                       * '<S447>/P_TCTLGC_LdcILimit_rad'
                                                       * '<S487>/P_TCTLGC_LdcILimit_rad'
                                                       */

/* Gradient Limit for P-Gain Transition */
extern const volatile real32_T P_TCTLGC_LdcILimit_rad;
                            /* Referenced by: '<S527>/P_TCTLGC_LdcILimit_rad' */

/* Gradient Limit for P-Gain Transition */
extern const volatile real32_T deadZone_Vxthd;/* Referenced by: '<S595>/Parameter5' */
extern const volatile real32_T deadZone_weightedGain_e1;/* Referenced by: '<S595>/Parameter1' */
extern const volatile real32_T deadZone_weightedGain_e2;/* Referenced by: '<S595>/Parameter2' */
extern const volatile real32_T deadZone_width;/* Referenced by: '<S595>/Parameter3' */

/* Declaration for custom storage class: Global */
extern uint8_T CDC_BtfQualifier;       /* '<S86>/Multiport Switch3' */
extern real32_T CDC_CtrlErrDistY;      /* '<S87>/Multiport Switch2' */

/* UNDEFINED */
extern real32_T CDC_CtrlErrHeading;    /* '<S91>/Multiport Switch2' */
extern boolean_T CDC_EnaCntrlByTgq;    /* '<S85>/Equal1' */

/* Enalbe flag for control by trajectory guidance qualifier output */
extern boolean_T CDC_EnaFreezeByTgq;   /* '<S85>/Equal' */

/* Enalbe flag for freeze by trajectory guidance qualifier output */
extern boolean_T CDC_EnaResetByTgq;    /* '<S85>/OR' */

/* Enalbe flag for reset by trajectory guidance qualifier output */
extern boolean_T CDC_EnaWatchdog;      /* '<S86>/AND1' */

/* 0: Watchdog inactive
   1: Watchdog active */
extern real32_T CDC_EstCurHeading;     /* '<S92>/Neg2' */

/* UNDEFINED */
extern real32_T CDC_EstDeltaTheta;     /* '<S93>/Add1' */

/* Course Angle Deviation calculated by means of "HAF estimator" */
extern real32_T CDC_FltDeltaTheta;     /* '<S94>/Switch' */

/* PT1 filtered Course Angle Deviation */
extern real32_T CDC_FltErrDistYTpl;    /* '<S89>/Switch' */

/* Lateral Deviation, PT1 filtered */
extern real32_T CDC_HldCtrlErrDistY;   /* '<S87>/Multiport Switch' */

/* UNDEFINED */
extern real32_T CDC_HldCtrlErrHeading; /* '<S91>/Multiport Switch1' */
extern real32_T CDC_PreErrCtrlHeading; /* '<S91>/Data Type Conversion' */
extern uint8_T CDC_RawBtfQualifier;    /* '<S86>/Sum' */

/* UNDEFINED */
extern real32_T CDC_RawCtrlErrDistY;   /* '<S87>/Multiport Switch1' */

/* UNDEFINED */
extern real32_T CDC_RawCtrlErrHeading; /* '<S91>/Multiport Switch' */
extern real32_T CDC_RawDeltaTheta;     /* '<S93>/Add' */

/* Course Angle Deviation */
extern real32_T CDC_RawErrDistYTpl;    /* '<S88>/Add' */

/* Lateral Deviation */
extern real32_T CDC_RawFltDeltaTheta;  /* '<S93>/Signal Conversion' */

/* PT1 filtered Course Angle Deviation */
extern real32_T CLM_CrvBySteerAngle;   /* '<S134>/Divide1' */

/* UNDEFINED */
extern boolean_T CLM_EnaDegrReq;       /* '<S106>/Multiport Switch1' */
extern boolean_T CLM_EnaGrdDeltaFCmd;  /* '<S115>/AND1' */
extern boolean_T CLM_EnaGrdFbcDc;      /* '<S120>/AND1' */
extern boolean_T CLM_EnaGrdFfcCrv;     /* '<S125>/AND1' */
extern boolean_T CLM_EnaSatDeltaFCmd;  /* '<S116>/GreaterThan' */
extern boolean_T CLM_EnaSatFbcDc;      /* '<S121>/GreaterThan' */
extern boolean_T CLM_EnaSatFfcCrv;     /* '<S126>/GreaterThan' */
extern real32_T CLM_GrdDeltaFCmd;      /* '<S115>/Multiport Switch' */

/* Steering Angle Command for DMC */
extern real32_T CLM_GrdFbcDc;          /* '<S120>/Multiport Switch' */

/* UNDEFINED */
extern real32_T CLM_GrdFfcCrv;         /* '<S125>/Multiport Switch' */

/* UNDEFINED */
extern boolean_T CLM_RawEnaDegrReq;    /* '<S107>/Switch' */

/* Reset condition for Flip-Flop which holds degradation request */
extern real32_T CLM_RawGrdDeltaFCmd;   /* '<S117>/Multiport Switch1' */

/* Steering Angle Command for DMC */
extern real32_T CLM_RawGrdFbcDc;       /* '<S122>/Multiport Switch1' */

/* UNDEFINED */
extern real32_T CLM_RawGrdFfcCrv;      /* '<S127>/Multiport Switch1' */

/* UNDEFINED */
extern real32_T CLM_SatDeltaFCmd;      /* '<S119>/Switch2' */

/* Steering Angle Command for DMC */
extern real32_T CLM_SatFbcDc;          /* '<S124>/Switch2' */

/* UNDEFINED */
extern real32_T CLM_SatFfcCrv;         /* '<S129>/Switch2' */

/* UNDEFINED */
extern real32_T CLM_ThdDeltaFCmdGrd;   /* '<S115>/Multiport Switch1' */
extern real32_T CLM_ThdDeltaFCmdSat;   /* '<S116>/Multiport Switch1' */
extern real32_T CLM_ThdFbcDcGrd;       /* '<S120>/1-D Lookup Table' */

/* UNDEFINED */
extern real32_T CLM_ThdFbcDcSat;       /* '<S121>/1-D Lookup Table2' */

/* UNDEFINED */
extern real32_T CLM_ThdFfcCrvGrd;      /* '<S125>/1-D Lookup Table' */

/* UNDEFINED */
extern real32_T CLM_ThdFfcCrvSat;      /* '<S126>/1-D Lookup Table2' */

/* UNDEFINED */
extern uint8_T DEV_BtfFfcQualifierPar; /* '<S181>/Add2' */
extern uint8_T DEV_BtfFfcQualifierRte; /* '<S158>/Add2' */
extern real32_T DEV_CoeffDeltaGainFfc; /* '<S144>/Y_TCTFFC_GainFFC_nu' */

/* Gain of the control signal part of the feedforward controller */
extern real32_T DEV_CrvTestSignal;     /* '<S194>/Multiport Switch' */

/* Test signal for curvature output of lateral KMC */
extern real32_T DEV_DeltaFTestSignal;  /* '<S196>/Multiport Switch' */

/* Test signal for steering angle command from LaKMC to DMC */
extern real32_T DEV_DlySetDeltaF2DotPar;/* '<S180>/Unit Delay1' */
extern real32_T DEV_DlySetDeltaF2DotRte;/* '<S157>/Unit Delay1' */

/* Feedforward control signal part for the steer angle interface calculated by means of the self steering gradient based on signal from VDY */
extern real32_T DEV_DlySetDeltaFDotPar;/* '<S180>/Unit Delay' */
extern real32_T DEV_DlySetDeltaFDotRte;/* '<S157>/Unit Delay' */

/* Feedforward control signal part for the steer angle interface calculated by means of the self steering gradient based on signal from VDY */
extern real32_T DEV_DlySetDeltaFPar;   /* '<S180>/Unit Delay2' */
extern real32_T DEV_DlySetDeltaFRte;   /* '<S157>/Unit Delay2' */

/* Feedforward control signal part for the steer angle interface calculated by means of the self steering gradient based on signal from VDY */
extern boolean_T DEV_EnaCntrlByTgq;    /* '<S140>/Equal1' */

/* Enalbe flag for control by trajectory guidance qualifier output */
extern uint8_T DEV_EnaCrvGen;          /* '<S195>/Bitwise Operator' */
extern uint8_T DEV_EnaDeltaFGen;       /* '<S197>/Bitwise Operator' */
extern boolean_T DEV_EnaFreezeByTgq;   /* '<S140>/Equal' */

/* Enalbe flag for freeze by trajectory guidance qualifier output */
extern boolean_T DEV_EnaResetByTgq;    /* '<S140>/OR' */

/* Enalbe flag for reset by trajectory guidance qualifier output */
extern real32_T DEV_HldReqDeltaFRte;   /* '<S143>/Multiport Switch1' */

/* Feedforward control signal part based on self steering gradient (given by VDY from RTE) for the steer angle interface */
extern real32_T DEV_ReqDeltaFRte;      /* '<S143>/Multiport Switch' */

/* Feedforward control signal part based on self steering gradient (given by VDY from RTE) for the steer angle interface */
extern boolean_T DEV_RstCrvGen;        /* '<S198>/FixPt Relational Operator' */
extern boolean_T DEV_RstDeltaFGen;     /* '<S199>/FixPt Relational Operator' */
extern real32_T DEV_SetDeltaF3DotPar;  /* '<S179>/Multiport Switch' */
extern real32_T DEV_SetDeltaF3DotRte;  /* '<S156>/Multiport Switch' */
extern real32_T DEV_SetDeltaFPar;      /* '<S191>/Switch2' */
extern real32_T DEV_SetDeltaFRte;      /* '<S168>/Switch2' */

/* Feedforward control signal part for the steer angle interface calculated by means of the self steering gradient based on signal from VDY */
extern real32_T DEV_TimeCrvGen;        /* '<S195>/Multiport Switch' */
extern real32_T DEV_TimeDeltaFGen;     /* '<S197>/Multiport Switch' */
extern real32_T DTE_CoeffA0TranferFcn; /* '<S271>/Product1' */

/* s^0 coefficient of the approximated vehicle dynamic transfer function's numerator */
extern real32_T DTE_CoeffA1TranferFcn; /* '<S272>/Product9' */

/* s^1 coefficient of the approximated vehicle dynamic transfer function's numerator */
extern real32_T DTE_CoeffB0TranferFcn; /* '<S273>/Add' */

/* s^0 coefficient of the approximated vehicle dynamic transfer function's denominator */
extern real32_T DTE_CoeffB1TranferFcn; /* '<S274>/Add1' */

/* s^1 coefficient of the approximated vehicle dynamic transfer function's denominator */
extern real32_T DTE_CoeffB2TranferFcn; /* '<S275>/Product9' */

/* s^2 coefficient of the approximated vehicle dynamic transfer function's denominator */
extern real32_T DTE_CoeffDenS0LaDmc;   /* '<S241>/1-D Lookup Table' */

/* s^0 coefficient of the approximated LaDMC transfer function's denominator */
extern real32_T DTE_CoeffDenS1LaDmc;   /* '<S241>/1-D Lookup Table1' */

/* s^1 coefficient of the approximated LaDMC transfer function's denominator */
extern real32_T DTE_CoeffDenS2LaDmc;   /* '<S241>/1-D Lookup Table2' */

/* s^2 coefficient of the approximated LaDMC transfer function's denominator */
extern real32_T DTE_CoeffDenS3LaDmc;   /* '<S241>/1-D Lookup Table3' */

/* s^3 coefficient of the approximated LaDMC transfer function's denominator */
extern real32_T DTE_CoeffNumS0LaDmc;   /* '<S241>/1-D Lookup Table4' */

/* s^0 coefficient of the approximated LaDMC transfer function's numerator */
extern real32_T DTE_CoeffNumS1LaDmc;   /* '<S241>/1-D Lookup Table5' */

/* s^1 coefficient of the approximated LaDMC transfer function's numerator */
extern real32_T DTE_Delta2DotForCrv;   /* '<S227>/Divide1' */

/* Second order discrete derivative of ego vehicle delta for lateral dynamic transfer function */
extern real32_T DTE_Delta2DotLaDmc;    /* '<S247>/Divide1' */

/* Second order discrete derivative of ego vehicle delta for lateral dynamic transfer function */
extern real32_T DTE_Delta2DotVdyFcn;   /* '<S284>/Multiport Switch' */

/* steer angle of the front wheels by vehicle dynamic transfer function */
extern real32_T DTE_Delta3DotLaDmc;    /* '<S248>/Divide1' */

/* Third order discrete derivative of ego vehicle delta for lateral dynamic transfer function */
extern real32_T DTE_DeltaByVdyFcn;     /* '<S277>/Subtract' */

/* Current steer angle of the front wheels */
extern real32_T DTE_DeltaDotForCrv;    /* '<S228>/Divide' */

/* First order discrete derivative of ego vehicle delta for lateral dynamic transfer function */
extern real32_T DTE_DeltaDotLaDmc;     /* '<S250>/Add' */

/* First order discrete derivative of ego vehicle delta for lateral dynamic transfer function */
extern real32_T DTE_DeltaF2DotPar;     /* '<S174>/Divide1' */
extern real32_T DTE_DeltaF2DotRte;     /* '<S151>/Divide1' */
extern real32_T DTE_DeltaF3DotPar;     /* '<S175>/Divide1' */
extern real32_T DTE_DeltaF3DotRte;     /* '<S152>/Divide1' */
extern real32_T DTE_DeltaFDotPar;      /* '<S177>/Add' */
extern real32_T DTE_DeltaFDotRte;      /* '<S154>/Add' */
extern real32_T DTE_DeltaFPar;         /* '<S142>/Mul1' */
extern real32_T DTE_DeltaFRte;         /* '<S141>/Mul1' */
extern real32_T DTE_DeltaVdyFcn;       /* '<S288>/Switch2' */
extern real32_T DTE_DlyCurSteerAngle;  /* '<S277>/Multiport_Switch1' */

/* Current steer angle of the front wheels */
extern real32_T DTE_DlyDeltaDotVdyFcn; /* '<S285>/Unit Delay1' */
extern real32_T DTE_DlyDeltaVdyFcn;    /* '<S285>/Unit Delay' */
extern real32_T DTE_DlySetCrvDotLaDmc; /* '<S225>/Unit Delay1' */
extern real32_T DTE_DlySetCrvLaDmc;    /* '<S225>/Unit Delay' */
extern real32_T DTE_DlySetDelta2DotLaDmc;/* '<S252>/Unit Delay1' */
extern real32_T DTE_DlySetDeltaDotLaDmc;/* '<S252>/Unit Delay' */
extern real32_T DTE_DlySetDeltaLaDmc;  /* '<S252>/Unit Delay2' */
extern boolean_T DTE_EnaCtrlByTgq;     /* '<S201>/Equal1' */

/* Enalbe flag for control by trajectory guidance qualifier output */
extern boolean_T DTE_EnaFreezeByTgq;   /* '<S201>/Equal' */

/* Enalbe flag for freeze by trajectory guidance qualifier output */
extern boolean_T DTE_EnaResetByTgq;    /* '<S201>/OR' */

/* Enalbe flag for reset by trajectory guidance qualifier output */
extern real32_T DTE_EstCrvByBnkAgl;    /* '<S206>/Multiport Switch' */

/* Estimating curvature by road bank angle compensation */
extern real32_T DTE_FltDlyCurSteerAngle;/* '<S282>/Add' */

/* Current steer angle of the front wheels */
extern real32_T DTE_HldReqCrvByBnkAgl; /* '<S212>/Multiport Switch1' */

/* Holding value for required vehicle curvature by road bank angle compensation */
extern real32_T DTE_HldReqCrvByDstrb;  /* '<S239>/Multiport Switch1' */

/* Holding value for required curvature by disturbance compensator */
extern real32_T DTE_HldReqDeltaByBnkAgl;/* '<S213>/Multiport Switch1' */

/* Holding value for required Delta by road bank angle compensation */
extern real32_T DTE_HldReqDeltaByDstrb;/* '<S267>/Multiport Switch1' */

/* Holding value for required Delta by disturbance compensator */
extern real32_T DTE_KappaAngleLaDmc;   /* '<S229>/1-D Lookup Table' */

/* Grid points of the y-Axis of the LaDMC Look Up Table "Kappa To Angle". */
extern real32_T DTE_LmtEstCrvByBnkAgl; /* '<S207>/Multiport Switch2' */

/* Gradient limiter value for Estimating curvature by road bank angle compensation */
extern real32_T DTE_LmtReqCrvByBnkAgl; /* '<S214>/Switch2' */

/* Limiting value for required vehicle curvature by road bank angle compensation */
extern real32_T DTE_LmtReqCrvByDstrb;  /* '<S240>/Switch2' */

/* Limiting value for required curvature by disturbance compensator */
extern real32_T DTE_LmtReqDeltaByBnkAgl;/* '<S215>/Switch2' */

/* Limiting value for required Delta by road bank angle compensation */
extern real32_T DTE_LmtReqDeltaByDstrb;/* '<S268>/Switch2' */

/* Limiting value for required Delta by disturbance compensator */
extern real32_T DTE_LmtVehVelX;        /* '<S217>/Switch2' */

/* Vehicle Speed after limit for avoiding a potential division by zero */
extern real32_T DTE_MaxCrvByBnkAgl;    /* '<S212>/1-D Lookup Table' */

/* Max required vehicle curvature by road bank angle compensation */
extern real32_T DTE_MaxDeltaByBnkAgl;  /* '<S213>/1-D Lookup Table' */

/* Max required Delta by road bank angle compensation */
extern real32_T DTE_MaxReqCrvByDstrb;  /* '<S239>/1-D Lookup Table' */

/* Max required curvature by disturbance compensator */
extern real32_T DTE_MaxReqDeltaByDstrb;/* '<S267>/1-D Lookup Table' */

/* Max required Delta by disturbance compensator */
extern real32_T DTE_NdlySetCrvLaDmc;   /* '<S238>/Multiport_Switch1' */
extern real32_T DTE_NdlySetDeltaLaDmc; /* '<S266>/Multiport_Switch1' */
extern real32_T DTE_Psi2DotVdyFcn;     /* '<S280>/Divide' */

/* First1 order discrete derivative of ego vehicle yaw rate for vehicle dynamic transfer function */
extern real32_T DTE_Psi3DotVdyFcn;     /* '<S279>/Divide1' */

/* Second order discrete derivative of ego vehicle yaw rate for vehicle dynamic transfer function */
extern real32_T DTE_RawCrvLaDmc;       /* '<S229>/Product1' */
extern real32_T DTE_RawDeltaDotLaDmc;  /* '<S249>/Divide1' */

/* Raw first order discrete derivative of ego vehicle delta for lateral dynamic transfer function */
extern real32_T DTE_RawDeltaFDotPar;   /* '<S176>/Divide1' */
extern real32_T DTE_RawDeltaFDotRte;   /* '<S153>/Divide1' */
extern real32_T DTE_RawFltEstCrvByBnkAgl;/* '<S208>/Add' */

/* Raw low pass filtered value for Estimating curvature by road bank angle compensation */
extern real32_T DTE_RawLmtEstCrvByBnkAgl;/* '<S210>/Multiport Switch1' */

/* Raw gradient limiter value for Estimating curvature by road bank angle compensation */
extern real32_T DTE_RawReqCrvByBnkAgl; /* '<S207>/Product1' */

/* Raw required vehicle curvature by road bank angle compensation */
extern real32_T DTE_RawReqCrvByDstrb;  /* '<S238>/Product' */

/* Raw required curvature by disturbance compensator */
extern real32_T DTE_RawReqDeltaByBnkAgl;/* '<S205>/Product3' */
extern real32_T DTE_RawReqDeltaByDstrb;/* '<S266>/Product' */

/* Raw required Delta by disturbance compensator */
extern real32_T DTE_ReqCrvByBnkAgl;    /* '<S212>/Multiport Switch' */

/* Required vehicle curvature by road bank angle compensation */
extern real32_T DTE_ReqCrvByDstrb;     /* '<S239>/Multiport Switch' */

/* Required curvature by disturbance compensator */
extern real32_T DTE_ReqDeltaByBnkAgl;  /* '<S213>/Multiport Switch' */

/* Required Delta by road bank angle compensation */
extern real32_T DTE_ReqDeltaByDstrb;   /* '<S267>/Multiport Switch' */

/* Required Delta by disturbance compensator */
extern real32_T DTE_ResCrvDenLaDmc;    /* '<S226>/Add3' */
extern real32_T DTE_ResDeltaDenLaDmc;  /* '<S246>/Add1' */

/* Vehicle dynamic transfer function's denominator result for delta */
extern real32_T DTE_ResDeltaDenPar;    /* '<S173>/Add1' */
extern real32_T DTE_ResDeltaDenRte;    /* '<S150>/Add1' */
extern real32_T DTE_ResDeltaDenVdyFcn; /* '<S281>/Add1' */

/* Vehicle dynamic transfer function's denominator result for delta */
extern real32_T DTE_SetCrv2DotLaDmc;   /* '<S224>/Multiport Switch' */
extern real32_T DTE_SetCrvGainLaDmc;   /* '<S238>/1-D Lookup Table' */
extern real32_T DTE_SetCrvLaDmc;       /* '<S232>/Switch2' */
extern real32_T DTE_SetDelta3DotLaDmc; /* '<S253>/Multiport Switch' */

/* Third order discrete derivative of ego vehicle set delta for lateral dynamic transfer function */
extern real32_T DTE_SetDeltaGainLaDmc; /* '<S266>/1-D Lookup Table' */

/* Gain of the Disturbance Compensator set value */
extern real32_T DTE_SetDeltaLaDmc;     /* '<S263>/Switch2' */
extern real32_T EST_AngleCurSteer;     /* '<S391>/Multiport Switch' */

/* Current steer angle of the front wheels */
extern real32_T EST_AngleLaDMCSteer;   /* '<S391>/Product' */

/* Current steer angle of the front wheels from LaDMC */
extern real32_T EST_AnglePObsDTheta;   /* '<S354>/Multiport Switch' */
extern real32_T EST_AnglePObsDThetaFreeze;/* '<S354>/Multiport Switch1' */
extern real32_T EST_AnglePObsDThetaLmt0;/* '<S355>/Multiport Switch' */
extern real32_T EST_AnglePObsDThetaLmt0Raw;/* '<S359>/Multiport Switch1' */
extern real32_T EST_AnglePObsDThetaLmt1;/* '<S361>/Multiport Switch1' */
extern real32_T EST_AnglePObsDThetaLmt2;/* '<S363>/Multiport Switch1' */
extern real32_T EST_AnglePObsDThetaSat;/* '<S365>/Switch2' */
extern real32_T EST_AnglePObsDThetaSel;/* '<S358>/Multiport Switch1' */
extern real32_T EST_AnglePObsDThetaThd;/* '<S354>/1-D Lookup Table' */
extern real32_T EST_AnglePObsDThetaThd0;/* '<S355>/1-D Lookup Table' */
extern real32_T EST_AngleVDYSteer;     /* '<S391>/Divide1' */

/* Current steer angle of the front wheels from VDY */
extern real32_T EST_BetaDotPobs;       /* '<S312>/Signal Conversion' */
extern real32_T EST_BetaDotSObs;       /* '<S375>/Signal Conversion' */
extern real32_T EST_BetaSObs;          /* '<S374>/Multiport Switch' */
extern uint16_T EST_BtfQualifierByBeta;/* '<S374>/Shift Arithmetic' */
extern uint16_T EST_BtfQualifierByEna; /* '<S404>/Add2' */
extern uint16_T EST_BtfQualifierByHdr; /* '<S346>/Add2' */
extern real32_T EST_CoeffA11StateSpace;/* '<S407>/Subtract' */

/* Single Track State Space Coefficitents A11 */
extern real32_T EST_CoeffA12StateSpace;/* '<S408>/Subtract' */

/* Single Track State Space Coefficitents A12 */
extern real32_T EST_CoeffA21StateSpace;/* '<S409>/Divide' */

/* Single Track State Space Coefficitents A21 */
extern real32_T EST_CoeffA22StateSpace;/* '<S410>/Divide' */

/* Single Track State Space Coefficitents A22 */
extern real32_T EST_CoeffAXStateSpace[2];/* '<S385>/Mul' */
extern real32_T EST_CoeffB11StateSpace;/* '<S411>/Product' */
extern real32_T EST_CoeffB21StateSpace;/* '<S412>/Divide' */
extern real32_T EST_CoeffL11Pobs;      /* '<S311>/1-D Lookup Table' */

/* First element of first column of gain matrix of the Luenberger observer to estimate the course angle */
extern real32_T EST_CoeffL11Sobs;      /* '<S387>/1-D Lookup Table' */
extern real32_T EST_CoeffL12Pobs;      /* '<S311>/1-D Lookup Table4' */

/* First element of second column of gain matrix of the Luenberger observer to estimate the course angle */
extern real32_T EST_CoeffL13Pobs;      /* '<S311>/1-D Lookup Table9' */

/* First element of third column of gain matrix of the Luenberger observer to estimate the course angle */
extern real32_T EST_CoeffL21Pobs;      /* '<S311>/1-D Lookup Table1' */

/* Second element of first column of gain matrix of the Luenberger observer to estimate the course angle */
extern real32_T EST_CoeffL21Sobs;      /* '<S387>/1-D Lookup Table1' */
extern real32_T EST_CoeffL22Pobs;      /* '<S311>/1-D Lookup Table5' */

/* Second element of second column of gain matrix of the Luenberger observer to estimate the course angle */
extern real32_T EST_CoeffL23Pobs;      /* '<S311>/1-D Lookup Table10' */

/* Second element of third column of gain matrix of the Luenberger observer to estimate the course angle */
extern real32_T EST_CoeffL31Pobs;      /* '<S311>/1-D Lookup Table2' */

/* Third element of first column of gain matrix of the Luenberger observer to estimate the course angle */
extern real32_T EST_CoeffL32Pobs;      /* '<S311>/1-D Lookup Table6' */

/* Third element of second column of gain matrix of the Luenberger observer to estimate the course angle */
extern real32_T EST_CoeffL33Pobs;      /* '<S311>/1-D Lookup Table11' */

/* Third element of third column of gain matrix of the Luenberger observer to estimate the course angle */
extern real32_T EST_CoeffL41Pobs;      /* '<S311>/1-D Lookup Table3' */

/* Fourth element of first column of gain matrix of the Luenberger observer to estimate the course angle */
extern real32_T EST_CoeffL42Pobs;      /* '<S311>/1-D Lookup Table7' */

/* Fourth element of second column of gain matrix of the Luenberger observer to estimate the course angle */
extern real32_T EST_CoeffL43Pobs;      /* '<S311>/1-D Lookup Table12' */

/* Fourth element of third column of gain matrix of the Luenberger observer to estimate the course angle */
extern real32_T EST_CoeffLPobs[12];    /* '<S311>/FeedbackMatrixL' */
extern real32_T EST_CoeffLYPObs[4];    /* '<S310>/Mul' */
extern real32_T EST_CoeffLYStateSpace[2];/* '<S387>/Product' */
extern real32_T EST_CrvPiObsCrvFlt;    /* '<S392>/Add' */

/* Target curvature after low pass filter for plant observer */
extern real32_T EST_CrvPlObsIn;        /* '<S389>/Multiport Switch' */

/* Curvature input for plant observer */
extern real32_T EST_CurSteerAngle;     /* '<S73>/Data Type Conversion' */

/* Current steer angle of the front wheels */
extern real32_T EST_DThetaDotPobs;     /* '<S312>/Signal Conversion2' */
extern real32_T EST_DYDotPobs;         /* '<S312>/Signal Conversion3' */
extern real32_T EST_DeltaYPlObsIn;     /* '<S390>/Multiport Switch' */
extern real32_T EST_DistFromCgToGud;   /* '<S413>/Add' */

/* Distance between Center of Gravity and point to be guided */
extern real32_T EST_DistPObsDY;        /* '<S368>/Multiport Switch' */
extern real32_T EST_DistPObsDYFreeze;  /* '<S368>/Multiport Switch1' */
extern real32_T EST_DistPObsDYGrdnt;   /* '<S366>/Multiport Switch' */
extern real32_T EST_DistPObsDYGrdntThd;/* '<S366>/1-D Lookup Table1' */
extern real32_T EST_DistPObsDYSat;     /* '<S371>/Switch2' */
extern real32_T EST_DistPObsDYSel;     /* '<S366>/Multiport Switch1' */
extern real32_T EST_DistPObsDYThd;     /* '<S368>/1-D Lookup Table' */
extern real32_T EST_DistPobsDYGrdntRaw;/* '<S369>/Multiport Switch1' */
extern real32_T EST_DistYDevByGrdntLmt1;/* '<S398>/Multiport Switch1' */

/* Lateral distance deviation after first gradient limit */
extern real32_T EST_DistYDevStep;      /* '<S395>/Subtract3' */
extern real32_T EST_DistYDevTrajFromCur;/* '<S390>/Subtract' */

/* Deviation between Trajectory distance Y and current distance Y */
extern real32_T EST_DlyCurSteerAngle;  /* '<S386>/Multiport_Switch' */

/* Current steering angle after delay */
extern boolean_T EST_EnaActvtGrdntLmt1;/* '<S395>/GreaterThan3' */

/* Enable flag for first gradient limit activation */
extern boolean_T EST_EnaActvtGrdntLmt2;/* '<S396>/AND' */

/* Enable flag for second gradient limit activation */
extern boolean_T EST_EnaBetaSatSObs;   /* '<S374>/GreaterThan' */
extern boolean_T EST_EnaByMeanHdr;     /* '<S346>/GreaterThan1' */
extern boolean_T EST_EnaByMulHdrPerc;  /* '<S346>/GreaterThan' */
extern boolean_T EST_EnaCntrlByTgq;    /* '<S403>/Equal1' */
extern boolean_T EST_EnaFreezeByTgq;   /* '<S403>/Equal' */

/* Enalbe flag for freeze by trajectory guidance qualifier output */
extern boolean_T EST_EnaLmt2ByDistY;   /* '<S396>/OR' */

/* Enable flag for second gradient limt */
extern boolean_T EST_EnaLmtByDistY;    /* '<S395>/GreaterThan' */
extern boolean_T EST_EnaPObsDThetaLmt0;/* '<S355>/AND' */
extern boolean_T EST_EnaPObsDThetaLmt1;/* '<S356>/GreaterThan3' */
extern boolean_T EST_EnaPObsDThetaLmt2;/* '<S357>/AND' */
extern boolean_T EST_EnaPObsDThetaRst1;/* '<S356>/NOT' */
extern boolean_T EST_EnaPObsDThetaRst2;/* '<S357>/OR1' */
extern boolean_T EST_EnaPObsDThetaSat; /* '<S354>/GreaterThan' */
extern boolean_T EST_EnaPObsDYGrdnt;   /* '<S366>/AND' */
extern boolean_T EST_EnaPObsDYGrdntRaw;/* '<S366>/GreaterThan3' */
extern boolean_T EST_EnaPObsDYSat;     /* '<S368>/GreaterThan' */
extern boolean_T EST_EnaResetByTgq;    /* '<S403>/OR' */

/* Enalbe flag for reset by trajectory guidance qualifier output */
extern real32_T EST_ErrVehYawRate;     /* '<S387>/Subtract' */
extern real32_T EST_EstBetaPobs;       /* '<S318>/Switch2' */
extern real32_T EST_EstBetaSObs;       /* '<S379>/Switch2' */
extern real32_T EST_EstDThetaPobs;     /* '<S320>/Switch2' */
extern real32_T EST_EstDYPobs;         /* '<S321>/Switch2' */
extern real32_T EST_EstPsiDotPobs;     /* '<S319>/Switch2' */
extern real32_T EST_EstPsiDotSObs;     /* '<S382>/Switch2' */
extern real32_T EST_FacDThetaWghtHdrSel;/* '<S349>/Multiport Switch3' */
extern real32_T EST_FacDYWghtHdrSel;   /* '<S349>/Multiport Switch4' */
extern real32_T EST_FltDThetaDotPObs;  /* '<S341>/Multiport Switch' */
extern real32_T EST_FltDYDotPObs;      /* '<S336>/Multiport Switch' */
extern real32_T EST_HdrPercByDY;       /* '<S335>/Multiport Switch1' */
extern real32_T EST_HdrPercByTheta;    /* '<S340>/Multiport Switch1' */
extern real32_T EST_HldBetaSObs;       /* '<S374>/Multiport Switch1' */
extern real32_T EST_LmtBetaSObs;       /* '<S388>/Switch2' */
extern real32_T EST_LmtHdrPercByDY;    /* '<S337>/Switch2' */
extern real32_T EST_LmtHdrPercByTheta; /* '<S342>/Switch2' */
extern real32_T EST_LmtVehVelX;        /* '<S406>/Multiport Switch1' */

/* Vehicle Speed after limit for avoiding a potential division by zero */
extern real32_T EST_MeanHdrPerc;       /* '<S346>/Divide1' */
extern uint8_T EST_ModeSelParHdr;      /* '<S345>/Multiport Switch' */
extern real32_T EST_MulHdrPerc;        /* '<S346>/Divide' */
extern real32_T EST_Psi2DotPobs;       /* '<S312>/Signal Conversion1' */
extern real32_T EST_Psi2DotSObs;       /* '<S375>/Signal Conversion1' */
extern real32_T EST_RatioSteerGear;    /* '<S391>/1-D Lookup Table' */
extern real32_T EST_RawBetaSObs;       /* '<S374>/Multiport Switch2' */
extern real32_T EST_RawEstBetaPobs;    /* '<S323>/Init' */
extern real32_T EST_RawEstDThetaPobs;  /* '<S327>/Init' */
extern real32_T EST_RawEstDYPobs;      /* '<S329>/Init' */
extern real32_T EST_RawEstPsiDotPobs;  /* '<S325>/Init' */
extern real32_T EST_RawFltDThetaDotPObs;/* '<S343>/Add' */
extern real32_T EST_RawFltDYDotPObs;   /* '<S338>/Add' */
extern real32_T EST_RawHdrPercByDY;    /* '<S335>/Product' */
extern real32_T EST_RawHdrPercByTheta; /* '<S340>/Product' */
extern real32_T EST_ThdBetaSatSObs;    /* '<S374>/1-D Lookup Table' */
extern real32_T EST_ThdMeanHdrSel;     /* '<S348>/Multiport Switch1' */
extern real32_T EST_ThdMulHdrSel;      /* '<S348>/Multiport Switch2' */
extern real32_T FFC_HldReqFfcCrv;      /* '<S80>/Unit Delay1' */

/* Curvature control signal part of the feedforward controller */
extern real32_T FFC_ReqFfcCrv;         /* '<S80>/Multiport Switch2' */

/* Curvature control signal part of the feedforward controller */
extern uint8_T LGC_ActiveLgcParamSet_nu;/* '<S81>/Data Type Conversion12' */

/*
   Active LGC Paramter Set:
   1: Lane Centering
   2: Object Following
   3: Safety Function
 */
extern boolean_T LGC_CacIntReset_nu;   /* '<S81>/Data Type Conversion16' */
extern boolean_T LGC_CacPT1Reset_nu;   /* '<S81>/Data Type Conversion14' */
extern real32_T LGC_CdcCmd_rad;        /* '<S81>/Data Type Conversion4' */
extern real32_T LGC_Cmpn2DotLaDmcCas;  /* '<S466>/Multiport Switch' */
extern real32_T LGC_Cmpn2DotLaDmcCdc;  /* '<S506>/Multiport Switch' */
extern real32_T LGC_Cmpn2DotLaDmcLdc;  /* '<S548>/Multiport Switch' */
extern real32_T LGC_CmpnDotLaDmcCas;   /* '<S475>/Switch2' */
extern real32_T LGC_CmpnDotLaDmcCdc;   /* '<S515>/Switch2' */
extern real32_T LGC_CmpnDotLaDmcLdc;   /* '<S557>/Switch2' */
extern real32_T LGC_CmpnLaDmcCas;      /* '<S472>/Switch2' */
extern real32_T LGC_CmpnLaDmcCdc;      /* '<S512>/Switch2' */
extern real32_T LGC_CmpnLaDmcLdc;      /* '<S554>/Switch2' */
extern real32_T LGC_CoeffDGainCac;     /* '<S426>/Multiport Switch3' */
extern real32_T LGC_CoeffDGainLcCac;   /* '<S426>/Y_TCTLGC_LdcDGain_radspm1' */
extern real32_T LGC_CoeffDGainLcLdc;   /* '<S430>/Y_TCTLGC_LdcDGain_radspm' */

/* Gain of the Y-Coorindate Controller's differential part in dependence on the vehicle's velocity */
extern real32_T LGC_CoeffDGainLdc;     /* '<S430>/Multiport Switch3' */

/* Gain of the Y-Coorindate Controller's differential part in dependence on the vehicle's velocity */
extern real32_T LGC_CoeffDGainOfCac;   /* '<S426>/Y_TCTLGC_LdcOfDGain_radspm' */
extern real32_T LGC_CoeffDGainOfLdc;   /* '<S430>/Y_TCTLGC_LdcOfDGain_radspm' */

/* Gain of the Y-Coorindate Controller's differential part in dependence on the vehicle's velocity */
extern real32_T LGC_CoeffDGainSfCac;   /* '<S426>/Y_TCTLGC_LdcSfDGain_radspm' */
extern real32_T LGC_CoeffDGainSfLdc;   /* '<S430>/Y_TCTLGC_LdcSfDGain_radspm' */

/* Gain of the Y-Coorindate Controller's differential part in dependence on the vehicle's velocity */
extern real32_T LGC_CoeffIGainCac;     /* '<S427>/Multiport Switch1' */
extern real32_T LGC_CoeffIGainLcCac;   /* '<S427>/Y_TCTLGC_LdcIGain_radpsm' */
extern real32_T LGC_CoeffIGainLcLdc;   /* '<S431>/Y_TCTLGC_LdcIGain_radpsm' */
extern real32_T LGC_CoeffIGainLdc;     /* '<S431>/Multiport Switch1' */
extern real32_T LGC_CoeffIGainOfCac;   /* '<S427>/Y_TCTLGC_LdcOfIGain_radpsm' */
extern real32_T LGC_CoeffIGainOfLdc;   /* '<S431>/Y_TCTLGC_LdcOfIGain_radpsm' */
extern real32_T LGC_CoeffIGainSfCac;   /* '<S427>/Y_TCTLGC_LdcSfIGain_radpsm' */
extern real32_T LGC_CoeffIGainSfLdc;   /* '<S431>/Y_TCTLGC_LdcSfIGain_radpsm' */
extern real32_T LGC_CoeffMainPGainCac; /* '<S428>/Multiport Switch7' */
extern real32_T LGC_CoeffMainPGainLdc; /* '<S432>/Multiport Switch7' */
extern real32_T LGC_CoeffNumS0LaDmc;   /* '<S425>/1-D Lookup Table' */

/* s^0 coefficient of the approximated LaDMC transfer function's numerator */
extern real32_T LGC_CoeffNumS1LaDmc;   /* '<S425>/1-D Lookup Table1' */

/* s^1 coefficient of the approximated LaDMC transfer function's numerator */
extern real32_T LGC_CoeffPGainByCrvCac;
                                      /* '<S428>/Y_TCTLGC_LdcPGainCrv_radpm1' */
extern real32_T LGC_CoeffPGainByCrvLdc;
                                      /* '<S432>/Y_TCTLGC_LdcPGainCrv_radpm1' */
extern real32_T LGC_CoeffPGainCac;     /* '<S428>/Multiport Switch2' */
extern real32_T LGC_CoeffPGainLcCac;   /* '<S428>/Y_TCTLGC_LdcP_radpm1' */
extern real32_T LGC_CoeffPGainLcLdc;   /* '<S432>/Y_TCTLGC_LdcP_radpm1' */

/* Proportional gain of the Y-Coordinate Controller in dependence on the vehicle's velocity */
extern real32_T LGC_CoeffPGainLdc;     /* '<S432>/Multiport Switch2' */

/* Proportional gain of the Y-Coordinate Controller */
extern real32_T LGC_CoeffPGainOfCac;   /* '<S428>/Y_TCTLGC_LdcOfP_radpm1' */
extern real32_T LGC_CoeffPGainOfLdc;   /* '<S432>/Y_TCTLGC_LdcOfP_radpm1' */

/* Lateral Deviation Controller P-Gain for object following mode */
extern real32_T LGC_CoeffPGainSfCac;   /* '<S428>/Y_TCTLGC_LdcSfP_radpm1' */
extern real32_T LGC_CoeffPGainSfLdc;   /* '<S432>/Y_TCTLGC_LdcSfP_radpm1' */

/* Proportional gain of the Y-Coordinate Controller in dependence on the vehicle's velocity */
extern real32_T LGC_CoeffPT1GainCac;   /* '<S429>/Multiport Switch3' */
extern real32_T LGC_CoeffPT1GainLcCac; /* '<S429>/Y_TCTLGC_LdcDGain_radspm' */
extern real32_T LGC_CoeffPT1GainLcLdc; /* '<S433>/Y_TCTLGC_LdcDGain_radspm' */
extern real32_T LGC_CoeffPT1GainLdc;   /* '<S433>/Multiport Switch3' */
extern real32_T LGC_CoeffPT1GainOfCac; /* '<S429>/Y_TCTLGC_LdcOfDGain_radspm' */
extern real32_T LGC_CoeffPT1GainOfLdc; /* '<S433>/Y_TCTLGC_LdcOfDGain_radspm' */
extern real32_T LGC_CoeffPT1GainSfCac; /* '<S429>/Y_TCTLGC_LdcSfDGain_radspm' */
extern real32_T LGC_CoeffPT1GainSfLdc; /* '<S433>/Y_TCTLGC_LdcSfDGain_radspm' */
extern real32_T LGC_CoeffPole1LaDmc;   /* '<S425>/1-D Lookup Table2' */

/* Pole 1 of the approximated LaDMC transfer function */
extern real32_T LGC_CoeffPole2LaDmc;   /* '<S425>/1-D Lookup Table3' */

/* Pole 2 of the approximated LaDMC transfer function */
extern real32_T LGC_CrvReqBAC_1pm;     /* '<S565>/Switch' */
extern real32_T LGC_CrvReqDte_1pm;     /* '<S81>/Data Type Conversion5' */

/*
   Required curvature by disturbance compensator
 */
extern real32_T LGC_CrvReqFfcFrz_1pm;  /* '<S570>/Switch2' */
extern real32_T LGC_CrvReqFfcGrdLimT1_1pm;/* '<S576>/Switch' */
extern real32_T LGC_CrvReqFfcGrdLimT2_1pm;/* '<S578>/Add1' */
extern real32_T LGC_CrvReqFfcGrdLim_1pm;/* '<S577>/Switch2' */
extern real32_T LGC_CtrlCrv_1pm;       /* '<S564>/Divide' */
extern real32_T LGC_CtrlCrv_DE_1pm;    /* '<S419>/Switch' */
extern real32_T LGC_CtrlErrHeadAglCrtd_rad;/* '<S81>/Data Type Conversion' */
extern real32_T LGC_CtrlErrMainPGain;  /* '<S540>/Mul4' */

/* UNDEFINED */
extern real32_T LGC_CtrlErrMainPGainCas;/* '<S458>/Mul4' */

/* UNDEFINED */
extern real32_T LGC_CtrlErrMainPGainCdc;/* '<S498>/Mul4' */

/* UNDEFINED */
extern real32_T LGC_DeltaByBnkAglComp_deg;/* '<S588>/Divide' */
extern real32_T LGC_DeltaFBAC_deg;     /* '<S81>/Data Type Conversion17' */
extern real32_T LGC_DeltaFCmdCdc;      /* '<S484>/Multiport Switch' */

/* UNDEFINED */
extern real32_T LGC_DeltaFCmdDC_deg;   /* '<S587>/Divide' */
extern real32_T LGC_DeltaFCmdFFC_deg;  /* '<S81>/Data Type Conversion10' */
extern real32_T LGC_DeltaFCmdUnlimited_deg;/* '<S586>/Divide' */
extern real32_T LGC_DeltaFCmd_deg;     /* '<S81>/Data Type Conversion3' */

/*
   Steering Angle Command for DMC
 */
extern real32_T LGC_DeltaFCmd_rad;     /* '<S439>/Sum' */
extern real32_T LGC_DeltaFDGainCas;    /* '<S446>/Multiport Switch9' */

/* UNDEFINED */
extern real32_T LGC_DeltaFDGainCdc;    /* '<S486>/Multiport Switch9' */

/* UNDEFINED */
extern real32_T LGC_DeltaFDGainLdc;    /* '<S526>/Multiport Switch9' */

/* UNDEFINED */
extern real32_T LGC_DeltaFIGainCas;    /* '<S452>/Switch2' */

/* UNDEFINED */
extern real32_T LGC_DeltaFIGainCdc;    /* '<S492>/Switch2' */

/* UNDEFINED */
extern real32_T LGC_DeltaFIGainLdc;    /* '<S532>/Switch2' */

/* UNDEFINED */
extern real32_T LGC_DeltaFPGainCas;    /* '<S458>/Mul8' */

/* UNDEFINED */
extern real32_T LGC_DeltaFPGainCdc;    /* '<S498>/Mul8' */

/* UNDEFINED */
extern real32_T LGC_DeltaFPGainLdc;    /* '<S540>/Mul8' */

/* UNDEFINED */
extern real32_T LGC_DeltaFPT1GainCas;  /* '<S448>/Multiport Switch8' */

/* UNDEFINED */
extern real32_T LGC_DeltaFPT1GainCdc;  /* '<S488>/Multiport Switch8' */

/* UNDEFINED */
extern real32_T LGC_DeltaFPT1GainLdc;  /* '<S535>/Multiport Switch1' */

/* UNDEFINED */
extern uint8_T LGC_EnaActObjFollow;    /* '<S422>/Data Type Conversion1' */

/* Enable flag for activating object following */
extern uint8_T LGC_EnaActSafetyFcn;    /* '<S422>/Data Type Conversion' */

/* Enable flag for activating safety function */
extern boolean_T LGC_EnaCntrlByTgq;    /* '<S417>/Equal1' */

/* Enalbe flag for control by trajectory guidance qualifier output */
extern boolean_T LGC_EnaFreezeByTgq;   /* '<S417>/Equal' */

/* Enalbe flag for freeze by trajectory guidance qualifier output */
extern boolean_T LGC_EnaModeChangeCas; /* '<S457>/NotEqual' */

/* UNDEFINED */
extern boolean_T LGC_EnaModeChangeCdc; /* '<S497>/NotEqual' */

/* UNDEFINED */
extern boolean_T LGC_EnaModeChangeDtct;/* '<S539>/NotEqual' */
extern boolean_T LGC_EnaPGainGrdLmtCas;/* '<S457>/OR3' */

/* UNDEFINED */
extern boolean_T LGC_EnaPGainGrdLmtCdc;/* '<S497>/OR3' */

/* UNDEFINED */
extern boolean_T LGC_EnaPGainGrdLmtLdc;/* '<S539>/OR3' */
extern int8_T LGC_EnaPGainGrdSignCas;  /* '<S457>/Multiport Switch7' */

/* UNDEFINED */
extern int8_T LGC_EnaPGainGrdSignCdc;  /* '<S497>/Multiport Switch7' */

/* UNDEFINED */
extern real32_T LGC_EnaPGainGrdSignLdc;/* '<S539>/Multiport Switch7' */
extern boolean_T LGC_EnaPGainGrdThdCas;/* '<S457>/GreaterThan' */

/* UNDEFINED */
extern boolean_T LGC_EnaPGainGrdThdCdc;/* '<S497>/GreaterThan' */

/* UNDEFINED */
extern boolean_T LGC_EnaPGainGrdThdLdc;/* '<S539>/GreaterThan' */
extern boolean_T LGC_EnaResetByTgq;    /* '<S417>/OR' */

/* Enalbe flag for reset by trajectory guidance qualifier output */
extern boolean_T LGC_EnaRstByDistY;    /* '<S435>/AND1' */
extern boolean_T LGC_EnaRstByStandStill;/* '<S435>/Less Than' */
extern boolean_T LGC_EnaRstByTrq;      /* '<S435>/GreaterThan2' */
extern boolean_T LGC_EnaRstIntCac;     /* '<S434>/OR3' */
extern boolean_T LGC_EnaRstIntLdc;     /* '<S434>/OR5' */
extern boolean_T LGC_EnaRstPT1Cac;     /* '<S434>/OR2' */
extern boolean_T LGC_EnaRstPT1Ldc;     /* '<S434>/OR4' */
extern boolean_T LGC_EnableCtrl_nu;    /* '<S81>/Data Type Conversion7' */

/*
   Enalbe flag for control by trajectory guidance qualifier output
 */
extern real32_T LGC_ErrCourse2DotCas;  /* '<S465>/Multiport Switch' */

/* UNDEFINED */
extern real32_T LGC_ErrCourse2DotCdc;  /* '<S505>/Multiport Switch' */

/* UNDEFINED */
extern real32_T LGC_ErrCourseDotCas;   /* '<S464>/Multiport Switch' */

/* UNDEFINED */
extern real32_T LGC_ErrCourseDotCdc;   /* '<S504>/Multiport Switch' */

/* UNDEFINED */
extern real32_T LGC_ErrCtrlCourseCas;  /* '<S461>/Multiport Switch5' */

/* UNDEFINED */
extern real32_T LGC_ErrCtrlCourseCdc;  /* '<S501>/Multiport Switch5' */

/* UNDEFINED */
extern real32_T LGC_ErrCtrlDistY;      /* '<S543>/Multiport Switch5' */

/* UNDEFINED */
extern real32_T LGC_ErrDistY2Dot;      /* '<S547>/Multiport Switch' */

/* UNDEFINED */
extern real32_T LGC_ErrDistYDot;       /* '<S546>/Multiport Switch' */

/* UNDEFINED */
extern real32_T LGC_FFCrv_1pm;         /* '<S568>/Switch4' */
extern real32_T LGC_FltErrCourseCas;   /* '<S463>/Multiport Switch4' */

/* UNDEFINED */
extern real32_T LGC_FltErrCourseCdc;   /* '<S503>/Multiport Switch4' */

/* UNDEFINED */
extern real32_T LGC_FltErrCourseDotCas;/* '<S468>/Add' */

/* UNDEFINED */
extern real32_T LGC_FltErrCourseDotCdc;/* '<S508>/Add' */

/* UNDEFINED */
extern real32_T LGC_FltPT1YErr_met;    /* '<S545>/Multiport Switch4' */

/* UNDEFINED */
extern real32_T LGC_FltRawErrDistYDot; /* '<S550>/Add' */

/* UNDEFINED */
extern real32_T LGC_HldReqDeltaF;      /* '<S590>/Multiport Switch2' */

/* Steering Angle Command for DMC */
extern boolean_T LGC_Hold_nu;          /* '<S81>/Data Type Conversion6' */

/*
   Enalbe flag for freeze by trajectory guidance qualifier output
 */
extern real32_T LGC_LdcAloneICmd_rad;  /* '<S524>/Multiport Switch' */
extern real32_T LGC_LdcCmd_rad;        /* '<S444>/Multiport Switch' */
extern boolean_T LGC_LdcIntReset_nu;   /* '<S81>/Data Type Conversion15' */
extern boolean_T LGC_LdcPT1Reset_nu;   /* '<S81>/Data Type Conversion13' */
extern real32_T LGC_LmtCoeffPGainCas;  /* '<S458>/Multiport Switch5' */

/* Proportional gain of the Y-Coordinate Controller */
extern real32_T LGC_LmtCoeffPGainCdc;  /* '<S498>/Multiport Switch5' */

/* Proportional gain of the Y-Coordinate Controller */
extern real32_T LGC_LmtCoeffPGainLdc;  /* '<S540>/Multiport Switch5' */

/* Proportional gain of the Y-Coordinate Controller */
extern real32_T LGC_LmtReqDeltaF;      /* '<S592>/Switch2' */

/* Steering Angle Command for DMC */
extern real32_T LGC_LmtSelReqDeltaF;   /* '<S589>/Multiport Switch3' */

/* Steering Angle Command for DMC */
extern real32_T LGC_MaxReqDeltaF;      /* '<S589>/Product' */

/* Steering Angle Command for DMC */
extern real32_T LGC_RawCmpn2DotLaDmcCas;/* '<S466>/Sum2' */
extern real32_T LGC_RawCmpn2DotLaDmcCdc;/* '<S506>/Sum2' */
extern real32_T LGC_RawCmpn2DotLaDmcLdc;/* '<S548>/Sum2' */
extern real32_T LGC_RawErrCourseDotCas;/* '<S464>/Divide' */

/* UNDEFINED */
extern real32_T LGC_RawErrCourseDotCdc;/* '<S504>/Divide' */

/* UNDEFINED */
extern real32_T LGC_RawErrDistYDot;    /* '<S546>/Divide' */

/* UNDEFINED */
extern real32_T LGC_RawFfcDeltaF;      /* '<S574>/Divide1' */

/* UNDEFINED */
extern real32_T LGC_RawFltErrCourseCas;/* '<S478>/Add' */

/* UNDEFINED */
extern real32_T LGC_RawFltErrCourseCdc;/* '<S518>/Add' */

/* UNDEFINED */
extern real32_T LGC_RawFltErrCtrlDistY;/* '<S560>/Add' */

/* UNDEFINED */
extern real32_T LGC_RawLmtCoeffPGainCas;/* '<S459>/Multiport Switch1' */

/* Proportional gain of the Y-Coordinate Controller */
extern real32_T LGC_RawLmtCoeffPGainCdc;/* '<S499>/Multiport Switch1' */

/* Proportional gain of the Y-Coordinate Controller */
extern real32_T LGC_RawLmtCoeffPGainLdc;/* '<S541>/Multiport Switch1' */

/* Proportional gain of the Y-Coordinate Controller */
extern real32_T LGC_ReqDeltaF;         /* '<S590>/Multiport Switch' */

/* Steering Angle Command for DMC */
extern boolean_T LGC_Reset_nu;         /* '<S81>/Data Type Conversion9' */

/*
   Enalbe flag for reset by trajectory guidance qualifier output
 */
extern uint8_T LGC_SafetyFunctionActive_nu;/* '<S81>/Data Type Conversion11' */

/*
   Enable flag for activating safety function
 */
extern uint8_T LGC_StActParSet;        /* '<S422>/Data Type Conversion2' */

/* Active LGC Paramter Set:
   1: Lane Centering
   2: Object Following
   3: Safety Function */
extern real32_T LGC_SumCrvReqFbFrz_1pm;/* '<S569>/Switch2' */
extern real32_T LGC_SumCrvReqFbGrdLim_1pm;/* '<S580>/Switch2' */
extern real32_T LGC_SumCrvReqFbSatLim_1pm;/* '<S582>/Switch1' */
extern real32_T LGC_SumCrvReqFb_1pm;   /* '<S563>/Sum2' */
extern real32_T LGC_TgtCrv_DENoLatSlp_1pm;/* '<S81>/Constant' */
extern real32_T LGC_TgtCrv_DE_1pm;     /* '<S567>/Switch4' */
extern real32_T LGC_TgtCrv_NoDE_1pm;   /* '<S81>/Constant1' */
extern real32_T LGC_TimeDT1Cac;        /* '<S426>/Multiport Switch4' */
extern real32_T LGC_TimeDT1LcCac;      /* '<S426>/Y_TCTLGC_LdcDT1_sec' */
extern real32_T LGC_TimeDT1LcLdc;      /* '<S430>/Y_TCTLGC_LdcDT1_sec' */
extern real32_T LGC_TimeDT1Ldc;        /* '<S430>/Multiport Switch4' */
extern real32_T LGC_TimeDT1OfCac;      /* '<S426>/Y_TCTLGC_LdcOfDT1_sec' */
extern real32_T LGC_TimeDT1OfLdc;      /* '<S430>/Y_TCTLGC_LdcOfDT1_sec' */
extern real32_T LGC_TimeDT1SfCac;      /* '<S426>/Y_TCTLGC_LdcSfDT1_sec' */
extern real32_T LGC_TimeDT1SfLdc;      /* '<S430>/Y_TCTLGC_LdcSfDT1_sec' */
extern real32_T LGC_TimeFltErrCourse;  /* '<S463>/Y_TCTLGC_PT1YErrTime_sec' */
extern real32_T LGC_TimeFltErrCourseCdc;/* '<S503>/Y_TCTLGC_PT1YErrTime_sec' */
extern real32_T LGC_TimeFltErrDistY;   /* '<S545>/Y_TCTLGC_PT1YErrTime_sec' */
extern real32_T LGC_TimePT1Cac;        /* '<S429>/Multiport Switch4' */
extern real32_T LGC_TimePT1DeltaFCmd;
                                    /* '<S574>/Y_TCTLGC_PT1DeltaFCmdTime_sec' */
extern real32_T LGC_TimePT1LcCac;      /* '<S429>/Y_TCTLGC_LdcDT1_sec' */
extern real32_T LGC_TimePT1LcLdc;      /* '<S433>/Y_TCTLGC_LdcDT1_sec' */
extern real32_T LGC_TimePT1Ldc;        /* '<S433>/Multiport Switch4' */
extern real32_T LGC_TimePT1OfCac;      /* '<S429>/Y_TCTLGC_LdcOfDT1_sec' */
extern real32_T LGC_TimePT1OfLdc;      /* '<S433>/Y_TCTLGC_LdcOfDT1_sec' */
extern real32_T LGC_TimePT1SfCac;      /* '<S429>/Y_TCTLGC_LdcSfDT1_sec' */
extern real32_T LGC_TimePT1SfLdc;      /* '<S433>/Y_TCTLGC_LdcSfDT1_sec' */
extern real32_T LQR_DeltaF_Cmd_rad;    /* '<S82>/Add' */
extern real32_T LQR_DeltaF_Lead_Cmd_rad;/* '<S601>/MATLAB Function' */
extern real32_T LQR_DeltaF_feedback_rad;/* '<S595>/Add' */
extern real32_T LQR_DeltaF_feedforward_rad;/* '<S82>/Product' */
extern boolean_T LQR_EnaCntrlByTgq;    /* '<S597>/Equal1' */
extern boolean_T LQR_EnaFreezeByTgq;   /* '<S597>/Equal' */
extern boolean_T LQR_EnaResetByTgq;    /* '<S597>/OR' */
extern real32_T LQR_FltDeltaF_Cmd_rad; /* '<S603>/Switch' */
extern real32_T LQR_I_term_rad;        /* '<S614>/Switch2' */
extern real32_T LQR_KappaFlt;          /* '<S599>/Switch' */
extern real32_T LQR_MatK_k1;           /* '<S607>/1-D Lookup Table4' */
extern real32_T LQR_MatK_k2;           /* '<S607>/1-D Lookup Table1' */
extern real32_T LQR_MatK_k3;           /* '<S607>/1-D Lookup Table2' */
extern real32_T LQR_MatK_k4;           /* '<S607>/1-D Lookup Table3' */
extern real32_T LQR_ReqDeltaF_Limit_deg;/* '<S628>/Multiport Switch' */
extern real32_T LQR_YawrateFlt;        /* '<S632>/Switch' */
extern real32_T LQR_e1_contribution;   /* '<S595>/Product' */
extern real32_T LQR_e1dot_contribution;/* '<S595>/Product1' */
extern real32_T LQR_e2_contribution;   /* '<S595>/Product2' */
extern real32_T LQR_e2dot_contribution;/* '<S595>/Product3' */
extern real32_T LQR_heading_error;     /* '<S73>/Data Type Conversion12' */
extern real32_T LQR_heading_error_rate;/* '<S604>/Subtract' */
extern real32_T LQR_lateral_error;     /* '<S73>/Data Type Conversion11' */
extern real32_T LQR_lateral_error_rate;/* '<S604>/Switch' */
extern real32_T LQR_num_iteration;     /* '<S608>/Data Type Conversion3' */
extern real32_T LQR_ref_heading_rate;  /* '<S604>/Product4' */
extern real32_T LQR_yawrate_term;      /* '<S73>/Data Type Conversion15' */
extern real32_T TCTI_ActualTrqEPS;     /* '<Root>/TCTI_ActualTrqEPS' */
extern real32_T TCTI_NegReqTrajHeadTpl;/* '<S92>/Neg4' */

/* negative S_TPLFBT_CurHeading_rad */
extern real32_T TCTI_RoadBankAngle;    /* '<Root>/TCTI_RoadBankAngle' */
extern boolean_T TC_EnaUnplauUnitDelay_bool;/* '<S102>/Unit Delay' */
extern boolean_T TC_Freeze1RSFlipFlop_bool;/* '<S111>/Unit Delay' */
extern boolean_T TC_Freeze2RSFlipFlop_bool;/* '<S107>/Unit Delay' */
extern boolean_T TC_FreezeRSFlipFlop_bool;/* '<S110>/Unit Delay' */
extern boolean_T TC_HoldWarnRSFlipFlop_bool;/* '<S130>/Unit Delay' */
extern real32_T deadZone_gainkT;       /* '<S595>/MATLAB Function' */
extern real32_T deadZone_weightedError;/* '<S595>/MATLAB Function' */

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S90>/Data Type Duplicate' : Unused code path elimination
 * Block '<S90>/Data Type Propagation' : Unused code path elimination
 * Block '<S95>/Data Type Duplicate' : Unused code path elimination
 * Block '<S95>/Data Type Propagation' : Unused code path elimination
 * Block '<S115>/Constant1' : Unused code path elimination
 * Block '<S115>/Equal' : Unused code path elimination
 * Block '<S118>/Data Type Duplicate' : Unused code path elimination
 * Block '<S118>/Data Type Propagation' : Unused code path elimination
 * Block '<S119>/Data Type Duplicate' : Unused code path elimination
 * Block '<S119>/Data Type Propagation' : Unused code path elimination
 * Block '<S123>/Data Type Duplicate' : Unused code path elimination
 * Block '<S123>/Data Type Propagation' : Unused code path elimination
 * Block '<S124>/Data Type Duplicate' : Unused code path elimination
 * Block '<S124>/Data Type Propagation' : Unused code path elimination
 * Block '<S128>/Data Type Duplicate' : Unused code path elimination
 * Block '<S128>/Data Type Propagation' : Unused code path elimination
 * Block '<S129>/Data Type Duplicate' : Unused code path elimination
 * Block '<S129>/Data Type Propagation' : Unused code path elimination
 * Block '<S155>/Data Type Duplicate' : Unused code path elimination
 * Block '<S155>/Data Type Propagation' : Unused code path elimination
 * Block '<S162>/Data Type Duplicate' : Unused code path elimination
 * Block '<S162>/Data Type Propagation' : Unused code path elimination
 * Block '<S163>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S164>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S165>/Data Type Duplicate' : Unused code path elimination
 * Block '<S165>/Data Type Propagation' : Unused code path elimination
 * Block '<S166>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S167>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S168>/Data Type Duplicate' : Unused code path elimination
 * Block '<S168>/Data Type Propagation' : Unused code path elimination
 * Block '<S169>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S170>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S178>/Data Type Duplicate' : Unused code path elimination
 * Block '<S178>/Data Type Propagation' : Unused code path elimination
 * Block '<S185>/Data Type Duplicate' : Unused code path elimination
 * Block '<S185>/Data Type Propagation' : Unused code path elimination
 * Block '<S186>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S187>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S188>/Data Type Duplicate' : Unused code path elimination
 * Block '<S188>/Data Type Propagation' : Unused code path elimination
 * Block '<S189>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S190>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S191>/Data Type Duplicate' : Unused code path elimination
 * Block '<S191>/Data Type Propagation' : Unused code path elimination
 * Block '<S192>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S193>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S209>/Data Type Duplicate' : Unused code path elimination
 * Block '<S209>/Data Type Propagation' : Unused code path elimination
 * Block '<S211>/Data Type Duplicate' : Unused code path elimination
 * Block '<S211>/Data Type Propagation' : Unused code path elimination
 * Block '<S214>/Data Type Duplicate' : Unused code path elimination
 * Block '<S214>/Data Type Propagation' : Unused code path elimination
 * Block '<S215>/Data Type Duplicate' : Unused code path elimination
 * Block '<S215>/Data Type Propagation' : Unused code path elimination
 * Block '<S217>/Data Type Duplicate' : Unused code path elimination
 * Block '<S217>/Data Type Propagation' : Unused code path elimination
 * Block '<S232>/Data Type Duplicate' : Unused code path elimination
 * Block '<S232>/Data Type Propagation' : Unused code path elimination
 * Block '<S233>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S234>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S235>/Data Type Duplicate' : Unused code path elimination
 * Block '<S235>/Data Type Propagation' : Unused code path elimination
 * Block '<S236>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S237>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S240>/Data Type Duplicate' : Unused code path elimination
 * Block '<S240>/Data Type Propagation' : Unused code path elimination
 * Block '<S251>/Data Type Duplicate' : Unused code path elimination
 * Block '<S251>/Data Type Propagation' : Unused code path elimination
 * Block '<S257>/Data Type Duplicate' : Unused code path elimination
 * Block '<S257>/Data Type Propagation' : Unused code path elimination
 * Block '<S258>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S259>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S260>/Data Type Duplicate' : Unused code path elimination
 * Block '<S260>/Data Type Propagation' : Unused code path elimination
 * Block '<S261>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S262>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S263>/Data Type Duplicate' : Unused code path elimination
 * Block '<S263>/Data Type Propagation' : Unused code path elimination
 * Block '<S264>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S265>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S268>/Data Type Duplicate' : Unused code path elimination
 * Block '<S268>/Data Type Propagation' : Unused code path elimination
 * Block '<S283>/Data Type Duplicate' : Unused code path elimination
 * Block '<S283>/Data Type Propagation' : Unused code path elimination
 * Block '<S288>/Data Type Duplicate' : Unused code path elimination
 * Block '<S288>/Data Type Propagation' : Unused code path elimination
 * Block '<S289>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S290>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S291>/Data Type Duplicate' : Unused code path elimination
 * Block '<S291>/Data Type Propagation' : Unused code path elimination
 * Block '<S292>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S293>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S322>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S323>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S324>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S325>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S326>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S327>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S328>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S329>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S318>/Data Type Duplicate' : Unused code path elimination
 * Block '<S318>/Data Type Propagation' : Unused code path elimination
 * Block '<S319>/Data Type Duplicate' : Unused code path elimination
 * Block '<S319>/Data Type Propagation' : Unused code path elimination
 * Block '<S320>/Data Type Duplicate' : Unused code path elimination
 * Block '<S320>/Data Type Propagation' : Unused code path elimination
 * Block '<S321>/Data Type Duplicate' : Unused code path elimination
 * Block '<S321>/Data Type Propagation' : Unused code path elimination
 * Block '<S337>/Data Type Duplicate' : Unused code path elimination
 * Block '<S337>/Data Type Propagation' : Unused code path elimination
 * Block '<S339>/Data Type Duplicate' : Unused code path elimination
 * Block '<S339>/Data Type Propagation' : Unused code path elimination
 * Block '<S342>/Data Type Duplicate' : Unused code path elimination
 * Block '<S342>/Data Type Propagation' : Unused code path elimination
 * Block '<S344>/Data Type Duplicate' : Unused code path elimination
 * Block '<S344>/Data Type Propagation' : Unused code path elimination
 * Block '<S360>/Data Type Duplicate' : Unused code path elimination
 * Block '<S360>/Data Type Propagation' : Unused code path elimination
 * Block '<S362>/Data Type Duplicate' : Unused code path elimination
 * Block '<S362>/Data Type Propagation' : Unused code path elimination
 * Block '<S364>/Data Type Duplicate' : Unused code path elimination
 * Block '<S364>/Data Type Propagation' : Unused code path elimination
 * Block '<S365>/Data Type Duplicate' : Unused code path elimination
 * Block '<S365>/Data Type Propagation' : Unused code path elimination
 * Block '<S370>/Data Type Duplicate' : Unused code path elimination
 * Block '<S370>/Data Type Propagation' : Unused code path elimination
 * Block '<S371>/Data Type Duplicate' : Unused code path elimination
 * Block '<S371>/Data Type Propagation' : Unused code path elimination
 * Block '<S379>/Data Type Duplicate' : Unused code path elimination
 * Block '<S379>/Data Type Propagation' : Unused code path elimination
 * Block '<S380>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S381>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S382>/Data Type Duplicate' : Unused code path elimination
 * Block '<S382>/Data Type Propagation' : Unused code path elimination
 * Block '<S383>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S384>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S388>/Data Type Duplicate' : Unused code path elimination
 * Block '<S388>/Data Type Propagation' : Unused code path elimination
 * Block '<S393>/Data Type Duplicate' : Unused code path elimination
 * Block '<S393>/Data Type Propagation' : Unused code path elimination
 * Block '<S399>/Data Type Duplicate' : Unused code path elimination
 * Block '<S399>/Data Type Propagation' : Unused code path elimination
 * Block '<S401>/Data Type Duplicate' : Unused code path elimination
 * Block '<S401>/Data Type Propagation' : Unused code path elimination
 * Block '<S450>/Data Type Duplicate' : Unused code path elimination
 * Block '<S450>/Data Type Propagation' : Unused code path elimination
 * Block '<S452>/Data Type Duplicate' : Unused code path elimination
 * Block '<S452>/Data Type Propagation' : Unused code path elimination
 * Block '<S453>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S454>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S456>/Data Type Duplicate' : Unused code path elimination
 * Block '<S456>/Data Type Propagation' : Unused code path elimination
 * Block '<S460>/Data Type Duplicate' : Unused code path elimination
 * Block '<S460>/Data Type Propagation' : Unused code path elimination
 * Block '<S469>/Data Type Duplicate' : Unused code path elimination
 * Block '<S469>/Data Type Propagation' : Unused code path elimination
 * Block '<S472>/Data Type Duplicate' : Unused code path elimination
 * Block '<S472>/Data Type Propagation' : Unused code path elimination
 * Block '<S473>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S474>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S475>/Data Type Duplicate' : Unused code path elimination
 * Block '<S475>/Data Type Propagation' : Unused code path elimination
 * Block '<S476>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S477>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S479>/Data Type Duplicate' : Unused code path elimination
 * Block '<S479>/Data Type Propagation' : Unused code path elimination
 * Block '<S484>/Gain' : Unused code path elimination
 * Block '<S490>/Data Type Duplicate' : Unused code path elimination
 * Block '<S490>/Data Type Propagation' : Unused code path elimination
 * Block '<S492>/Data Type Duplicate' : Unused code path elimination
 * Block '<S492>/Data Type Propagation' : Unused code path elimination
 * Block '<S493>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S494>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S496>/Data Type Duplicate' : Unused code path elimination
 * Block '<S496>/Data Type Propagation' : Unused code path elimination
 * Block '<S500>/Data Type Duplicate' : Unused code path elimination
 * Block '<S500>/Data Type Propagation' : Unused code path elimination
 * Block '<S509>/Data Type Duplicate' : Unused code path elimination
 * Block '<S509>/Data Type Propagation' : Unused code path elimination
 * Block '<S512>/Data Type Duplicate' : Unused code path elimination
 * Block '<S512>/Data Type Propagation' : Unused code path elimination
 * Block '<S513>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S514>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S515>/Data Type Duplicate' : Unused code path elimination
 * Block '<S515>/Data Type Propagation' : Unused code path elimination
 * Block '<S516>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S517>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S519>/Data Type Duplicate' : Unused code path elimination
 * Block '<S519>/Data Type Propagation' : Unused code path elimination
 * Block '<S530>/Data Type Duplicate' : Unused code path elimination
 * Block '<S530>/Data Type Propagation' : Unused code path elimination
 * Block '<S532>/Data Type Duplicate' : Unused code path elimination
 * Block '<S532>/Data Type Propagation' : Unused code path elimination
 * Block '<S533>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S534>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S537>/Data Type Duplicate' : Unused code path elimination
 * Block '<S537>/Data Type Propagation' : Unused code path elimination
 * Block '<S538>/Data Type Duplicate' : Unused code path elimination
 * Block '<S538>/Data Type Propagation' : Unused code path elimination
 * Block '<S542>/Data Type Duplicate' : Unused code path elimination
 * Block '<S542>/Data Type Propagation' : Unused code path elimination
 * Block '<S551>/Data Type Duplicate' : Unused code path elimination
 * Block '<S551>/Data Type Propagation' : Unused code path elimination
 * Block '<S554>/Data Type Duplicate' : Unused code path elimination
 * Block '<S554>/Data Type Propagation' : Unused code path elimination
 * Block '<S555>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S556>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S557>/Data Type Duplicate' : Unused code path elimination
 * Block '<S557>/Data Type Propagation' : Unused code path elimination
 * Block '<S558>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S559>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S561>/Data Type Duplicate' : Unused code path elimination
 * Block '<S561>/Data Type Propagation' : Unused code path elimination
 * Block '<S579>/Data Type Duplicate' : Unused code path elimination
 * Block '<S579>/Data Type Propagation' : Unused code path elimination
 * Block '<S581>/Data Type Duplicate' : Unused code path elimination
 * Block '<S581>/Data Type Propagation' : Unused code path elimination
 * Block '<S591>/Data Type Duplicate' : Unused code path elimination
 * Block '<S591>/Data Type Propagation' : Unused code path elimination
 * Block '<S592>/Data Type Duplicate' : Unused code path elimination
 * Block '<S592>/Data Type Propagation' : Unused code path elimination
 * Block '<S593>/Data Type Duplicate' : Unused code path elimination
 * Block '<S593>/Data Type Propagation' : Unused code path elimination
 * Block '<S612>/Data Type Duplicate' : Unused code path elimination
 * Block '<S612>/Data Type Propagation' : Unused code path elimination
 * Block '<S611>/Data Type Duplicate' : Unused code path elimination
 * Block '<S611>/Data Type Propagation' : Unused code path elimination
 * Block '<S614>/Data Type Duplicate' : Unused code path elimination
 * Block '<S614>/Data Type Propagation' : Unused code path elimination
 * Block '<S615>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S616>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S595>/MatrixMultiply' : Unused code path elimination
 * Block '<S625>/Data Type Duplicate' : Unused code path elimination
 * Block '<S625>/Data Type Propagation' : Unused code path elimination
 * Block '<S629>/Data Type Duplicate' : Unused code path elimination
 * Block '<S629>/Data Type Propagation' : Unused code path elimination
 * Block '<S630>/Data Type Duplicate' : Unused code path elimination
 * Block '<S630>/Data Type Propagation' : Unused code path elimination
 * Block '<S633>/Data Type Duplicate' : Unused code path elimination
 * Block '<S633>/Data Type Propagation' : Unused code path elimination
 * Block '<S73>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S73>/Data Type Conversion13' : Eliminate redundant data type conversion
 * Block '<S73>/Data Type Conversion14' : Eliminate redundant data type conversion
 * Block '<S73>/Data Type Conversion2' : Eliminate redundant data type conversion
 * Block '<S73>/Data Type Conversion3' : Eliminate redundant data type conversion
 * Block '<S73>/Data Type Conversion4' : Eliminate redundant data type conversion
 * Block '<S74>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S74>/Data Type Conversion2' : Eliminate redundant data type conversion
 * Block '<S74>/Data Type Conversion3' : Eliminate redundant data type conversion
 * Block '<S74>/Data Type Conversion4' : Eliminate redundant data type conversion
 * Block '<S74>/Data Type Conversion5' : Eliminate redundant data type conversion
 * Block '<S74>/Data Type Conversion6' : Eliminate redundant data type conversion
 * Block '<S89>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S94>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S106>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S108>/Signal Copy' : Eliminate redundant signal conversion block
 * Block '<S109>/Signal Copy' : Eliminate redundant signal conversion block
 * Block '<S117>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S122>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S127>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S154>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S159>/Signal Copy' : Eliminate redundant signal conversion block
 * Block '<S160>/Signal Copy' : Eliminate redundant signal conversion block
 * Block '<S161>/Signal Copy' : Eliminate redundant signal conversion block
 * Block '<S177>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S182>/Signal Copy' : Eliminate redundant signal conversion block
 * Block '<S183>/Signal Copy' : Eliminate redundant signal conversion block
 * Block '<S184>/Signal Copy' : Eliminate redundant signal conversion block
 * Block '<S208>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S210>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S230>/Signal Copy' : Eliminate redundant signal conversion block
 * Block '<S231>/Signal Copy' : Eliminate redundant signal conversion block
 * Block '<S250>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S254>/Signal Copy' : Eliminate redundant signal conversion block
 * Block '<S255>/Signal Copy' : Eliminate redundant signal conversion block
 * Block '<S256>/Signal Copy' : Eliminate redundant signal conversion block
 * Block '<S282>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S286>/Signal Copy' : Eliminate redundant signal conversion block
 * Block '<S287>/Signal Copy' : Eliminate redundant signal conversion block
 * Block '<S314>/Signal Copy' : Eliminate redundant signal conversion block
 * Block '<S315>/Signal Copy' : Eliminate redundant signal conversion block
 * Block '<S316>/Signal Copy' : Eliminate redundant signal conversion block
 * Block '<S317>/Signal Copy' : Eliminate redundant signal conversion block
 * Block '<S338>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S343>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S359>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S361>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S363>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S369>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S377>/Signal Copy' : Eliminate redundant signal conversion block
 * Block '<S378>/Signal Copy' : Eliminate redundant signal conversion block
 * Block '<S392>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S398>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S400>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S449>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S451>/Signal Copy' : Eliminate redundant signal conversion block
 * Block '<S455>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S459>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S468>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S470>/Signal Copy' : Eliminate redundant signal conversion block
 * Block '<S471>/Signal Copy' : Eliminate redundant signal conversion block
 * Block '<S478>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S489>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S491>/Signal Copy' : Eliminate redundant signal conversion block
 * Block '<S495>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S499>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S508>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S510>/Signal Copy' : Eliminate redundant signal conversion block
 * Block '<S511>/Signal Copy' : Eliminate redundant signal conversion block
 * Block '<S518>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S529>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S531>/Signal Copy' : Eliminate redundant signal conversion block
 * Block '<S535>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S536>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S541>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S550>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S552>/Signal Copy' : Eliminate redundant signal conversion block
 * Block '<S553>/Signal Copy' : Eliminate redundant signal conversion block
 * Block '<S560>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S439>/Gain2' : Eliminated nontunable gain of 1
 * Block '<S81>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S81>/Data Type Conversion2' : Eliminate redundant data type conversion
 * Block '<S419>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S419>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S585>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S575>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S610>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S613>/Signal Copy' : Eliminate redundant signal conversion block
 * Block '<S595>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S595>/Signal Conversion1' : Eliminate redundant signal conversion block
 * Block '<S595>/Signal Conversion2' : Eliminate redundant signal conversion block
 * Block '<S595>/Signal Conversion3' : Eliminate redundant signal conversion block
 * Block '<S608>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S608>/Data Type Conversion2' : Eliminate redundant data type conversion
 * Block '<S599>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S632>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S157>/Constant10' : Unused code path elimination
 * Block '<S157>/Constant13' : Unused code path elimination
 * Block '<S157>/Constant6' : Unused code path elimination
 * Block '<S180>/Constant10' : Unused code path elimination
 * Block '<S180>/Constant13' : Unused code path elimination
 * Block '<S180>/Constant6' : Unused code path elimination
 * Block '<S225>/Constant10' : Unused code path elimination
 * Block '<S225>/Constant13' : Unused code path elimination
 * Block '<S252>/Constant10' : Unused code path elimination
 * Block '<S252>/Constant13' : Unused code path elimination
 * Block '<S252>/Constant6' : Unused code path elimination
 * Block '<S285>/Constant10' : Unused code path elimination
 * Block '<S285>/Constant13' : Unused code path elimination
 * Block '<S313>/Constant10' : Unused code path elimination
 * Block '<S313>/Constant12' : Unused code path elimination
 * Block '<S313>/Constant16' : Unused code path elimination
 * Block '<S313>/Constant6' : Unused code path elimination
 * Block '<S376>/Constant' : Unused code path elimination
 * Block '<S376>/Constant2' : Unused code path elimination
 * Block '<S467>/Constant' : Unused code path elimination
 * Block '<S467>/Constant3' : Unused code path elimination
 * Block '<S507>/Constant' : Unused code path elimination
 * Block '<S507>/Constant3' : Unused code path elimination
 * Block '<S549>/Constant' : Unused code path elimination
 * Block '<S549>/Constant3' : Unused code path elimination
 * Block '<S604>/Constant1' : Unused code path elimination
 * Block '<S604>/Product5' : Unused code path elimination
 * Block '<S595>/Constant' : Unused code path elimination
 * Block '<S596>/USE_TABLE' : Unused code path elimination
 * Block '<S596>/USE_TABLE1' : Unused code path elimination
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
 * hilite_system('TJATCT_model/TJATCT')    - opens subsystem TJATCT_model/TJATCT
 * hilite_system('TJATCT_model/TJATCT/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'TJATCT_model'
 * '<S73>'  : 'TJATCT_model/TJATCT'
 * '<S74>'  : 'TJATCT_model/TJATCT/Subsystem'
 * '<S75>'  : 'TJATCT_model/TJATCT/TCTCDC'
 * '<S76>'  : 'TJATCT_model/TJATCT/TCTCLM'
 * '<S77>'  : 'TJATCT_model/TJATCT/TCTDEV'
 * '<S78>'  : 'TJATCT_model/TJATCT/TCTDTE'
 * '<S79>'  : 'TJATCT_model/TJATCT/TCTEST'
 * '<S80>'  : 'TJATCT_model/TJATCT/TCTFFC'
 * '<S81>'  : 'TJATCT_model/TJATCT/TCTLGC'
 * '<S82>'  : 'TJATCT_model/TJATCT/TCTLQR'
 * '<S83>'  : 'TJATCT_model/TJATCT/TCTCDC/ControlDeviationDistY'
 * '<S84>'  : 'TJATCT_model/TJATCT/TCTCDC/ControlDeviationYawAngle'
 * '<S85>'  : 'TJATCT_model/TJATCT/TCTCDC/DetermineEnableCondition'
 * '<S86>'  : 'TJATCT_model/TJATCT/TCTCDC/Watchdog'
 * '<S87>'  : 'TJATCT_model/TJATCT/TCTCDC/ControlDeviationDistY/CtrlErrDistY'
 * '<S88>'  : 'TJATCT_model/TJATCT/TCTCDC/ControlDeviationDistY/ErrDistYTpl'
 * '<S89>'  : 'TJATCT_model/TJATCT/TCTCDC/ControlDeviationDistY/ErrDistYTpl/LowPassFilter2'
 * '<S90>'  : 'TJATCT_model/TJATCT/TCTCDC/ControlDeviationDistY/ErrDistYTpl/LowPassFilter2/Saturation Dynamic1'
 * '<S91>'  : 'TJATCT_model/TJATCT/TCTCDC/ControlDeviationYawAngle/CtrlErrHeadingAngle'
 * '<S92>'  : 'TJATCT_model/TJATCT/TCTCDC/ControlDeviationYawAngle/DeltaPsiEstimation'
 * '<S93>'  : 'TJATCT_model/TJATCT/TCTCDC/ControlDeviationYawAngle/FilterForHeading'
 * '<S94>'  : 'TJATCT_model/TJATCT/TCTCDC/ControlDeviationYawAngle/FilterForHeading/LowPassFilter2'
 * '<S95>'  : 'TJATCT_model/TJATCT/TCTCDC/ControlDeviationYawAngle/FilterForHeading/LowPassFilter2/Saturation Dynamic1'
 * '<S96>'  : 'TJATCT_model/TJATCT/TCTCDC/DetermineEnableCondition/Detect Decrease1'
 * '<S97>'  : 'TJATCT_model/TJATCT/TCTCLM/GradientLimit'
 * '<S98>'  : 'TJATCT_model/TJATCT/TCTCLM/PlausibilityCheck'
 * '<S99>'  : 'TJATCT_model/TJATCT/TCTCLM/GradientLimit/CalculateQualifier'
 * '<S100>' : 'TJATCT_model/TJATCT/TCTCLM/GradientLimit/FreezeAndWatchdog'
 * '<S101>' : 'TJATCT_model/TJATCT/TCTCLM/GradientLimit/GradientLimiterAndSaturation'
 * '<S102>' : 'TJATCT_model/TJATCT/TCTCLM/GradientLimit/HoldWarning'
 * '<S103>' : 'TJATCT_model/TJATCT/TCTCLM/GradientLimit/FreezeAndWatchdog/FreezeControlSignals'
 * '<S104>' : 'TJATCT_model/TJATCT/TCTCLM/GradientLimit/FreezeAndWatchdog/WatchdogAndMergeQualifierSignals'
 * '<S105>' : 'TJATCT_model/TJATCT/TCTCLM/GradientLimit/FreezeAndWatchdog/WatchdogAndMergeQualifierSignals/MergeQualifierSignals'
 * '<S106>' : 'TJATCT_model/TJATCT/TCTCLM/GradientLimit/FreezeAndWatchdog/WatchdogAndMergeQualifierSignals/Watchdog'
 * '<S107>' : 'TJATCT_model/TJATCT/TCTCLM/GradientLimit/FreezeAndWatchdog/WatchdogAndMergeQualifierSignals/Watchdog/RSFlipFlop'
 * '<S108>' : 'TJATCT_model/TJATCT/TCTCLM/GradientLimit/FreezeAndWatchdog/WatchdogAndMergeQualifierSignals/Watchdog/TurnOffDelay'
 * '<S109>' : 'TJATCT_model/TJATCT/TCTCLM/GradientLimit/FreezeAndWatchdog/WatchdogAndMergeQualifierSignals/Watchdog/TurnOffDelay1'
 * '<S110>' : 'TJATCT_model/TJATCT/TCTCLM/GradientLimit/FreezeAndWatchdog/WatchdogAndMergeQualifierSignals/Watchdog/TurnOffDelay/RSFlipFlop'
 * '<S111>' : 'TJATCT_model/TJATCT/TCTCLM/GradientLimit/FreezeAndWatchdog/WatchdogAndMergeQualifierSignals/Watchdog/TurnOffDelay1/RSFlipFlop'
 * '<S112>' : 'TJATCT_model/TJATCT/TCTCLM/GradientLimit/GradientLimiterAndSaturation/DeltaF'
 * '<S113>' : 'TJATCT_model/TJATCT/TCTCLM/GradientLimit/GradientLimiterAndSaturation/FbcDc'
 * '<S114>' : 'TJATCT_model/TJATCT/TCTCLM/GradientLimit/GradientLimiterAndSaturation/FfcCrv'
 * '<S115>' : 'TJATCT_model/TJATCT/TCTCLM/GradientLimit/GradientLimiterAndSaturation/DeltaF/DeltaFCmdGradientLimitation'
 * '<S116>' : 'TJATCT_model/TJATCT/TCTCLM/GradientLimit/GradientLimiterAndSaturation/DeltaF/DeltaFCmdSaturationLimit'
 * '<S117>' : 'TJATCT_model/TJATCT/TCTCLM/GradientLimit/GradientLimiterAndSaturation/DeltaF/DeltaFCmdGradientLimitation/GradientLimiter'
 * '<S118>' : 'TJATCT_model/TJATCT/TCTCLM/GradientLimit/GradientLimiterAndSaturation/DeltaF/DeltaFCmdGradientLimitation/GradientLimiter/Saturation Dynamic'
 * '<S119>' : 'TJATCT_model/TJATCT/TCTCLM/GradientLimit/GradientLimiterAndSaturation/DeltaF/DeltaFCmdSaturationLimit/Saturation Dynamic'
 * '<S120>' : 'TJATCT_model/TJATCT/TCTCLM/GradientLimit/GradientLimiterAndSaturation/FbcDc/FbcDcGradientLimitation'
 * '<S121>' : 'TJATCT_model/TJATCT/TCTCLM/GradientLimit/GradientLimiterAndSaturation/FbcDc/FbcDcSaturationLimit'
 * '<S122>' : 'TJATCT_model/TJATCT/TCTCLM/GradientLimit/GradientLimiterAndSaturation/FbcDc/FbcDcGradientLimitation/GradientLimiter'
 * '<S123>' : 'TJATCT_model/TJATCT/TCTCLM/GradientLimit/GradientLimiterAndSaturation/FbcDc/FbcDcGradientLimitation/GradientLimiter/Saturation Dynamic'
 * '<S124>' : 'TJATCT_model/TJATCT/TCTCLM/GradientLimit/GradientLimiterAndSaturation/FbcDc/FbcDcSaturationLimit/Saturation Dynamic'
 * '<S125>' : 'TJATCT_model/TJATCT/TCTCLM/GradientLimit/GradientLimiterAndSaturation/FfcCrv/FfcCrvGradientLimitation'
 * '<S126>' : 'TJATCT_model/TJATCT/TCTCLM/GradientLimit/GradientLimiterAndSaturation/FfcCrv/FfcCrvSaturationLimit'
 * '<S127>' : 'TJATCT_model/TJATCT/TCTCLM/GradientLimit/GradientLimiterAndSaturation/FfcCrv/FfcCrvGradientLimitation/GradientLimiter'
 * '<S128>' : 'TJATCT_model/TJATCT/TCTCLM/GradientLimit/GradientLimiterAndSaturation/FfcCrv/FfcCrvGradientLimitation/GradientLimiter/Saturation Dynamic'
 * '<S129>' : 'TJATCT_model/TJATCT/TCTCLM/GradientLimit/GradientLimiterAndSaturation/FfcCrv/FfcCrvSaturationLimit/Saturation Dynamic'
 * '<S130>' : 'TJATCT_model/TJATCT/TCTCLM/GradientLimit/HoldWarning/RSFlipFlop'
 * '<S131>' : 'TJATCT_model/TJATCT/TCTCLM/PlausibilityCheck/CalculateTimer'
 * '<S132>' : 'TJATCT_model/TJATCT/TCTCLM/PlausibilityCheck/CheckPreconditions'
 * '<S133>' : 'TJATCT_model/TJATCT/TCTCLM/PlausibilityCheck/PlausibilityCheckOfTheControlSignal'
 * '<S134>' : 'TJATCT_model/TJATCT/TCTCLM/PlausibilityCheck/SumCtrlSignal'
 * '<S135>' : 'TJATCT_model/TJATCT/TCTCLM/PlausibilityCheck/CalculateTimer/Detect Decrease'
 * '<S136>' : 'TJATCT_model/TJATCT/TCTCLM/PlausibilityCheck/PlausibilityCheckOfTheControlSignal/CalCrvThd'
 * '<S137>' : 'TJATCT_model/TJATCT/TCTCLM/PlausibilityCheck/PlausibilityCheckOfTheControlSignal/LimterWarningFlag'
 * '<S138>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control'
 * '<S139>' : 'TJATCT_model/TJATCT/TCTDEV/SignalGenerator'
 * '<S140>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/DetermineEnableCondition'
 * '<S141>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/FFC based on external Self Steering Gradient from RTE'
 * '<S142>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/FFC based on internal Self Steering Gradient from RTE'
 * '<S143>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/Subsystem'
 * '<S144>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/Subsystem1'
 * '<S145>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 1'
 * '<S146>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 2'
 * '<S147>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/DetermineEnableCondition/Detect Decrease'
 * '<S148>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 1/CoeffForPsiDotToDelta'
 * '<S149>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 1/SetFfcDelta'
 * '<S150>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 1/CoeffForPsiDotToDelta/CoeffForPsiDotToDelta'
 * '<S151>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 1/CoeffForPsiDotToDelta/DiscreteDerivativeDelta_2dot'
 * '<S152>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 1/CoeffForPsiDotToDelta/DiscreteDerivativeDelta_3dot'
 * '<S153>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 1/CoeffForPsiDotToDelta/DiscreteDerivativeDelta_dot'
 * '<S154>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 1/CoeffForPsiDotToDelta/DiscreteDerivativeDelta_dot/LowPassFilter'
 * '<S155>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 1/CoeffForPsiDotToDelta/DiscreteDerivativeDelta_dot/LowPassFilter/Saturation Dynamic1'
 * '<S156>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 1/SetFfcDelta/SetDelta3Dot'
 * '<S157>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 1/SetFfcDelta/SetFfcDelta'
 * '<S158>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 1/SetFfcDelta/SetFfcDelta1'
 * '<S159>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 1/SetFfcDelta/SetFfcDelta/Integrator'
 * '<S160>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 1/SetFfcDelta/SetFfcDelta/Integrator1'
 * '<S161>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 1/SetFfcDelta/SetFfcDelta/Integrator2'
 * '<S162>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 1/SetFfcDelta/SetFfcDelta/Integrator/Saturation Dynamic'
 * '<S163>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 1/SetFfcDelta/SetFfcDelta/Integrator/Unit Delay Enabled Resettable External IC'
 * '<S164>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 1/SetFfcDelta/SetFfcDelta/Integrator/Unit Delay Enabled Resettable External IC/Unit Delay Resettable External IC'
 * '<S165>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 1/SetFfcDelta/SetFfcDelta/Integrator1/Saturation Dynamic'
 * '<S166>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 1/SetFfcDelta/SetFfcDelta/Integrator1/Unit Delay Enabled Resettable External IC'
 * '<S167>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 1/SetFfcDelta/SetFfcDelta/Integrator1/Unit Delay Enabled Resettable External IC/Unit Delay Resettable External IC'
 * '<S168>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 1/SetFfcDelta/SetFfcDelta/Integrator2/Saturation Dynamic'
 * '<S169>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 1/SetFfcDelta/SetFfcDelta/Integrator2/Unit Delay Enabled Resettable External IC'
 * '<S170>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 1/SetFfcDelta/SetFfcDelta/Integrator2/Unit Delay Enabled Resettable External IC/Unit Delay Resettable External IC'
 * '<S171>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 2/CoeffForPsiDotToDelta'
 * '<S172>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 2/SetFfcDelta'
 * '<S173>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 2/CoeffForPsiDotToDelta/CoeffForPsiDotToDelta'
 * '<S174>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 2/CoeffForPsiDotToDelta/DiscreteDerivativeDelta_2dot'
 * '<S175>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 2/CoeffForPsiDotToDelta/DiscreteDerivativeDelta_3dot'
 * '<S176>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 2/CoeffForPsiDotToDelta/DiscreteDerivativeDelta_dot'
 * '<S177>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 2/CoeffForPsiDotToDelta/DiscreteDerivativeDelta_dot/LowPassFilter'
 * '<S178>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 2/CoeffForPsiDotToDelta/DiscreteDerivativeDelta_dot/LowPassFilter/Saturation Dynamic1'
 * '<S179>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 2/SetFfcDelta/SetDelta3Dot'
 * '<S180>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 2/SetFfcDelta/SetFfcDelta'
 * '<S181>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 2/SetFfcDelta/SetFfcDelta1'
 * '<S182>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 2/SetFfcDelta/SetFfcDelta/Integrator'
 * '<S183>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 2/SetFfcDelta/SetFfcDelta/Integrator1'
 * '<S184>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 2/SetFfcDelta/SetFfcDelta/Integrator2'
 * '<S185>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 2/SetFfcDelta/SetFfcDelta/Integrator/Saturation Dynamic'
 * '<S186>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 2/SetFfcDelta/SetFfcDelta/Integrator/Unit Delay Enabled Resettable External IC'
 * '<S187>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 2/SetFfcDelta/SetFfcDelta/Integrator/Unit Delay Enabled Resettable External IC/Unit Delay Resettable External IC'
 * '<S188>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 2/SetFfcDelta/SetFfcDelta/Integrator1/Saturation Dynamic'
 * '<S189>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 2/SetFfcDelta/SetFfcDelta/Integrator1/Unit Delay Enabled Resettable External IC'
 * '<S190>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 2/SetFfcDelta/SetFfcDelta/Integrator1/Unit Delay Enabled Resettable External IC/Unit Delay Resettable External IC'
 * '<S191>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 2/SetFfcDelta/SetFfcDelta/Integrator2/Saturation Dynamic'
 * '<S192>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 2/SetFfcDelta/SetFfcDelta/Integrator2/Unit Delay Enabled Resettable External IC'
 * '<S193>' : 'TJATCT_model/TJATCT/TCTDEV/Feedforward control/nverse DMC Dynamic (deltaF -> deltaFSet)*Q_d 2/SetFfcDelta/SetFfcDelta/Integrator2/Unit Delay Enabled Resettable External IC/Unit Delay Resettable External IC'
 * '<S194>' : 'TJATCT_model/TJATCT/TCTDEV/SignalGenerator/CrvGenerator'
 * '<S195>' : 'TJATCT_model/TJATCT/TCTDEV/SignalGenerator/CrvTimeGenerator'
 * '<S196>' : 'TJATCT_model/TJATCT/TCTDEV/SignalGenerator/DeltaFGenerator'
 * '<S197>' : 'TJATCT_model/TJATCT/TCTDEV/SignalGenerator/DeltaFTimeGenerator'
 * '<S198>' : 'TJATCT_model/TJATCT/TCTDEV/SignalGenerator/CrvTimeGenerator/Detect Increase'
 * '<S199>' : 'TJATCT_model/TJATCT/TCTDEV/SignalGenerator/DeltaFTimeGenerator/Detect Increase1'
 * '<S200>' : 'TJATCT_model/TJATCT/TCTDTE/CompensateBankAngleRoad'
 * '<S201>' : 'TJATCT_model/TJATCT/TCTDTE/DetermineEnableCondition'
 * '<S202>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator'
 * '<S203>' : 'TJATCT_model/TJATCT/TCTDTE/CompensateBankAngleRoad/CompensateByRoadBankAngle'
 * '<S204>' : 'TJATCT_model/TJATCT/TCTDTE/CompensateBankAngleRoad/SaturationLimiter'
 * '<S205>' : 'TJATCT_model/TJATCT/TCTDTE/CompensateBankAngleRoad/CompensateByRoadBankAngle/CalculateSteerAngle'
 * '<S206>' : 'TJATCT_model/TJATCT/TCTDTE/CompensateBankAngleRoad/CompensateByRoadBankAngle/CurvatureEstimation'
 * '<S207>' : 'TJATCT_model/TJATCT/TCTDTE/CompensateBankAngleRoad/CompensateByRoadBankAngle/FilterAndGradientLimit'
 * '<S208>' : 'TJATCT_model/TJATCT/TCTDTE/CompensateBankAngleRoad/CompensateByRoadBankAngle/CurvatureEstimation/LowPassFilter'
 * '<S209>' : 'TJATCT_model/TJATCT/TCTDTE/CompensateBankAngleRoad/CompensateByRoadBankAngle/CurvatureEstimation/LowPassFilter/Saturation Dynamic1'
 * '<S210>' : 'TJATCT_model/TJATCT/TCTDTE/CompensateBankAngleRoad/CompensateByRoadBankAngle/FilterAndGradientLimit/GradientLimiter'
 * '<S211>' : 'TJATCT_model/TJATCT/TCTDTE/CompensateBankAngleRoad/CompensateByRoadBankAngle/FilterAndGradientLimit/GradientLimiter/Saturation Dynamic'
 * '<S212>' : 'TJATCT_model/TJATCT/TCTDTE/CompensateBankAngleRoad/SaturationLimiter/SaturationOfReqCrv'
 * '<S213>' : 'TJATCT_model/TJATCT/TCTDTE/CompensateBankAngleRoad/SaturationLimiter/SaturationOfReqDelta'
 * '<S214>' : 'TJATCT_model/TJATCT/TCTDTE/CompensateBankAngleRoad/SaturationLimiter/SaturationOfReqCrv/Saturation Dynamic'
 * '<S215>' : 'TJATCT_model/TJATCT/TCTDTE/CompensateBankAngleRoad/SaturationLimiter/SaturationOfReqDelta/Saturation Dynamic'
 * '<S216>' : 'TJATCT_model/TJATCT/TCTDTE/DetermineEnableCondition/Detect Decrease1'
 * '<S217>' : 'TJATCT_model/TJATCT/TCTDTE/DetermineEnableCondition/Saturation Dynamic'
 * '<S218>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv'
 * '<S219>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/EstStrAngleByVehicleDynamic'
 * '<S220>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqCrvByDMC'
 * '<S221>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqDeltaByDMC'
 * '<S222>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqCrvByDMC/RawReqCrvByDMC'
 * '<S223>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqCrvByDMC/SaturationLimitForReqCrv'
 * '<S224>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqCrvByDMC/RawReqCrvByDMC/Crv2Dot'
 * '<S225>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqCrvByDMC/RawReqCrvByDMC/CrvByLaDmc'
 * '<S226>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqCrvByDMC/RawReqCrvByDMC/DenForDeltaDotToCrv'
 * '<S227>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqCrvByDMC/RawReqCrvByDMC/DiscreteDerivativeDelta2Dot'
 * '<S228>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqCrvByDMC/RawReqCrvByDMC/DiscreteDerivativeDeltaDot'
 * '<S229>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqCrvByDMC/RawReqCrvByDMC/KappaAngle'
 * '<S230>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqCrvByDMC/RawReqCrvByDMC/CrvByLaDmc/Integrator'
 * '<S231>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqCrvByDMC/RawReqCrvByDMC/CrvByLaDmc/Integrator1'
 * '<S232>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqCrvByDMC/RawReqCrvByDMC/CrvByLaDmc/Integrator/Saturation Dynamic'
 * '<S233>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqCrvByDMC/RawReqCrvByDMC/CrvByLaDmc/Integrator/Unit Delay Enabled Resettable External IC'
 * '<S234>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqCrvByDMC/RawReqCrvByDMC/CrvByLaDmc/Integrator/Unit Delay Enabled Resettable External IC/Unit Delay Resettable External IC'
 * '<S235>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqCrvByDMC/RawReqCrvByDMC/CrvByLaDmc/Integrator1/Saturation Dynamic'
 * '<S236>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqCrvByDMC/RawReqCrvByDMC/CrvByLaDmc/Integrator1/Unit Delay Enabled Resettable External IC'
 * '<S237>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqCrvByDMC/RawReqCrvByDMC/CrvByLaDmc/Integrator1/Unit Delay Enabled Resettable External IC/Unit Delay Resettable External IC'
 * '<S238>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqCrvByDMC/SaturationLimitForReqCrv/RawReqCrvByDstrb'
 * '<S239>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqCrvByDMC/SaturationLimitForReqCrv/SaturationOfReqCrv'
 * '<S240>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqCrvByDMC/SaturationLimitForReqCrv/SaturationOfReqCrv/Saturation Dynamic'
 * '<S241>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqDeltaByDMC/CoefficientOfLaDmcTransferFunction'
 * '<S242>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqDeltaByDMC/Inverse DMC Dynamic (delta -> delta_set)*Q_d'
 * '<S243>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqDeltaByDMC/SaturationLimitForReqDelta'
 * '<S244>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqDeltaByDMC/Inverse DMC Dynamic (delta -> delta_set)*Q_d/CoeffForPsiDotToDelta'
 * '<S245>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqDeltaByDMC/Inverse DMC Dynamic (delta -> delta_set)*Q_d/SetDelta'
 * '<S246>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqDeltaByDMC/Inverse DMC Dynamic (delta -> delta_set)*Q_d/CoeffForPsiDotToDelta/CoeffForPsiDotToDelta'
 * '<S247>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqDeltaByDMC/Inverse DMC Dynamic (delta -> delta_set)*Q_d/CoeffForPsiDotToDelta/DiscreteDerivativeDelta_2dot'
 * '<S248>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqDeltaByDMC/Inverse DMC Dynamic (delta -> delta_set)*Q_d/CoeffForPsiDotToDelta/DiscreteDerivativeDelta_3dot'
 * '<S249>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqDeltaByDMC/Inverse DMC Dynamic (delta -> delta_set)*Q_d/CoeffForPsiDotToDelta/DiscreteDerivativeDelta_dot'
 * '<S250>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqDeltaByDMC/Inverse DMC Dynamic (delta -> delta_set)*Q_d/CoeffForPsiDotToDelta/DiscreteDerivativeDelta_dot/LowPassFilter'
 * '<S251>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqDeltaByDMC/Inverse DMC Dynamic (delta -> delta_set)*Q_d/CoeffForPsiDotToDelta/DiscreteDerivativeDelta_dot/LowPassFilter/Saturation Dynamic1'
 * '<S252>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqDeltaByDMC/Inverse DMC Dynamic (delta -> delta_set)*Q_d/SetDelta/SetDelta'
 * '<S253>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqDeltaByDMC/Inverse DMC Dynamic (delta -> delta_set)*Q_d/SetDelta/SetDelta3Dot'
 * '<S254>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqDeltaByDMC/Inverse DMC Dynamic (delta -> delta_set)*Q_d/SetDelta/SetDelta/Integrator'
 * '<S255>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqDeltaByDMC/Inverse DMC Dynamic (delta -> delta_set)*Q_d/SetDelta/SetDelta/Integrator1'
 * '<S256>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqDeltaByDMC/Inverse DMC Dynamic (delta -> delta_set)*Q_d/SetDelta/SetDelta/Integrator2'
 * '<S257>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqDeltaByDMC/Inverse DMC Dynamic (delta -> delta_set)*Q_d/SetDelta/SetDelta/Integrator/Saturation Dynamic'
 * '<S258>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqDeltaByDMC/Inverse DMC Dynamic (delta -> delta_set)*Q_d/SetDelta/SetDelta/Integrator/Unit Delay Enabled Resettable External IC'
 * '<S259>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqDeltaByDMC/Inverse DMC Dynamic (delta -> delta_set)*Q_d/SetDelta/SetDelta/Integrator/Unit Delay Enabled Resettable External IC/Unit Delay Resettable External IC'
 * '<S260>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqDeltaByDMC/Inverse DMC Dynamic (delta -> delta_set)*Q_d/SetDelta/SetDelta/Integrator1/Saturation Dynamic'
 * '<S261>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqDeltaByDMC/Inverse DMC Dynamic (delta -> delta_set)*Q_d/SetDelta/SetDelta/Integrator1/Unit Delay Enabled Resettable External IC'
 * '<S262>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqDeltaByDMC/Inverse DMC Dynamic (delta -> delta_set)*Q_d/SetDelta/SetDelta/Integrator1/Unit Delay Enabled Resettable External IC/Unit Delay Resettable External IC'
 * '<S263>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqDeltaByDMC/Inverse DMC Dynamic (delta -> delta_set)*Q_d/SetDelta/SetDelta/Integrator2/Saturation Dynamic'
 * '<S264>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqDeltaByDMC/Inverse DMC Dynamic (delta -> delta_set)*Q_d/SetDelta/SetDelta/Integrator2/Unit Delay Enabled Resettable External IC'
 * '<S265>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqDeltaByDMC/Inverse DMC Dynamic (delta -> delta_set)*Q_d/SetDelta/SetDelta/Integrator2/Unit Delay Enabled Resettable External IC/Unit Delay Resettable External IC'
 * '<S266>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqDeltaByDMC/SaturationLimitForReqDelta/RawReqDeltaByDstrb'
 * '<S267>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqDeltaByDMC/SaturationLimitForReqDelta/SaturationOfReqDelta'
 * '<S268>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/CalculateDeltaAndCrv/ReqDeltaByDMC/SaturationLimitForReqDelta/SaturationOfReqDelta/Saturation Dynamic'
 * '<S269>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/EstStrAngleByVehicleDynamic/CoefficientOfVdyTransferFunction'
 * '<S270>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/EstStrAngleByVehicleDynamic/Inverse Vehicle Dynamics psi_dot -> delta'
 * '<S271>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/EstStrAngleByVehicleDynamic/CoefficientOfVdyTransferFunction/CalculateA0'
 * '<S272>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/EstStrAngleByVehicleDynamic/CoefficientOfVdyTransferFunction/CalculateA1'
 * '<S273>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/EstStrAngleByVehicleDynamic/CoefficientOfVdyTransferFunction/CalculateB0'
 * '<S274>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/EstStrAngleByVehicleDynamic/CoefficientOfVdyTransferFunction/CalculateB1'
 * '<S275>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/EstStrAngleByVehicleDynamic/CoefficientOfVdyTransferFunction/CalculateB2'
 * '<S276>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/EstStrAngleByVehicleDynamic/Inverse Vehicle Dynamics psi_dot -> delta/DenForPsiDotToDelta'
 * '<S277>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/EstStrAngleByVehicleDynamic/Inverse Vehicle Dynamics psi_dot -> delta/FilterForDelta'
 * '<S278>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/EstStrAngleByVehicleDynamic/Inverse Vehicle Dynamics psi_dot -> delta/NumForPsiDotToDelta'
 * '<S279>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/EstStrAngleByVehicleDynamic/Inverse Vehicle Dynamics psi_dot -> delta/DenForPsiDotToDelta/DiscreteDerivativePsi_2dot'
 * '<S280>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/EstStrAngleByVehicleDynamic/Inverse Vehicle Dynamics psi_dot -> delta/DenForPsiDotToDelta/DiscreteDerivativePsi_dot'
 * '<S281>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/EstStrAngleByVehicleDynamic/Inverse Vehicle Dynamics psi_dot -> delta/DenForPsiDotToDelta/VdyFcnDenForPsiDotToDelta'
 * '<S282>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/EstStrAngleByVehicleDynamic/Inverse Vehicle Dynamics psi_dot -> delta/FilterForDelta/LowPassFilter'
 * '<S283>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/EstStrAngleByVehicleDynamic/Inverse Vehicle Dynamics psi_dot -> delta/FilterForDelta/LowPassFilter/Saturation Dynamic1'
 * '<S284>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/EstStrAngleByVehicleDynamic/Inverse Vehicle Dynamics psi_dot -> delta/NumForPsiDotToDelta/Delta2DotByVdyFcn'
 * '<S285>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/EstStrAngleByVehicleDynamic/Inverse Vehicle Dynamics psi_dot -> delta/NumForPsiDotToDelta/DeltaByVdyFcn'
 * '<S286>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/EstStrAngleByVehicleDynamic/Inverse Vehicle Dynamics psi_dot -> delta/NumForPsiDotToDelta/DeltaByVdyFcn/Integrator'
 * '<S287>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/EstStrAngleByVehicleDynamic/Inverse Vehicle Dynamics psi_dot -> delta/NumForPsiDotToDelta/DeltaByVdyFcn/Integrator1'
 * '<S288>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/EstStrAngleByVehicleDynamic/Inverse Vehicle Dynamics psi_dot -> delta/NumForPsiDotToDelta/DeltaByVdyFcn/Integrator/Saturation Dynamic'
 * '<S289>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/EstStrAngleByVehicleDynamic/Inverse Vehicle Dynamics psi_dot -> delta/NumForPsiDotToDelta/DeltaByVdyFcn/Integrator/Unit Delay Enabled Resettable External IC'
 * '<S290>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/EstStrAngleByVehicleDynamic/Inverse Vehicle Dynamics psi_dot -> delta/NumForPsiDotToDelta/DeltaByVdyFcn/Integrator/Unit Delay Enabled Resettable External IC/Unit Delay Resettable External IC'
 * '<S291>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/EstStrAngleByVehicleDynamic/Inverse Vehicle Dynamics psi_dot -> delta/NumForPsiDotToDelta/DeltaByVdyFcn/Integrator1/Saturation Dynamic'
 * '<S292>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/EstStrAngleByVehicleDynamic/Inverse Vehicle Dynamics psi_dot -> delta/NumForPsiDotToDelta/DeltaByVdyFcn/Integrator1/Unit Delay Enabled Resettable External IC'
 * '<S293>' : 'TJATCT_model/TJATCT/TCTDTE/DisturbanceEstimator/EstStrAngleByVehicleDynamic/Inverse Vehicle Dynamics psi_dot -> delta/NumForPsiDotToDelta/DeltaByVdyFcn/Integrator1/Unit Delay Enabled Resettable External IC/Unit Delay Resettable External IC'
 * '<S294>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver'
 * '<S295>' : 'TJATCT_model/TJATCT/TCTEST/InputSourceSelect'
 * '<S296>' : 'TJATCT_model/TJATCT/TCTEST/ModelCoefficientsCalculation'
 * '<S297>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver'
 * '<S298>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ESTQualifier'
 * '<S299>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/SideSlipAngleObserver'
 * '<S300>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/CalculatePlantObserver'
 * '<S301>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR'
 * '<S302>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/CalculatePlantObserver/PlantObserverCoefficient'
 * '<S303>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/CalculatePlantObserver/PlantObserverEstimation'
 * '<S304>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/CalculatePlantObserver/PlantObserverCoefficient/CalculateAX'
 * '<S305>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/CalculatePlantObserver/PlantObserverCoefficient/CalculateBX'
 * '<S306>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/CalculatePlantObserver/PlantObserverCoefficient/CalculateHK'
 * '<S307>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/CalculatePlantObserver/PlantObserverCoefficient/CalculateLY'
 * '<S308>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/CalculatePlantObserver/PlantObserverCoefficient/CalculateAX/Subsystem'
 * '<S309>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/CalculatePlantObserver/PlantObserverCoefficient/CalculateAX/SystemMatrixA'
 * '<S310>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/CalculatePlantObserver/PlantObserverCoefficient/CalculateLY/CalculateLYPobs'
 * '<S311>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/CalculatePlantObserver/PlantObserverCoefficient/CalculateLY/FeedbackMatrix'
 * '<S312>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/CalculatePlantObserver/PlantObserverEstimation/CalculatePlantObserver'
 * '<S313>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/CalculatePlantObserver/PlantObserverEstimation/EstimationByPlantObserver'
 * '<S314>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/CalculatePlantObserver/PlantObserverEstimation/EstimationByPlantObserver/Integrator1'
 * '<S315>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/CalculatePlantObserver/PlantObserverEstimation/EstimationByPlantObserver/Integrator2'
 * '<S316>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/CalculatePlantObserver/PlantObserverEstimation/EstimationByPlantObserver/Integrator3'
 * '<S317>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/CalculatePlantObserver/PlantObserverEstimation/EstimationByPlantObserver/Integrator4'
 * '<S318>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/CalculatePlantObserver/PlantObserverEstimation/EstimationByPlantObserver/Saturation Dynamic1'
 * '<S319>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/CalculatePlantObserver/PlantObserverEstimation/EstimationByPlantObserver/Saturation Dynamic2'
 * '<S320>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/CalculatePlantObserver/PlantObserverEstimation/EstimationByPlantObserver/Saturation Dynamic3'
 * '<S321>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/CalculatePlantObserver/PlantObserverEstimation/EstimationByPlantObserver/Saturation Dynamic4'
 * '<S322>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/CalculatePlantObserver/PlantObserverEstimation/EstimationByPlantObserver/Integrator1/Unit Delay Enabled Resettable External IC'
 * '<S323>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/CalculatePlantObserver/PlantObserverEstimation/EstimationByPlantObserver/Integrator1/Unit Delay Enabled Resettable External IC/Unit Delay Resettable External IC'
 * '<S324>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/CalculatePlantObserver/PlantObserverEstimation/EstimationByPlantObserver/Integrator2/Unit Delay Enabled Resettable External IC'
 * '<S325>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/CalculatePlantObserver/PlantObserverEstimation/EstimationByPlantObserver/Integrator2/Unit Delay Enabled Resettable External IC/Unit Delay Resettable External IC'
 * '<S326>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/CalculatePlantObserver/PlantObserverEstimation/EstimationByPlantObserver/Integrator3/Unit Delay Enabled Resettable External IC'
 * '<S327>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/CalculatePlantObserver/PlantObserverEstimation/EstimationByPlantObserver/Integrator3/Unit Delay Enabled Resettable External IC/Unit Delay Resettable External IC'
 * '<S328>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/CalculatePlantObserver/PlantObserverEstimation/EstimationByPlantObserver/Integrator4/Unit Delay Enabled Resettable External IC'
 * '<S329>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/CalculatePlantObserver/PlantObserverEstimation/EstimationByPlantObserver/Integrator4/Unit Delay Enabled Resettable External IC/Unit Delay Resettable External IC'
 * '<S330>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/DetectionOfHighDynamicRequests'
 * '<S331>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/PlantObserverOutputLimit'
 * '<S332>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/DetectionOfHighDynamicRequests/HighDynamicRequestByDY'
 * '<S333>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/DetectionOfHighDynamicRequests/HighDynamicRequestByTheta'
 * '<S334>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/DetectionOfHighDynamicRequests/HighdynamicRequests'
 * '<S335>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/DetectionOfHighDynamicRequests/HighDynamicRequestByDY/HdrPercByDYDot'
 * '<S336>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/DetectionOfHighDynamicRequests/HighDynamicRequestByDY/LowPassFilter'
 * '<S337>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/DetectionOfHighDynamicRequests/HighDynamicRequestByDY/HdrPercByDYDot/Saturation Dynamic'
 * '<S338>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/DetectionOfHighDynamicRequests/HighDynamicRequestByDY/LowPassFilter/LowPassFilter'
 * '<S339>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/DetectionOfHighDynamicRequests/HighDynamicRequestByDY/LowPassFilter/LowPassFilter/Saturation Dynamic1'
 * '<S340>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/DetectionOfHighDynamicRequests/HighDynamicRequestByTheta/HdrPercByTheta'
 * '<S341>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/DetectionOfHighDynamicRequests/HighDynamicRequestByTheta/LowPassFilter'
 * '<S342>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/DetectionOfHighDynamicRequests/HighDynamicRequestByTheta/HdrPercByTheta/Saturation Dynamic'
 * '<S343>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/DetectionOfHighDynamicRequests/HighDynamicRequestByTheta/LowPassFilter/LowPassFilter'
 * '<S344>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/DetectionOfHighDynamicRequests/HighDynamicRequestByTheta/LowPassFilter/LowPassFilter/Saturation Dynamic1'
 * '<S345>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/DetectionOfHighDynamicRequests/HighdynamicRequests/DetermineActiveParamSet'
 * '<S346>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/DetectionOfHighDynamicRequests/HighdynamicRequests/EnableFlagForHDR'
 * '<S347>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/DetectionOfHighDynamicRequests/HighdynamicRequests/SelectHDRParameters'
 * '<S348>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/DetectionOfHighDynamicRequests/HighdynamicRequests/SelectHDRParameters/CalculateHdrThreshold'
 * '<S349>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/DetectionOfHighDynamicRequests/HighdynamicRequests/SelectHDRParameters/CalculateHdrWeightFactor'
 * '<S350>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/PlantObserverOutputLimit/CalculateDeltaTheta'
 * '<S351>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/PlantObserverOutputLimit/CalculateLateralDeviation'
 * '<S352>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/PlantObserverOutputLimit/CalculateDeltaTheta/OutputSourceSelectForDeltaTheta'
 * '<S353>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/PlantObserverOutputLimit/CalculateDeltaTheta/QualifierByDeltaTheta'
 * '<S354>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/PlantObserverOutputLimit/CalculateDeltaTheta/SaturationOfCourseAngleDeviation'
 * '<S355>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/PlantObserverOutputLimit/CalculateDeltaTheta/OutputSourceSelectForDeltaTheta/GradientLimiter'
 * '<S356>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/PlantObserverOutputLimit/CalculateDeltaTheta/OutputSourceSelectForDeltaTheta/GradientLimiter1'
 * '<S357>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/PlantObserverOutputLimit/CalculateDeltaTheta/OutputSourceSelectForDeltaTheta/GradientLimiter2'
 * '<S358>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/PlantObserverOutputLimit/CalculateDeltaTheta/OutputSourceSelectForDeltaTheta/SelectDeltaTheta'
 * '<S359>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/PlantObserverOutputLimit/CalculateDeltaTheta/OutputSourceSelectForDeltaTheta/GradientLimiter/GradientLimiter'
 * '<S360>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/PlantObserverOutputLimit/CalculateDeltaTheta/OutputSourceSelectForDeltaTheta/GradientLimiter/GradientLimiter/Saturation Dynamic'
 * '<S361>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/PlantObserverOutputLimit/CalculateDeltaTheta/OutputSourceSelectForDeltaTheta/GradientLimiter1/GradientLimiter'
 * '<S362>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/PlantObserverOutputLimit/CalculateDeltaTheta/OutputSourceSelectForDeltaTheta/GradientLimiter1/GradientLimiter/Saturation Dynamic'
 * '<S363>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/PlantObserverOutputLimit/CalculateDeltaTheta/OutputSourceSelectForDeltaTheta/GradientLimiter2/GradientLimiter'
 * '<S364>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/PlantObserverOutputLimit/CalculateDeltaTheta/OutputSourceSelectForDeltaTheta/GradientLimiter2/GradientLimiter/Saturation Dynamic'
 * '<S365>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/PlantObserverOutputLimit/CalculateDeltaTheta/SaturationOfCourseAngleDeviation/Saturation Dynamic'
 * '<S366>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/PlantObserverOutputLimit/CalculateLateralDeviation/OutputSourceSelectForDeltaY'
 * '<S367>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/PlantObserverOutputLimit/CalculateLateralDeviation/QualifierByLateralDeviation'
 * '<S368>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/PlantObserverOutputLimit/CalculateLateralDeviation/SaturationOfLateralDeviation'
 * '<S369>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/PlantObserverOutputLimit/CalculateLateralDeviation/OutputSourceSelectForDeltaY/GradientLimiter'
 * '<S370>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/PlantObserverOutputLimit/CalculateLateralDeviation/OutputSourceSelectForDeltaY/GradientLimiter/Saturation Dynamic'
 * '<S371>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/ PlantObserver/PlantObserverOutputAndHDR/PlantObserverOutputLimit/CalculateLateralDeviation/SaturationOfLateralDeviation/Saturation Dynamic'
 * '<S372>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/SideSlipAngleObserver/CalculateBetaAndPsiDot'
 * '<S373>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/SideSlipAngleObserver/CoeffStateSpace'
 * '<S374>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/SideSlipAngleObserver/SaturationOfSideSlipAngle'
 * '<S375>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/SideSlipAngleObserver/CalculateBetaAndPsiDot/CalculateStateSpace'
 * '<S376>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/SideSlipAngleObserver/CalculateBetaAndPsiDot/EstBetaAndPsiDot'
 * '<S377>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/SideSlipAngleObserver/CalculateBetaAndPsiDot/EstBetaAndPsiDot/Integrator'
 * '<S378>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/SideSlipAngleObserver/CalculateBetaAndPsiDot/EstBetaAndPsiDot/Integrator1'
 * '<S379>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/SideSlipAngleObserver/CalculateBetaAndPsiDot/EstBetaAndPsiDot/Integrator/Saturation Dynamic'
 * '<S380>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/SideSlipAngleObserver/CalculateBetaAndPsiDot/EstBetaAndPsiDot/Integrator/Unit Delay Enabled Resettable External IC'
 * '<S381>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/SideSlipAngleObserver/CalculateBetaAndPsiDot/EstBetaAndPsiDot/Integrator/Unit Delay Enabled Resettable External IC/Unit Delay Resettable External IC'
 * '<S382>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/SideSlipAngleObserver/CalculateBetaAndPsiDot/EstBetaAndPsiDot/Integrator1/Saturation Dynamic'
 * '<S383>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/SideSlipAngleObserver/CalculateBetaAndPsiDot/EstBetaAndPsiDot/Integrator1/Unit Delay Enabled Resettable External IC'
 * '<S384>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/SideSlipAngleObserver/CalculateBetaAndPsiDot/EstBetaAndPsiDot/Integrator1/Unit Delay Enabled Resettable External IC/Unit Delay Resettable External IC'
 * '<S385>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/SideSlipAngleObserver/CoeffStateSpace/CalculateAX'
 * '<S386>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/SideSlipAngleObserver/CoeffStateSpace/CalculateBX'
 * '<S387>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/SideSlipAngleObserver/CoeffStateSpace/CalculateLY'
 * '<S388>' : 'TJATCT_model/TJATCT/TCTEST/CalculateObserver/SideSlipAngleObserver/SaturationOfSideSlipAngle/Saturation Dynamic'
 * '<S389>' : 'TJATCT_model/TJATCT/TCTEST/InputSourceSelect/InputSourceSelectForCurvature'
 * '<S390>' : 'TJATCT_model/TJATCT/TCTEST/InputSourceSelect/InputSourceSelectForDeltaY'
 * '<S391>' : 'TJATCT_model/TJATCT/TCTEST/InputSourceSelect/SelectSourceOfSteerAngle'
 * '<S392>' : 'TJATCT_model/TJATCT/TCTEST/InputSourceSelect/InputSourceSelectForCurvature/LowPassFilter'
 * '<S393>' : 'TJATCT_model/TJATCT/TCTEST/InputSourceSelect/InputSourceSelectForCurvature/LowPassFilter/Saturation Dynamic1'
 * '<S394>' : 'TJATCT_model/TJATCT/TCTEST/InputSourceSelect/InputSourceSelectForDeltaY/GradientLimit'
 * '<S395>' : 'TJATCT_model/TJATCT/TCTEST/InputSourceSelect/InputSourceSelectForDeltaY/GradientLimit/GradientLimiter1'
 * '<S396>' : 'TJATCT_model/TJATCT/TCTEST/InputSourceSelect/InputSourceSelectForDeltaY/GradientLimit/GradientLimiter2'
 * '<S397>' : 'TJATCT_model/TJATCT/TCTEST/InputSourceSelect/InputSourceSelectForDeltaY/GradientLimit/QualifierByLateralDeviationInput'
 * '<S398>' : 'TJATCT_model/TJATCT/TCTEST/InputSourceSelect/InputSourceSelectForDeltaY/GradientLimit/GradientLimiter1/GradientLimiter'
 * '<S399>' : 'TJATCT_model/TJATCT/TCTEST/InputSourceSelect/InputSourceSelectForDeltaY/GradientLimit/GradientLimiter1/GradientLimiter/Saturation Dynamic'
 * '<S400>' : 'TJATCT_model/TJATCT/TCTEST/InputSourceSelect/InputSourceSelectForDeltaY/GradientLimit/GradientLimiter2/GradientLimiter'
 * '<S401>' : 'TJATCT_model/TJATCT/TCTEST/InputSourceSelect/InputSourceSelectForDeltaY/GradientLimit/GradientLimiter2/GradientLimiter/Saturation Dynamic'
 * '<S402>' : 'TJATCT_model/TJATCT/TCTEST/ModelCoefficientsCalculation/CalcSingleTrackStateSpaceCoefficitents'
 * '<S403>' : 'TJATCT_model/TJATCT/TCTEST/ModelCoefficientsCalculation/DetermineEnableCondition'
 * '<S404>' : 'TJATCT_model/TJATCT/TCTEST/ModelCoefficientsCalculation/QualifierByEnableCondition'
 * '<S405>' : 'TJATCT_model/TJATCT/TCTEST/ModelCoefficientsCalculation/CalcSingleTrackStateSpaceCoefficitents/CalcSingleTrackStateSpaceCoefficitents'
 * '<S406>' : 'TJATCT_model/TJATCT/TCTEST/ModelCoefficientsCalculation/CalcSingleTrackStateSpaceCoefficitents/EnforceMinimumVelocityMagnitude'
 * '<S407>' : 'TJATCT_model/TJATCT/TCTEST/ModelCoefficientsCalculation/CalcSingleTrackStateSpaceCoefficitents/CalcSingleTrackStateSpaceCoefficitents/CalculateA11'
 * '<S408>' : 'TJATCT_model/TJATCT/TCTEST/ModelCoefficientsCalculation/CalcSingleTrackStateSpaceCoefficitents/CalcSingleTrackStateSpaceCoefficitents/CalculateA12'
 * '<S409>' : 'TJATCT_model/TJATCT/TCTEST/ModelCoefficientsCalculation/CalcSingleTrackStateSpaceCoefficitents/CalcSingleTrackStateSpaceCoefficitents/CalculateA21'
 * '<S410>' : 'TJATCT_model/TJATCT/TCTEST/ModelCoefficientsCalculation/CalcSingleTrackStateSpaceCoefficitents/CalcSingleTrackStateSpaceCoefficitents/CalculateA22'
 * '<S411>' : 'TJATCT_model/TJATCT/TCTEST/ModelCoefficientsCalculation/CalcSingleTrackStateSpaceCoefficitents/CalcSingleTrackStateSpaceCoefficitents/CalculateB11'
 * '<S412>' : 'TJATCT_model/TJATCT/TCTEST/ModelCoefficientsCalculation/CalcSingleTrackStateSpaceCoefficitents/CalcSingleTrackStateSpaceCoefficitents/CalculateB21'
 * '<S413>' : 'TJATCT_model/TJATCT/TCTEST/ModelCoefficientsCalculation/CalcSingleTrackStateSpaceCoefficitents/CalcSingleTrackStateSpaceCoefficitents/DistFromCgToGud'
 * '<S414>' : 'TJATCT_model/TJATCT/TCTEST/ModelCoefficientsCalculation/DetermineEnableCondition/Detect Decrease'
 * '<S415>' : 'TJATCT_model/TJATCT/TCTLGC/CalPreCondition'
 * '<S416>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters'
 * '<S417>' : 'TJATCT_model/TJATCT/TCTLGC/DetermineEnableCondition'
 * '<S418>' : 'TJATCT_model/TJATCT/TCTLGC/LGC_LatCtrlMode'
 * '<S419>' : 'TJATCT_model/TJATCT/TCTLGC/OutPutProcess'
 * '<S420>' : 'TJATCT_model/TJATCT/TCTLGC/CalPreCondition/CalculateGainsAndCompensation'
 * '<S421>' : 'TJATCT_model/TJATCT/TCTLGC/CalPreCondition/Reset PT1s and Integrators'
 * '<S422>' : 'TJATCT_model/TJATCT/TCTLGC/CalPreCondition/SafetyFunctionActive'
 * '<S423>' : 'TJATCT_model/TJATCT/TCTLGC/CalPreCondition/CalculateGainsAndCompensation/CalculateCacGain'
 * '<S424>' : 'TJATCT_model/TJATCT/TCTLGC/CalPreCondition/CalculateGainsAndCompensation/CalculateLdcGains'
 * '<S425>' : 'TJATCT_model/TJATCT/TCTLGC/CalPreCondition/CalculateGainsAndCompensation/GainCoeffForLaDmcCompensation'
 * '<S426>' : 'TJATCT_model/TJATCT/TCTLGC/CalPreCondition/CalculateGainsAndCompensation/CalculateCacGain/DGainCac'
 * '<S427>' : 'TJATCT_model/TJATCT/TCTLGC/CalPreCondition/CalculateGainsAndCompensation/CalculateCacGain/IGainCac'
 * '<S428>' : 'TJATCT_model/TJATCT/TCTLGC/CalPreCondition/CalculateGainsAndCompensation/CalculateCacGain/PGainCac'
 * '<S429>' : 'TJATCT_model/TJATCT/TCTLGC/CalPreCondition/CalculateGainsAndCompensation/CalculateCacGain/PTGainCac'
 * '<S430>' : 'TJATCT_model/TJATCT/TCTLGC/CalPreCondition/CalculateGainsAndCompensation/CalculateLdcGains/DGainLdc'
 * '<S431>' : 'TJATCT_model/TJATCT/TCTLGC/CalPreCondition/CalculateGainsAndCompensation/CalculateLdcGains/IGainLdc'
 * '<S432>' : 'TJATCT_model/TJATCT/TCTLGC/CalPreCondition/CalculateGainsAndCompensation/CalculateLdcGains/PGainLdc'
 * '<S433>' : 'TJATCT_model/TJATCT/TCTLGC/CalPreCondition/CalculateGainsAndCompensation/CalculateLdcGains/PT1GacinLdc'
 * '<S434>' : 'TJATCT_model/TJATCT/TCTLGC/CalPreCondition/Reset PT1s and Integrators/Reset Bitmasks'
 * '<S435>' : 'TJATCT_model/TJATCT/TCTLGC/CalPreCondition/Reset PT1s and Integrators/ResetFlag'
 * '<S436>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc'
 * '<S437>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure'
 * '<S438>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure'
 * '<S439>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Subsystem'
 * '<S440>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/CalGainLdc'
 * '<S441>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/ErrCtrlCourse'
 * '<S442>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/CalGainLdc/DeltaFCmdCas'
 * '<S443>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/CalGainLdc/PGainCas'
 * '<S444>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/CalGainLdc/DeltaFCmdCas/CmdLdc'
 * '<S445>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/CalGainLdc/DeltaFCmdCas/DGainAndIGainAndPT1Gain'
 * '<S446>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/CalGainLdc/DeltaFCmdCas/DGainAndIGainAndPT1Gain/DGainCas'
 * '<S447>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/CalGainLdc/DeltaFCmdCas/DGainAndIGainAndPT1Gain/IGainCas'
 * '<S448>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/CalGainLdc/DeltaFCmdCas/DGainAndIGainAndPT1Gain/PT1GainCas'
 * '<S449>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/CalGainLdc/DeltaFCmdCas/DGainAndIGainAndPT1Gain/DGainCas/LowPassFilter1'
 * '<S450>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/CalGainLdc/DeltaFCmdCas/DGainAndIGainAndPT1Gain/DGainCas/LowPassFilter1/Saturation Dynamic1'
 * '<S451>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/CalGainLdc/DeltaFCmdCas/DGainAndIGainAndPT1Gain/IGainCas/Integrator'
 * '<S452>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/CalGainLdc/DeltaFCmdCas/DGainAndIGainAndPT1Gain/IGainCas/Integrator/Saturation Dynamic'
 * '<S453>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/CalGainLdc/DeltaFCmdCas/DGainAndIGainAndPT1Gain/IGainCas/Integrator/Unit Delay Enabled Resettable External IC'
 * '<S454>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/CalGainLdc/DeltaFCmdCas/DGainAndIGainAndPT1Gain/IGainCas/Integrator/Unit Delay Enabled Resettable External IC/Unit Delay Resettable External IC'
 * '<S455>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/CalGainLdc/DeltaFCmdCas/DGainAndIGainAndPT1Gain/PT1GainCas/LowPassFilter'
 * '<S456>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/CalGainLdc/DeltaFCmdCas/DGainAndIGainAndPT1Gain/PT1GainCas/LowPassFilter/Saturation Dynamic1'
 * '<S457>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/CalGainLdc/PGainCas/EnableForPGainLimit'
 * '<S458>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/CalGainLdc/PGainCas/PGainLimit'
 * '<S459>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/CalGainLdc/PGainCas/PGainLimit/GradientLimiter'
 * '<S460>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/CalGainLdc/PGainCas/PGainLimit/GradientLimiter/Saturation Dynamic'
 * '<S461>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/ErrCtrlCourse/CalError'
 * '<S462>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/ErrCtrlCourse/Filter to compensate the conjugated complex Poles of the LaDMC'
 * '<S463>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/ErrCtrlCourse/FilterErrCtrlDistY'
 * '<S464>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/ErrCtrlCourse/Filter to compensate the conjugated complex Poles of the LaDMC/Discrete Derivative 1'
 * '<S465>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/ErrCtrlCourse/Filter to compensate the conjugated complex Poles of the LaDMC/Discrete Derivative 2'
 * '<S466>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/ErrCtrlCourse/Filter to compensate the conjugated complex Poles of the LaDMC/Filter to compensate the conjugated complex Poles of the LaDMC'
 * '<S467>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/ErrCtrlCourse/Filter to compensate the conjugated complex Poles of the LaDMC/IntForCmpn'
 * '<S468>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/ErrCtrlCourse/Filter to compensate the conjugated complex Poles of the LaDMC/Discrete Derivative 1/LowPassFilter'
 * '<S469>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/ErrCtrlCourse/Filter to compensate the conjugated complex Poles of the LaDMC/Discrete Derivative 1/LowPassFilter/Saturation Dynamic1'
 * '<S470>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/ErrCtrlCourse/Filter to compensate the conjugated complex Poles of the LaDMC/IntForCmpn/Integrator'
 * '<S471>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/ErrCtrlCourse/Filter to compensate the conjugated complex Poles of the LaDMC/IntForCmpn/Integrator1'
 * '<S472>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/ErrCtrlCourse/Filter to compensate the conjugated complex Poles of the LaDMC/IntForCmpn/Integrator/Saturation Dynamic'
 * '<S473>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/ErrCtrlCourse/Filter to compensate the conjugated complex Poles of the LaDMC/IntForCmpn/Integrator/Unit Delay Enabled Resettable External IC'
 * '<S474>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/ErrCtrlCourse/Filter to compensate the conjugated complex Poles of the LaDMC/IntForCmpn/Integrator/Unit Delay Enabled Resettable External IC/Unit Delay Resettable External IC'
 * '<S475>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/ErrCtrlCourse/Filter to compensate the conjugated complex Poles of the LaDMC/IntForCmpn/Integrator1/Saturation Dynamic'
 * '<S476>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/ErrCtrlCourse/Filter to compensate the conjugated complex Poles of the LaDMC/IntForCmpn/Integrator1/Unit Delay Enabled Resettable External IC'
 * '<S477>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/ErrCtrlCourse/Filter to compensate the conjugated complex Poles of the LaDMC/IntForCmpn/Integrator1/Unit Delay Enabled Resettable External IC/Unit Delay Resettable External IC'
 * '<S478>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/ErrCtrlCourse/FilterErrCtrlDistY/LowPassFilter'
 * '<S479>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cac of Cascade Structure for Ldc/ErrCtrlCourse/FilterErrCtrlDistY/LowPassFilter/Saturation Dynamic1'
 * '<S480>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/CalGainCdc'
 * '<S481>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/ErrCtrlCourseAngle'
 * '<S482>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/CalGainCdc/DeltaFCmdCdc'
 * '<S483>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/CalGainCdc/PGainCdc'
 * '<S484>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/CalGainCdc/DeltaFCmdCdc/CmdLdc'
 * '<S485>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/CalGainCdc/DeltaFCmdCdc/DGainAndIGainAndPT1Gain'
 * '<S486>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/CalGainCdc/DeltaFCmdCdc/DGainAndIGainAndPT1Gain/DGainCac'
 * '<S487>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/CalGainCdc/DeltaFCmdCdc/DGainAndIGainAndPT1Gain/IGainCac'
 * '<S488>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/CalGainCdc/DeltaFCmdCdc/DGainAndIGainAndPT1Gain/PT1GainCac'
 * '<S489>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/CalGainCdc/DeltaFCmdCdc/DGainAndIGainAndPT1Gain/DGainCac/LowPassFilter1'
 * '<S490>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/CalGainCdc/DeltaFCmdCdc/DGainAndIGainAndPT1Gain/DGainCac/LowPassFilter1/Saturation Dynamic1'
 * '<S491>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/CalGainCdc/DeltaFCmdCdc/DGainAndIGainAndPT1Gain/IGainCac/Integrator'
 * '<S492>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/CalGainCdc/DeltaFCmdCdc/DGainAndIGainAndPT1Gain/IGainCac/Integrator/Saturation Dynamic'
 * '<S493>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/CalGainCdc/DeltaFCmdCdc/DGainAndIGainAndPT1Gain/IGainCac/Integrator/Unit Delay Enabled Resettable External IC'
 * '<S494>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/CalGainCdc/DeltaFCmdCdc/DGainAndIGainAndPT1Gain/IGainCac/Integrator/Unit Delay Enabled Resettable External IC/Unit Delay Resettable External IC'
 * '<S495>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/CalGainCdc/DeltaFCmdCdc/DGainAndIGainAndPT1Gain/PT1GainCac/LowPassFilter'
 * '<S496>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/CalGainCdc/DeltaFCmdCdc/DGainAndIGainAndPT1Gain/PT1GainCac/LowPassFilter/Saturation Dynamic1'
 * '<S497>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/CalGainCdc/PGainCdc/EnableForPGainLimit'
 * '<S498>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/CalGainCdc/PGainCdc/PGainLimit'
 * '<S499>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/CalGainCdc/PGainCdc/PGainLimit/GradientLimiter'
 * '<S500>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/CalGainCdc/PGainCdc/PGainLimit/GradientLimiter/Saturation Dynamic'
 * '<S501>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/ErrCtrlCourseAngle/CalError'
 * '<S502>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/ErrCtrlCourseAngle/Filter to compensate the conjugated complex Poles of the LaDMC'
 * '<S503>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/ErrCtrlCourseAngle/FilterErrCourseAngle'
 * '<S504>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/ErrCtrlCourseAngle/Filter to compensate the conjugated complex Poles of the LaDMC/Discrete Derivative 1'
 * '<S505>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/ErrCtrlCourseAngle/Filter to compensate the conjugated complex Poles of the LaDMC/Discrete Derivative 2'
 * '<S506>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/ErrCtrlCourseAngle/Filter to compensate the conjugated complex Poles of the LaDMC/Filter to compensate the conjugated complex Poles of the LaDMC'
 * '<S507>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/ErrCtrlCourseAngle/Filter to compensate the conjugated complex Poles of the LaDMC/IntForCmpn'
 * '<S508>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/ErrCtrlCourseAngle/Filter to compensate the conjugated complex Poles of the LaDMC/Discrete Derivative 1/LowPassFilter'
 * '<S509>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/ErrCtrlCourseAngle/Filter to compensate the conjugated complex Poles of the LaDMC/Discrete Derivative 1/LowPassFilter/Saturation Dynamic1'
 * '<S510>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/ErrCtrlCourseAngle/Filter to compensate the conjugated complex Poles of the LaDMC/IntForCmpn/Integrator'
 * '<S511>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/ErrCtrlCourseAngle/Filter to compensate the conjugated complex Poles of the LaDMC/IntForCmpn/Integrator1'
 * '<S512>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/ErrCtrlCourseAngle/Filter to compensate the conjugated complex Poles of the LaDMC/IntForCmpn/Integrator/Saturation Dynamic'
 * '<S513>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/ErrCtrlCourseAngle/Filter to compensate the conjugated complex Poles of the LaDMC/IntForCmpn/Integrator/Unit Delay Enabled Resettable External IC'
 * '<S514>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/ErrCtrlCourseAngle/Filter to compensate the conjugated complex Poles of the LaDMC/IntForCmpn/Integrator/Unit Delay Enabled Resettable External IC/Unit Delay Resettable External IC'
 * '<S515>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/ErrCtrlCourseAngle/Filter to compensate the conjugated complex Poles of the LaDMC/IntForCmpn/Integrator1/Saturation Dynamic'
 * '<S516>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/ErrCtrlCourseAngle/Filter to compensate the conjugated complex Poles of the LaDMC/IntForCmpn/Integrator1/Unit Delay Enabled Resettable External IC'
 * '<S517>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/ErrCtrlCourseAngle/Filter to compensate the conjugated complex Poles of the LaDMC/IntForCmpn/Integrator1/Unit Delay Enabled Resettable External IC/Unit Delay Resettable External IC'
 * '<S518>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/ErrCtrlCourseAngle/FilterErrCourseAngle/LowPassFilter'
 * '<S519>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Cdc of Cascade Structure/ErrCtrlCourseAngle/FilterErrCourseAngle/LowPassFilter/Saturation Dynamic1'
 * '<S520>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/CalGainLdc'
 * '<S521>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/ErrCtrlDistY'
 * '<S522>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/CalGainLdc/DeltaFCmdLdc'
 * '<S523>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/CalGainLdc/PGainLdc'
 * '<S524>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/CalGainLdc/DeltaFCmdLdc/CmdLdc'
 * '<S525>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/CalGainLdc/DeltaFCmdLdc/DGainAndIGainAndPT1Gain'
 * '<S526>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/CalGainLdc/DeltaFCmdLdc/DGainAndIGainAndPT1Gain/DGainLdc'
 * '<S527>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/CalGainLdc/DeltaFCmdLdc/DGainAndIGainAndPT1Gain/IGainLdc'
 * '<S528>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/CalGainLdc/DeltaFCmdLdc/DGainAndIGainAndPT1Gain/PT1GainLdc'
 * '<S529>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/CalGainLdc/DeltaFCmdLdc/DGainAndIGainAndPT1Gain/DGainLdc/LowPassFilter1'
 * '<S530>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/CalGainLdc/DeltaFCmdLdc/DGainAndIGainAndPT1Gain/DGainLdc/LowPassFilter1/Saturation Dynamic1'
 * '<S531>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/CalGainLdc/DeltaFCmdLdc/DGainAndIGainAndPT1Gain/IGainLdc/Integrator'
 * '<S532>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/CalGainLdc/DeltaFCmdLdc/DGainAndIGainAndPT1Gain/IGainLdc/Integrator/Saturation Dynamic'
 * '<S533>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/CalGainLdc/DeltaFCmdLdc/DGainAndIGainAndPT1Gain/IGainLdc/Integrator/Unit Delay Enabled Resettable External IC'
 * '<S534>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/CalGainLdc/DeltaFCmdLdc/DGainAndIGainAndPT1Gain/IGainLdc/Integrator/Unit Delay Enabled Resettable External IC/Unit Delay Resettable External IC'
 * '<S535>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/CalGainLdc/DeltaFCmdLdc/DGainAndIGainAndPT1Gain/PT1GainLdc/GradientLimiter'
 * '<S536>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/CalGainLdc/DeltaFCmdLdc/DGainAndIGainAndPT1Gain/PT1GainLdc/LowPassFilter'
 * '<S537>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/CalGainLdc/DeltaFCmdLdc/DGainAndIGainAndPT1Gain/PT1GainLdc/GradientLimiter/Saturation Dynamic'
 * '<S538>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/CalGainLdc/DeltaFCmdLdc/DGainAndIGainAndPT1Gain/PT1GainLdc/LowPassFilter/Saturation Dynamic1'
 * '<S539>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/CalGainLdc/PGainLdc/EnableForPGainLimit'
 * '<S540>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/CalGainLdc/PGainLdc/PGainLimit'
 * '<S541>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/CalGainLdc/PGainLdc/PGainLimit/GradientLimiter'
 * '<S542>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/CalGainLdc/PGainLdc/PGainLimit/GradientLimiter/Saturation Dynamic'
 * '<S543>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/ErrCtrlDistY/CalError'
 * '<S544>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/ErrCtrlDistY/Filter to compensate the conjugated complex Poles of the LaDMC'
 * '<S545>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/ErrCtrlDistY/FilterErrCtrlDistY'
 * '<S546>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/ErrCtrlDistY/Filter to compensate the conjugated complex Poles of the LaDMC/Discrete Derivative 1'
 * '<S547>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/ErrCtrlDistY/Filter to compensate the conjugated complex Poles of the LaDMC/Discrete Derivative 2'
 * '<S548>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/ErrCtrlDistY/Filter to compensate the conjugated complex Poles of the LaDMC/Filter to compensate the conjugated complex Poles of the LaDMC'
 * '<S549>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/ErrCtrlDistY/Filter to compensate the conjugated complex Poles of the LaDMC/Subsystem'
 * '<S550>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/ErrCtrlDistY/Filter to compensate the conjugated complex Poles of the LaDMC/Discrete Derivative 1/LowPassFilter'
 * '<S551>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/ErrCtrlDistY/Filter to compensate the conjugated complex Poles of the LaDMC/Discrete Derivative 1/LowPassFilter/Saturation Dynamic1'
 * '<S552>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/ErrCtrlDistY/Filter to compensate the conjugated complex Poles of the LaDMC/Subsystem/Integrator'
 * '<S553>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/ErrCtrlDistY/Filter to compensate the conjugated complex Poles of the LaDMC/Subsystem/Integrator1'
 * '<S554>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/ErrCtrlDistY/Filter to compensate the conjugated complex Poles of the LaDMC/Subsystem/Integrator/Saturation Dynamic'
 * '<S555>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/ErrCtrlDistY/Filter to compensate the conjugated complex Poles of the LaDMC/Subsystem/Integrator/Unit Delay Enabled Resettable External IC'
 * '<S556>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/ErrCtrlDistY/Filter to compensate the conjugated complex Poles of the LaDMC/Subsystem/Integrator/Unit Delay Enabled Resettable External IC/Unit Delay Resettable External IC'
 * '<S557>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/ErrCtrlDistY/Filter to compensate the conjugated complex Poles of the LaDMC/Subsystem/Integrator1/Saturation Dynamic'
 * '<S558>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/ErrCtrlDistY/Filter to compensate the conjugated complex Poles of the LaDMC/Subsystem/Integrator1/Unit Delay Enabled Resettable External IC'
 * '<S559>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/ErrCtrlDistY/Filter to compensate the conjugated complex Poles of the LaDMC/Subsystem/Integrator1/Unit Delay Enabled Resettable External IC/Unit Delay Resettable External IC'
 * '<S560>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/ErrCtrlDistY/FilterErrCtrlDistY/LowPassFilter'
 * '<S561>' : 'TJATCT_model/TJATCT/TCTLGC/CalcNonLinearParameters/Ldc of Cascade Structure/ErrCtrlDistY/FilterErrCtrlDistY/LowPassFilter/Saturation Dynamic1'
 * '<S562>' : 'TJATCT_model/TJATCT/TCTLGC/DetermineEnableCondition/Detect Decrease'
 * '<S563>' : 'TJATCT_model/TJATCT/TCTLGC/OutPutProcess/CSS - Command Source Select1'
 * '<S564>' : 'TJATCT_model/TJATCT/TCTLGC/OutPutProcess/Conversion of steer angle rqeuest to curvature request'
 * '<S565>' : 'TJATCT_model/TJATCT/TCTLGC/OutPutProcess/Dynamic BAC Gain1'
 * '<S566>' : 'TJATCT_model/TJATCT/TCTLGC/OutPutProcess/Enable Output'
 * '<S567>' : 'TJATCT_model/TJATCT/TCTLGC/OutPutProcess/Enable Output1'
 * '<S568>' : 'TJATCT_model/TJATCT/TCTLGC/OutPutProcess/Enable Output2'
 * '<S569>' : 'TJATCT_model/TJATCT/TCTLGC/OutPutProcess/Freeze1'
 * '<S570>' : 'TJATCT_model/TJATCT/TCTLGC/OutPutProcess/Freeze2'
 * '<S571>' : 'TJATCT_model/TJATCT/TCTLGC/OutPutProcess/LimitGradient_CrvReqFFC'
 * '<S572>' : 'TJATCT_model/TJATCT/TCTLGC/OutPutProcess/Limitation of the summed curvature command's gradient'
 * '<S573>' : 'TJATCT_model/TJATCT/TCTLGC/OutPutProcess/Saturation of the curvature command'
 * '<S574>' : 'TJATCT_model/TJATCT/TCTLGC/OutPutProcess/Subsystem'
 * '<S575>' : 'TJATCT_model/TJATCT/TCTLGC/OutPutProcess/get_bit = 1'
 * '<S576>' : 'TJATCT_model/TJATCT/TCTLGC/OutPutProcess/LimitGradient_CrvReqFFC/CalcGrd'
 * '<S577>' : 'TJATCT_model/TJATCT/TCTLGC/OutPutProcess/LimitGradient_CrvReqFFC/FreezeCrv2'
 * '<S578>' : 'TJATCT_model/TJATCT/TCTLGC/OutPutProcess/LimitGradient_CrvReqFFC/GrdLimit'
 * '<S579>' : 'TJATCT_model/TJATCT/TCTLGC/OutPutProcess/LimitGradient_CrvReqFFC/GrdLimit/Saturation Dynamic'
 * '<S580>' : 'TJATCT_model/TJATCT/TCTLGC/OutPutProcess/Limitation of the summed curvature command's gradient/FreezeCrv2'
 * '<S581>' : 'TJATCT_model/TJATCT/TCTLGC/OutPutProcess/Limitation of the summed curvature command's gradient/Saturation Dynamic'
 * '<S582>' : 'TJATCT_model/TJATCT/TCTLGC/OutPutProcess/Saturation of the curvature command/Saturation'
 * '<S583>' : 'TJATCT_model/TJATCT/TCTLGC/OutPutProcess/Subsystem/Dynamic BAC Gain'
 * '<S584>' : 'TJATCT_model/TJATCT/TCTLGC/OutPutProcess/Subsystem/LimitDeltaF'
 * '<S585>' : 'TJATCT_model/TJATCT/TCTLGC/OutPutProcess/Subsystem/LowPassFilter'
 * '<S586>' : 'TJATCT_model/TJATCT/TCTLGC/OutPutProcess/Subsystem/rad_to_deg'
 * '<S587>' : 'TJATCT_model/TJATCT/TCTLGC/OutPutProcess/Subsystem/rad_to_deg1'
 * '<S588>' : 'TJATCT_model/TJATCT/TCTLGC/OutPutProcess/Subsystem/rad_to_deg3'
 * '<S589>' : 'TJATCT_model/TJATCT/TCTLGC/OutPutProcess/Subsystem/LimitDeltaF/GradientLimitForDeltaF'
 * '<S590>' : 'TJATCT_model/TJATCT/TCTLGC/OutPutProcess/Subsystem/LimitDeltaF/SaturationLimitForDeltaF'
 * '<S591>' : 'TJATCT_model/TJATCT/TCTLGC/OutPutProcess/Subsystem/LimitDeltaF/GradientLimitForDeltaF/Saturation Dynamic1'
 * '<S592>' : 'TJATCT_model/TJATCT/TCTLGC/OutPutProcess/Subsystem/LimitDeltaF/SaturationLimitForDeltaF/Saturation Dynamic'
 * '<S593>' : 'TJATCT_model/TJATCT/TCTLGC/OutPutProcess/Subsystem/LowPassFilter/Saturation Dynamic1'
 * '<S594>' : 'TJATCT_model/TJATCT/TCTLGC/OutPutProcess/get_bit = 1/MATLAB Function'
 * '<S595>' : 'TJATCT_model/TJATCT/TCTLQR/DeltaF_feedback'
 * '<S596>' : 'TJATCT_model/TJATCT/TCTLQR/DeltaF_feedforward'
 * '<S597>' : 'TJATCT_model/TJATCT/TCTLQR/DetermineEnableCondition'
 * '<S598>' : 'TJATCT_model/TJATCT/TCTLQR/FeedforwardGain'
 * '<S599>' : 'TJATCT_model/TJATCT/TCTLQR/LowPassFilter1'
 * '<S600>' : 'TJATCT_model/TJATCT/TCTLQR/Subsystem'
 * '<S601>' : 'TJATCT_model/TJATCT/TCTLQR/Subsystem1'
 * '<S602>' : 'TJATCT_model/TJATCT/TCTLQR/YawrateFilter'
 * '<S603>' : 'TJATCT_model/TJATCT/TCTLQR/butterWorthLowpassFilter'
 * '<S604>' : 'TJATCT_model/TJATCT/TCTLQR/DeltaF_feedback/ComputeLateralErrors'
 * '<S605>' : 'TJATCT_model/TJATCT/TCTLQR/DeltaF_feedback/IGainLdc'
 * '<S606>' : 'TJATCT_model/TJATCT/TCTLQR/DeltaF_feedback/MATLAB Function'
 * '<S607>' : 'TJATCT_model/TJATCT/TCTLQR/DeltaF_feedback/MatK'
 * '<S608>' : 'TJATCT_model/TJATCT/TCTLQR/DeltaF_feedback/UpdateMatrix'
 * '<S609>' : 'TJATCT_model/TJATCT/TCTLQR/DeltaF_feedback/ComputeLateralErrors/Subsystem'
 * '<S610>' : 'TJATCT_model/TJATCT/TCTLQR/DeltaF_feedback/ComputeLateralErrors/Subsystem/LowPassFilter1'
 * '<S611>' : 'TJATCT_model/TJATCT/TCTLQR/DeltaF_feedback/ComputeLateralErrors/Subsystem/Saturation Dynamic1'
 * '<S612>' : 'TJATCT_model/TJATCT/TCTLQR/DeltaF_feedback/ComputeLateralErrors/Subsystem/LowPassFilter1/Saturation Dynamic1'
 * '<S613>' : 'TJATCT_model/TJATCT/TCTLQR/DeltaF_feedback/IGainLdc/Integrator'
 * '<S614>' : 'TJATCT_model/TJATCT/TCTLQR/DeltaF_feedback/IGainLdc/Integrator/Saturation Dynamic'
 * '<S615>' : 'TJATCT_model/TJATCT/TCTLQR/DeltaF_feedback/IGainLdc/Integrator/Unit Delay Enabled Resettable External IC'
 * '<S616>' : 'TJATCT_model/TJATCT/TCTLQR/DeltaF_feedback/IGainLdc/Integrator/Unit Delay Enabled Resettable External IC/Unit Delay Resettable External IC'
 * '<S617>' : 'TJATCT_model/TJATCT/TCTLQR/DeltaF_feedback/UpdateMatrix/MATLAB Function'
 * '<S618>' : 'TJATCT_model/TJATCT/TCTLQR/DeltaF_feedback/UpdateMatrix/MATLAB Function1'
 * '<S619>' : 'TJATCT_model/TJATCT/TCTLQR/DeltaF_feedback/UpdateMatrix/MATLAB Function2'
 * '<S620>' : 'TJATCT_model/TJATCT/TCTLQR/DeltaF_feedback/UpdateMatrix/MATLAB Function3'
 * '<S621>' : 'TJATCT_model/TJATCT/TCTLQR/DeltaF_feedback/UpdateMatrix/vel_process'
 * '<S622>' : 'TJATCT_model/TJATCT/TCTLQR/DeltaF_feedforward/MATLAB Function'
 * '<S623>' : 'TJATCT_model/TJATCT/TCTLQR/DetermineEnableCondition/Detect Decrease'
 * '<S624>' : 'TJATCT_model/TJATCT/TCTLQR/DetermineEnableCondition/Detect Decrease1'
 * '<S625>' : 'TJATCT_model/TJATCT/TCTLQR/LowPassFilter1/Saturation Dynamic1'
 * '<S626>' : 'TJATCT_model/TJATCT/TCTLQR/Subsystem/LimitDeltaF'
 * '<S627>' : 'TJATCT_model/TJATCT/TCTLQR/Subsystem/LimitDeltaF/GradientLimitForDeltaF'
 * '<S628>' : 'TJATCT_model/TJATCT/TCTLQR/Subsystem/LimitDeltaF/SaturationLimitForDeltaF'
 * '<S629>' : 'TJATCT_model/TJATCT/TCTLQR/Subsystem/LimitDeltaF/GradientLimitForDeltaF/Saturation Dynamic1'
 * '<S630>' : 'TJATCT_model/TJATCT/TCTLQR/Subsystem/LimitDeltaF/SaturationLimitForDeltaF/Saturation Dynamic'
 * '<S631>' : 'TJATCT_model/TJATCT/TCTLQR/Subsystem1/MATLAB Function'
 * '<S632>' : 'TJATCT_model/TJATCT/TCTLQR/YawrateFilter/LowPassFilter1'
 * '<S633>' : 'TJATCT_model/TJATCT/TCTLQR/YawrateFilter/LowPassFilter1/Saturation Dynamic1'
 * '<S634>' : 'TJATCT_model/TJATCT/TCTLQR/butterWorthLowpassFilter/MATLAB Function'
 */
#endif                                 /* RTW_HEADER_TJATCT_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
