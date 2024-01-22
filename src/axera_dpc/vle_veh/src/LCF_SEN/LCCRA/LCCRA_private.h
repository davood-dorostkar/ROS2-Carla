/**********************************Model
 Property********************************
 *
 * Company             : SENSETIME
 *
 * Tool Version        : Ver2.0
 *
 * Model Name          : LCCRA
 *
 * Model Long Name     : LCCRA

 *

 * Model Advisor       : Not Check

 *

 * Model Version       : Ver1.0

 *

 * Model Author        : ZhuHe

 *

 * Model Reviewer      :

 *

 * Model Review Data   :

 *

 * Model Cycle Time    : 50ms


 ************************************Auto
 Coder**********************************
 *
 * File                             : LCCRA_private.h
 *
 * FileType                         : Code Header File
 *
 * Real-Time Workshop file version  : 9.4 (R2020b) 29-Jul-2020
 *
 * TLC version                      : 9.4 (Aug 20 2020)
 *
 * C source code generated on       : Fri Feb  3 11:44:07 2023
 *
 * Copyright (C) by SenseTime Group Limited. All rights reserved.
 *******************************************************************************/

#ifndef RTW_HEADER_LCCRA_private_h_
#define RTW_HEADER_LCCRA_private_h_
#include "rtwtypes.h"

/* Imported (extern) block signals */
extern real32_T LBP_LeLnClthPosY0_met;      /* '<Root>/Inport' */
extern real32_T LBP_LeLnClthHeading_rad;    /* '<Root>/Inport1' */
extern real32_T LBP_LeLnClthCrv_1pm;        /* '<Root>/Inport2' */
extern real32_T LBP_LeLnClthCrvChng_1pm2;   /* '<Root>/Inport3' */
extern boolean_T LBP_LeLnValid_bool;        /* '<Root>/Inport4' */
extern real32_T LBP_RiLnClthPosY0_met;      /* '<Root>/Inport5' */
extern real32_T LBP_RiLnClthHeading_rad;    /* '<Root>/Inport6' */
extern real32_T LBP_RiLnClthCrv_1pm;        /* '<Root>/Inport7' */
extern real32_T LBP_RiLnClthCrvChng_1pm2;   /* '<Root>/Inport8' */
extern boolean_T LBP_RiLnValid_bool;        /* '<Root>/Inport9' */
extern real32_T ALP_LeLnClthPosY0_met;      /* '<Root>/Inport10' */
extern real32_T ALP_LeLnClthHeading_rad;    /* '<Root>/Inport11' */
extern real32_T ALP_LeLnClthCrv_1pm;        /* '<Root>/Inport12' */
extern real32_T ALP_LeLnClthCrvChng_1pm2;   /* '<Root>/Inport13' */
extern boolean_T ALP_LeLnValid_bool;        /* '<Root>/Inport14' */
extern real32_T ALP_RiLnClthPosY0_met;      /* '<Root>/Inport15' */
extern real32_T ALP_RiLnClthHeading_rad;    /* '<Root>/Inport16' */
extern real32_T ALP_RiLnClthCrv_1pm;        /* '<Root>/Inport17' */
extern real32_T ALP_RiLnClthCrvChng_1pm2;   /* '<Root>/Inport18' */
extern boolean_T ALP_RiLnValid_bool;        /* '<Root>/Inport19' */
extern real32_T VED_EgoVelocity_mps;        /* '<Root>/Inport31' */
extern real32_T VED_EgoYawRate_rps;         /* '<Root>/Inport35' */
extern real32_T VED_EgoClthCrv_1pm;         /* '<Root>/Inport36' */
extern real32_T VED_EgoClthCrvChng_1pm2;    /* '<Root>/Inport37' */
extern real32_T ABPR_LnWidth_met;           /* '<Root>/Inport38' */
extern boolean_T ABPR_LnWidthValid_bool;    /* '<Root>/Inport22' */
extern real32_T LCFRCV_SysCycleTimeSen_sec; /* '<Root>/Inport20' */

/* Imported (extern) pointer block signals */
extern BusObject *Fusion_TargetObject_str; /* '<Root>/Inport26' */

#endif /* RTW_HEADER_LCCRA_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
