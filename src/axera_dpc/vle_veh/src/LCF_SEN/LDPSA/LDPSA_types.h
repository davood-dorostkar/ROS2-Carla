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
 * File                             : LDPSA_types.h
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

#ifndef RTW_HEADER_LDPSA_types_h_
#define RTW_HEADER_LDPSA_types_h_
#include "rtwtypes.h"

/* Model Code Variants */
#ifndef DEFINED_TYPEDEF_FOR_E_LDPState_nu_
#define DEFINED_TYPEDEF_FOR_E_LDPState_nu_

/* System state */
typedef uint8_T E_LDPState_nu;

/* enum E_LDPState_nu */
#define E_LDPState_nu_OFF              ((E_LDPState_nu)0U)       /* Default value */
#define E_LDPState_nu_PASSIVE          ((E_LDPState_nu)1U)
#define E_LDPState_nu_STANDBY          ((E_LDPState_nu)2U)
#define E_LDPState_nu_ACTIVE           ((E_LDPState_nu)3U)
#define E_LDPState_nu_SUSPENDED        ((E_LDPState_nu)4U)
#define E_LDPState_nu_RAMPOUT          ((E_LDPState_nu)5U)
#define E_LDPState_nu_ERROR            ((E_LDPState_nu)6U)
#define E_LDPState_nu_REQUEST          ((E_LDPState_nu)7U)
#endif
#endif                                 /* RTW_HEADER_LDPSA_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
