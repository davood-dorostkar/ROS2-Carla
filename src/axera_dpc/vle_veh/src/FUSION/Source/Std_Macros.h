/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * wen mingshu <wen mingshu@senseauto.com>
 */
/*****************************************************************************

AUTOLIV ELECTRONIC document.

-------------------------------------

DUPLICATION or DISCLOSURE PROHIBITED without prior written consent

******************************************************************************


******************************************************************************/
#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_SOURCE_STD_MACROS_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_SOURCE_STD_MACROS_H_

// #include "TM_Global_Types.h"
#include "TM_PlatF_Types.h"
#include "Compiler.h"

#include "Compiler_Cfg.h"
// #include "TM_Global_Types.h"
#include "TM_PlatF_Types.h"
#include "TM_Soc_Ips.h"
#include "glob.h"
#include "Project_specific_Types.h"
#include "Fusion_Std_Types.h"

/*****************************************************************************
STANDARD MACROS
******************************************************************************/
#define STD_MIN(a, b) (((a) < (b)) ? (a) : (b))
#define STD_MAX(a, b) (((a) > (b)) ? (a) : (b))
#define STD_ABS(a) (((a) >= (0)) ? (a) : -(a))
#define STD_LIMIT(x, min, max) \
    (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))
#define STD_RANGE(x, min, max) \
    ((((x) >= (min)) && ((x) <= (max))) ? (TRUE) : (FALSE))

/***************************************************************************
Evolution of the component

Created by : E. Kagan

$Log: Std_Macros.h  $

******************************************************************************/

#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_SOURCE_STD_MACROS_H_
