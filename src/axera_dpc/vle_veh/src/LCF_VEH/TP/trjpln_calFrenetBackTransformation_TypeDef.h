/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * wangsifan <wangsifan@senseauto.com>
 */
/*********************************************************************

Type:           C-Header

CPU:            All

DESCRIPTION:
********************************************************************************

<HISTORY>
#<version>  <modify-date>   <owner>     <comment>

<NEW>
</NEW>
</HISTORY>

*</>********************************************************************/

#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_VEH_TP_TRJPLN_CALFRENETTRANSFORMATION_TYPEDEF_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_VEH_TP_TRJPLN_CALFRENETTRANSFORMATION_TYPEDEF_H_
#ifdef __cplusplus
extern "C" {
#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_VEH_TP_TRJPLN_CALFRENETTRANSFORMATION_TYPEDEF_H_

// #ifdef MATH_LOCAL_SFUNCTION
#include "trjpln_TrajPlanMathDefs.h"
// #endif
typedef struct {
    uint8 uiNoTrajFound_nu;
    uint8 uiTPLFBT_TrajGuiEnable_nu;
    float32 fTPLFBT_CridrRightSeg1_Crv_1pm;
    float32 fTPLFBT_CridrRightSeg1_ChngOfCrv_1pm2;
    float32 fTPLFBT_TrajDistY_met;
    float32 fTPLFBT_TrajDistY1stDeriv_mps;
    float32 fTPLFBT_TrajDistY2ndDeriv_nu;
    float32 fTPLFBT_TrajDistY3rdDeriv_nu;
    float32 fTPLFBT_TrajVelRefCurve_mps;
    float32 fTPLFBT_TrajAclRefCurve_mps2;
    uint8 uiReplanModeArcLength_nu;
    float32 fDistYCurValuePreview_met;
    float32 fDistY1stDerivCurValuePreview_mps;
    float32 fDevHeading_rad;
    float32 fDevDistY_met;
    float32 fYDotTrajFromHeadingPreview_mps;
    float32 fYDot2TrajFromKappaPrevAndDT_mps2;
    float32 fYDot3TrajFromKappaPrevAndDT_nu;
    float32 fYDotDotTrajFromKappaPreview_mps2;
} TRJPLN_calFBTInType_t;

typedef struct {
    float32 fTrajDistY_met;
    float32 fTrajHeading_rad;
    float32 fTrajTgtCrv_1pm;
    float32 fTrajTgtCrvGrd_1pms;
    float32 fTgtCrvGrdTrajInclPrevAndDT_1pms;
    float32 fHeadingCurValuePreview_rad;
    float32 fDistYCurValuePreview_met;
    float32 fTrajHeadingInclPreview_rad;
    float32 fTgtCrvInclPreviewAndDeadTime_1pm;
    float32 fTgtCrvTrajInclPreview_1pm;
} TRJPLN_calFBTOutType_t;
#ifdef __cplusplus
}
#endif
#endif