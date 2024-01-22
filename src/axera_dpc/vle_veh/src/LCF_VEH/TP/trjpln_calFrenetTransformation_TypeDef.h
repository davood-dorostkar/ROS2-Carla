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

*********************************************************************/

#ifndef TRJPLN_CALFRENETTRANSFORMATION_TYPEDEF_H
#define TRJPLN_CALFRENETTRANSFORMATION_TYPEDEF_H

//#ifdef MATH_LOCAL_SFUNCTION
#include "trjpln_TrajPlanMathDefs.h"
//#endif
typedef struct {
    float32 fDevDistY_met;
    float32 fDevHeading_rad;
    float32 fCridrRightSg1_Crv_1pm;
    float32 fCridrRightSg1_ChOfCrv_1pm2;
    float32 fCridrRightSg1_Length_met;
    float32 fCridrLeftSg1_DistY_met;
    float32 fCridrLeftSg1_Heading_rad;
    float32 fCridrLeftSg1_Crv_1pm;
    float32 fCridrLeftSg1_ChOfCrv_1pm2;
    float32 fCridrLeftSg1_Length_met;
    float32 fTgtTrajSg1_DistY_met;
    float32 fTgtTrajSg1_Heading_rad;
    float32 fTgtTrajSg1_Crv_1pm;
    float32 fTgtTrajSg1_ChngOfCrv_1pm2;
    float32 fTgtTrajSg1_Length_met;
    uint8 uiTrajPlanEnbl_nu;
    uint8 uiTrigRecalc_nu;
    float32 fPlanningHorizon_sec;
    float32 fCurCrvVehMotion_1pm;
    float32 fStartCrv_1pm;
    float32 fEgoVelX_mps;
    float32 fEgoAclX_mps2;
    float32 fTgtCrvTrajPrev_1pm;
    float32 fTgtTrajSg1_DistX_met;
    uint8 uiReplanModeArcLength_nu;
    uint8 uiLeftCridrActive_nu;
    float32 fDistYTrajPrev_met;
    float32 fHeadingTrajPrev_rad;
    float32 fCurHdngAndDeltaHdng_rad;
    float32 fPreviewTimeHeading_sec;
    float32 fDeadTime_sec;
    uint8 uiTrigTgtTrajCalc_nu;
    float32 fCridrRightSg1_DistX_met;
    float32 fCridrLeftSg1_DistX_met;
    uint8 uiUseTgtTrajX0_nu;
    uint8 uiUseCridrRightX0_nu;
    float32 fCridrRightSg1_Heading_rad;
    float32 fCridrRightSg1_DistY_met;
    uint8 uiReplanCurValues_nu;
} TRJPLN_calFTInTypeV3_t;

typedef struct {
    float32 fDistYCurValue_met;
    float32 fDistY1stDerivCurValue_nu;
    float32 fDistY2ndDerivCurValue_nu;
    float32 fTrajVel_mps;
    float32 fTrajAcl_mps2;
    float32 afTgtDistY_met[15];
    float32 afTgtDistY1stDeriv_nu[15];
    float32 afTgtDistY2ndDeriv_nu[15];
    float32 afDistYLeft_met[100];
    float32 fTrajDistYPrevious_met;
    float32 fTrajDistYPrev1stDeriv_nu;
    float32 fTrajDistYPrev2ndDeriv_nu;
    uint8 uiNumTgtPoints_nu;
    float32 afTgtPoints_nu[15];
    float32 fTrajPlanningHorizon_sec;
    float32 fPlanHorizonVisRange_sec;
    float32 f1stDerivDevHeading_nu;
    float32 f2ndDerivDevHeading_nu;
    float32 fDstYCurValuePreview_met;
    float32 fDstY1stDrvCurValPrvw_mps;
    float32 fFrenTrafHeadingPrvw_sec;
    uint8 uiNumPointsCridrLeft_nu;
    float32 afCridrRight_Crv_1pm[15];
    uint8 uiNumPointsCridrRight_nu;
} TRJPLN_calFTOutType_t;
#endif
