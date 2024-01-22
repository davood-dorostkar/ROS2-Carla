/*********************************************************************
Type:           C-Header

        CPU:            All

DESCRIPTION:

********************************************************************************

<HISTORY>
<NEW>
</NEW>
</HISTORY>

*********************************************************************/

#ifndef TRJPLN_CALOPTTRAJEKTORIE_TYPEDEF_H
#define TRJPLN_CALOPTTRAJEKTORIE_TYPEDEF_H

#include "trjpln_TrajPlanMathDefs.h"

typedef struct {
    float32 fDistYInit_nu;
    float32 fDistY1stDerivInit_nu;
    float32 fDistY2ndDerivInit_nu;
    float32 fKd_nu;
    float32 fKt_nu;
    float32 fTrajCalcEgoVelX_mps;
    float32 afTrajCalcTgtDistY_met[15];
    float32 afTrajCalcTgtDistY1stDeriv_nu[15];
    float32 afrajCalcTgtDistY2ndDeriv_nu[15];
    float32 fLatCtrlCoordCridrRightSeg1_Crv_1pm;
    float32 fTolDistYRight_met;
    float32 fTolDistYLeft_met;
    uint8 uiCheckFtire_nu;
    uint8 uiCheckCridrBoundaries_nu;
    uint8 uiCheckObjects_nu;
    uint8 uiCheckJerk_nu;
    uint8 uiCheckLaneCross_nu;
    float32 fAclPotVecMin_mps2;
    float32 fAclPotVecMax_mps2;
    float32 afteTrajCalcTgt_nu[15];
    uint8 uiTrajCalcNumTgtPoints_nu;
    float32 fTrajCalcCircleRadius_met;
    float32 fTrajCalcCircleDistance_met;
    float32 fTrajCalcCridrRightSeg1_Length_met;
    float32 fTrajCalcCridrLeftSeg1_Length_met;
    float32 fObstacle_DistX_met;
    float32 fObstacle_DistY_met;
    float32 fObstacle_Width_met;
    float32 afTrajCalcDistYLeft_met[100];
    uint8 uiReplanModeArcLength_nu;
    float32 fWeightArcLengthPlan_nu;
    float32 fTrajCalcPlanHorizonTraj_sec;
    float32 fTrajCalcPlanHorizonVisibility_sec;
    float32 fTrajCalcTrajVel_mps;
    float32 fTrajCalcTrajAcl_mps2;
    float32 fTrajCalcDeadTime_sec;
    uint8 uiTrajCalcNCircles_nu;
    uint8 uiTrajCalcNumCridrPointsLeft_nu;
    float32 fTrajCalcObjVelX_mps;
    float32 fTrajCalcObjAclX_mps2;
    float32 fCrvChng_1pm2;
    float32 fAllwdJerkMax_mps3;
    float32 fLaneWidth_met;
    float32 fVehicleWidth_met;
    float32 fAllowedTimetoCross_sec;
} TRJPLN_calOptInTypeV3_t;

typedef struct {
    uint8 uiTrajStatus_nu;
    float32 afTrajParam_nu[6];
    float32 fMaxAclTraj_mps2;
    float32 fTrajCalcTimeTrajEnd_sec;
    float32 fArcLengthTrajEnd_met;
    float32 fOptimalTolerance_nu;
    float32 fOptimalCost_nu;
    uint8 uiFtireMinOK_nu;
    uint8 uiFtireMaxOK_nu;
    uint8 uiCollDetRightOK_nu;
    uint8 uiCollDetLeftOK_nu;
    uint8 uiCollDetObjOK_nu;
    uint8 uiMatrixInverseOK_nu;
    uint8 uiLaneCrossOK_nu;
    uint8 uiTrajLengthOK_nu;
    float32 fMaxJerkTraj_mps3;
    uint8 uiMaxJerkOK_nu;
} TRJPLN_calOptOutTypeV4_t;

#endif
