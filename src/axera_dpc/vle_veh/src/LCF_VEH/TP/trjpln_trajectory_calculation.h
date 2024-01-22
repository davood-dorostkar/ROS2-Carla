#ifndef TRJPLN_TRAJECTORY_CALCULATION_H
#define TRJPLN_TRAJECTORY_CALCULATION_H
#ifdef __cplusplus
extern "C" {
#endif
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include <string.h>
#include "TM_Global_Types.h"
#include "trjpln_consts.h"
#include "trajectory_plan_ext.h"
#include "trjpln_calOptTrajektorie.h"
/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/
#define TRJPLN_GetBit(source, bitmask) (((source) & (bitmask)) == (bitmask))
#define TPLTJC_CalcTrajDistYEquation(P0, P1, P2, P3, P4, P5, T, T2, T3, T4, \
                                     T5)                                    \
    (P0 + P1 * T + P2 * T2 + P3 * T3 + P4 * T4 + P5 * T5)
#define TPLTJC_CalcTrajDistY1stDervEquation(P1, P2, P3, P4, P5, T, T2, T3, T4) \
    (P1 + P2 * T * 2.f + P3 * T2 * 3.f + P4 * T3 * 4.f + P5 * T4 * 5.f)
#define TPLTJC_CalcTrajDistY2rdDervEquation(P2, P3, P4, P5, T, T2, T3) \
    (P2 * 2.f + P3 * T * 6.f + P4 * T2 * 12.f + P5 * T3 * 20.f)
#define TPLTJC_CalcTrajDistY3rdDervEquation(P3, P4, P5, T, T2) \
    (P3 * 6.f + P4 * T * 24.f + P5 * T2 * 60.f)

typedef struct {
    boolean bTrigTrajReplan;   // trajectory replan enable signal ,[0-1]
    boolean bReplanCurValues;  // replan mode with curvature switch ,[0-1]
    boolean bReplanTgtValues;  // we will replan target values while plan mode
                               // changed or input replan signal is TRUE from
                               // other module ,[0-1]
    boolean bReplanModeArcLength;  // replan mode with arc length switch while
                                   // ego velocity less than 25kmp ,[0-1]
    boolean bTrigCustFctActn;      // same value with bReplanCurValues, [0-1]
    float32 fDelayVehGui_sec;      // vehichle delay time from the lookup talbe
    // result of ego velocity float32-todo ,[0-60]
    float32 fCycleTimeVeh_sec;       // cycle time, range[0.001,0.03]
    float32 fEgoVelX_mps;            // ego vehicle VelX, [-20, 150]
    uint8 uiTrajPlanServQu_nu;       // todo, [0,255]
    float32 fWeightTgtDistY_nu;      // todo, [0,1]
    float32 fWeightEndTime_nu;       // todo, [0,1]
    float32 fDistYToLeTgtArea_met;   // todo, [0,10]
    float32 fDistYToRiTgtArea_met;   // todo, [0,10]
    float32 fFTireAclMax_mps2;       // todo, [-20,20]
    float32 fFTireAclMin_mps2;       // todo, [-20,20]
    float32 fPredictionTimeCrv_sec;  // Prediction time of the curvature,
                                     // saterated, ramped up,[0-1]
    float32 fObstacleVelX_mps;       // todo, [-20,150]
    float32 fObstacleAccelX_mps2;    // todo, [-20,20]
    float32 fObstacleWidth_met;      // todo, [0,150]
    float32 fObstacleDistX_met;      // todo, [-1000,1000]
    float32 fObstacleDistY_met;      // todo, [-1000,1000]
    float32 fRiCorridorLength_met;   // right corridor valid length in right
                                     // corridor cooridnate system, [0 ... 150]
    float32 fLeCorridorLength_met;   // left corridor valid length in right
                                     // corridor coordinate system, [0 ... 150]
    float32 fRiCorridorCurve_1pm;  // right corridor curvature in right corridor
                                   // cooridnate system,[-0.1 ... 0.1]
    float32 afLeDistY_met[LEFT_DISTY_ARRAY_SIZE];  // the frenet transformed
                                                   // left corridor DistanceY
                                                   // (to the reference right
                                                   // corridor), [0-10]
    float32 afTargetDistY_met[TARGET_POINTS_ARRAY_SIZE];  // the frenet
                                                          // transformed target
                                                          // corridor DistanceY
                                                          // (to the reference
                                                          // right corridor),
                                                          // [0-10]
    float32 fTargetDistY1stDeriv_mps[TARGET_POINTS_ARRAY_SIZE];  // the frenet
                                                                 // transformed
                                                                 // target
                                                                 // corridor
    // VelocityY (to the reference right
    // corridor),[-20-20]
    float32 fTargetDistY2ndDeriv_mps2[TARGET_POINTS_ARRAY_SIZE];  // the frenet
                                                                  // transformed
                                                                  // target
                                                                  // corridor
                                                                  // AccelY (to
                                                                  // the
                                                                  // reference
                                                                  // right
                                                                  // corridor),
                                                                  // [-20-20]
    float32 fCurDistY_met;  // same value with the input "fDevDistY_met",
                            // vehicle ego lateral distance in right corridor
                            // coordinate system,[0-10]
    float32 fCurDistY1stDeriv_mps;  // the 1st deriviation of vehicle ego
                                    // lateral distance(or the lateral speed) in
                                    // right corridor coordinate system,
                                    // [-20-20]
    float32 fCurDistY2ndDeriv_mps2;  // the 2nd deriviation of vehicle ego
                                     // lateral distance(or the lateral
                                     // acceleration) in right corridor
                                     // coordinate system, [-20-20]
    float32 afTargetPoints_nu[TARGET_POINTS_ARRAY_SIZE];  // the time or the arc
                                                          // length to the
                                                          // target points from
                                                          // current position,
                                                          // [-100-100]
    float32 fTrajDistYPrev_met;                           // todo, [0-10]
    float32 fTrajDistY1stToPrev_mps;                      // todo, [-20-20]
    float32 fTrajDistY2ndToPrev_mps2;                     // todo, [-20-20]
    uint8 uiNumOfTgtPoints_nu;  // the number of target corridor sample point,
                                // [0-255]
    float32 fTrajVelRefCurve_mps;  // The tangential directional velocity
                                   // relative to the reference right corridor,
                                   // [-20-20]
    float32 fTrajPlanningHorizon_sec;  // todo, [0-60]
    float32 fPreviewTimeHeading_sec;   // todo, [0-300]
    float32 fTrajAclRefCurve_mps2;  // The tangential directional acceleration
                                    // relative to the reference right corridor,
                                    // [-20-20]
    uint8 uiNumOfPointsCridrLeft_nu;   // the number of sample points for left
                                       // corridor in right corridor coordinate
                                       // system , [0-255]
    float32 fPlanHorizonVisRange_sec;  // the farest sample point Dsitance/Time
                                       // of target corridor sample points in
                                       // right corridor coordinate system ,
                                       // [0-300]
    float32 fRiCorridorCrvChng_1pm2;   // right corridor curvature change in
                                       // right corridor cooridnate
                                       // system,[-0.001 ... 0.001]
    float32 fMaxJerkAllowed_mps3;      // Maximum Jerk Allowed in the trajectory
                                       // planning, [0,50]
    float32 fLeCorridorPosY0_met;  // left corridor lateral position in right
                                   // corridor coordinate system, [0 ... 10]
} TRJPLN_TrajectoryCalcInReq_t;

typedef struct {
    float32 fTrajDistY_met;  // calculated DistY at moved distance(which is
                             // current time) of planned optimal trajectory,
                             // [-100,100]
    float32 fTrajDistY1stDeriv_mps;   // calculated VelY at moved distance(which
                                      // is current time) of planned optimal
                                      // trajectory, [-20,20]
    float32 fTrajDistY2ndDeriv_mps2;  // calculated AccelY at moved
                                      // distance(which is current time) of
                                      // planned optimal trajectory, [-20,20]
    float32 fTrajDistY3rdDeriv_mps3;  // calculated AccelYDerivative at moved
                                      // distance(which is current time) of
                                      // planned optimal trajectory, [-1,1]
    uint16 uiQuStatusTrajPlan_nu;  // the calculated optimal trajectory quality
                                   // check result , [0,65535],/*! Bitfield
                                   // indicates the planner status: 1: NOT OK 0:
                                   // OK \n0 bit: min acceleration check \n1
                                   // bit: max acceleration check \n2 bit: right
                                   // corridor boundary collision check \n3 bit:
                                   // left corridor boundary collision check \n4
                                   // bit: object collision check \n5 bit:
                                   // matrix invertible  \n6 bit: trajectory
                                   // length \n7 bit: max jerk check \n8 bit:
                                   // physical max curvature
    float32 afTrajParam_nu[TRAJ_PARAM_ARRAY_SIZE];  // the calculated optimal
                                                    // trajectory parameter for
                                                    // fifth order polynomial
                                                    // equation ,
                                                    // [-10000000,10000000]
    boolean bTrajEnd;   // the check result of whether planned trajectory have
                        // already finished since last replan, [0,1]
    boolean bLengthOK;  // the check result of trajectory length validation for
                        // planned trajectory , [0,1]
    boolean bMatrixInverseOK;  // the check result of matrix inverse validation
                               // for planned trajectory, the planned trajectory
                               // is invalid if matrix inverse not ok, [0,1]
    float32 fYDtTrjFmHeadPrev_mps;    // calculated VelY at moved distance(which
                                      // is current time with PreviewTime Of
                                      // Heading) of planned optimal trajectory,
                                      // [-20,20]
    float32 fYDt2TrjFmKpPrevDT_mps2;  // calculated AccelY at moved
                                      // distance(which is current time with
                                      // vehicle delay time and PreviewTime Of
                                      // curvature) of planned optimal
                                      // trajectory, [-20,20]
    float32 fYDt3TrjFmKpPrevDT_mps3;  // calculated AccelYDerviative at moved
                                      // distance(which is current time with
                                      // vehicle delay time and PreviewTime Of
                                      // curvature) of planned optimal
                                      // trajectory, [-1,1]
    float32 fYD2TrjFmKpPrev_mps2;  // calculated AccelY at moved distance(which
                                   // is current time with PreviewTime Of
                                   // curvature) of planned optimal trajectory,
                                   // [-20,20]
    float32 fEndPointTrajectory_nu;  // the distance/time for the final end
                                     // point of planned trajectory from start
                                     // point of trajectory planned trigger ego
                                     // distance/time,[0,100]
    float32 fTrajDistYFmPrev_met;    // calculated DistY at moved distance(which
                                     // is current time with PreviewTime Of
                                     // Heading) of planned optimal trajectory,
                                     // [-100,100]
    float32 fPassedTrajLenPercent_per;  // the passed percent of ego car with
                                        // planned trajectory , [0,1]
    float32 fMaxJerkTraj_mps3;          // the max jerk of lateral
    // acceleration(acceleration derivative ) for
    // planned trajectory, [-10,10]
    boolean bMaxJerkOK;        // Max jerk check result, [0,1]
    float32 fMaxAclTraj_mps2;  // lateral acceleration of the planned
                               // trajectory., [-20,20]
    float32 fOptimalCost_nu;   // cost of the optimal trajectory, [0,100000]
    float32
        fWeightTargetDistY_nu;  // used weight target disty by planning,[0,100]
    float32 fWeightEndTime_nu;  // used weight end time by planning, [0,100]
} TRJPLN_TrajectoryCalcOutPro_t;

typedef struct {
    const float32* fMeasTrajVariable_nu;
    const boolean* bTrigCustFctActn_nu;
    const boolean* bReplanModeArcLength_nu;
    const float32* fDelayVehGui_sec;
    const float32* fPreviewTimeHeading_sec;
    const float32* fPredictionTimeCrv_sec;
    const boolean* bTrigTrajReplan_nu;
    const float32* fTimeTrajEnd_sec;
} TPLTJC_LateralTrajecotryCalIn_t;

typedef struct {
    float32* fDStart_nu;
    float32* fTrajDistY_met;
    float32* fTrajDistY1stDeriv_mps;
    float32* fTrajDistY2ndDeriv_mps2;
    float32* fTrajDistY3rdDeriv_mps3;
    float32* fYDtTrjFmHeadPrev_mps;
    float32* fYDt2TrjFmKpPrevDT_mps2;
    float32* fYDt3TrjFmKpPrevDT_mps3;
    float32* fYDt2TrjFmKpPrev_mps2;
    float32* fTrajDistYFmPrev_met;
} TPLTJC_LateralTrajecotryCalOut_t;

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
void LCF_TRJPLN_TrajectoryCalc_Exec(
    const TRJPLN_TrajectoryCalcInReq_t* reqPorts,
    const TRJPLN_TrajectoryPlanParam_t* paras,
    TRJPLN_TrajectoryCalcOutPro_t* proPorts,
    TRJPLN_TrajecotryCalcDebug_t* debug);
void LCF_TPLTJC_DebugOutput(TRJPLN_TrajectoryCalcOutPro_t proPorts,
                            TRJPLN_TrajecotryCalcDebug_t* debug);
void LCF_TPLTJC_PreProcess(
    const TRJPLN_TrajectoryCalcInReq_t sTrajCalcModuleInput,
    const TRJPLN_TrajectoryPlanParam_t* paras,
    float32 fLastTrajDistY,
    float32 fLastTrajDistY1st,
    float32 fLastTrajDistY2nd,
    TRJPLN_calOptInTypeV3_t* pCalcOptTrajInput,
    float32* fWeightTgtDistY_nu,
    float32* fWeightEndTime_nu);
void LCF_TPLTJC_Timer(float32 fDeltaTime_sec,
                      boolean bReset,
                      float32 fDefaultTime_sec,
                      float32* fRemainTime_sec);
void LCF_TPLTJC_PostProcess(
    const TRJPLN_calOptOutTypeV4_t sCalculatedOptTraj,
    const TRJPLN_TrajectoryCalcInReq_t* reqPorts,
    TRJPLN_TrajectoryCalcOutPro_t* pTrajecoryCalcModulOut);
void TPLTJC_CalcPassedTrajLenPercent(float32 fEndPointTrajectory_nu,
                                     const float32 afTrajParam_nu[6],
                                     float32 fDStart_nu,
                                     float32 fDT_nu,
                                     float32* fPassedTrajLenPercent_per);
void TPLTJC_CalcTraj(const TPLTJC_LateralTrajecotryCalIn_t sInput,
                     const float32 afTrajParam_nu[6],
                     TPLTJC_LateralTrajecotryCalOut_t sOutput);
void TPLTJC_CalcMeasTrajVariable(boolean bTrigTrajReplan,
                                 float32 fSysCycleTimeVeh_sec,
                                 float32 fTrajVelRefCurve_mps,
                                 boolean bReplanModeArcLength,
                                 float32* fMeasTrajVariable_nu);
void LCF_TRJPLN_TrajectoryCalc_Reset(void);

void TPLTJC_SetBit(uint8 uiBitIndex, boolean bBitSetVal, uint16* pTargetBit);

#ifdef __cplusplus
}
#endif
#endif
