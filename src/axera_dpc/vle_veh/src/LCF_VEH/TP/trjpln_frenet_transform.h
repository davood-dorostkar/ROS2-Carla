#ifndef TRJPLN_FRENET_TRANSFORM_H
#define TRJPLN_FRENET_TRANSFORM_H
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
#include "trjpln_calFrenetTransformation.h"
/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

typedef struct {
    float32 fLeCorridorPosX0_met;  // left corridor longitudinal start position
                                   // in right corridor coordinate system, [-100
                                   // ... 100]
    float32 fLeCorridorPosY0_met;  // left corridor lateral position in right
                                   // corridor coordinate system, [0 ... 10]
    float32 fLeCorridorHeadingAgl_rad;  // left corridor heading angle in right
                                        // corridor cooridnate system,
                                        // [-0.78539816 ... 0.78539816]
    float32 fLeCorridorCurve_1pm;  // left corridor curvature in right corridor
                                   // coordinate system, [-0.1 ... 0.1]
    float32 fLeCorridorCrvChng_1pm2;  // left corridor curvature change in right
                                      // corridor cooridinate system, [-0.001
                                      // ... 0.001]
    float32 fLeCorridorLength_met;    // left corridor valid length in right
                                      // corridor coordinate system, [0 ... 150]
    float32 fRiCorridorPosX0_met;  // right corridor longitudinal start position
                                   // in right corridor coordinate system, [-100
                                   // ... 100]
    float32 fRiCorridorPosY0_met;  // right corridor lateral position in right
                                   // corridor coordinate system, [-10 ... 10]
    float32 fRiCorridorHeadingAgl_rad;  // right corridor heading angle in right
                                        // corridor coordinate system,
                                        // [-0.78539816 ... 0.78539816]
    float32 fRiCorridorCurve_1pm;  // right corridor curvature in right corridor
                                   // cooridnate system,[-0.1 ... 0.1]
    float32 fRiCorridorCrvChng_1pm2;  // right corridor curvature change in
                                      // right corridor cooridnate
                                      // system,[-0.001 ... 0.001]
    float32 fRiCorridorLength_met;    // right corridor valid length in right
                                      // corridor cooridnate system, [0 ... 150]
    float32 fTargetCorridorPosX0_met;  // target corridor longitudinal start
                                       // position in right corridor coordinate
                                       // system, [-100 ... 100]
    float32 fTargetCorridorPosY0_met;  // target corridor lateral position in
                                       // right corridor coordinate system,[-10
                                       // ... 10]
    float32 fTargetCorridorHeading_rad;  // target corridor heading angle in
                                         // right corridor coordinate
                                         // system,[-0.78539816 ... 0.78539816]
    float32 fTargetCorridorCurve_1pm;    // target corridor curvature in right
                                         // corridor coordinate system,[-0.1 ...
                                         // 0.1]
    float32 fTargetCorridorCrvChng_1pm2;  // target corridor curvature change in
                                          // right corridor coordinate
                                          // system,[-0.001 ... 0.001]
    float32 fTargetCorridorLength_met;  // target corridor valid length in right
                                        // corridor coordinate system,[0 ...
                                        // 150]
    float32 fDevDistY_met;  // vehicle ego lateral distance in right corridor
                            // coordinate system,[-15 ... 15]
    float32 fDevHeadingAngle_rad;   // vehicle heading angle in right corridor
                                    // coordinate system, [-0.78539816 ...
                                    // 0.78539816]
    float32 fReplanDevDistY_met;    // deviation in lateral position with the
                                    // distY of last replan triggered if current
                                    // replan is triggered, [-10 ... 10]
    float32 fReplanDevHeading_rad;  // heading angle deviation by replanning,
                                    // [-0.78539816 ... 0.78539816]
    float32 fEgoVelX_mps;           // ego vehicle VelX, [-20, 150]
    float32 fEgoAccelX_mps2;        // ego longitude acceleration, [-20,20]
    float32 fEgoCurve_1pm;          // ego curvature , [-0.15�, 0.15]
    float32 fPlanningHorizon_sec;   // max Planning horizon(time) of the
                                    // trajectory, [0,60]
    boolean bTrajPlanEnble;         // trajectory plan enable signal ,[0,1]
    boolean bTrigTrajReplan;        // trajectory replan enable signal ,[0,1]
    boolean bReplanModeArcLength;   // replan mode with arc length switch while
                                    // ego velocity less than 25kmp ,[0,1]
    float32 fDelayVehGui_sec;       // vehichle delay time from the lookup talbe
    // result of ego velocity float32,todo ,[0,60]
    boolean bTrigReplanTgtTraj;    // whether target trajectory replaned is
                                   // triggered, set true if bReplanTgtValues ||
                                   // bReplanCurValues, [0,1]
    float32 fTrajDistYPrev_met;    // Lateral deviation of planned trajectory,
                                   // [-100,100]
    float32 fTrajHeadingPrev_rad;  // Heading angle of planned
                                   // trajectory,[-6,6]
    float32 fTrajTgtCrvPrev_1pm;   // Curvature of planned trajectory,[-1,1]
    float32 fPredictionTimeHead_sec;  // Prediction time of the heading,
                                      // saturated, ramped up,[0,1]
    boolean bReplanCurValues;      // replan mode with curvature switch ,[0,1]
    float32 fKappaSumCommand_1pm;  // todo,[-0.15,0.15]
} TRJPLN_FrenetTransformInReq_t;

typedef struct {
    float32 fCurDistY_met;  // same value with the input "fDevDistY_met",
                            // vehicle ego lateral distance in right corridor
                            // coordinate system,[0,10]
    float32 fCurDistY1stDeriv_mps;  // the 1st deriviation of vehicle ego
                                    // lateral distance(or the lateral speed) in
                                    // right corridor coordinate system,
                                    // [-20,20]
    float32 fCurDistY2ndDeriv_mps2;  // the 2nd deriviation of vehicle ego
                                     // lateral distance(or the lateral
                                     // acceleration) in right corridor
                                     // coordinate system, [-20,20]
    float32 fTrajVelRefCurve_mps;    // The tangential directional velocity
    // relative to the reference right corridor,
    // [-20,20]
    float32 fTrajAclRefCurve_mps2;  // The tangential directional accleration
                                    // relative to the reference right corridor,
                                    // [-20,20]
    float32 afTargetDistY_met[TARGET_POINTS_ARRAY_SIZE];  // the frenet
                                                          // transformed target
                                                          // corridor DistanceY
                                                          // (to the reference
                                                          // right corridor),
                                                          // [0,10]
    float32 fTargetDistY1stDeriv_mps[TARGET_POINTS_ARRAY_SIZE];  // the frenet
                                                                 // transformed
                                                                 // target
                                                                 // corridor
    // VelocityY (to the reference right
    // corridor),[-20��20]
    float32 fTargetDistY2ndDeriv_mps2[TARGET_POINTS_ARRAY_SIZE];  // the frenet
                                                                  // transformed
                                                                  // target
                                                                  // corridor
                                                                  // AccelY (to
                                                                  // the
                                                                  // reference
                                                                  // right
                                                                  // corridor),
                                                                  // [-20,20]
    float32 afLeDistY_met[LEFT_DISTY_ARRAY_SIZE];  // the frenet transformed
                                                   // left corridor DistanceY
                                                   // (to the reference right
                                                   // corridor), [0,10]
    float32 afTargetPoints_nu[TARGET_POINTS_ARRAY_SIZE];  // the time or the arc
                                                          // length to the
                                                          // target points from
                                                          // current position,
                                                          // [-100,100]
    float32 fTrajDistYPrev_met;  // relate to S_TPLFBT_TrajDistYPrev_met todo,
                                 // [0,10]
    float32 fTrajDistY1stToPrev_mps;   // relate to S_TPLFBT_TrajDistYPrev_met
                                       // todo, [-20,20]
    float32 fTrajDistY2ndToPrev_mps2;  // relate to S_TPLFBT_TrajDistYPrev_met
                                       // todo, [-20,20]
    uint8 uiNumOfTgtPoints_nu;  // the number of target corridor sample point,
                                // [0,255]
    float32 fTrajPlanningHorizon_sec;  // todo, [0,60]
    float32 fDistY1stToDevHead_mps;    // FirstDerivDeltaHeading of ego in right
                                       // corridor frent coordinate system,
                                       // [-20,20]
    float32 fDistY2ndToDevHead_mps2;  // SecondDerivDeltaHeading of ego in right
                                      // corridor frent coordinate system,
                                      // [-20,20]
    float32 fCurDistYPreview_met;     // the ego previewed DistY value after
    // S_TPLCEN_PredictionTimeHead_sec, [0,10]
    float32 fCurDistY1stToPrev_mps;    // the ego previewed VelY value after
                                       // S_TPLCEN_PredictionTimeHead_sec,
                                       // [-20,20]
    float32 fPreviewTimeHeading_sec;   // todo, [0,300]
    float32 fPlanHorizonVisRange_sec;  // the farest sample point Dsitance/Time
                                       // of target corridor sample points in
                                       // right corridor coordinate system ,
                                       // [0,300]
    uint8 uiNumOfPointsCridrLeft_nu;   // the number of sample points for left
                                       // corridor in right corridor coordinate
                                       // system , [0,255]
} TRJPLN_FrenetTransformOutPro_t;

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
void LCF_TRJPLN_FrenetTransform_Exec(
    const TRJPLN_FrenetTransformInReq_t* reqPorts,
    const TRJPLN_TrajectoryPlanParam_t* paras,
    TRJPLN_FrenetTransformOutPro_t* proPorts,
    TRJPLN_FrenetTransfDebug_t* debug);
void LCF_TRJPLN_FrenetTransform_Reset(void);

#ifdef __cplusplus
}
#endif
#endif
