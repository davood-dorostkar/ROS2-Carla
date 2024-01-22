#ifndef TRAJECTORY_PLAN_EXT_H
#define TRAJECTORY_PLAN_EXT_H
#ifdef __cplusplus
extern "C" {
#endif
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "TM_Global_Types.h"
#include "Rte_Type.h"
/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

typedef struct {
    float32 fEgoVehWidth_met;
    float32 fEgoVehLength_met;
    float32 fCycleTime_sec;
    //...
} TRJPLN_TrajectoryPlanParam_t;

#ifndef Rte_TypeDef_TRJPLN_LatenCompDebug_t
typedef struct {
    uint32 uiVersionNum_nu;  // value example
    //...
} TRJPLN_LatenCompDebug_t;
#define Rte_TypeDef_TRJPLN_LatenCompDebug_t
#endif

#ifndef Rte_TypeDef_TRJPLN_CalcEnableDebug_t
typedef struct {
    boolean bTrajPlanEnble;   // trajectory plan enable signal ,[0��1]
    boolean bTrigTrajReplan;  // trajectory replan enable signal ,[0��1]
    boolean bReplanModeArcLength;  // replan mode with arc length switch while
                                   // ego velocity less than 25kmp ,[0��1]
    boolean bReplanCurValues;  // replan mode with curvature switch ,[0��1]
    boolean bReplanTgtValues;  // we will replan target values while plan mode
                               // changed or input replan signal is TRUE from
                               // other module ,[0��1]
    boolean bTrigCustFctChange;  // set bTrigCustFctChange TRUE while last cycle
                                 // and current cycle both are CtrlActived, and
                                 // are actived by different
                                 // ControllingFunction, [0��1]
    boolean bTrajGuiQuChange;    // Signal indicates the trajectory guidance
                                 // qualifier change,[0��1]
    boolean bTrigCustFctActn;    // same value with bReplanCurValues, [0��1]
    boolean bTrigReplanTgtTraj;  // whether target trajectory replaned is
                                 // triggered, set true if bReplanTgtValues ||
                                 // bReplanCurValues, [0��1]
    boolean bEnblSpecPlanStrategy;  // we need a special plan strategy for TJA
                                    // while TJA function is controlling, [0��1]
    float32 fDelayVehGui_sec;  // vehichle delay time from the lookup talbe
    // result of ego velocity float32��todo ,[0��60]
    boolean bTrigLargeDeviation;  // Trigger indicates if the deviation is too
                                  // large,[0��1]
    float32 fPredictionTimeCrv_sec;   // Prediction time of the curvature,
                                      // saterated, ramped up,[0��1]
    float32 fPredictionTimeHead_sec;  // Prediction time of the heading,
                                      // saturated, ramped up,[0��1]
    boolean bCorridorJumpDetected;    // Boolean indicates if there is jump in
                                      // corridor info,[0��1]
    boolean bLatDMCReqFinished;       // Indicates if DMC and EPS handshake is
                                      // finished, [0��1]
} TRJPLN_CalcEnableDebug_t;
#define Rte_TypeDef_TRJPLN_CalcEnableDebug_t
#endif

#ifndef Rte_TypeDef_TRJPLN_FrenetTransfDebug_t
typedef struct {
    float32 fCurDistY_met;  // same value with the input "fDevDistY_met",
                            // vehicle ego lateral distance in right corridor
                            // coordinate system,[0��10]
    float32 fCurDistY1stDeriv_mps;  // the 1st deriviation of vehicle ego
                                    // lateral distance(or the lateral speed) in
                                    // right corridor coordinate system,
                                    // [-20��20]
    float32 fCurDistY2ndDeriv_mps2;  // the 2nd deriviation of vehicle ego
                                     // lateral distance(or the lateral
                                     // acceleration) in right corridor
                                     // coordinate system, [-20��20]
    float32 fTrajVelRefCurve_mps;    // The tangential directional velocity
    // relative to the reference right corridor,
    // [-20��20]
    float32 fTrajAclRefCurve_mps2;  // The tangential directional accleration
                                    // relative to the reference right corridor,
                                    // [-20��20]
    float32 afTargetDistY_met[15];  // the frenet transformed target corridor
                                    // DistanceY (to the reference right
                                    // corridor), [0��10]
    float32 fTargetDistY1stDeriv_mps[15];   // the frenet transformed target
                                            // corridor VelocityY (to the
                                            // reference right
                                            // corridor),[-20��20]
    float32 fTargetDistY2ndDeriv_mps2[15];  // the frenet transformed target
                                            // corridor AccelY (to the reference
                                            // right corridor), [-20��20]
    float32 afLeDistY_met[100];  // the frenet transformed left corridor
                                 // DistanceY (to the reference right corridor),
                                 // [0��10]
    float32 afTargetPoints_nu[15];  // the time or the arc length to the target
                                    // points from current position, [-100��100]
    float32 fTrajDistYPrev_met;  // relate to S_TPLFBT_TrajDistYPrev_met todo,
                                 // [0��10]
    float32 fTrajDistY1stToPrev_mps;   // relate to S_TPLFBT_TrajDistYPrev_met
                                       // todo, [-20��20]
    float32 fTrajDistY2ndToPrev_mps2;  // relate to S_TPLFBT_TrajDistYPrev_met
                                       // todo, [-20��20]
    uint8 uiNumOfTgtPoints_nu;  // the number of target corridor sample point,
                                // [0��255]
    float32 fTrajPlanningHorizon_sec;  // todo, [0��60]
    float32 fDistY1stToDevHead_mps;    // FirstDerivDeltaHeading of ego in right
                                       // corridor frent coordinate system,
                                       // [-20��20]
    float32 fDistY2ndToDevHead_mps2;  // SecondDerivDeltaHeading of ego in right
                                      // corridor frent coordinate system,
                                      // [-20��20]
    float32 fCurDistYPreview_met;     // the ego previewed DistY value after
    // S_TPLCEN_PredictionTimeHead_sec, [0��10]
    float32 fCurDistY1stToPrev_mps;    // the ego previewed VelY value after
                                       // S_TPLCEN_PredictionTimeHead_sec,
                                       // [-20��20]
    float32 fPreviewTimeHeading_sec;   // todo, [0��300]
    float32 fPlanHorizonVisRange_sec;  // the farest sample point Dsitance/Time
                                       // of target corridor sample points in
                                       // right corridor coordinate system ,
                                       // [0��300]
    uint8 uiNumOfPointsCridrLeft_nu;   // the number of sample points for left
                                       // corridor in right corridor coordinate
                                       // system , [0��255]
} TRJPLN_FrenetTransfDebug_t;
#define Rte_TypeDef_TRJPLN_FrenetTransfDebug_t
#endif

#ifndef Rte_TypeDef_TRJPLN_TrajecotryCalcDebug_t
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
    float32 afTrajParam_nu[6];  // the calculated optimal trajectory parameter
                                // for fifth order polynomial equation ,
                                // [-10000000,10000000]
    boolean bTrajEnd;   // the check result of whether planned trajectory have
                        // already finished since last replan, [0,1]
    boolean bLengthOK;  // the check result of trajectory length validation for
                        // planned trajectory , [0,1]
    boolean bMatrixInverseOK;  // the check result of matrix inverse validation
                               // for planned trajectory, the planned trajectory
                               // is invalid if matrix inverse not ok, [0,1]
    float32 fEndPointTrajectory_nu;  // the distance/time for the final end
                                     // point of planned trajectory from start
                                     // point of trajectory planned trigger ego
                                     // distance/time,[0,100]
    float32 fPassedTrajLenPercent_per;  // the passed percent of ego car with
                                        // planned trajectory , [0,1]
    float32 fMaxJerkTraj_mps3;          // the max jerk of lateral
    // acceleration(acceleration derivative ) for
    // planned trajectory, [-10,10]
    boolean bMaxJerkOK;        // Max jerk check result, [0,1]
    float32 fMaxAclTraj_mps2;  // Max. lateral acceleration of the planned
                               // trajectory., [-20,20]
    float32 fOptimalCost_nu;   // cost of the optimal trajectory, [0,100000]
    float32
        fWeightTargetDistY_nu;  // used weight target disty by planning,[0,100]
    float32 fWeightEndTime_nu;  // used weight end time by planning, [0,100]
} TRJPLN_TrajecotryCalcDebug_t;
#define Rte_TypeDef_TRJPLN_TrajecotryCalcDebug_t
#endif

#ifndef Rte_TypeDef_TRJPLN_FrenetBackDebug_t
typedef struct {
    float32 fTrajHeading_rad;  // Target heading angle for kinematic controller,
                               // [-6,6]
    float32
        fTrajDistY_met;  // Lateral deviation of planned trajectory, [-100,100]
    float32 fTrajTgtCurve_1pm;     // Target Curvature for kinematic controller,
                                   // [-0.1,0.1]
    float32 fTrajTgtCrvGrd_1pms;   // Target Curvature gradient for kinematic
                                   // controller, todo
    uint8 uiTrajGuiQualifier_nu;   // qualifier value of trajectory guidence,
                                   // [0,5] .E_LCF_TGQ_REQ_OFF= 0, E_LCF_TGQ_REQ
                                   // = 1, E_LCF_TGQ_REQ_FREEZE= 3,
                                   // E_LCF_TGQ_REQ_FFC	= 4,
                                   // E_LCF_TGQ_REQ_REFCHNG= 5
    float32 fTrajHeadingPrev_rad;  // Heading angle of planned
                                   // trajectory,[-6��6]
    float32 fTrajTgtCrvPrev_1pm;  // Curvature of planned trajectory,[-1��1]
    float32 fCurHeading_rad;      // Current vehicle heading angle under
                                  // consideration of preview time, [-6,6]
    float32 fCurDistY_met;        // Current vehicle lateral deviation under
                                  // consideration of preview time, [-100,100]
    float32 fTrajHeadInclPrev_rad;  // Target heading angle for kinematic
                                    // controller under consideration of preview
                                    // time, [-6,6]
    float32 fCtrlErrDistY_met;      // todo, [-100,100]
    float32 fCtrlErrHeadingAngle_rad;  // todo, [-6,6]
    float32 fCtrlErrHeadAglPrev_rad;   // todo,[-6��6]
    float32 fTrajDistYPrev_met;     // Lateral deviation of planned trajectory,
                                    // [-100��100]
    float32 fDeltaTargetCrv_1pm;    // Difference between planned trajectory
                                    // curvature and target corridor curvature ,
                                    // [-0.15,0.15]
    float32 fDeltaTargetPosY0_met;  // difference between planned trajectory
                                    // lateral distance and target corridor
                                    // lateral distance , [-10,10]
    float32 fDeltaTargetHeading_rad;  // Difference between planned trajectory
                                      // heading and target corridor heading,
                                      // [-6,6]
    boolean bUseTargetCorridor;       // todo, [0,1]
    boolean bTargetSwitch;            // todo, [0,1]
    boolean bGradLimitActive;         // todo, [0,1]
    boolean bPlausiCheckStatus;  // Indicates if the target disty is plausible,
                                 // [0,1]
    uint16 uiS_QuStatusTrajPlan_nu;  // Bitfield indicates the status of
                                     // trajectory planner, including
                                     // information such as collision with
                                     // corridor/object, etc. [0,65535], 1 Not
                                     // OK, 0 OK \n0 bit: min acceleration check
                                     // \n1 bit: max acceleration check  \n2
                                     // bit: right corridor boundary collision
                                     // check  \n3 bit: left corridor boundary
                                     // collision check  \n4 bit: object
                                     // collision check  \n5 bit: matrix
                                     // invertible   \n6 bit: trajectory length
                                     // \n7 bit: max jerk check  \n8 bit: lane
                                     // cross check  \n9 bit: target lateral
                                     // distance  \n10 bit: vehicle lateral
                                     // distance  \n11 bit: right corridor
                                     // transformation  \n12 bit: target
                                     // corridor transformation  \n13 bit: left
                                     // corridor transformation  \n14 bit: input
                                     // target corridor heading \n
    float32 fTrajTgtCrvGrdPrev_1pms;  // Target trajectory curvature gradient,
                                      // [-1, 1]
    uint16 uiD_QuStatusTrajPlan_nu;   // Bitfield indicates the status of
    // trajectory planner,[0,65535],1 Not OK, 0
    // OK \n0 bit: min acceleration check  \n1
    // bit: max acceleration check  \n2 bit:
    // right corridor boundary collision check
    // \n3 bit: left corridor boundary
    // collision check  \n4 bit: object
    // collision check  \n5 bit: matrix
    // invertible   \n6 bit: trajectory length
    // \n7 bit: max jerk check  \n8 bit: lane
    // cross check  \n9 bit: target lateral
    // distance  \n10 bit: vehicle lateral
    // distance  \n11 bit: right corridor
    // transformation  \n12 bit: target
    // corridor transformation  \n13 bit: left
    // corridor transformation  \n14 bit: input
    // target corridor heading \n
} TRJPLN_FrenetBackDebug_t;
#define Rte_TypeDef_TRJPLN_FrenetBackDebug_t
#endif

#ifndef Rte_TypeDef_TRJPLN_TrajectoryPlanDebug_t
typedef struct {
    uint32 uiVersionNum_nu;
    TRJPLN_LatenCompDebug_t pLatenCompDebug;
    TRJPLN_CalcEnableDebug_t pCalcEnableDebug;
    TRJPLN_FrenetTransfDebug_t pFrentTransfDebug;
    TRJPLN_TrajecotryCalcDebug_t pTrajCalcDebug;
    TRJPLN_FrenetBackDebug_t pFrenetBackDebug;
} TRJPLN_TrajectoryPlanDebug_t;
#define Rte_TypeDef_TRJPLN_TrajectoryPlanDebug_t
#endif

#ifndef Rte_TypeDef_TRJPLN_TrajectoryPlanInReq_t
typedef struct {
    float32 fEgoVelX_mps;          // ego vehicle VelX, [-20, 150]
    float32 fEgoAccelX_mps2;       // ego longitude acceleration, [-20��20]
    float32 fEgoCurve_1pm;         // ego curvature , [-0.15�� 0.15]
    float32 fKappaSumCommand_1pm;  // todo,[-0.15��0.15]
    float32 fCycleTimeVeh_sec;     // cycle time, range[0.001,0.03]
    float32 fEPSManualTrqActVal_Nm;  // the acture manual torque value from EPS,
                                     // [-100,100]
    boolean bSysStOffLatDMC;         // system state of lateral DMC module
    // ,[0��1],Indicates TRUE if DMC_LAT_STATUS = 0
    // (LCF_LADMC_OFF)
    boolean bSysStReqLatDMC;     // Indicates TRUE if DMC_LAT_STATUS = 6
                                 // (LCF_LADMC_REQ),[0��1]
    uint8 uiOdometerState_nu;    // vehicle odometer state,  [0,1], 1: odometer
                                 // state OK, 0: odometer state wrong
    float32 fEgoYawRate_rps;     // ego vehicle yawrate, [-1,1]
    uint32 uiVehSync4LCO_us;     // time stamp of VED signals for latency
                                 // compensation.[0,4294967295]
    uint32 uiSenToVehTStamp_us;  // the time stamp of the signals from sen part,
                                 // [0,4294967295]

    float32 fPlanningHorizon_sec;  // max Planning horizon(time) of the
                                   // trajectory, [0��60]
    uint8 uiSysStateLCF_nu;        // lateral control function system state enum
    // value, [0,6],E_LCF_SYSSTATE_NOTPRESENT = 0;
    // E_LCF_SYSSTATE_DISABLED = 1;
    // E_LCF_SYSSTATE_PASSIVE = 2;
    // E_LCF_SYSSTATE_REQUEST = 3;
    // E_LCF_SYSSTATE_CONTROLLING = 4;
    // E_LCF_SYSSTATE_RAMPOUT = 5; E_LCF_SYSSTATE_ERROR
    // = 6; \n\n
    float32 fPredTimeCurve_sec;    // todo,[0��60]
    float32 fPredTimeHeadAng_sec;  //[0��60]
    boolean bTriggerReplan;  // trigger replan signal from CSCLTA module,[0��1]
    float32 fLeCridBndPosX0_met;     // the PosX0 value of left corridor bound,
                                     // [-300, 300]
    float32 fLeCridBndPosY0_met;     // the PosY0 value of left corridor bound,
                                     // [-15, 15]
    float32 fLeCridBndHeadAng_rad;   // the heading angle value of left corridor
                                     // bound, [-0.78539816, 0.78539816]
    float32 fLeCridBndCrv_1pm;       // the curve value of left corridor bound,
                                     // [-0.1, 0.1]
    float32 fLeCridBndCrvChng_1pm2;  // the curve deviation value of left
                                     // corridor bound, [-0.001, 0.001]
    float32 fLeCridBndLength_met;    // the length value of left corridor bound,
                                     // [0, 150]
    float32 fRiCridBndPosX0_met;     // the PosX0 value of right corridor bound,
                                     // [-300, 300]
    float32 fRiCridBndPosY0_met;     // the PosY0 value of right corridor bound,
                                     // [-15, 15]
    float32 fRiCridBndHeadAng_rad;  // the heading angle value of right corridor
                                    // bound, [-0.78539816, 0.78539816]
    float32 fRiCridBndCrv_1pm;      // the curve value of right corridor bound,
                                    // [-0.1, 0.1]
    float32 fRiCridBndCrvChng_1pm2;  // the curve deviation value of right
                                     // corridor bound, [-0.001, 0.001]
    float32 fRiCridBndLength_met;  // the length value of right corridor bound,
                                   // [0, 150]
    float32 fTgtTrajPosX0_met;     // the PosX0 value of target corridor bound,
                                   // [-300, 300]
    float32 fTgtTrajPosY0_met;     // the PosY0 value of target corridor bound,
                                   // [-15, 15]
    float32 fTgtTrajHeadingAng_rad;  // the heading angle value of target
                                     // corridor bound, [-0.78539816,
                                     // 0.78539816]
    float32 fTgtTrajCurve_1pm;     // the curve value of target corridor bound,
                                   // [-0.1, 0.1]
    float32 fTgtTrajCrvChng_1pm2;  // the curve deviation value of target
                                   // corridor bound, [-0.001, 0.001]
    float32 fTgtTrajLength_met;    // the length value of target corridor bound,
                                   // [0, 150]
    boolean bLatencyCompActivated;  // the trigger flag for latency compensation
                                    // function, [0,1] 1: latency compensation
                                    // enable, 0: latency compensation disable
    float32 fSensorTimeStamp_sec;   // time stamp of the camera signal from
                                    // camera sensor,[0,4295]
    uint8 uiTrajPlanServQu_nu;      // todo, [0,255]
    float32 fWeightTgtDistY_nu;     // The importance factor of the lateral
                                    // deviation, [0,1]
    float32 fWeightEndTime_nu;  // The importance factor of the time required
                                // for the planned trajectory, [0,1]
    float32 fDistYToLeTgtArea_met;  // lateral tolerance left boundary value ,
                                    // [0,10]
    float32 fDistYToRiTgtArea_met;  // lateral tolerance right boundary value ,
                                    // [0,10]
    float32 fFTireAclMax_mps2;  // lateral acceleration upper limiting value,
                                // [-20,20]
    float32 fFTireAclMin_mps2;  // lateral acceleration lower limiting value,
                                // [-20,20]
    float32 fObstacleVelX_mps;  // the obstacle velocity X in target trajectory
                                // if it is existed , [-20,150]
    float32 fObstacleAccelX_mps2;  // the obstacle accel X in target trajectory
                                   // if it is existed , [-20,20]
    float32 fObstacleWidth_met;    // the obstacle width in target trajectory if
                                   // it is existed , [0,150]
    float32 fObstacleDistX_met;  // the obstacle distance X in target trajectory
                                 // if it is existed , [-1000,1000]
    float32 fObstacleDistY_met;  // the obstacle distance X in target trajectory
                                 // if it is existed , [-1000,1000]
    float32 fMaxJerkAllowed_mps3;  // Maximum Jerk Allowed in the trajectory
                                   // planning, [0,50]
    uint8
        uiTrajGuiQualifier_nu;  // qualifier value of trajectory guidence, The
                                // qualifier indicates if/how the target
                                // curvature should be considered, [0,5]
                                // .E_LCF_TGQ_REQ_OFF= 0, E_LCF_TGQ_REQ	= 1,
                                // E_LCF_TGQ_REQ_FREEZE= 3, E_LCF_TGQ_REQ_FFC
                                // = 4, E_LCF_TGQ_REQ_REFCHNG= 5

    uint8 uiLeLnQuality_per;   // quality of left lane, [0, 100]
    uint8 uiRiLnQuality_per;   // quality of right lane, [0, 100]
    uint8 uiLeCrvQuality_per;  // quality of left lane curve, [0, 100]
    uint8 uiRiCrvQuality_per;  // quality of right lane curve, [0, 100]

    uint8 uiLatCtrlMode_nu;  // lateral control mode, [0,8]
                             // ,E_TJASTM_LATCTRLMD_PASSIVE=
                             // 0,E_TJASTM_LATCTRLMD_LC=
                             // 1,E_TJASTM_LATCTRLMD_OF=
                             // 2,E_TJASTM_LATCTRLMD_CMB=
                             // 3,E_TJASTM_LATCTRLMD_SALC=
                             // 4,E_TJASTM_LATCTRLMD_LC_RQ=
                             // 5,E_TJASTM_LATCTRLMD_OF_RQ=
                             // 6,E_TJASTM_LATCTRLMD_CMB_RQ= 7,
                             // E_TJASTM_LATCTRLMD_SALC_RQ= 8

    uint8 uiControllingFunction_nu;  // function type of lateral controlling,
                                     // [0,7], E_LCF_OFF_nu= 0,E_LCF_TJA_nu=
                                     // 1,E_LCF_LDP_nu= 2,E_LCF_LDPOC_nu=
                                     // 3,E_LCF_RDP_nu= 4,E_LCF_ALCA_nu=
                                     // 5,E_LCF_AOLC_nu= 6, E_LCF_ESA_nu= 7
} TRJPLN_TrajectoryPlanInReq_t;
#define Rte_TypeDef_TRJPLN_TrajectoryPlanInReq_t
#endif

#ifndef Rte_TypeDef_TRJPLN_TrajectoryPlanOutPro_t
typedef struct {
    boolean bReplanCurValues;  // replan mode with curvature switch ,[0��1]

    uint8 uiTrajGuiQualifier_nu;  // qualifier value of trajectory guidence,
                                  // [0,5] .E_LCF_TGQ_REQ_OFF= 0, E_LCF_TGQ_REQ
                                  // = 1, E_LCF_TGQ_REQ_FREEZE= 3,
                                  // E_LCF_TGQ_REQ_FFC	= 4,
                                  // E_LCF_TGQ_REQ_REFCHNG= 5
    float32 fCurDistY_met;        // Current vehicle lateral deviation under
                                  // consideration of preview time, [-100,100]
    float32
        fTrajDistY_met;  // Lateral deviation of planned trajectory, [-100,100]
    float32 fTrajTgtCurve_1pm;  // Target Curvature for kinematic controller,
                                // [-0.1,0.1]
    float32 fCurHeading_rad;    // Current vehicle heading angle under
                                // consideration of preview time, [-6,6]
    float32 fTrajHeadInclPrev_rad;  // Target heading angle for kinematic
                                    // controller under consideration of preview
                                    // time, [-6,6]
    float32 fTrajHeading_rad;  // Target heading angle for kinematic controller,
                               // [-6,6]
    float32 fTrajTgtCrvGrd_1pms;   // Target Curvature gradient for kinematic
                                   // controller, [-0.1,0.1]
    float32 fTrajHeadingPrev_rad;  // Heading angle of planned
                                   // trajectory,[-6��6]
    float32 fTrajTgtCrvPrev_1pm;  // Curvature of planned trajectory,[-1��1]
    float32 fCtrlErrDistY_met;    // todo, [-100,100]
    float32 fCtrlErrHeadingAngle_rad;  // todo, [-6,6]
    float32 fCtrlErrHeadAglPrev_rad;   // todo,[-6��6]
    float32 fTrajDistYPrev_met;     // Lateral deviation of planned trajectory,
                                    // [-100��100]
    float32 fDeltaTargetCrv_1pm;    // Difference between planned trajectory
                                    // curvature and target corridor curvature ,
                                    // [-0.15,0.15]
    float32 fDeltaTargetPosY0_met;  // difference between planned trajectory
                                    // lateral distance and target corridor
                                    // lateral distance , [-10,10]
    float32 fDeltaTargetHeading_rad;  // Difference between planned trajectory
                                      // heading and target corridor heading,
                                      // [-6,6]
    boolean bUseTargetCorridor;       // todo, [0,1]
    boolean bTargetSwitch;            // todo, [0,1]
    boolean bGradLimitActive;         // todo, [0,1]
    boolean bPlausiCheckStatus;  // Indicates if the target disty is plausible,
                                 // [0,1]
    uint16 uiS_QuStatusTrajPlan_nu;  // Bitfield indicates the status of
                                     // trajectory planner, including
                                     // information such as collision with
                                     // corridor/object, etc. [0,65535], 1 Not
                                     // OK, 0 OK \n0 bit: min acceleration check
                                     // \n1 bit: max acceleration check  \n2
                                     // bit: right corridor boundary collision
                                     // check  \n3 bit: left corridor boundary
                                     // collision check  \n4 bit: object
                                     // collision check  \n5 bit: matrix
                                     // invertible   \n6 bit: trajectory length
                                     // \n7 bit: max jerk check  \n8 bit: lane
                                     // cross check  \n9 bit: target lateral
                                     // distance  \n10 bit: vehicle lateral
                                     // distance  \n11 bit: right corridor
                                     // transformation  \n12 bit: target
                                     // corridor transformation  \n13 bit: left
                                     // corridor transformation  \n14 bit: input
                                     // target corridor heading \n
    float32 fTrajTgtCrvGrdPrev_1pms;  // Target trajectory curvature gradient,
                                      // [-1, 1]
    uint16 uiD_QuStatusTrajPlan_nu;   // Bitfield indicates the status of
    // trajectory planner,[0,65535],1 Not OK, 0
    // OK \n0 bit: min acceleration check  \n1
    // bit: max acceleration check  \n2 bit:
    // right corridor boundary collision check
    // \n3 bit: left corridor boundary
    // collision check  \n4 bit: object
    // collision check  \n5 bit: matrix
    // invertible   \n6 bit: trajectory length
    // \n7 bit: max jerk check  \n8 bit: lane
    // cross check  \n9 bit: target lateral
    // distance  \n10 bit: vehicle lateral
    // distance  \n11 bit: right corridor
    // transformation  \n12 bit: target
    // corridor transformation  \n13 bit: left
    // corridor transformation  \n14 bit: input
    // target corridor heading \n
} TRJPLN_TrajectoryPlanOutPro_t;
#define Rte_TypeDef_TRJPLN_TrajectoryPlanOutPro_t
#endif

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
void LCF_TrajectoryPlan_Reset(void);
void LCF_TrajectoryPlan_Exec(const TRJPLN_TrajectoryPlanInReq_t* reqPorts,
                             const TRJPLN_TrajectoryPlanParam_t* paras,
                             TRJPLN_TrajectoryPlanOutPro_t* proPorts,
                             TRJPLN_TrajectoryPlanDebug_t* debug);

#ifdef __cplusplus
}
#endif
#endif
