#ifndef TRJPLN_FRENET_BACK_H
#define TRJPLN_FRENET_BACK_H
#ifdef __cplusplus
extern "C" {
#endif
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "TM_Global_Types.h"
#include "trajectory_plan_ext.h"
#include "trjpln_consts.h"
#include "trjpln_calFrenetBackTransformation.h"
#include "trjpln_trajectory_calculation.h"
/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

typedef struct {
    uint8 uiTrajGuiQualifier_nu;  // qualifier value of trajectory guidence, The
                                  // qualifier indicates if/how the target
                                  // curvature should be considered, [0,5]
                                  // .E_LCF_TGQ_REQ_OFF= 0, E_LCF_TGQ_REQ	= 1,
                                  // E_LCF_TGQ_REQ_FREEZE= 3, E_LCF_TGQ_REQ_FFC
                                  // = 4, E_LCF_TGQ_REQ_REFCHNG= 5
    float32 fPredictionTimeCrv_sec;  // Prediction time of the curvature,
                                     // saterated, ramped up,[0��1]
    float32 fRiCorridorCurve_1pm;  // right corridor curvature in right corridor
                                   // cooridnate system,[-0.1 ... 0.1]
    float32 fRiCorridorCrvChng_1pm2;   // right corridor curvature change in
                                       // right corridor cooridnate
                                       // system,[-0.001 ... 0.001]
    float32 fTargetCorridorPosX0_met;  // target corridor longitudinal start
                                       // position in right corridor coordinate
                                       // system, [-100 ... 100]
    float32 fTargetCorridorCurve_1pm;  // target corridor curvature in right
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
    float32 fDevHeadingAngle_rad;  // vehicle heading angle in right corridor
                                   // coordinate system, [-0.78539816 ...
                                   // 0.78539816]
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
    float32 fTrajDistY_met;  // calculated DistY at moved distance(which is
                             // current time) of planned optimal trajectory,
                             // [-100,100]
    float32 fTrajDistY1stDeriv_mps;   // calculated VelY at moved distance(which
                                      // is current time) of planned optimal
                                      // trajectory, [-20,20]
    float32 fTrajDistY2ndDeriv_mps2;  // calculated AccelY at moved
                                      // distance(which is current time) of
                                      // planned optimal trajectory, [-20,20]
    float32 fTrajDistY3rdDeriv_mps3;  // calculated AccelYDerviative at moved
                                      // distance(which is current time) of
                                      // planned optimal trajectory, [-1,1]
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
    float32 fYD2TrjFmKpPrev_mps2;   // calculated AccelY at moved distance(which
                                    // is current time with PreviewTime Of
                                    // curvature) of planned optimal trajectory,
                                    // [-20,20]
    float32 fTrajVelRefCurve_mps;   // The tangential directional velocity
                                    // relative to the reference right corridor,
                                    // [-20��20]
    float32 fTrajAclRefCurve_mps2;  // The tangential directional acceleration
                                    // relative to the reference right corridor,
                                    // [-20��20]
    float32 fCurDistYPreview_met;   // the ego previewed DistY value after
                                    // S_TPLCEN_PredictionTimeHead_sec, [0��10]
    float32 fCurDistY1stToPrev_mps;  // the ego previewed VelY value after
                                     // S_TPLCEN_PredictionTimeHead_sec,
                                     // [-20��20]
    boolean bTrajPlanEnble;          // trajectory plan enable signal ,[0��1]
    boolean bReplanModeArcLength;    // replan mode with arc length switch while
                                     // ego velocity less than 25kmp ,[0��1]
    float32 fDelayVehGui_sec;       // vehichle delay time from the lookup talbe
                                    // result of ego velocity float32 ,[0��60]
    boolean bTrigTrajReplan;        // trajectory replan enable signal ,[0��1]
    boolean bEnblSpecPlanStrategy;  // we need a special plan strategy for TJA
                                    // while TJA function is controlling, [0��1]
    float32 fCycleTimeVeh_sec;      // cycle time, range[0.001,0.03]
    boolean bReplanCurValues;       // replan mode with curvature switch ,[0��1]
    float32 fTargetPathY0_met;  // lateral position of the target path (target
                                // corridor with X0 = 0),[-10 ... 10]
    float32 fTargetPathHeading_rad;  // heading angle of the target path (target
                                     // corridor with X0 = 0),[-0.78539816 ...
                                     // 0.78539816]
    float32 fLeCorridorPosY0_met;    // left corridor lateral position in right
                                     // corridor coordinate system, [0 ... 10]
    float32 fRiCorridorPosY0_met;    // right corridor lateral position in right
                                     // corridor coordinate system, [-10 ... 10]
    uint8 uiSPlausiCheckStatus_nu;  // Bitfield indicates the information of the
                                    // plausibility check in TPLLCO 1: NOT OK 0:
                                    // OK,[0 ... 255],	Bit 0: vehicle's lateral
                                    // position \nBit 1: right corridor
                                    // transformation \nBit 2: left corridor
                                    // transformation
    uint8 uiSysStateLCF_nu;  // lateral control function system state enum
                             // value, [0,6],E_LCF_SYSSTATE_NOTPRESENT = 0;
                             // E_LCF_SYSSTATE_DISABLED = 1;
                             // E_LCF_SYSSTATE_PASSIVE = 2;
                             // E_LCF_SYSSTATE_REQUEST = 3;
                             // E_LCF_SYSSTATE_CONTROLLING = 4;
                             // E_LCF_SYSSTATE_RAMPOUT = 5; E_LCF_SYSSTATE_ERROR
                             // = 6; \n\n
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
    float32 fEgoVelX_mps;    // ego vehicle VelX, [-20, 150]
} TRJPLN_FrenetBackInReq_t;

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
    float32 fTrajTgtCrvPrev_1pm;   // Curvature of planned trajectory,[-1��1]
    float32 fCurHeading_rad;       // Current vehicle heading angle under
                                   // consideration of preview time, [-6,6]
    float32 fCurDistY_met;         // Current vehicle lateral deviation under
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
} TRJPLN_FrenetBackOutPro_t;

typedef struct {
    const boolean* pbReplanCurValues;
    const float32* pfSysCycleTimeVeh_sec;
    const float32* pfTrajVelRefCurve_mps;
    const float32* pfTargetCorridorCrv_1pm;
    const float32* pfTgtCridrChngOfCrv_1pm2;
    const float32* pfTargetCridrLength_met;
    const float32* pfTargetCorridorPosX0_met;
    const float32* pfPredictionTimeCrve_sec;
    const float32* pfDelayVehGui_sec;
    const boolean* pbTrigTrajReplan;
    const float32* pfTgtCrvTrajInclPrevAndDeadTime_1pm;
    const float32* pfTgtCrvGrdTrajInclPrevAndDeadTime_1pm2;
    const boolean* pbEnblSpecPlanStrategy;
    const uint8* puiTrajGuiQualifier_nu;
} TRJPLN_CalcTargetCurveGradIn_st;

typedef struct {
    const float32* pfSysCycleTimeVeh_sec;
    const float32* pfTrajDistYPrev_met;
    const float32* pfTrajHeading_rad;
    const float32* pfTrajHeadingInclPreview_rad;
    const boolean* pbReplanCurValues;
    const float32* pfTargetCorridorCrv_1pm;
    const uint8* puiLatCtrlMode_nu;
    const float32* pfTargetPathY0_met;
    const float32* pfTargetPathHeading_rad;
    const float32* pfVehVelX_mps;
    const float32* pfTgtCrvInclPrevAndDeadTime_1pm;
} TRJPLN_TargeValueSelectionIn_st;
typedef struct {
    float32* pfTrajDistY_met;
    float32* pfTrajHeadInclPrev_rad;
    float32* pfTrajTgtCrv_1pm;
    float32* pfDeltaTargetCrv_1pm;
    float32* pfDeltaTargetPosY0_met;
    float32* pfDeltaTargetHeading_rad;
    boolean* pbUseTargetCorridor;
    boolean* pbTargetSwitch;
    boolean* pbGradLimitActive;
    float32* pfTrajHeading_rad;
} TRJPLN_TargeValueSelectionOut_st;
/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
void LCF_TRJPLN_FrenetBack_Exec(const TRJPLN_FrenetBackInReq_t* reqPorts,
                                const TRJPLN_TrajectoryPlanParam_t* paras,
                                TRJPLN_FrenetBackOutPro_t* proPorts,
                                TRJPLN_FrenetBackDebug_t* debug);
void TPJPLN_FBT_SetupTrajGuiFromQualifier(uint8 uiSysStateLCF_nu,
                                          boolean bTrajPlanInvalid,
                                          boolean bTrajPlanEnable,
                                          uint8 uiTrajGuiQualifier_nu,
                                          uint8* puiTrajGuiQualifier_nu);
void TPJPLN_FBT_CheckTrajPlanStatus(uint8 uiSysStateLCF_nu,
                                    uint8 uiPlausiCheckStatus_nu,
                                    uint16 uiQuStatusTrajPlan_nu,
                                    float32 fSysCycleTimeVeh_sec,
                                    boolean bPlausiCheckStatus,
                                    uint16* uiDQuStatusTrajPlan,
                                    uint16* uiSQuStatusTrajPlan,
                                    boolean* bTrajPlanInvalid);
void TRJPLN_FBT_TgtDistYPlausiCheck(float32 fDevDistY_met,
                                    float32 fRiCorridorPosY0_met,
                                    float32 fLeCorridorPosY0_met,
                                    float32 fTrajDistY_met,
                                    uint8 uiTrajGuiQualifier_nu,
                                    boolean* pbPlausiCheckStatus);
void TRJPLN_FBT_TargetValueSelector(TRJPLN_TargeValueSelectionIn_st sInput,
                                    TRJPLN_TargeValueSelectionOut_st sOutput);
void TRJPLN_FBT_CalcTargetCurveGrad(
    const TRJPLN_CalcTargetCurveGradIn_st sInput,
    float32* fMeasKappaTgtInclPrevAndDeadTime_1pm,
    float32* fTgtCrvInclPrevAndDeadTime_1pm,
    float32* fTrajTgtCrvGrd_1pm2);
void LCF_TRJPLN_FrenetBack_Reset(void);

#ifdef __cplusplus
}
#endif
#endif
