#ifndef TRJPLN_LATENCY_COMPENSATION_H
#define TRJPLN_LATENCY_COMPENSATION_H
#ifdef __cplusplus
extern "C" {
#endif
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include <string.h>
#include "TM_Global_Types.h"
#include "trajectory_plan_ext.h"
#include "trjpln_consts.h"
#include "tue_common_libs.h"
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
    float32 fCycleTimeVeh_sec;    // cycle time, range[0.001,0.03]
    float32 fEgoYawRate_rps;      // ego vehicle yawrate, [-1,1]
    float32 fEgoVelX_mps;         // ego vehicle VelX, [-20, 150]

} TRJPLN_LatencyCompensationCommon_t;

typedef struct {
    uint8 uiControllingFunction_nu;  // function type of lateral controlling,
                                     // [0,7], E_LCF_OFF_nu= 0,E_LCF_TJA_nu=
                                     // 1,E_LCF_LDP_nu= 2,E_LCF_LDPOC_nu=
                                     // 3,E_LCF_RDP_nu= 4,E_LCF_ALCA_nu=
                                     // 5,E_LCF_AOLC_nu= 6, E_LCF_ESA_nu= 7
    uint8 uiLatCtrlMode_nu;          // lateral control mode, [0,8]
                                     // ,E_TJASTM_LATCTRLMD_PASSIVE=
                                     // 0,E_TJASTM_LATCTRLMD_LC=
                                     // 1,E_TJASTM_LATCTRLMD_OF=
                                     // 2,E_TJASTM_LATCTRLMD_CMB=
                                     // 3,E_TJASTM_LATCTRLMD_SALC=
                                     // 4,E_TJASTM_LATCTRLMD_LC_RQ=
                                     // 5,E_TJASTM_LATCTRLMD_OF_RQ=
                                     // 6,E_TJASTM_LATCTRLMD_CMB_RQ= 7,
                                     // E_TJASTM_LATCTRLMD_SALC_RQ= 8
    uint8 uiLeLnQuality_per;         // quality of left lane, [0, 100]
    uint8 uiRiLnQuality_per;         // quality of right lane, [0, 100]
    uint8 uiLeCrvQuality_per;        // quality of left lane curve, [0, 100]
    uint8 uiRiCrvQuality_per;        // quality of right lane curve, [0, 100]

} TRJPLN_LatencyCompensationConsisCheck_t;

typedef struct {
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

} TRJPLN_LatencyCompensationAllCorridor_t;

typedef struct {
    uint8 uiOdometerState_nu;  // vehicle odometer state,  [0,1], 1: odometer
                               // state OK, 0: odometer state wrong
    uint32 uiVehSync4LCO_us;   // time stamp of VED signals for latency
                               // compensation.[0,4294967295]
    uint8 uiSysStateLCF_nu;    // lateral control function system state enum
                               // value, [0,6],E_LCF_SYSSTATE_NOTPRESENT = 0;
                               // E_LCF_SYSSTATE_DISABLED = 1;
                               // E_LCF_SYSSTATE_PASSIVE = 2;
                               // E_LCF_SYSSTATE_REQUEST = 3;
                               // E_LCF_SYSSTATE_CONTROLLING = 4;
    // E_LCF_SYSSTATE_RAMPOUT = 5; E_LCF_SYSSTATE_ERROR
    // = 6; \n\n
    boolean bLatencyCompActivated;  // the trigger flag for latency compensation
                                    // function, [0,1] 1: latency compensation
                                    // enable, 0: latency compensation disable
    float32 fSensorTimeStamp_sec;   // time stamp of the camera signal from
                                    // camera sensor,[0,4295]
    uint32 uiSenToVehTStamp_us;  // the time stamp of the signals from sen part,
                                 // [0,4294967295]
} TRJPLN_LatencyCompensationLatComIn_t;

typedef struct {
    TRJPLN_LatencyCompensationAllCorridor_t sAllCorridor;
    TRJPLN_LatencyCompensationConsisCheck_t sConsisCheckIn;
    TRJPLN_LatencyCompensationCommon_t sCommonInput;
    TRJPLN_LatencyCompensationLatComIn_t sLatCompIn;
} TRJPLN_LatencyCompensationInReq_t;

typedef struct {
    float32 fCCCDeviation_met;  // Lateral deviation of CCC (course consistency
                                // check),[-1000 ... 1000]
    float32 fCCCTestPointDistX_met;  // DistX of the test point,[0 ... 1000]
    float32 fCCCLength_met;          // Length of the reference,[0 ... 100]
    boolean bCCCReset;               // reference reset,[0 ... 1]
    float32 fCCCPassedPerc_nu;  // Testpoint DistX / Length of the reference,[0
                                // ... 1]
    boolean bCCCWarning;  // CCC Deviation over the threshold detected,[0 ... 1]
    boolean bCCCInvalid;  // CCC failed,[0 ... 1]
} TRJPLN_LatencyCompensationCCCOut_t;

typedef struct {
    float32 fMeasDeltaTime_sec;  // Delta between LD and VDY time stamps,[0,1]
    float32 fRightOrientation_rad;  // Orientation of the right corridor at the
                                    // foot of the perpendicular,[-0.78539816
                                    // ... 0.78539816]
    float32 fDeltaProjPosX_met;     // the distanceX value between ego and
                                    // projected point, float32,Projection on x
    // axle of the foot of the perpendicular from
    // reference point (e.g. front axle middle) to
    // the right corridor boundary in the vehicle
    // coordinate system.,[-10 ... 10]
    float32 fDeltaProjPosY_met;   // the distanceY value between ego and
                                  // projected point, float32,Projection on y
                                  // axle of the foot of the perpendicular from
                                  // reference point (e.g. front axle middle) to
                                  // the right corridor boundary in the vehicle
                                  // coordinate system.,[-10 ... 10]
    float32 fDeltaCoordPosX_met;  // the ego X value, consider preview distanc,
                                  // ego moved distance since last reference
                                  // corridor reset
    float32 fDeltaCoordPosY_met;  // the ego Y value, consider preview distanc,
                                  // ego moved distance since last reference
                                  // corridor reset
    float32 fPosRightPosX_met;  // the orthogonal projected x value from ego car
                                // on right reference corridor
    float32 fPosRightPosY_met;  // the orthogonal projected y value from ego car
                                // on right reference corridor
} TRJPLN_LatencyCompensationCorriParamOut_t;

typedef struct {
    float32 fTargetPathY0_met;  // lateral position of the target path (target
                                // corridor with X0 = 0),[-10 ... 10]
    float32 fTargetPathHeading_rad;  // heading angle of the target path (target
                                     // corridor with X0 = 0),[-0.78539816 ...
                                     // 0.78539816]
    float32 fTargetPathCurve_rad;    // curvature of the target path (target
                                     // corridor with X0 = 0),[-0.1 ... 0.1]
} TRJPLN_LatencyCompensationTargetPath_t;

typedef struct {
    uint8
        uiDPlausiCheckStatus_nu;  // Bitfield indicates the detailed information
                                  // of the plausibility check in TPLLCO 1: NOT
                                  // OK 0: OK,[0 ... 255].	Bit 0: vehicle's
                                  // lateral position \nBit 1: right corridor
                                  // disty \nBit 2: right corridor heading \nBit
                                  // 3: right corridor curvature \nBit 4: left
                                  // corridor disty \nBit 5: left corridor
                                  // heading \nBit 6: left corridor curvature
    uint8 uiSPlausiCheckStatus_nu;  // Bitfield indicates the information of the
                                    // plausibility check in TPLLCO 1: NOT OK 0:
                                    // OK,[0 ... 255],	Bit 0: vehicle's lateral
                                    // position \nBit 1: right corridor
                                    // validation check \nBit 2: right corridor
                                    // transformation \nBit 3: left corridor
                                    // transformation \nBit 4: target corridor
                                    // heading threshold check \nBit 5: CCC
                                    // invalid check result
    float32 fRiCorridorTransDev_met[4];  // Deviation of the lateral position
                                         // between the orginal and transformed
                                         // right corridor boundary,[0 ... 1]
    float32 fTargetCorridorTransDev_met[4];  // Deviation of the lateral
                                             // position between the orginal and
                                             // transformed target corridor,[0
                                             // ... 1]
    float32 fLeCorridorTransDev_met[4];  // Deviation of the lateral position
                                         // between the orginal and transformed
                                         // left corridor boundary.,[0 ... 1]
} TRJPLN_LatencyCompensationPlasiCheck_t;

typedef struct {
    boolean bTriggerReplan;  // whether we need a trajectory replan flag, if
                             // trajectory plan requirement mode changed, uint8,
                             // trigger for replanning, [0,1]
    float32 fDevDistY_met;   // vehicle ego lateral distance in right corridor
                             // coordinate system,[-15 ... 15]
    float32 fDevHeadingAngle_rad;   // vehicle heading angle in right corridor
                                    // coordinate system, [-0.78539816 ...
                                    // 0.78539816]
    float32 fReplanDevDistY_met;    // deviation in lateral position with the
                                    // distY of last replan triggered if current
                                    // replan is triggered, [-10 ... 10]
    float32 fReplanDevHeading_rad;  // heading angle deviation by replanning,
                                    // [-0.78539816 ... 0.78539816]
    float32 fLeCorridorPosX0_met;   // left corridor longitudinal start position
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
} TRJPLN_LatencyCompensationCorridor_t;

typedef struct {
    TRJPLN_LatencyCompensationCCCOut_t sConsisCheckOut;
    TRJPLN_LatencyCompensationCorriParamOut_t sCompenCorriParam;
    TRJPLN_LatencyCompensationTargetPath_t sTargetPath;
    TRJPLN_LatencyCompensationPlasiCheck_t sPlasiCheck;
    TRJPLN_LatencyCompensationCorridor_t sCompCorridor;
} TRJPLN_LatencyCompensationOutPro_t;

typedef struct {
    float32 fEgoYawAngle_rad;
    float32 fEgoDistX_met;
    float32 fEgoDistY_met;
} TRJPLN_LatencyCompensationEgoMotion_t;

typedef struct {
    float32 fEgoDevPosXSum_met;
    float32 fEgoDevPosYSum_met;
    float32 fDevHeadingEgo_rad;
} TPLLCO_EgoDeviationSum_t;

typedef struct {
    float32 fDevOdoEgoDist_met;
    float32 fDevOdoEgoHead_rad;
} TPLLCO_EgoOdoVeh_t;

typedef struct {
    float32 fX0_met;
    float32 fY0_met;
    float32 fHeading_rad;
    float32 fCurve_1pm;
    float32 fCurveChng_1pm2;
    float32 fLength_met;
} TRJPLN_TrajectoryParams_t;

typedef struct {
    const boolean* bTriggerReplan_nu;
    const float32* fDevHeadingEgo_rad;
    const float32* fCtrlPtDevX_met;
    const float32* fCtrlPtDevY_met;
    const float32* fVehVelX_mps;
    const TPLLCO_EgoOdoVeh_t* sDevOdoVeh;
    const TRJPLN_LatencyCompensationCorriParamOut_t* sCoordParams;
} TRJPLN_SetDevReplanDeltaIn_t;
/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
void LCF_TRJPLN_LatencyCompensation_Exec(
    const TRJPLN_LatencyCompensationInReq_t* reqPorts,
    const TRJPLN_TrajectoryPlanParam_t* paras,
    TRJPLN_LatencyCompensationOutPro_t* proPorts,
    TRJPLN_LatenCompDebug_t* debug);
void LCF_TRJPLN_LatencyCompensation_Reset(void);

void CourseConsistenceCheck(
    const TRJPLN_LatencyCompensationConsisCheck_t* pConsisCheckIn,
    const TRJPLN_LatencyCompensationAllCorridor_t* pAllCorridor,
    const TRJPLN_LatencyCompensationCommon_t* pCommonInput,
    TRJPLN_LatencyCompensationCCCOut_t* pCCCOut,
    TRJPLN_LatencyCompensationEgoMotion_t* pEgoMotion,
    TRJPLN_LatencyCompensationAllCorridor_t* pAllCorridorWithCCC);
void LatencyCompensation(
    const TRJPLN_LatencyCompensationLatComIn_t* pLatCompIn,
    const TRJPLN_LatencyCompensationCommon_t* pCommonInput,
    const TRJPLN_LatencyCompensationEgoMotion_t* pEgoMotion,
    const TRJPLN_LatencyCompensationAllCorridor_t* pAllCorridorWithCCC,
    const TRJPLN_LatencyCompensationConsisCheck_t* pConsisCheckIn,
    TRJPLN_LatencyCompensationCorriParamOut_t* pCompenCorriParam,
    TRJPLN_LatencyCompensationCorridor_t* pCompCorridor);
void CorridorTrjecotryCalc(const float32 fPosX0_met,
                           const float32 fPosY0_met,
                           const float32 fHeadingAgl_rad,
                           const float32 fCurve_1pm,
                           const float32 fCurveChng_1pm2,
                           TRJPLN_LatencyCompensationTargetPath_t* pTargetPath);
void PlausibilityCheck(
    const TRJPLN_LatencyCompensationCorridor_t* pCompCorridor,
    const TRJPLN_LatencyCompensationCorriParamOut_t* pCompenCorriParam,
    const TRJPLN_LatencyCompensationAllCorridor_t* pAllCorridorWithCCC,
    const float32 fEgoVelX_mps,
    TRJPLN_LatencyCompensationPlasiCheck_t* pPlasiCheck);
void EgoMotionCalculation(const float32 fYawrate,
                          const float32 fCycletime,
                          const float32 fVelX,
                          const boolean bCCCReset,
                          TRJPLN_LatencyCompensationEgoMotion_t* pEgoMotion);
void TestPointPositionCal(float32 fTgtTrajLength,
                          float32 fTgtTrajPosX0,
                          float32 fTgtTrajPosY0,
                          float32 fTgtTrajHeading,
                          float32 fTgtTrajCurve,
                          float32 fTgtTrajCurveChng,
                          float32* fTestPointDistX,
                          float32* fTestPointDistY);

void TestPointNewPositionCal(float32 fTestPointDistX,
                             float32 fTestPointDistY,
                             TRJPLN_LatencyCompensationEgoMotion_t sEgoMotion,
                             float32* fCCCTestPointDistX,
                             float32* fCCCTestPointDistY);

void CorridorFrozenCheck(
    boolean bCCCReset,
    TRJPLN_LatencyCompensationAllCorridor_t pAllCorridor,
    float32 fEgoMovedDistX,
    float32* fCCCLength,
    float32* fValidLength,
    TRJPLN_LatencyCompensationAllCorridor_t* pAllCorrdorsReference);

void CCCResetCheck(float32 fCCCLength,
                   float32 fTestNewPointX,
                   TRJPLN_LatencyCompensationConsisCheck_t sLaneData,
                   float32* fCCCPassedPerc,
                   boolean* bCCCReset);

void CCCWarningCheck(
    TRJPLN_LatencyCompensationAllCorridor_t sAllCorrdorsReference,
    float32 fCCCTestPointDistX,
    float32 fCCCTestPointDistY,
    float32 fCCCPassedPerc,
    float32 fValidLength,
    float32* fCCCDeviation,
    boolean* bCCCWarning,
    boolean* bCCCInvalid);

void CalcCurrentVehPosition(
    const TRJPLN_LatencyCompensationLatComIn_t sLatCompIn,
    const TRJPLN_LatencyCompensationCommon_t sCommonInput,
    float32* fMeasDeltaTime_sec,
    TPLLCO_EgoOdoVeh_t* pDevOdoVeh,
    TPLLCO_EgoDeviationSum_t* pDevEgo,
    boolean* bTriggerReplan);
void TranslateRotateCoriCoord(
    const TPLLCO_EgoDeviationSum_t sDevEgo,
    const TRJPLN_LatencyCompensationEgoMotion_t sEgoMotion,
    const TRJPLN_LatencyCompensationAllCorridor_t sAllCorridor,
    float32 fEgoVelX_mps,
    uint8 uiLatCtrlMode_nu,
    TRJPLN_LatencyCompensationCorriParamOut_t* pCompenCorriParam,
    float32 fCtrlPtDev_met[2]);
void LatencyCompensationOutput(
    const TPLLCO_EgoOdoVeh_t sDevOdoVeh,
    float32 fDevHeadingEgo_rad,
    boolean bTriggerReplan,
    uint8 uiSysStateLCF_nu,
    uint8 uiLatCtrlMode_nu,
    const TRJPLN_LatencyCompensationCorriParamOut_t sCompenCorriParam,
    float32 fCtrlPtDev_met[2],
    float32 fEgoVelX_mps,
    const TRJPLN_LatencyCompensationAllCorridor_t sAllCorridor,
    TRJPLN_LatencyCompensationCorridor_t* pCompCorridor);
void TPLLCO_TrajecotryTranslation(const float32 fInputX_met,
                                  const TRJPLN_TrajectoryParams_t sInputCorri,
                                  TRJPLN_TrajectoryParams_t* pOutputCorri);
void TPLLCO_Corridor_TrajectoryEquation(
    const float32 fX_met,
    const TRJPLN_TrajectoryParams_t sInputCorri,
    float32* fOutX0_met,
    float32* fOutY0_met,
    float32* fOutHeading_rad);
void TPLLCO_CoordinateTranformation(const float32 fCridCoordSysPosX_met,
                                    const float32 fCridCoordSysPosY_met,
                                    const float32 fCridCoordSysOrien_rad,
                                    const TRJPLN_TrajectoryParams_t sInputCorri,
                                    const float32 fCorridTrafoLengthMin_met,
                                    TRJPLN_TrajectoryParams_t* pOutCorri);
void TPLLCO_TranjParamsCopyValue(TRJPLN_TrajectoryParams_t sInputCorri,
                                 float32* fX0_met,
                                 float32* fY0_met,
                                 float32* fHeading_rad,
                                 float32* fCurve_1pm,
                                 float32* fCurveChng_1pm2,
                                 float32* fLength_met);
void TPLLCO_SetDevReplanDelta(boolean bTriggerReplanDeviaCal,
                              const TRJPLN_SetDevReplanDeltaIn_t sInputParams,
                              const TRJPLN_TrajectoryParams_t sRightClothoid,
                              uint8 uiLatCtrlMode_nu,
                              float32* fDevDistY_met,
                              float32* fDevHeading_rad,
                              float32* fReplanDistY_met,
                              float32* fReplanDevHeading_rad);
void TPLLCO_SetBit(uint8 uiBitIndex, boolean bBitSetVal, uint8* pTargetBit);
void DistYDeviationPlausibilityCheck(
    const TRJPLN_TrajectoryParams_t sOriginCorridor,
    const TRJPLN_TrajectoryParams_t sLatenCompenCorridor,
    const float32 fRightOrientation_rad,
    const float32 fPosRightX_met,
    const float32 fPosRightY_met,
    const uint8 uiNumForIterator,
    float32 fDeviationDistY_met[4],
    boolean* bOutOfRange);
#ifdef __cplusplus
}
#endif
#endif
