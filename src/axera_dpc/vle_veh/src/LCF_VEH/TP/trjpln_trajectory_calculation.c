/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "trjpln_trajectory_calculation.h"
/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE2_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
static TRJPLN_calOptOutTypeV4_t TRJPLN_Ss_OptTrajectoryOutput_nu = {
    0};  // used in function LCF_TRJPLN_TrajectoryCalc_Exec
static float32 TRJPLN_Sf_PreProcAllowTimeToCorss_sec =
    0.f;  // used in function LCF_TPLTJC_PreProcess
static float32 TRJPLN_Sf_CalcMeasAccuDistance_met =
    0.f;  // used in function TPLTJC_CalcMeasTrajVariable
static float32 TRJPLN_Sf_CalcMeasAccuTime_sec =
    0.f;  // used in function TPLTJC_CalcMeasTrajVariable
static float32 TRJPLN_Sf_CalcTrajPrevTimeHead_sec =
    0.f;  // used in function TPLTJC_CalcTraj
static float32 TRJPLN_Sf_CalcTrajPrevTimeCrve_sec =
    0.f;  // used in function TPLTJC_CalcTraj
static float32 TRJPLN_Sf_CalcTrajDelayVehGui_sec =
    0.f;  // used in function TPLTJC_CalcTraj
static float32 TRJPLN_Sf_CalcTrajDStart_nu =
    0.f;  // used in function TPLTJC_CalcTraj
static float32 TRJPLN_Sa_TJCPostProcTrajParam_nu[6] = {
    0};  // used in function LCF_TPLTJC_PostProcess
static float32 TRJPLN_Sf_TJCPostProcTimeTrajEnd_sec =
    0.f;  // used in function LCF_TPLTJC_PostProcess
static float32 TRJPLN_Sf_TJCPostProceArcLengthTrajEnd_met =
    0.f;  // used in function LCF_TPLTJC_PostProcess
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  Functionname: LCF_TRJPLN_TrajectoryCalc_Exec */ /*!

                             @brief: optimal trajectory calculation

                             @description:The task of the trajectory planning
                             module is to bring the vehicle
                             from a start
                             state into an end state while considering the
                             following criteria:
                             comfort (i.e., jerk optimal),
                             no detour (optimal distance), speed (time optimal).

                             @param[in]
                             const TRJPLN_TrajectoryCalcInReq_t* reqPorts:
                             optimal trajectory
                             calculated module input
                             const TRJPLN_TrajectoryPlanParam_t* paras: system
                             parameters
                             @param[out]
                             TRJPLN_TrajectoryCalcOutPro_t* proPorts: optimal
                             trajectory
                             calculated result

                             @return
                             @uml
                             @startuml
                             start
                             :LCF_TPLTJC_PreProcess;
                             note:input data mapping for \noptimal trajectory
                             calculation
                             if (replan is triggered) then (yes)
                             :TrajCalc_CalcOptTrajectory;
                             note:Calculation of the trajectory parameters,
                             \nCalculation of the cost
                             function, \nCheck restrictions, Select the best
                             trajectory
                             endif
                             :LCF_TPLTJC_PostProcess;
                             note:calculated optimal trajectory parameters
                             \nmapping to output structure
                             :LCF_TPLTJC_DebugOutput;
                             note:output current module's output to \ndatalogger
                             debug structure
                             end
                             @enduml
                             *****************************************************************************/
void LCF_TRJPLN_TrajectoryCalc_Exec(
    const TRJPLN_TrajectoryCalcInReq_t* reqPorts,
    const TRJPLN_TrajectoryPlanParam_t* paras,
    TRJPLN_TrajectoryCalcOutPro_t* proPorts,
    TRJPLN_TrajecotryCalcDebug_t* debug) {
    TRJPLN_calOptInTypeV3_t TRJPLN_Ss_OptTrajectoryInput_nu = {0};

    // input data mapping for optimal trajectory calculation
    LCF_TPLTJC_PreProcess(
        *reqPorts, paras, proPorts->fTrajDistY_met,
        proPorts->fTrajDistY1stDeriv_mps, proPorts->fTrajDistY2ndDeriv_mps2,
        &TRJPLN_Ss_OptTrajectoryInput_nu, &proPorts->fWeightTargetDistY_nu,
        &proPorts->fWeightEndTime_nu);

    if (reqPorts->bTrigTrajReplan) {
        // Calculation of the trajectory parameters, Calculation of the cost
        // function, Check restrictions, Select the best trajectory
        TRJPLN_Ss_OptTrajectoryOutput_nu =
            TrajCalc_CalcOptTrajectory(&TRJPLN_Ss_OptTrajectoryInput_nu);
    }

    // calculated optimal trajectory parameters mapping to output structure
    LCF_TPLTJC_PostProcess(TRJPLN_Ss_OptTrajectoryOutput_nu, reqPorts,
                           proPorts);

    // output to debug
    LCF_TPLTJC_DebugOutput(*proPorts, debug);
}

/*****************************************************************************
  Functionname: LCF_TPLTJC_DebugOutput */ /*!

                                     @brief: debug output wrapper

                                     @description:

                                     @param[in]

                                     @param[out]

                                     @return
                                     *****************************************************************************/
void LCF_TPLTJC_DebugOutput(TRJPLN_TrajectoryCalcOutPro_t proPorts,
                            TRJPLN_TrajecotryCalcDebug_t* debug) {
    debug->fTrajDistY_met = proPorts.fTrajDistY_met;
    debug->fTrajDistY1stDeriv_mps = proPorts.fTrajDistY1stDeriv_mps;
    debug->fTrajDistY2ndDeriv_mps2 = proPorts.fTrajDistY2ndDeriv_mps2;
    debug->fTrajDistY3rdDeriv_mps3 = proPorts.fTrajDistY3rdDeriv_mps3;
    debug->uiQuStatusTrajPlan_nu = proPorts.uiQuStatusTrajPlan_nu;
    memcpy(debug->afTrajParam_nu, proPorts.afTrajParam_nu,
           sizeof(proPorts.afTrajParam_nu));
    debug->bTrajEnd = proPorts.bTrajEnd;
    debug->bLengthOK = proPorts.bLengthOK;
    debug->bMatrixInverseOK = proPorts.bMatrixInverseOK;
    debug->fEndPointTrajectory_nu = proPorts.fEndPointTrajectory_nu;
    debug->fPassedTrajLenPercent_per = proPorts.fPassedTrajLenPercent_per;
    debug->fMaxJerkTraj_mps3 = proPorts.fMaxJerkTraj_mps3;
    debug->bMaxJerkOK = proPorts.bMaxJerkOK;
    debug->fMaxAclTraj_mps2 = proPorts.fMaxAclTraj_mps2;
    debug->fOptimalCost_nu = proPorts.fOptimalCost_nu;
    debug->fWeightTargetDistY_nu = proPorts.fWeightTargetDistY_nu;
    debug->fWeightEndTime_nu = proPorts.fWeightEndTime_nu;
}

/*****************************************************************************
  Functionname: LCF_TPLTJC_PreProcess */ /*!

                                      @brief: pre-process of trajectory
                                      calculation module

                                      @description: 1. select ego data based on
                                      replan trigger signal;
                                                      2. cost function factor
                                      calculate based on lookup result
                                      with input parameters;
                                                      3. max accleration
                                      threshold calculate;
                                                      4. other input data
                                      mapping;

                                      @param[in]
                                                      const
                                      TRJPLN_TrajectoryCalcInReq_t
                                      sTrajCalcModuleInput: the
                                      input of trajectory calcualtion module
                                                      const
                                      TRJPLN_TrajectoryPlanParam_t* paras:
                                      system parameters
                                                      float32 fLastTrajDistY:
                                      the trajectory DistY output data of
                                      ego in last cycle
                                                      float32 fLastTrajDistY1st:
                                      the trajectory VelY output data
                                      of ego in last cycle
                                                      float32 fLastTrajDistY2nd:
                                      the trajectory AccelY output data
                                      of ego in last cycle
                                      @param[out]
                                                      TRJPLN_calOptInTypeV3_t*
                                      pCalcOptTrajInput: the input
                                      structure of optimal trajectory
                                      calculation function
                                                      float32*
                                      fWeightTgtDistY_nu: the DistY relate
                                      weight factor
                                      of cost function
                                                      float32*
                                      fWeightEndTime_nu: the EndTime relate
                                      weight factor
                                      of cost function

                                      @return
                                      *****************************************************************************/
void LCF_TPLTJC_PreProcess(
    const TRJPLN_TrajectoryCalcInReq_t sTrajCalcModuleInput,
    const TRJPLN_TrajectoryPlanParam_t* paras,
    float32 fLastTrajDistY,
    float32 fLastTrajDistY1st,
    float32 fLastTrajDistY2nd,
    TRJPLN_calOptInTypeV3_t* pCalcOptTrajInput,
    float32* fWeightTgtDistY_nu,
    float32* fWeightEndTime_nu) {
    float32 fDistYInit_met = 0.f;
    float32 fDistY1stDerivInit_nu = 0.f;
    float32 fDistY2ndDerivInit_nu = 0.f;
    float32 fAclPotVecMax_mps2 = 0.f;

    // update ego distance while current/target replan is triggered
    // or the intput value would be the last same input value of last cycle
    if (sTrajCalcModuleInput.bReplanCurValues) {
        fDistYInit_met = sTrajCalcModuleInput.fCurDistY_met;
        fDistY1stDerivInit_nu = sTrajCalcModuleInput.fCurDistY1stDeriv_mps;
        fDistY2ndDerivInit_nu = sTrajCalcModuleInput.fCurDistY2ndDeriv_mps2;
    } else {
        if (sTrajCalcModuleInput.bReplanTgtValues) {
            fDistYInit_met = sTrajCalcModuleInput.fTrajDistYPrev_met;
            fDistY1stDerivInit_nu =
                sTrajCalcModuleInput.fTrajDistY1stToPrev_mps;
            fDistY2ndDerivInit_nu =
                sTrajCalcModuleInput.fTrajDistY2ndToPrev_mps2;
        } else {
            fDistYInit_met = fLastTrajDistY;
            fDistY1stDerivInit_nu = fLastTrajDistY1st;
            fDistY2ndDerivInit_nu = fLastTrajDistY2nd;
        }
    }
    LCF_TPLTJC_Timer(sTrajCalcModuleInput.fCycleTimeVeh_sec,
                     sTrajCalcModuleInput.bReplanCurValues,
                     TPLTJC_ALLOWEDTIMETOCROSS_SEC,
                     &TRJPLN_Sf_PreProcAllowTimeToCorss_sec);

    // calculate the weight factor(0~20/0~45) of DistY and EndTime based on the
    // LookUp result with input factor value(0%~100%)
    if (TPLTJC_PARAMCOSTFCTINTERNAL_NU) {
        *fWeightTgtDistY_nu = TPLTJC_WEIGHTTGTDISTY_NU;

        float32 afTableInputX1[TPLTJC_WEIGHTENDTIME_TABLENUM_NU] =
            TPLTJC_PLANNINGHORIZON_TABLEX_SEC;
        float32 afTableInputY1[TPLTJC_WEIGHTENDTIME_TABLENUM_NU] =
            TPLTJC_WEIGHTENDTIME_TABLEY_NU;
        *fWeightEndTime_nu = TUE_CML_LookUpTable2D(
            sTrajCalcModuleInput.fTrajPlanningHorizon_sec, afTableInputX1,
            afTableInputY1, TPLTJC_WEIGHTENDTIME_TABLENUM_NU);
    } else {
        float32 afTableInputX1[TPLTJC_WGHTTGTDISTYCUSTFCT_TABLENUM_NU] =
            TPLTJC_WGHTTGTDISTYCUSTFCT_TABLEX_NU;
        float32 afTableInputY1[TPLTJC_WGHTTGTDISTYCUSTFCT_TABLENUM_NU] =
            TPLTJC_WGHTTGTDISTYCHARACT_TABLEY_NU;
        *fWeightTgtDistY_nu = TUE_CML_LookUpTable2D(
            sTrajCalcModuleInput.fWeightTgtDistY_nu, afTableInputX1,
            afTableInputY1, TPLTJC_WGHTTGTDISTYCUSTFCT_TABLENUM_NU);

        float32 afTableInputX2[TPLTJC_WGHTTGTTIMECHARACT_TABLENUM_NU] =
            TPLTJC_WGHTTGTTIMECUSTFCT_TABLEX_NU;
        float32 afTableInputY2[TPLTJC_WGHTTGTTIMECHARACT_TABLENUM_NU] =
            TPLTJC_WGHTTGTTIMECHARACT_TABLEY_NU;
        *fWeightEndTime_nu = TUE_CML_LookUpTable2D(
            sTrajCalcModuleInput.fWeightEndTime_nu, afTableInputX2,
            afTableInputY2, TPLTJC_WGHTTGTTIMECHARACT_TABLENUM_NU);
    }
    // calculate the max accleration threshold based on the LookUp result with
    // ego velocity
    float32 afTableInputX[TPLTJC_MAXACLY_TABLENUM_NU] =
        TPLTJC_VEHVELX_TABLEX_MPS;
    float32 afTableInputY[TPLTJC_MAXACLY_TABLENUM_NU] =
        TPLTJC_MAXACLY_TABLEY_MPS2;
    fAclPotVecMax_mps2 =
        TUE_CML_LookUpTable2D(sTrajCalcModuleInput.fEgoVelX_mps, afTableInputX,
                              afTableInputY, TPLTJC_MAXACLY_TABLENUM_NU);
    fAclPotVecMax_mps2 =
        MIN(fAclPotVecMax_mps2, sTrajCalcModuleInput.fFTireAclMax_mps2);

    // output process
    pCalcOptTrajInput->fDistYInit_nu = fDistYInit_met;
    pCalcOptTrajInput->fDistY1stDerivInit_nu = fDistY1stDerivInit_nu;
    pCalcOptTrajInput->fDistY2ndDerivInit_nu = fDistY2ndDerivInit_nu;
    pCalcOptTrajInput->fKd_nu = *fWeightTgtDistY_nu;
    pCalcOptTrajInput->fKt_nu = *fWeightEndTime_nu;
    pCalcOptTrajInput->fTrajCalcEgoVelX_mps = sTrajCalcModuleInput.fEgoVelX_mps;
    memcpy(pCalcOptTrajInput->afTrajCalcTgtDistY_met,
           sTrajCalcModuleInput.afTargetDistY_met,
           sizeof(pCalcOptTrajInput->afTrajCalcTgtDistY_met));
    memcpy(pCalcOptTrajInput->afTrajCalcTgtDistY1stDeriv_nu,
           sTrajCalcModuleInput.fTargetDistY1stDeriv_mps,
           sizeof(pCalcOptTrajInput->afTrajCalcTgtDistY1stDeriv_nu));
    memcpy(pCalcOptTrajInput->afrajCalcTgtDistY2ndDeriv_nu,
           sTrajCalcModuleInput.fTargetDistY2ndDeriv_mps2,
           sizeof(pCalcOptTrajInput->afrajCalcTgtDistY2ndDeriv_nu));
    pCalcOptTrajInput->fLatCtrlCoordCridrRightSeg1_Crv_1pm =
        sTrajCalcModuleInput.fRiCorridorCurve_1pm;
    pCalcOptTrajInput->fTolDistYRight_met =
        sTrajCalcModuleInput.fDistYToRiTgtArea_met;
    pCalcOptTrajInput->fTolDistYLeft_met =
        sTrajCalcModuleInput.fDistYToLeTgtArea_met;
    // Each trajectory has to be in compliance with the maximum actionable
    // acceleration (FTIRE_CHECK).
    // The bits of the qualifier(TrajServiceQualifier_nu) are checked.
    pCalcOptTrajInput->uiCheckFtire_nu =
        TRJPLN_GetBit(sTrajCalcModuleInput.uiTrajPlanServQu_nu, (0x01 << (2)));
    pCalcOptTrajInput->uiCheckCridrBoundaries_nu =
        TRJPLN_GetBit(sTrajCalcModuleInput.uiTrajPlanServQu_nu, (0x01 << (0)));
    pCalcOptTrajInput->uiCheckObjects_nu =
        TRJPLN_GetBit(sTrajCalcModuleInput.uiTrajPlanServQu_nu, (0x01 << (1)));
    pCalcOptTrajInput->uiCheckJerk_nu =
        TRJPLN_GetBit(sTrajCalcModuleInput.uiTrajPlanServQu_nu, (0x01 << (3)));
    pCalcOptTrajInput->uiCheckLaneCross_nu =
        TRJPLN_GetBit(sTrajCalcModuleInput.uiTrajPlanServQu_nu, (0x01 << (4)));
    pCalcOptTrajInput->fAclPotVecMin_mps2 =
        TPLTJC_FTIREACLMINSCALING_NU * sTrajCalcModuleInput.fFTireAclMin_mps2;
    pCalcOptTrajInput->fAclPotVecMax_mps2 = fAclPotVecMax_mps2;
    memcpy(pCalcOptTrajInput->afteTrajCalcTgt_nu,
           sTrajCalcModuleInput.afTargetPoints_nu,
           sizeof(pCalcOptTrajInput->afteTrajCalcTgt_nu));
    pCalcOptTrajInput->uiTrajCalcNumTgtPoints_nu =
        sTrajCalcModuleInput.uiNumOfTgtPoints_nu;
    // The ego vehicle has a rectangular base and is approximated by N
    // overlapping equal-sized circles.
    // The number of circles to be used is set via the parameter
    // TPLTJC_NUMCIRCLES_NU(default value 4).
    // The radiusand the distance between circles is calculated here.
    float32 fTemp_nu =
        (SQR(paras->fEgoVehWidth_met) / 16.f) +
        (SQR(paras->fEgoVehLength_met) / SQR(TPLTJC_NUMCIRCLES_NU));
    pCalcOptTrajInput->fTrajCalcCircleRadius_met = SQRT(fTemp_nu);
    pCalcOptTrajInput->fTrajCalcCircleDistance_met =
        SQRT(fTemp_nu - (SQR(paras->fEgoVehWidth_met) / 16.f));
    pCalcOptTrajInput->fTrajCalcCridrRightSeg1_Length_met =
        sTrajCalcModuleInput.fRiCorridorLength_met;
    pCalcOptTrajInput->fTrajCalcCridrLeftSeg1_Length_met =
        sTrajCalcModuleInput.fLeCorridorLength_met;
    pCalcOptTrajInput->fObstacle_DistX_met =
        sTrajCalcModuleInput.fObstacleDistX_met;
    pCalcOptTrajInput->fObstacle_DistY_met =
        sTrajCalcModuleInput.fObstacleDistY_met;
    pCalcOptTrajInput->fObstacle_Width_met =
        sTrajCalcModuleInput.fObstacleWidth_met;
    memcpy(pCalcOptTrajInput->afTrajCalcDistYLeft_met,
           sTrajCalcModuleInput.afLeDistY_met,
           sizeof(pCalcOptTrajInput->afTrajCalcDistYLeft_met));
    pCalcOptTrajInput->uiReplanModeArcLength_nu =
        sTrajCalcModuleInput.bReplanModeArcLength;
    pCalcOptTrajInput->fWeightArcLengthPlan_nu = TPLTJC_WEIGHTDISTBASED_NU;
    pCalcOptTrajInput->fTrajCalcPlanHorizonTraj_sec =
        sTrajCalcModuleInput.fTrajPlanningHorizon_sec;
    pCalcOptTrajInput->fTrajCalcPlanHorizonVisibility_sec =
        sTrajCalcModuleInput.fPlanHorizonVisRange_sec;
    pCalcOptTrajInput->fTrajCalcTrajVel_mps =
        sTrajCalcModuleInput.fTrajVelRefCurve_mps;
    pCalcOptTrajInput->fTrajCalcTrajAcl_mps2 =
        sTrajCalcModuleInput.fTrajAclRefCurve_mps2;
    pCalcOptTrajInput->fTrajCalcDeadTime_sec =
        sTrajCalcModuleInput.fDelayVehGui_sec;
    pCalcOptTrajInput->uiTrajCalcNCircles_nu = TPLTJC_NUMCIRCLES_NU;
    pCalcOptTrajInput->uiTrajCalcNumCridrPointsLeft_nu =
        sTrajCalcModuleInput.uiNumOfPointsCridrLeft_nu;
    pCalcOptTrajInput->fTrajCalcObjVelX_mps =
        sTrajCalcModuleInput.fObstacleVelX_mps;
    pCalcOptTrajInput->fTrajCalcObjAclX_mps2 =
        sTrajCalcModuleInput.fObstacleAccelX_mps2;
    pCalcOptTrajInput->fCrvChng_1pm2 =
        sTrajCalcModuleInput.fRiCorridorCrvChng_1pm2;
    pCalcOptTrajInput->fAllwdJerkMax_mps3 =
        sTrajCalcModuleInput.fMaxJerkAllowed_mps3;
    pCalcOptTrajInput->fLaneWidth_met =
        sTrajCalcModuleInput.fLeCorridorPosY0_met;
    pCalcOptTrajInput->fVehicleWidth_met = paras->fEgoVehWidth_met;
    pCalcOptTrajInput->fAllowedTimetoCross_sec =
        TRJPLN_Sf_PreProcAllowTimeToCorss_sec;
}

/*****************************************************************************
  Functionname: LCF_TPLTJC_Timer                                           */ /*!

     @brief: timer util function

     @description: remain time reduce while reset is not triggered

     @param[in]
                           float32 fDeltaTime_sec: cycle time which would be
   reduced every invoke
                           boolean bReset: out remain time would be set to
   default
   time while reset is TRUE
                           float32 fDefaultTime_sec: default remain time while
   reset triggered
     @param[out]
                           float32* fRemainTime_sec: remain time of timer,
   always
   positive
     @return
   *****************************************************************************/
void LCF_TPLTJC_Timer(float32 fDeltaTime_sec,
                      boolean bReset,
                      float32 fDefaultTime_sec,
                      float32* fRemainTime_sec) {
    if (bReset) {
        *fRemainTime_sec = fDefaultTime_sec;
    } else {
        *fRemainTime_sec -= fDeltaTime_sec;
    }
    *fRemainTime_sec = MAX(0.f, *fRemainTime_sec);
}

/*****************************************************************************
  Functionname: LCF_TPLTJC_PostProcess */ /*!

                                     @brief: validation and output process based
                                     on the calculated optimal trajectory

                                     @description:
                                                   1. calculate ego moved
                                     distance/time since last replan
                                     triggered
                                                   2. optimal trajectory
                                     calculate validation assignment
                                                   3. trajectory end check and
                                     trajectory passed percent
                                     calculate
                                                   4. convert calculated fifth
                                     order polynomial to lateral
                                     distance,velocity parameter

                                     @param[in]
                                                   const
                                     TRJPLN_calOptOutTypeV4_t
                                     sCalculatedOptTraj: calculated
                                     optimal trajectory with fifth order
                                     polynomial parameters
                                                   const
                                     TRJPLN_TrajectoryCalcInReq_t* reqPorts:
                                     trajectory
                                     calculation input data

                                     @param[out]
                                                   TRJPLN_TrajectoryCalcOutPro_t*
                                     pTrajecoryCalcModulOut:
                                     trajectory calculation output structure
                                     @return
                                     @uml
                                     @startuml
                                     start
                                     :TPLTJC_CalcMeasTrajVariable;
                                     note:ego accumulate move distance/time\n
                                     since last replan triggered
                                     if(trajectory calculation successful) then
                                     (yes)
                                     :update best trajectory value;
                                     endif
                                     :set trajectory calculate status;
                                     :TPLTJC_CalcTraj;
                                     note:convert calculated Fifth order
                                     polynomial's\n parameters to frenet's
                                     parameters, \nsuch as DistY, DistY
                                     derivation ...
                                     :TPLTJC_CalcPassedTrajLenPercent;
                                     note:calculate how much of DistY of the
                                     \ntrajectory has already been realized.
                                     \nThis value is a percentage.
                                     end
                                     @enduml
                                     *****************************************************************************/
void LCF_TPLTJC_PostProcess(
    const TRJPLN_calOptOutTypeV4_t sCalculatedOptTraj,
    const TRJPLN_TrajectoryCalcInReq_t* reqPorts,
    TRJPLN_TrajectoryCalcOutPro_t* pTrajecoryCalcModulOut) {
    float32 fMeasTrajVariable_nu = 0.f;
    float32 fDStart_nu = 0;

    // ego accumulate move distance/time since last replan triggered
    TPLTJC_CalcMeasTrajVariable(
        reqPorts->bTrigTrajReplan, reqPorts->fCycleTimeVeh_sec,
        reqPorts->fTrajVelRefCurve_mps, reqPorts->bReplanModeArcLength,
        &fMeasTrajVariable_nu);

    pTrajecoryCalcModulOut->bMatrixInverseOK =
        sCalculatedOptTraj.uiMatrixInverseOK_nu;
    // update best trajectory parameters if optimal trajectory calculation
    // successful
    if (pTrajecoryCalcModulOut->bMatrixInverseOK) {
        memcpy(TRJPLN_Sa_TJCPostProcTrajParam_nu,
               sCalculatedOptTraj.afTrajParam_nu,
               sizeof(sCalculatedOptTraj.afTrajParam_nu));
        TRJPLN_Sf_TJCPostProcTimeTrajEnd_sec =
            sCalculatedOptTraj.fTrajCalcTimeTrajEnd_sec;
        TRJPLN_Sf_TJCPostProceArcLengthTrajEnd_met =
            sCalculatedOptTraj.fArcLengthTrajEnd_met;
    }
    memcpy(pTrajecoryCalcModulOut->afTrajParam_nu,
           &TRJPLN_Sa_TJCPostProcTrajParam_nu,
           sizeof(TRJPLN_Sa_TJCPostProcTrajParam_nu));
    // optimal trajectory calculation validation
    TPLTJC_SetBit(0u, !(sCalculatedOptTraj.uiFtireMinOK_nu),
                  &pTrajecoryCalcModulOut->uiQuStatusTrajPlan_nu);
    TPLTJC_SetBit(1u, !(sCalculatedOptTraj.uiFtireMaxOK_nu),
                  &pTrajecoryCalcModulOut->uiQuStatusTrajPlan_nu);
    TPLTJC_SetBit(2u, !(sCalculatedOptTraj.uiCollDetRightOK_nu),
                  &pTrajecoryCalcModulOut->uiQuStatusTrajPlan_nu);
    TPLTJC_SetBit(3u, !(sCalculatedOptTraj.uiCollDetLeftOK_nu),
                  &pTrajecoryCalcModulOut->uiQuStatusTrajPlan_nu);
    TPLTJC_SetBit(4u, !(sCalculatedOptTraj.uiCollDetObjOK_nu),
                  &pTrajecoryCalcModulOut->uiQuStatusTrajPlan_nu);
    TPLTJC_SetBit(5u, !(sCalculatedOptTraj.uiMatrixInverseOK_nu),
                  &pTrajecoryCalcModulOut->uiQuStatusTrajPlan_nu);
    TPLTJC_SetBit(6u, !(sCalculatedOptTraj.uiTrajLengthOK_nu),
                  &pTrajecoryCalcModulOut->uiQuStatusTrajPlan_nu);
    TPLTJC_SetBit(7u, !(sCalculatedOptTraj.uiMaxJerkOK_nu),
                  &pTrajecoryCalcModulOut->uiQuStatusTrajPlan_nu);
    TPLTJC_SetBit(8u, !(sCalculatedOptTraj.uiLaneCrossOK_nu),
                  &pTrajecoryCalcModulOut->uiQuStatusTrajPlan_nu);

    pTrajecoryCalcModulOut->bLengthOK = sCalculatedOptTraj.uiTrajLengthOK_nu;
    pTrajecoryCalcModulOut->bMaxJerkOK = sCalculatedOptTraj.uiMaxJerkOK_nu;
    pTrajecoryCalcModulOut->fMaxJerkTraj_mps3 =
        sCalculatedOptTraj.fMaxJerkTraj_mps3;
    pTrajecoryCalcModulOut->fMaxAclTraj_mps2 =
        sCalculatedOptTraj.fMaxAclTraj_mps2;
    pTrajecoryCalcModulOut->fOptimalCost_nu =
        sCalculatedOptTraj.fOptimalCost_nu;

    // the distance of last planned point from ego car
    pTrajecoryCalcModulOut->fEndPointTrajectory_nu =
        reqPorts->bReplanModeArcLength
            ? TRJPLN_Sf_TJCPostProceArcLengthTrajEnd_met
            : TRJPLN_Sf_TJCPostProcTimeTrajEnd_sec;
    // check whether the planned trajectory is finished based on ego accumulate
    // moved distance
    pTrajecoryCalcModulOut->bTrajEnd =
        (fMeasTrajVariable_nu >=
         (pTrajecoryCalcModulOut->fEndPointTrajectory_nu -
          reqPorts->fCycleTimeVeh_sec));

    const TPLTJC_LateralTrajecotryCalIn_t sCalcTrajInput = {
        &fMeasTrajVariable_nu,               // fMeasTrajVariable_nu;
        &reqPorts->bTrigCustFctActn,         // bTrigCustFctActn_nu;
        &reqPorts->bReplanModeArcLength,     // bReplanModeArcLength_nu;
        &reqPorts->fDelayVehGui_sec,         // fDelayVehGui_sec;
        &reqPorts->fPreviewTimeHeading_sec,  // fPreviewTimeHeading_sec;
        &reqPorts->fPredictionTimeCrv_sec,   // fPredictionTimeCrv_sec;
        &reqPorts->bTrigTrajReplan,          // bTrigTrajReplan_nu;
        &sCalculatedOptTraj.fTrajCalcTimeTrajEnd_sec  // fTimeTrajEnd_sec;
    };
    TPLTJC_LateralTrajecotryCalOut_t sCalcTrajOutput = {
        &fDStart_nu,                              // fDStart_nu;
        &pTrajecoryCalcModulOut->fTrajDistY_met,  // fTrajDistY_met;
        &pTrajecoryCalcModulOut
             ->fTrajDistY1stDeriv_mps,  // fTrajDistY1stDeriv_mps;
        &pTrajecoryCalcModulOut
             ->fTrajDistY2ndDeriv_mps2,  // fTrajDistY2ndDeriv_mps2;
        &pTrajecoryCalcModulOut
             ->fTrajDistY3rdDeriv_mps3,  // fTrajDistY3rdDeriv_mps3;
        &pTrajecoryCalcModulOut
             ->fYDtTrjFmHeadPrev_mps,  // fYDtTrjFmHeadPrev_mps;
        &pTrajecoryCalcModulOut
             ->fYDt2TrjFmKpPrevDT_mps2,  // fYDt2TrjFmKpPrevDT_mps2;
        &pTrajecoryCalcModulOut
             ->fYDt3TrjFmKpPrevDT_mps3,  // fYDt3TrjFmKpPrevDT_mps3;
        &pTrajecoryCalcModulOut
             ->fYD2TrjFmKpPrev_mps2,                   // fYDt2TrjFmKpPrev_mps2;
        &pTrajecoryCalcModulOut->fTrajDistYFmPrev_met  // fTrajDistYFmPrev_met;
    };

    // convert calculated Fifth order polynomial's parameters to frenet's
    // parameters, such as DistY, DistY derivation ...
    TPLTJC_CalcTraj(sCalcTrajInput, sCalculatedOptTraj.afTrajParam_nu,
                    sCalcTrajOutput);

    // to calculate how much of DistY of the trajectory has already been
    // realized. This value is a percentage.
    TPLTJC_CalcPassedTrajLenPercent(
        pTrajecoryCalcModulOut->fEndPointTrajectory_nu,
        sCalculatedOptTraj.afTrajParam_nu, fDStart_nu,
        pTrajecoryCalcModulOut->fTrajDistY_met,
        &pTrajecoryCalcModulOut->fPassedTrajLenPercent_per);
}

/*****************************************************************************
  Functionname:TPLTJC_CalcPassedTrajLenPercent */ /*!

                             @brief: Calculation of the percentage of the
                             trajectory length that has been
                             passed

                             @description:The interface shall be used to
                             propagate how much of DistY of the
                             trajectory
                             has already been realized. This value is a
                             percentage.


                             @param[in]
                             float32 fEndPointTrajectory_nu: end of trajectory
                             const float32 afTrajParam_nu[6]: Fifth order
                             polynomial of calculated
                             trajectory
                             float32 fDStart_nu: start of trajectory
                             float32 fDT_nu: current ego distanceY of calculated
                             trajectory
                             @param[out]
                             float32* fPassedTrajLenPercent_per: passed percent
                             of calculated
                             trajectory in lateral dimention
                             @return
                             *****************************************************************************/
void TPLTJC_CalcPassedTrajLenPercent(float32 fEndPointTrajectory_nu,
                                     const float32 afTrajParam_nu[6],
                                     float32 fDStart_nu,
                                     float32 fDT_nu,
                                     float32* fPassedTrajLenPercent_per) {
    const float32 fT_a = fEndPointTrajectory_nu;
    const float32 fT_a2 = fT_a * fT_a;
    const float32 fT_a3 = fT_a * fT_a2;
    const float32 fT_a4 = fT_a * fT_a3;
    const float32 fT_a5 = fT_a * fT_a4;

    // TrajPlanTrajCalc_RatioTrajTrajGui_nu is a percentage and shall be
    // calculated as follows: TrajPlanTrajCalc_RatioTrajTrajGui_nu=(d(t) - p0) /
    // (d(te) - p0)
    // p0=d_start_nu d(t) is the current lateral deviation of the generated
    // trajectory p0 is the first parameter of the generated trajectory . This
    // corresponds
    // to the initial lateral deviation of the generated trajectory ( for t = 0
    // ) d(te) is the lateral deviation at the end of the trajectory at the time
    // te = FasQcBplBtb_tTrajektorieEnde

    // d_start_nu = corresponds to the initial lateral deviation of the
    // generated trajectory ( t = 0 ) when activating the customer function .
    // That is when you activate the customer function
    float32 fD_Te_nu = TPLTJC_CalcTrajDistYEquation(
        afTrajParam_nu[0], afTrajParam_nu[1], afTrajParam_nu[2],
        afTrajParam_nu[3], afTrajParam_nu[4], afTrajParam_nu[5], fT_a, fT_a2,
        fT_a3, fT_a4, fT_a5);

    *fPassedTrajLenPercent_per =
        fABS((fDT_nu - fDStart_nu) / SafeDiv(fD_Te_nu - fDStart_nu));
}

/*****************************************************************************
  Functionname:TPLTJC_CalcTraj                                            */ /*!

        @brief:generate lateral trajectory

        @description:Determine the the lateral distance and its derivatives for
      the
                              trajectory based on the parameter vector of the
      preceeding subsystem


        @param[in]
                              const TPLTJC_LateralTrajecotryCalIn_t sInput:
      related
      time to be the input of equation
                              const float32 afTrajParam_nu[6]: Fifth order
      polynomial of calculated optimal trajectory

        @param[out]
                              TPLTJC_LateralTrajecotryCalOut_t sOutput:
      calculated
      trajectory parameters
        @return
      *****************************************************************************/
void TPLTJC_CalcTraj(const TPLTJC_LateralTrajecotryCalIn_t sInput,
                     const float32 afTrajParam_nu[6],
                     TPLTJC_LateralTrajecotryCalOut_t sOutput) {
    if (*sInput.bTrigTrajReplan_nu) {
        TRJPLN_Sf_CalcTrajPrevTimeHead_sec = *sInput.fPreviewTimeHeading_sec;
        TRJPLN_Sf_CalcTrajPrevTimeCrve_sec = *sInput.fPredictionTimeCrv_sec;
        TRJPLN_Sf_CalcTrajDelayVehGui_sec = *sInput.fDelayVehGui_sec;
    }
    float32 fTempB = 0.f;
    float32 fTempC = 0.f;
    float32 fTempD = 0.f;

    if (*sInput.bReplanModeArcLength_nu) {
        fTempB = *sInput.fMeasTrajVariable_nu;
        fTempC = *sInput.fMeasTrajVariable_nu;
        fTempD = *sInput.fMeasTrajVariable_nu;
    } else {
        fTempB =
            TRJPLN_Sf_CalcTrajPrevTimeHead_sec + *sInput.fMeasTrajVariable_nu;
        fTempC = TRJPLN_Sf_CalcTrajPrevTimeCrve_sec +
                 *sInput.fMeasTrajVariable_nu +
                 TRJPLN_Sf_CalcTrajDelayVehGui_sec;
        fTempD =
            TRJPLN_Sf_CalcTrajPrevTimeCrve_sec + *sInput.fMeasTrajVariable_nu;
        if (TPLTJC_TIMETRAJENDENABLE_NU) {
            fTempB = fTempB <= *sInput.fTimeTrajEnd_sec
                         ? fTempB
                         : *sInput.fTimeTrajEnd_sec;
            fTempC = fTempC <= *sInput.fTimeTrajEnd_sec
                         ? fTempC
                         : *sInput.fTimeTrajEnd_sec;
            fTempD = fTempD <= *sInput.fTimeTrajEnd_sec
                         ? fTempD
                         : *sInput.fTimeTrajEnd_sec;
        }
    }

    // prepare the equation factor calculation
    const float32 fP0 = afTrajParam_nu[0];
    const float32 fP1 = afTrajParam_nu[1];
    const float32 fP2 = afTrajParam_nu[2];
    const float32 fP3 = afTrajParam_nu[3];
    const float32 fP4 = afTrajParam_nu[4];
    const float32 fP5 = afTrajParam_nu[5];

    // Time t_a: no dead time, no preview
    const float32 fT_a = *sInput.fMeasTrajVariable_nu;
    const float32 fT_a2 = fT_a * fT_a;
    const float32 fT_a3 = fT_a * fT_a2;
    const float32 fT_a4 = fT_a * fT_a3;
    const float32 fT_a5 = fT_a * fT_a4;

    // Time t_b: no dead time, with preview time for heading
    const float32 fT_b = fTempB;
    const float32 fT_b2 = fT_b * fT_b;
    const float32 fT_b3 = fT_b * fT_b2;
    const float32 fT_b4 = fT_b * fT_b3;
    const float32 fT_b5 = fT_b * fT_b4;

    // Time t_c: with dead time, with preview time for curvature
    const float32 fT_c = fTempC;
    const float32 fT_c2 = fT_c * fT_c;
    const float32 fT_c3 = fT_c * fT_c2;

    // Time t_d: no dead time, with preview time for curvature
    const float32 fT_d = fTempD;
    const float32 fT_d2 = fT_d * fT_d;
    const float32 fT_d3 = fT_d * fT_d2;

    if (*sInput.bTrigCustFctActn_nu) {
        TRJPLN_Sf_CalcTrajDStart_nu = fP0;
    }

    *sOutput.fDStart_nu = TRJPLN_Sf_CalcTrajDStart_nu;

    // Time t_a: no dead time, no preview
    // Time t_b: no dead time, with preview time for heading
    // Time t_c: with dead time, with preview time for curvature
    // Time t_d: no dead time, with preview time for curvature
    *sOutput.fTrajDistY_met = TPLTJC_CalcTrajDistYEquation(
        fP0, fP1, fP2, fP3, fP4, fP5, fT_a, fT_a2, fT_a3, fT_a4, fT_a5);
    *sOutput.fTrajDistYFmPrev_met = TPLTJC_CalcTrajDistYEquation(
        fP0, fP1, fP2, fP3, fP4, fP5, fT_b, fT_b2, fT_b3, fT_b4, fT_b5);
    *sOutput.fTrajDistY1stDeriv_mps = TPLTJC_CalcTrajDistY1stDervEquation(
        fP1, fP2, fP3, fP4, fP5, fT_a, fT_a2, fT_a3, fT_a4);
    *sOutput.fYDtTrjFmHeadPrev_mps = TPLTJC_CalcTrajDistY1stDervEquation(
        fP1, fP2, fP3, fP4, fP5, fT_b, fT_b2, fT_b3, fT_b4);
    *sOutput.fTrajDistY2ndDeriv_mps2 = TPLTJC_CalcTrajDistY2rdDervEquation(
        fP2, fP3, fP4, fP5, fT_a, fT_a2, fT_a3);
    *sOutput.fYDt2TrjFmKpPrev_mps2 = TPLTJC_CalcTrajDistY2rdDervEquation(
        fP2, fP3, fP4, fP5, fT_d, fT_d2, fT_d3);
    *sOutput.fYDt2TrjFmKpPrevDT_mps2 = TPLTJC_CalcTrajDistY2rdDervEquation(
        fP2, fP3, fP4, fP5, fT_c, fT_c2, fT_c3);
    *sOutput.fTrajDistY3rdDeriv_mps3 =
        TPLTJC_CalcTrajDistY3rdDervEquation(fP3, fP4, fP5, fT_a, fT_a2);
    *sOutput.fYDt3TrjFmKpPrevDT_mps3 =
        TPLTJC_CalcTrajDistY3rdDervEquation(fP3, fP4, fP5, fT_c, fT_c2);
}

/*****************************************************************************
  Functionname: TPLTJC_CalcMeasTrajVariable */ /*!

                                @brief: ego accumulate moving distance/time
                                after last replan triggered

                                @description: ego accumulate moving
                                distance/time after last replan triggered

                                @param[in]
                                    boolean bTrigTrajReplan: replan trigger flag
                                    float32 fSysCycleTimeVeh_sec: cycle time
                                    float32 fTrajVelRefCurve_mps: ego tangential
                                velocity
                                    boolean bReplanModeArcLength: the trajectory
                                plan mode is
                                ArcLength based or time based

                                @param[out]
                                    float32* fMeasTrajVariable_nu: ego
                                accumulate moving distance/time
                                after last replan triggered
                                @return
                                *****************************************************************************/
void TPLTJC_CalcMeasTrajVariable(boolean bTrigTrajReplan,
                                 float32 fSysCycleTimeVeh_sec,
                                 float32 fTrajVelRefCurve_mps,
                                 boolean bReplanModeArcLength,
                                 float32* fMeasTrajVariable_nu) {
    if (bTrigTrajReplan) {
        TRJPLN_Sf_CalcMeasAccuDistance_met =
            fSysCycleTimeVeh_sec * fTrajVelRefCurve_mps;
        TRJPLN_Sf_CalcMeasAccuTime_sec = fSysCycleTimeVeh_sec;
    }

    if (bReplanModeArcLength) {
        // trajectory plan mode is based on ArcLength
        *fMeasTrajVariable_nu = TRJPLN_Sf_CalcMeasAccuDistance_met;
    } else {
        // trajectory plan mode is based on time
        *fMeasTrajVariable_nu = TRJPLN_Sf_CalcMeasAccuTime_sec;
    }
    TRJPLN_Sf_CalcMeasAccuDistance_met =
        MIN(TRJPLN_Sf_CalcMeasAccuDistance_met +
                (fSysCycleTimeVeh_sec * fTrajVelRefCurve_mps),
            TPLTJC_MAXARCLENGTH_MET);
    TRJPLN_Sf_CalcMeasAccuTime_sec = MIN(
        TRJPLN_Sf_CalcMeasAccuTime_sec + fSysCycleTimeVeh_sec, TPLTJC_TMAX_SEC);
}

/*****************************************************************************
  Functionname:    LCF_TRJPLN_TrajectoryCalc_Reset */ /*!

                         @brief: reset function of trajectory calculation module

                         @description: reset function of trajectory calculation
                         module

                         @param[in] void
                         @param[out] void

                         @return
                         *****************************************************************************/
void LCF_TRJPLN_TrajectoryCalc_Reset(void) {
    memset(&TRJPLN_Ss_OptTrajectoryOutput_nu, 0,
           sizeof(TRJPLN_calOptOutTypeV4_t));
    TRJPLN_Sf_PreProcAllowTimeToCorss_sec = 0.f;
    TRJPLN_Sf_CalcMeasAccuDistance_met = 0.f;
    TRJPLN_Sf_CalcMeasAccuTime_sec = 0.f;
    TRJPLN_Sf_CalcTrajPrevTimeHead_sec = 0.f;
    TRJPLN_Sf_CalcTrajPrevTimeCrve_sec = 0.f;
    TRJPLN_Sf_CalcTrajDelayVehGui_sec = 0.f;
    TRJPLN_Sf_CalcTrajDStart_nu = 0.f;

    memset(TRJPLN_Sa_TJCPostProcTrajParam_nu, 0,
           sizeof(TRJPLN_Sa_TJCPostProcTrajParam_nu));
    TRJPLN_Sf_TJCPostProcTimeTrajEnd_sec = 0.f;
    TRJPLN_Sf_TJCPostProceArcLengthTrajEnd_met = 0.f;
}

/*****************************************************************************
  Functionname:TPLTJC_SetBit                                            */ /*!

              @brief: set bit value based on input bit index and value

              @description:set bit value based on input bit index and value

              @param[in]:
                                    uint8 uiBitIndex: bit index of insert place
                                    boolean bBitSetVal: bit set TRUE or FALSE
              @param[in\out]
                                     uint8* pTargetBit: inserted bit
              @return
            *****************************************************************************/
void TPLTJC_SetBit(uint8 uiBitIndex, boolean bBitSetVal, uint16* pTargetBit) {
    if (bBitSetVal) {
        TUE_CML_Setbit_M(*pTargetBit, uiBitIndex);
    } else {
        TUE_CML_Clrbit_M(*pTargetBit, uiBitIndex);
    }
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
