/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "trajectory_plan.h"

/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE2_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
// input values of submodules
static TRJPLN_LatencyCompensationInReq_t g_TRJPLN_LatenCompInput;
static TRJPLN_CalculationEnableInReq_t g_TRJPLN_CalcEnableInput;
static TRJPLN_FrenetTransformInReq_t g_TRJPLN_FrenetTransfInput;
static TRJPLN_TrajectoryCalcInReq_t g_TRJPLN_TrajCalcInput;
static TRJPLN_FrenetBackInReq_t g_TRJPLN_FrenetBackInput;
// output values of submodules
static TRJPLN_LatencyCompensationOutPro_t g_TRJPLN_LatenCompOutput;
static TRJPLN_CalculationEnableOutPro_t g_TRJPLN_CalcEnableOutput;
static TRJPLN_FrenetTransformOutPro_t g_TRJPLN_FrenetTransfOutput;
static TRJPLN_TrajectoryCalcOutPro_t g_TRJPLN_TrajCalcOutput;
static TRJPLN_FrenetBackOutPro_t g_TRJPLN_FrenetBackOutput;
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"

typedef struct {
    float x;
    float y;
} TPPoint_t;

// uint8 TPNum = 100;
// float TPLength = 40.0;

// TPPoint_t TPPoints[TPNum] = {0};

typedef struct {
    float c0;
    float c1;
    float c2;
    float c3;
    float RangeEnd;
} SenseviewLine_t;
#define ASW_QM_CORE2_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
SenseviewLine_t RightCtrlLf;
SenseviewLine_t RightCtrlRi;
SenseviewLine_t RightCtrlCent;
SenseviewLine_t RightCtrlTarget;
TPPoint_t RightEgo;
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
static void TP_Senseview();

/*****************************************************************************
  Functionname:LCF_TrajectoryPlan_Reset */ /*!

                                    @brief reset function of trajectory plan
                                    module

                                    @description: invoke sub-moudles' reset
                                    functions and memset all the global
                                    values

                                    @param[in] void
                                    @param[out] void

                                    @return
                                    *****************************************************************************/
void LCF_TrajectoryPlan_Reset(void) {
    LCF_TRJPLN_LatencyCompensation_Reset();
    LCF_TRJPLN_CalculationReset_Exec();
    LCF_TRJPLN_FrenetTransform_Reset();
    LCF_TRJPLN_TrajectoryCalc_Reset();
    LCF_TRJPLN_FrenetBack_Reset();

    memset(&g_TRJPLN_LatenCompInput, 0,
           sizeof(TRJPLN_LatencyCompensationInReq_t));
    memset(&g_TRJPLN_CalcEnableInput, 0,
           sizeof(TRJPLN_CalculationEnableInReq_t));
    memset(&g_TRJPLN_FrenetTransfInput, 0,
           sizeof(TRJPLN_FrenetTransformInReq_t));
    memset(&g_TRJPLN_TrajCalcInput, 0, sizeof(TRJPLN_TrajectoryCalcInReq_t));
    memset(&g_TRJPLN_FrenetBackInput, 0, sizeof(TRJPLN_FrenetBackInReq_t));

    memset(&g_TRJPLN_LatenCompOutput, 0,
           sizeof(TRJPLN_LatencyCompensationOutPro_t));
    memset(&g_TRJPLN_CalcEnableOutput, 0,
           sizeof(TRJPLN_CalculationEnableOutPro_t));
    memset(&g_TRJPLN_FrenetTransfOutput, 0,
           sizeof(TRJPLN_FrenetTransformOutPro_t));
    memset(&g_TRJPLN_TrajCalcOutput, 0, sizeof(TRJPLN_TrajectoryCalcOutPro_t));
    memset(&g_TRJPLN_FrenetBackOutput, 0, sizeof(TRJPLN_FrenetBackOutPro_t));
}

/*****************************************************************************
  Functionname:LCF_TrajectoryPlan_Exec */ /*!

                                     @brief main function of trajectory plan
                                     module

                                     @description: invoke wrapper functions
                                     sub-modules

                                     @param[in]reqPorts: trajectory input
                                     signals
                                                 paras: algorithm parameters of
                                     trajectory plan

                                     @param[out]proPorts: trajectory output
                                     signals
                                                 debug:debug output signals,
                                     which only be used in datalogger
                                     in debug procedure

                                     @return
                                     @uml
                                     @startuml
                                     start
                                     :LatencyCompensationInputWrapper;
                                     note:set the input data of latency
                                     compensation module
                                     :LCF_TRJPLN_LatencyCompensation_Exec;
                                     note:latency compenstation sub-module entry
                                     function
                                     :CalculationEnableInputWrapper;
                                     note:set the input data of calculation
                                     enable module
                                     :LCF_TRJPLN_CalculationEnable_Exec;
                                     note:calculation enable sub-module entry
                                     function
                                     :FrenetTransformInputWrapper;
                                     note:set the input data of frenet transform
                                     module
                                     :LCF_TRJPLN_FrenetTransform_Exec;
                                     note:frenet transform sub-module entry
                                     function
                                     :TrajectoryCalcInputWrapper;
                                     note:set the input data of trajectory
                                     calculation module
                                     :LCF_TRJPLN_TrajectoryCalc_Exec;
                                     note:trajectory calculation sub-module
                                     entry function
                                     :FrenetBackInputWrapper;
                                     note:set the input data of frenet back
                                     module
                                     :LCF_TRJPLN_FrenetBack_Exec;
                                     note:frenet back sub-module entry function
                                     :TrajectoryOutputWrapper;
                                     note:set the output of whole trajectory
                                     calculation module
                                     end
                                     @enduml
                                     *****************************************************************************/
void LCF_TrajectoryPlan_Exec(const TRJPLN_TrajectoryPlanInReq_t* reqPorts,
                             const TRJPLN_TrajectoryPlanParam_t* paras,
                             TRJPLN_TrajectoryPlanOutPro_t* proPorts,
                             TRJPLN_TrajectoryPlanDebug_t* debug) {
    debug->uiVersionNum_nu = TRAJECTORY_PLAN_VERSION_NUM;
    // step 1, latency compensation process
    LatencyCompensationInputWrapper(reqPorts, &g_TRJPLN_LatenCompInput);
    LCF_TRJPLN_LatencyCompensation_Exec(&g_TRJPLN_LatenCompInput, paras,
                                        &g_TRJPLN_LatenCompOutput,
                                        &debug->pLatenCompDebug);

    // step 2, calcualtion enable process
    CalculationEnableInputWrapper(
        reqPorts, &g_TRJPLN_LatenCompOutput,
        g_TRJPLN_FrenetBackOutput.fCtrlErrHeadAglPrev_rad,
        g_TRJPLN_FrenetBackOutput.fCtrlErrDistY_met,
        g_TRJPLN_TrajCalcOutput.bTrajEnd, &g_TRJPLN_CalcEnableInput);
    LCF_TRJPLN_CalculationEnable_Exec(&g_TRJPLN_CalcEnableInput, paras,
                                      &g_TRJPLN_CalcEnableOutput,
                                      &debug->pCalcEnableDebug);

    // step 3, frenet transform process
    FrenetTransformInputWrapper(
        reqPorts, &g_TRJPLN_LatenCompOutput, &g_TRJPLN_CalcEnableOutput,
        &g_TRJPLN_FrenetBackOutput, &g_TRJPLN_FrenetTransfInput);
    LCF_TRJPLN_FrenetTransform_Exec(&g_TRJPLN_FrenetTransfInput, paras,
                                    &g_TRJPLN_FrenetTransfOutput,
                                    &debug->pFrentTransfDebug);

    // step 4, optimal trajectory calculation process
    TrajectoryCalcInputWrapper(
        reqPorts, &g_TRJPLN_CalcEnableOutput, &g_TRJPLN_LatenCompOutput,
        &g_TRJPLN_FrenetTransfOutput, &g_TRJPLN_TrajCalcInput);
    LCF_TRJPLN_TrajectoryCalc_Exec(&g_TRJPLN_TrajCalcInput, paras,
                                   &g_TRJPLN_TrajCalcOutput,
                                   &debug->pTrajCalcDebug);

    // step 5, frenet back process
    FrenetBackInputWrapper(reqPorts, &g_TRJPLN_CalcEnableOutput,
                           &g_TRJPLN_LatenCompOutput, &g_TRJPLN_TrajCalcOutput,
                           &g_TRJPLN_FrenetTransfOutput,
                           &g_TRJPLN_FrenetBackInput);
    LCF_TRJPLN_FrenetBack_Exec(&g_TRJPLN_FrenetBackInput, paras,
                               &g_TRJPLN_FrenetBackOutput,
                               &debug->pFrenetBackDebug);

    // step 6, trajectory plan output
    TrajectoryOutputWrapper(&g_TRJPLN_FrenetBackOutput,
                            &g_TRJPLN_CalcEnableOutput, proPorts);

    TP_Senseview();
}

static void TP_Senseview() {
    // TRJPLN_FrenetBackOutPro_t* p_TRJPLN_FrenetBackOutput,
    // TRJPLN_LatencyCompensationOutPro_t* p_TRJPLN_LatenCompOutput) {
    // float C0Right = p_TRJPLN_FrenetBackOutput->fTrajDistY_met;
    // float C1Right = p_TRJPLN_FrenetBackOutput->fTrajHeading_rad;
    // float C2Right = p_TRJPLN_FrenetBackOutput->fTrajTgtCurve_1pm;
    // float C3Right = p_TRJPLN_FrenetBackOutput->fTrajTgtCrvGrd_1pms;

    // float DeltaX0 =
    //     p_TRJPLN_LatenCompOutput->sCompenCorriParam.fDeltaProjPosX_met;
    // float DeltaY0 =
    //     p_TRJPLN_LatenCompOutput->sCompenCorriParam.fDeltaProjPosY_met;
    // float DeltaOri =
    //     p_TRJPLN_LatenCompOutput->sCompenCorriParam.fRightOrientation_rad;

    // for (uint8 i = 0; i < TPNum; i++) {
    //     float s = TPLength / TPNum * i;
    //     TPPoint_t TPRightPoint;
    //     TPRightPoint.y = s;
    //     TPRightPoint.x = C0Right + C1Right * s + 1 / 2 * C2Right * s * s +
    //                      1 / 6 * C3Right * s * s * s;
    //     TPPoints[i].y = s;
    //     TPPoints[i].x = C0Right + C1Right * s + 1 / 2 * C2Right * s * s +
    //                     1 / 6 * C3Right * s * s * s;
    // }
    RightCtrlLf.c0 =
        -g_TRJPLN_LatenCompOutput.sCompCorridor.fLeCorridorPosY0_met;
    RightCtrlLf.c1 =
        -g_TRJPLN_LatenCompOutput.sCompCorridor.fLeCorridorHeadingAgl_rad;
    RightCtrlLf.c2 =
        -1 / 2 * g_TRJPLN_LatenCompOutput.sCompCorridor.fLeCorridorCurve_1pm;
    RightCtrlLf.c3 =
        -1 / 6 * g_TRJPLN_LatenCompOutput.sCompCorridor.fLeCorridorCrvChng_1pm2;
    RightCtrlLf.RangeEnd =
        g_TRJPLN_LatenCompOutput.sCompCorridor.fLeCorridorLength_met;

    RightCtrlRi.c0 =
        -g_TRJPLN_LatenCompOutput.sCompCorridor.fRiCorridorPosY0_met;
    RightCtrlRi.c1 =
        -g_TRJPLN_LatenCompOutput.sCompCorridor.fRiCorridorHeadingAgl_rad;
    RightCtrlRi.c2 =
        -1 / 2 * g_TRJPLN_LatenCompOutput.sCompCorridor.fRiCorridorCurve_1pm;
    RightCtrlRi.c3 =
        -1 / 6 * g_TRJPLN_LatenCompOutput.sCompCorridor.fRiCorridorCrvChng_1pm2;
    RightCtrlRi.RangeEnd =
        g_TRJPLN_LatenCompOutput.sCompCorridor.fRiCorridorLength_met;

    RightCtrlCent.c0 =
        -g_TRJPLN_LatenCompOutput.sCompCorridor.fTargetCorridorPosY0_met;
    RightCtrlCent.c1 =
        -g_TRJPLN_LatenCompOutput.sCompCorridor.fTargetCorridorHeading_rad;
    RightCtrlCent.c2 =
        -1 / 2 *
        g_TRJPLN_LatenCompOutput.sCompCorridor.fTargetCorridorCurve_1pm;
    RightCtrlCent.c3 =
        -1 / 6 *
        g_TRJPLN_LatenCompOutput.sCompCorridor.fTargetCorridorCrvChng_1pm2;
    RightCtrlCent.RangeEnd =
        g_TRJPLN_LatenCompOutput.sCompCorridor.fTargetCorridorLength_met;

    RightEgo.x = -g_TRJPLN_FrenetBackOutput.fCurDistY_met;
    RightEgo.y = -g_TRJPLN_FrenetBackOutput.fCurHeading_rad;

    RightCtrlTarget.c0 = -g_TRJPLN_FrenetBackOutput.fTrajDistY_met;
    RightCtrlTarget.c1 = -g_TRJPLN_FrenetBackOutput.fTrajHeading_rad;
    RightCtrlTarget.c2 = -1 / 2 * g_TRJPLN_FrenetBackOutput.fTrajTgtCurve_1pm;
    RightCtrlTarget.c3 = -1 / 6 * g_TRJPLN_FrenetBackOutput.fTrajTgtCrvGrd_1pms;
    RightCtrlTarget.RangeEnd = RightCtrlCent.RangeEnd;
}

/*****************************************************************************
  Functionname:LatencyCompensationInputWrapper */ /*!

                             @brief input wrapper function of latency
                             compensation sub-module

                             @description

                             @param[in]pTrajPlanInput: the input value from the
                             trajectory plan input

                             @param[out]pLatCompInput: the input value for the
                             latency compensation
                             sub-module

                             @return
                             *****************************************************************************/
void LatencyCompensationInputWrapper(
    const TRJPLN_TrajectoryPlanInReq_t* pTrajPlanInput,
    TRJPLN_LatencyCompensationInReq_t* pLatCompInput) {
    pLatCompInput->sLatCompIn.bLatencyCompActivated =
        pTrajPlanInput->bLatencyCompActivated;
    pLatCompInput->sCommonInput.fCycleTimeVeh_sec =
        pTrajPlanInput->fCycleTimeVeh_sec;
    pLatCompInput->sCommonInput.fEgoVelX_mps = pTrajPlanInput->fEgoVelX_mps;
    pLatCompInput->sCommonInput.fEgoYawRate_rps =
        pTrajPlanInput->fEgoYawRate_rps;
    pLatCompInput->sAllCorridor.fLeCridBndCrvChng_1pm2 =
        pTrajPlanInput->fLeCridBndCrvChng_1pm2;
    pLatCompInput->sAllCorridor.fLeCridBndCrv_1pm =
        pTrajPlanInput->fLeCridBndCrv_1pm;
    pLatCompInput->sAllCorridor.fLeCridBndHeadAng_rad =
        pTrajPlanInput->fLeCridBndHeadAng_rad;
    pLatCompInput->sAllCorridor.fLeCridBndLength_met =
        pTrajPlanInput->fLeCridBndLength_met;
    pLatCompInput->sAllCorridor.fLeCridBndPosX0_met =
        pTrajPlanInput->fLeCridBndPosX0_met;
    pLatCompInput->sAllCorridor.fLeCridBndPosY0_met =
        pTrajPlanInput->fLeCridBndPosY0_met;
    pLatCompInput->sAllCorridor.fRiCridBndCrvChng_1pm2 =
        pTrajPlanInput->fRiCridBndCrvChng_1pm2;
    pLatCompInput->sAllCorridor.fRiCridBndCrv_1pm =
        pTrajPlanInput->fRiCridBndCrv_1pm;
    pLatCompInput->sAllCorridor.fRiCridBndHeadAng_rad =
        pTrajPlanInput->fRiCridBndHeadAng_rad;
    pLatCompInput->sAllCorridor.fRiCridBndLength_met =
        pTrajPlanInput->fRiCridBndLength_met;
    pLatCompInput->sAllCorridor.fRiCridBndPosX0_met =
        pTrajPlanInput->fRiCridBndPosX0_met;
    pLatCompInput->sAllCorridor.fRiCridBndPosY0_met =
        pTrajPlanInput->fRiCridBndPosY0_met;
    pLatCompInput->sLatCompIn.fSensorTimeStamp_sec =
        pTrajPlanInput->fSensorTimeStamp_sec;
    pLatCompInput->sAllCorridor.fTgtTrajCrvChng_1pm2 =
        pTrajPlanInput->fTgtTrajCrvChng_1pm2;
    pLatCompInput->sAllCorridor.fTgtTrajCurve_1pm =
        pTrajPlanInput->fTgtTrajCurve_1pm;
    pLatCompInput->sAllCorridor.fTgtTrajHeadingAng_rad =
        pTrajPlanInput->fTgtTrajHeadingAng_rad;
    pLatCompInput->sAllCorridor.fTgtTrajLength_met =
        pTrajPlanInput->fTgtTrajLength_met;
    pLatCompInput->sAllCorridor.fTgtTrajPosX0_met =
        pTrajPlanInput->fTgtTrajPosX0_met;
    pLatCompInput->sAllCorridor.fTgtTrajPosY0_met =
        pTrajPlanInput->fTgtTrajPosY0_met;
    pLatCompInput->sConsisCheckIn.uiControllingFunction_nu =
        pTrajPlanInput->uiControllingFunction_nu;
    pLatCompInput->sConsisCheckIn.uiLatCtrlMode_nu =
        pTrajPlanInput->uiLatCtrlMode_nu;
    pLatCompInput->sConsisCheckIn.uiLeCrvQuality_per =
        pTrajPlanInput->uiLeCrvQuality_per;
    pLatCompInput->sConsisCheckIn.uiLeLnQuality_per =
        pTrajPlanInput->uiLeLnQuality_per;
    pLatCompInput->sLatCompIn.uiOdometerState_nu =
        pTrajPlanInput->uiOdometerState_nu;
    pLatCompInput->sConsisCheckIn.uiRiCrvQuality_per =
        pTrajPlanInput->uiRiCrvQuality_per;
    pLatCompInput->sConsisCheckIn.uiRiLnQuality_per =
        pTrajPlanInput->uiRiLnQuality_per;
    pLatCompInput->sLatCompIn.uiSenToVehTStamp_us =
        pTrajPlanInput->uiSenToVehTStamp_us;
    pLatCompInput->sLatCompIn.uiSysStateLCF_nu =
        pTrajPlanInput->uiSysStateLCF_nu;
    pLatCompInput->sCommonInput.uiTrajGuiQualifier_nu =
        pTrajPlanInput->uiTrajGuiQualifier_nu;
    pLatCompInput->sLatCompIn.uiVehSync4LCO_us =
        pTrajPlanInput->uiVehSync4LCO_us;
}

/*****************************************************************************
  Functionname:CalculationEnableInputWrapper */ /*!

                               @brief input wrapper function of calculation
                               enable sub-module

                               @description

                               @param[in]pTrajPlanInput: the input value from
                               the trajectory plan input
                               pLatCompOutput: the output value from the latency
                               compensation
                               sub-module
                               fCtrlErrHeadAglPrev_rad: the preview control
                               error heading angle
                               value from frenet transform sub-module
                               fCtrlErrDistY_met: the control error distanceY
                               value from frenet
                               transform sub-module
                               bTrajecotryEnd:the trajectory end flag value from
                               calculation enable
                               sub-module

                               @param[out]pCalEnableInput: the input value for
                               the calculation enable
                               sub-module

                               @return
                               *****************************************************************************/
void CalculationEnableInputWrapper(
    const TRJPLN_TrajectoryPlanInReq_t* pTrajPlanInput,
    const TRJPLN_LatencyCompensationOutPro_t* pLatCompOutput,
    const float32 fCtrlErrHeadAglPrev_rad,
    const float32 fCtrlErrDistY_met,
    const boolean bTrajecotryEnd,
    TRJPLN_CalculationEnableInReq_t* pCalEnableInput) {
    pCalEnableInput->uiControllingFunction_nu =
        pTrajPlanInput->uiControllingFunction_nu;
    pCalEnableInput->fCycleTimeVeh_sec = pTrajPlanInput->fCycleTimeVeh_sec;
    pCalEnableInput->fEgoVelX_mps = pTrajPlanInput->fEgoVelX_mps;
    pCalEnableInput->fEPSManualTrqActVal_Nm =
        pTrajPlanInput->fEPSManualTrqActVal_Nm;
    pCalEnableInput->uiTrajGuiQualifier_nu =
        pTrajPlanInput->uiTrajGuiQualifier_nu;
    pCalEnableInput->uiSysStateLCF_nu = pTrajPlanInput->uiSysStateLCF_nu;
    pCalEnableInput->bLCOTriggerReplan =
        pLatCompOutput->sCompCorridor.bTriggerReplan;
    pCalEnableInput->bTrajecotryEnd = bTrajecotryEnd;
    pCalEnableInput->fCtrlErrDistY_met = fCtrlErrDistY_met;
    pCalEnableInput->fCtrlErrHeadAglPrev_rad = fCtrlErrHeadAglPrev_rad;
    pCalEnableInput->bSysStOffLatDMC = pTrajPlanInput->bSysStOffLatDMC;
    pCalEnableInput->fPredTimeCurve_sec = pTrajPlanInput->fPredTimeCurve_sec;
    pCalEnableInput->fPredTimeHeadAng_sec =
        pTrajPlanInput->fPredTimeHeadAng_sec;
    pCalEnableInput->bLTATriggerReplan = pTrajPlanInput->bTriggerReplan;
    pCalEnableInput->uiLatCtrlMode_nu = pTrajPlanInput->uiLatCtrlMode_nu;
    pCalEnableInput->fDevDistY_met =
        pLatCompOutput->sCompCorridor.fDevDistY_met;
    pCalEnableInput->fDevHeadingAngle_rad =
        pLatCompOutput->sCompCorridor.fDevHeadingAngle_rad;
    pCalEnableInput->fTargetPathY0_met =
        pLatCompOutput->sTargetPath.fTargetPathY0_met;
    pCalEnableInput->fTargetPathHeading_rad =
        pLatCompOutput->sTargetPath.fTargetPathHeading_rad;
    pCalEnableInput->bSysStReqLatDMC = pTrajPlanInput->bSysStReqLatDMC;
}

/*****************************************************************************
  Functionname:FrenetTransformInputWrapper */ /*!

                                 @brief input wrapper function of frenet
                                 transform sub-module

                                 @description

                                 @param[in]pTrajPlanInput: the input value from
                                 the trajectory plan input
                                     pLatCompOutput: the output value from the
                                 latency compensation
                                 sub-module
                                     pCalcEnableOutput: the output value from
                                 the calculation enable
                                 sub-module
                                     pFrenBackOutput: the output value from the
                                 frenet back sub-module

                                 @param[out]pFrenetTrans: the input value for
                                 the frenet transform sub-module

                                 @return
                                 *****************************************************************************/
void FrenetTransformInputWrapper(
    const TRJPLN_TrajectoryPlanInReq_t* pTrajPlanInput,
    const TRJPLN_LatencyCompensationOutPro_t* pLatCompOutput,
    const TRJPLN_CalculationEnableOutPro_t* pCalcEnableOutput,
    const TRJPLN_FrenetBackOutPro_t* pFrenBackOutput,
    TRJPLN_FrenetTransformInReq_t* pFrenetTrans) {
    pFrenetTrans->fLeCorridorPosX0_met =
        pLatCompOutput->sCompCorridor.fLeCorridorPosX0_met;
    pFrenetTrans->fLeCorridorPosY0_met =
        pLatCompOutput->sCompCorridor.fLeCorridorPosY0_met;
    pFrenetTrans->fLeCorridorHeadingAgl_rad =
        pLatCompOutput->sCompCorridor.fLeCorridorHeadingAgl_rad;
    pFrenetTrans->fLeCorridorCurve_1pm =
        pLatCompOutput->sCompCorridor.fLeCorridorCurve_1pm;
    pFrenetTrans->fLeCorridorCrvChng_1pm2 =
        pLatCompOutput->sCompCorridor.fLeCorridorCrvChng_1pm2;
    pFrenetTrans->fLeCorridorLength_met =
        pLatCompOutput->sCompCorridor.fLeCorridorLength_met;
    pFrenetTrans->fRiCorridorPosX0_met =
        pLatCompOutput->sCompCorridor.fRiCorridorPosX0_met;
    pFrenetTrans->fRiCorridorPosY0_met =
        pLatCompOutput->sCompCorridor.fRiCorridorPosY0_met;
    pFrenetTrans->fRiCorridorHeadingAgl_rad =
        pLatCompOutput->sCompCorridor.fRiCorridorHeadingAgl_rad;
    pFrenetTrans->fRiCorridorCurve_1pm =
        pLatCompOutput->sCompCorridor.fRiCorridorCurve_1pm;
    pFrenetTrans->fRiCorridorCrvChng_1pm2 =
        pLatCompOutput->sCompCorridor.fRiCorridorCrvChng_1pm2;
    pFrenetTrans->fRiCorridorLength_met =
        pLatCompOutput->sCompCorridor.fRiCorridorLength_met;
    pFrenetTrans->fTargetCorridorPosX0_met =
        pLatCompOutput->sCompCorridor.fTargetCorridorPosX0_met;
    pFrenetTrans->fTargetCorridorPosY0_met =
        pLatCompOutput->sCompCorridor.fTargetCorridorPosY0_met;
    pFrenetTrans->fTargetCorridorHeading_rad =
        pLatCompOutput->sCompCorridor.fTargetCorridorHeading_rad;
    pFrenetTrans->fTargetCorridorCurve_1pm =
        pLatCompOutput->sCompCorridor.fTargetCorridorCurve_1pm;
    pFrenetTrans->fTargetCorridorCrvChng_1pm2 =
        pLatCompOutput->sCompCorridor.fTargetCorridorCrvChng_1pm2;
    pFrenetTrans->fTargetCorridorLength_met =
        pLatCompOutput->sCompCorridor.fTargetCorridorLength_met;
    pFrenetTrans->fDevDistY_met = pLatCompOutput->sCompCorridor.fDevDistY_met;
    pFrenetTrans->fDevHeadingAngle_rad =
        pLatCompOutput->sCompCorridor.fDevHeadingAngle_rad;
    pFrenetTrans->fReplanDevDistY_met =
        pLatCompOutput->sCompCorridor.fReplanDevDistY_met;
    pFrenetTrans->fReplanDevHeading_rad =
        pLatCompOutput->sCompCorridor.fReplanDevHeading_rad;
    pFrenetTrans->fEgoVelX_mps = pTrajPlanInput->fEgoVelX_mps;
    pFrenetTrans->fEgoAccelX_mps2 = pTrajPlanInput->fEgoAccelX_mps2;
    pFrenetTrans->fEgoCurve_1pm = pTrajPlanInput->fEgoCurve_1pm;
    pFrenetTrans->fPlanningHorizon_sec = pTrajPlanInput->fPlanningHorizon_sec;
    pFrenetTrans->bTrajPlanEnble = pCalcEnableOutput->bTrajPlanEnble;
    pFrenetTrans->bTrigTrajReplan = pCalcEnableOutput->bTrigTrajReplan;
    pFrenetTrans->bReplanModeArcLength =
        pCalcEnableOutput->bReplanModeArcLength;
    pFrenetTrans->fDelayVehGui_sec = pCalcEnableOutput->fDelayVehGui_sec;
    pFrenetTrans->bTrigReplanTgtTraj = pCalcEnableOutput->bTrigReplanTgtTraj;
    pFrenetTrans->fTrajDistYPrev_met = pFrenBackOutput->fTrajDistYPrev_met;
    pFrenetTrans->fTrajHeadingPrev_rad = pFrenBackOutput->fTrajHeadingPrev_rad;
    pFrenetTrans->fTrajTgtCrvPrev_1pm = pFrenBackOutput->fTrajTgtCrvPrev_1pm;
    pFrenetTrans->fPredictionTimeHead_sec =
        pCalcEnableOutput->fPredictionTimeHead_sec;
    pFrenetTrans->bReplanCurValues = pCalcEnableOutput->bReplanCurValues;
    pFrenetTrans->fKappaSumCommand_1pm = pTrajPlanInput->fKappaSumCommand_1pm;
}

/*****************************************************************************
  Functionname:TrajectoryCalcInputWrapper */ /*!

                                  @brief input wrapper function of trajectory
                                  calculation sub-module

                                  @description

                                  @param[in]pTrajPlanInput: the input value from
                                  the trajectory plan input
                                        pCalcEnableOutput: the output value from
                                  the calculation enable
                                  sub-module
                                        pLatCompOutput: the output value from
                                  the latency compensation
                                  sub-module
                                        pFrenetTransfOutput: the output value
                                  from the frenet transform
                                  sub-module

                                  @param[out]pTrajectoryCalcInput: the input
                                  value for the trajectory calculation
                                  sub-module

                                  @return
                                  *****************************************************************************/
void TrajectoryCalcInputWrapper(
    const TRJPLN_TrajectoryPlanInReq_t* pTrajPlanInput,
    const TRJPLN_CalculationEnableOutPro_t* pCalcEnableOutput,
    const TRJPLN_LatencyCompensationOutPro_t* pLatCompOutput,
    const TRJPLN_FrenetTransformOutPro_t* pFrenetTransfOutput,
    TRJPLN_TrajectoryCalcInReq_t* pTrajectoryCalcInput) {
    pTrajectoryCalcInput->bTrigTrajReplan = pCalcEnableOutput->bTrigTrajReplan;
    pTrajectoryCalcInput->bReplanCurValues =
        pCalcEnableOutput->bReplanCurValues;
    pTrajectoryCalcInput->bReplanTgtValues =
        pCalcEnableOutput->bReplanTgtValues;
    pTrajectoryCalcInput->bReplanModeArcLength =
        pCalcEnableOutput->bReplanModeArcLength;
    pTrajectoryCalcInput->bTrigCustFctActn =
        pCalcEnableOutput->bTrigCustFctActn;
    pTrajectoryCalcInput->fDelayVehGui_sec =
        pCalcEnableOutput->fDelayVehGui_sec;
    pTrajectoryCalcInput->fCycleTimeVeh_sec = pTrajPlanInput->fCycleTimeVeh_sec;
    pTrajectoryCalcInput->fEgoVelX_mps = pTrajPlanInput->fEgoVelX_mps;
    pTrajectoryCalcInput->uiTrajPlanServQu_nu =
        pTrajPlanInput->uiTrajPlanServQu_nu;
    pTrajectoryCalcInput->fWeightTgtDistY_nu =
        pTrajPlanInput->fWeightTgtDistY_nu;
    pTrajectoryCalcInput->fWeightEndTime_nu = pTrajPlanInput->fWeightEndTime_nu;
    pTrajectoryCalcInput->fDistYToLeTgtArea_met =
        pTrajPlanInput->fDistYToLeTgtArea_met;
    pTrajectoryCalcInput->fDistYToRiTgtArea_met =
        pTrajPlanInput->fDistYToRiTgtArea_met;
    pTrajectoryCalcInput->fFTireAclMax_mps2 = pTrajPlanInput->fFTireAclMax_mps2;
    pTrajectoryCalcInput->fFTireAclMin_mps2 = pTrajPlanInput->fFTireAclMin_mps2;
    pTrajectoryCalcInput->fPredictionTimeCrv_sec =
        pCalcEnableOutput->fPredictionTimeCrv_sec;
    pTrajectoryCalcInput->fObstacleVelX_mps = pTrajPlanInput->fObstacleVelX_mps;
    pTrajectoryCalcInput->fObstacleAccelX_mps2 =
        pTrajPlanInput->fObstacleAccelX_mps2;
    pTrajectoryCalcInput->fObstacleWidth_met =
        pTrajPlanInput->fObstacleWidth_met;
    pTrajectoryCalcInput->fObstacleDistX_met =
        pTrajPlanInput->fObstacleDistX_met;
    pTrajectoryCalcInput->fObstacleDistY_met =
        pTrajPlanInput->fObstacleDistY_met;
    pTrajectoryCalcInput->fRiCorridorLength_met =
        pLatCompOutput->sCompCorridor.fRiCorridorLength_met;
    pTrajectoryCalcInput->fLeCorridorLength_met =
        pLatCompOutput->sCompCorridor.fLeCorridorLength_met;
    pTrajectoryCalcInput->fRiCorridorCurve_1pm =
        pLatCompOutput->sCompCorridor.fRiCorridorCurve_1pm;
    memcpy(pTrajectoryCalcInput->afLeDistY_met,
           pFrenetTransfOutput->afLeDistY_met,
           sizeof(pFrenetTransfOutput->afLeDistY_met));
    memcpy(pTrajectoryCalcInput->afTargetDistY_met,
           pFrenetTransfOutput->afTargetDistY_met,
           sizeof(pFrenetTransfOutput->afTargetDistY_met));
    memcpy(pTrajectoryCalcInput->fTargetDistY1stDeriv_mps,
           pFrenetTransfOutput->fTargetDistY1stDeriv_mps,
           sizeof(pFrenetTransfOutput->fTargetDistY1stDeriv_mps));
    memcpy(pTrajectoryCalcInput->fTargetDistY2ndDeriv_mps2,
           pFrenetTransfOutput->fTargetDistY2ndDeriv_mps2,
           sizeof(pFrenetTransfOutput->fTargetDistY2ndDeriv_mps2));
    pTrajectoryCalcInput->fCurDistY_met = pFrenetTransfOutput->fCurDistY_met;
    pTrajectoryCalcInput->fCurDistY1stDeriv_mps =
        pFrenetTransfOutput->fCurDistY1stDeriv_mps;
    pTrajectoryCalcInput->fCurDistY2ndDeriv_mps2 =
        pFrenetTransfOutput->fCurDistY2ndDeriv_mps2;
    memcpy(pTrajectoryCalcInput->afTargetPoints_nu,
           pFrenetTransfOutput->afTargetPoints_nu,
           sizeof(pFrenetTransfOutput->afTargetPoints_nu));
    pTrajectoryCalcInput->fTrajDistYPrev_met =
        pFrenetTransfOutput->fTrajDistYPrev_met;
    pTrajectoryCalcInput->fTrajDistY1stToPrev_mps =
        pFrenetTransfOutput->fTrajDistY1stToPrev_mps;
    pTrajectoryCalcInput->fTrajDistY2ndToPrev_mps2 =
        pFrenetTransfOutput->fTrajDistY2ndToPrev_mps2;
    pTrajectoryCalcInput->uiNumOfTgtPoints_nu =
        pFrenetTransfOutput->uiNumOfTgtPoints_nu;
    pTrajectoryCalcInput->fTrajVelRefCurve_mps =
        pFrenetTransfOutput->fTrajVelRefCurve_mps;
    pTrajectoryCalcInput->fTrajPlanningHorizon_sec =
        pFrenetTransfOutput->fTrajPlanningHorizon_sec;
    pTrajectoryCalcInput->fPreviewTimeHeading_sec =
        pFrenetTransfOutput->fPreviewTimeHeading_sec;
    pTrajectoryCalcInput->fTrajAclRefCurve_mps2 =
        pFrenetTransfOutput->fTrajAclRefCurve_mps2;
    pTrajectoryCalcInput->uiNumOfPointsCridrLeft_nu =
        pFrenetTransfOutput->uiNumOfPointsCridrLeft_nu;
    pTrajectoryCalcInput->fPlanHorizonVisRange_sec =
        pFrenetTransfOutput->fPlanHorizonVisRange_sec;
    pTrajectoryCalcInput->fRiCorridorCrvChng_1pm2 =
        pLatCompOutput->sCompCorridor.fRiCorridorCrvChng_1pm2;
    pTrajectoryCalcInput->fMaxJerkAllowed_mps3 =
        pTrajPlanInput->fMaxJerkAllowed_mps3;
    pTrajectoryCalcInput->fLeCorridorPosY0_met =
        pLatCompOutput->sCompCorridor.fLeCorridorPosY0_met;
}

/*****************************************************************************
  Functionname:FrenetBackInputWrapper */ /*!

                                      @brief input wrapper function of frenet
                                      back sub-module

                                      @description

                                      @param[in]pTrajPlanInput: the input value
                                      from the trajectory plan input
                                                    pCalcEnableOutput: the
                                      output value from the calculation
                                      enable sub-module
                                                    pLatCompOutput: the output
                                      value from the latency
                                      compensation sub-module
                                                    pTrajCalcOutput: the output
                                      value from the trajectory
                                      calculation sub-module
                                                    pFrenetTransfOutput: the
                                      output value from the frenet
                                      transform sub-module

                                      @param[out]pFrenetBackInput: the input
                                      value for the frenet back sub-module

                                      @return
                                      *****************************************************************************/
void FrenetBackInputWrapper(
    const TRJPLN_TrajectoryPlanInReq_t* pTrajPlanInput,
    const TRJPLN_CalculationEnableOutPro_t* pCalcEnableOutput,
    const TRJPLN_LatencyCompensationOutPro_t* pLatCompOutput,
    const TRJPLN_TrajectoryCalcOutPro_t* pTrajCalcOutput,
    const TRJPLN_FrenetTransformOutPro_t* pFrenetTransfOutput,
    TRJPLN_FrenetBackInReq_t* pFrenetBackInput) {
    pFrenetBackInput->uiTrajGuiQualifier_nu =
        pTrajPlanInput->uiTrajGuiQualifier_nu;
    pFrenetBackInput->fPredictionTimeCrv_sec =
        pCalcEnableOutput->fPredictionTimeCrv_sec;
    pFrenetBackInput->fRiCorridorCurve_1pm =
        pLatCompOutput->sCompCorridor.fRiCorridorCurve_1pm;
    pFrenetBackInput->fRiCorridorCrvChng_1pm2 =
        pLatCompOutput->sCompCorridor.fRiCorridorCrvChng_1pm2;
    pFrenetBackInput->fTargetCorridorPosX0_met =
        pLatCompOutput->sCompCorridor.fTargetCorridorPosX0_met;
    pFrenetBackInput->fTargetCorridorCurve_1pm =
        pLatCompOutput->sCompCorridor.fTargetCorridorCurve_1pm;
    pFrenetBackInput->fTargetCorridorCrvChng_1pm2 =
        pLatCompOutput->sCompCorridor.fTargetCorridorCrvChng_1pm2;
    pFrenetBackInput->fTargetCorridorLength_met =
        pLatCompOutput->sCompCorridor.fTargetCorridorLength_met;
    pFrenetBackInput->fDevDistY_met =
        pLatCompOutput->sCompCorridor.fDevDistY_met;
    pFrenetBackInput->fDevHeadingAngle_rad =
        pLatCompOutput->sCompCorridor.fDevHeadingAngle_rad;
    pFrenetBackInput->uiQuStatusTrajPlan_nu =
        pTrajCalcOutput->uiQuStatusTrajPlan_nu;
    pFrenetBackInput->fTrajDistY_met = pTrajCalcOutput->fTrajDistY_met;
    pFrenetBackInput->fTrajDistY1stDeriv_mps =
        pTrajCalcOutput->fTrajDistY1stDeriv_mps;
    pFrenetBackInput->fTrajDistY2ndDeriv_mps2 =
        pTrajCalcOutput->fTrajDistY2ndDeriv_mps2;
    pFrenetBackInput->fTrajDistY3rdDeriv_mps3 =
        pTrajCalcOutput->fTrajDistY3rdDeriv_mps3;
    pFrenetBackInput->fYDtTrjFmHeadPrev_mps =
        pTrajCalcOutput->fYDtTrjFmHeadPrev_mps;
    pFrenetBackInput->fYDt2TrjFmKpPrevDT_mps2 =
        pTrajCalcOutput->fYDt2TrjFmKpPrevDT_mps2;
    pFrenetBackInput->fYDt3TrjFmKpPrevDT_mps3 =
        pTrajCalcOutput->fYDt3TrjFmKpPrevDT_mps3;
    pFrenetBackInput->fYD2TrjFmKpPrev_mps2 =
        pTrajCalcOutput->fYD2TrjFmKpPrev_mps2;
    pFrenetBackInput->fTrajVelRefCurve_mps =
        pFrenetTransfOutput->fTrajVelRefCurve_mps;
    pFrenetBackInput->fTrajAclRefCurve_mps2 =
        pFrenetTransfOutput->fTrajAclRefCurve_mps2;
    pFrenetBackInput->fCurDistYPreview_met =
        pFrenetTransfOutput->fCurDistYPreview_met;
    pFrenetBackInput->fCurDistY1stToPrev_mps =
        pFrenetTransfOutput->fCurDistY1stToPrev_mps;
    pFrenetBackInput->bTrajPlanEnble = pCalcEnableOutput->bTrajPlanEnble;
    pFrenetBackInput->bReplanModeArcLength =
        pCalcEnableOutput->bReplanModeArcLength;
    pFrenetBackInput->fDelayVehGui_sec = pCalcEnableOutput->fDelayVehGui_sec;
    pFrenetBackInput->bTrigTrajReplan = pCalcEnableOutput->bTrigTrajReplan;
    pFrenetBackInput->bEnblSpecPlanStrategy =
        pCalcEnableOutput->bEnblSpecPlanStrategy;
    pFrenetBackInput->fCycleTimeVeh_sec = pTrajPlanInput->fCycleTimeVeh_sec;
    pFrenetBackInput->bReplanCurValues = pCalcEnableOutput->bReplanCurValues;
    pFrenetBackInput->fTargetPathY0_met =
        pLatCompOutput->sTargetPath.fTargetPathY0_met;
    pFrenetBackInput->fTargetPathHeading_rad =
        pLatCompOutput->sTargetPath.fTargetPathHeading_rad;
    pFrenetBackInput->fLeCorridorPosY0_met =
        pLatCompOutput->sCompCorridor.fLeCorridorPosY0_met;
    pFrenetBackInput->fRiCorridorPosY0_met =
        pLatCompOutput->sCompCorridor.fRiCorridorPosY0_met;
    pFrenetBackInput->uiSPlausiCheckStatus_nu =
        pLatCompOutput->sPlasiCheck.uiSPlausiCheckStatus_nu;
    pFrenetBackInput->uiSysStateLCF_nu = pTrajPlanInput->uiSysStateLCF_nu;
    pFrenetBackInput->uiLatCtrlMode_nu = pTrajPlanInput->uiLatCtrlMode_nu;
    pFrenetBackInput->fEgoVelX_mps = pTrajPlanInput->fEgoVelX_mps;
}

/*****************************************************************************
  Functionname:TrajectoryOutputWrapper */ /*!

                                     @brief output wrapper function from
                                     sub-modules to trajectory plan module

                                     @description: output value assignment of
                                     trajectory plan module

                                     @param[in]pFrenetBackOutput: the output
                                     value from the frenet back sub-module
                                                 pCalcEnableOutput: the output
                                     value from the calculation
                                     enable sub-module

                                     @param[out]pTrajectoryPlanOutput: the
                                     output value of the trajectory plan module

                                     @return
                                     *****************************************************************************/
void TrajectoryOutputWrapper(
    const TRJPLN_FrenetBackOutPro_t* pFrenetBackOutput,
    const TRJPLN_CalculationEnableOutPro_t* pCalcEnableOutput,
    TRJPLN_TrajectoryPlanOutPro_t* pTrajectoryPlanOutput) {
    pTrajectoryPlanOutput->bReplanCurValues =
        pCalcEnableOutput->bReplanCurValues;
    pTrajectoryPlanOutput->uiTrajGuiQualifier_nu =
        pFrenetBackOutput->uiTrajGuiQualifier_nu;
    pTrajectoryPlanOutput->fCurDistY_met = pFrenetBackOutput->fCurDistY_met;
    pTrajectoryPlanOutput->fTrajDistY_met = pFrenetBackOutput->fTrajDistY_met;
    pTrajectoryPlanOutput->fTrajTgtCurve_1pm =
        pFrenetBackOutput->fTrajTgtCurve_1pm;
    pTrajectoryPlanOutput->fCurHeading_rad = pFrenetBackOutput->fCurHeading_rad;
    pTrajectoryPlanOutput->fTrajHeadInclPrev_rad =
        pFrenetBackOutput->fTrajHeadInclPrev_rad;
    pTrajectoryPlanOutput->fTrajHeading_rad =
        pFrenetBackOutput->fTrajHeading_rad;
    pTrajectoryPlanOutput->fTrajTgtCrvGrd_1pms =
        pFrenetBackOutput->fTrajTgtCrvGrd_1pms;
    pTrajectoryPlanOutput->fTrajHeadingPrev_rad =
        pFrenetBackOutput->fTrajHeadingPrev_rad;
    pTrajectoryPlanOutput->fTrajTgtCrvPrev_1pm =
        pFrenetBackOutput->fTrajTgtCrvPrev_1pm;
    pTrajectoryPlanOutput->fCtrlErrDistY_met =
        pFrenetBackOutput->fCtrlErrDistY_met;
    pTrajectoryPlanOutput->fCtrlErrHeadingAngle_rad =
        pFrenetBackOutput->fCtrlErrHeadingAngle_rad;
    pTrajectoryPlanOutput->fCtrlErrHeadAglPrev_rad =
        pFrenetBackOutput->fCtrlErrHeadAglPrev_rad;
    pTrajectoryPlanOutput->fTrajDistYPrev_met =
        pFrenetBackOutput->fTrajDistYPrev_met;
    pTrajectoryPlanOutput->fDeltaTargetCrv_1pm =
        pFrenetBackOutput->fDeltaTargetCrv_1pm;
    pTrajectoryPlanOutput->fDeltaTargetPosY0_met =
        pFrenetBackOutput->fDeltaTargetPosY0_met;
    pTrajectoryPlanOutput->fDeltaTargetHeading_rad =
        pFrenetBackOutput->fDeltaTargetHeading_rad;
    pTrajectoryPlanOutput->bUseTargetCorridor =
        pFrenetBackOutput->bUseTargetCorridor;
    pTrajectoryPlanOutput->bTargetSwitch = pFrenetBackOutput->bTargetSwitch;
    pTrajectoryPlanOutput->bGradLimitActive =
        pFrenetBackOutput->bGradLimitActive;
    pTrajectoryPlanOutput->bPlausiCheckStatus =
        pFrenetBackOutput->bPlausiCheckStatus;
    pTrajectoryPlanOutput->uiS_QuStatusTrajPlan_nu =
        pFrenetBackOutput->uiS_QuStatusTrajPlan_nu;
    pTrajectoryPlanOutput->fTrajTgtCrvGrdPrev_1pms =
        pFrenetBackOutput->fTrajTgtCrvGrdPrev_1pms;
    pTrajectoryPlanOutput->uiD_QuStatusTrajPlan_nu =
        pFrenetBackOutput->uiD_QuStatusTrajPlan_nu;
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */