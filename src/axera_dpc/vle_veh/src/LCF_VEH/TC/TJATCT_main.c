/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "TJATCT.h"
#include "TJATCT_main.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
#define CAL_START_CODE
#include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
const volatile uint8_T Cal_TestFlag_EnableFF = 0;
const volatile uint8_T Cal_TestFlag_EnableTP = 1;
const volatile uint8_T Cal_TestFlag_EnableOL = 1;

const volatile real32_T Cal_Test_CoeffMainPGainLdc = 0.16;
const volatile real32_T Cal_Test_CoeffPGainLdc = 1;
const volatile real32_T Cal_Test_CoeffIGainLdc = 0.6;
const volatile real32_T Cal_Test_CoeffDGainLdc = 0.01;
const volatile real32_T Cal_Test_TimeDT1Ldc = 0.015;
const volatile real32_T Cal_Test_CoeffPT1GainLdc = 0.0;
const volatile real32_T Cal_Test_TimePT1Ldc = 0.1;

const volatile real32_T Cal_Test_CoeffMainPGainCac = 0.008;
const volatile real32_T Cal_Test_CoeffPGainCac = 0.01;
const volatile real32_T Cal_Test_CoeffIGainCac = 0.12;
const volatile real32_T Cal_Test_CoeffDGainCac = 0.1;
const volatile real32_T Cal_Test_TimeDT1Cac = 0.0005;
const volatile real32_T Cal_Test_CoeffPT1GainCac = 0.0;
const volatile real32_T Cal_Test_TimePT1Cac = 0.18;

const volatile real32_T Cal_Test_CDCTimeFltCurHeading = 0.16;
const volatile real32_T Cal_LQR_e1_gains[9] = {
    0.02, 0.02, 0.015, 0.013, 0.007, 0.0045, 0.0045, 0.004, 0.004};
const volatile real32_T Cal_LQR_e1dot_gains[9] = {
    0.004, 0.002, 0.003, 0.003, 0.002, 0.0015, 0.001, 0.001, 0.001};
const volatile real32_T Cal_LQR_e2_gains[9] = {0.25, 0.25, 0.22, 0.18, 0.12,
                                               0.07, 0.07, 0.05, 0.05};
const volatile real32_T Cal_LQR_e2dot_gains[9] = {
    0.03, 0.028, 0.026, 0.024, 0.022, 0.015, 0.01, 0.015, 0.015};

const volatile real32_T Cal_LQR_Feedforward_gains[9] = {0.8, 0.8, 0.8, 0.8, 0.8,
                                                        0.7, 0.7, 0.6, 0.6};

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
#define CAL_STOP_CODE
#include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
#define ASW_QM_CORE2_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
// TJATCT variables define
real32_T TCTI_TimeSysCycle;        /* '<Root>/TCTI_TimeSysCycle'
                                    * Cycle Time for VEH task
                                    */
real32_T TCTI_VehicleVelX;         /* '<Root>/TCTI_VehicleVelX'
                                                                   * Vehicle Speed in
                                      miles/hour, computed based on the wheel speeds
                                                                     Negative values
                                      when vehicle moves backward
                                                                   */
real32_T TCTI_VehYawRate;          /* '<Root>/TCTI_VehYawRate' */
real32_T TCTI_WhlSteerAngleVdy;    /* '<Root>/TCTI_WhlSteerAngleVdy'
                                    * Offset
                                    * compensated steering wheel angle
                                    */
real32_T TCTI_SteerAngleLaDmc;     /* '<Root>/TCTI_SteerAngleLaDmc'
                                    * Offset
                                    * compensated steer angle at the front wheels
                                    * provided by the LatDMC
                                    */
boolean_T TCTI_EnaReplanCurValues; /* '<Root>/TCTI_EnaReplanCurValues'
                                    * UNDEFINED
                                    */
uint8_T TCTI_BtfTrajGuiQualifier;  /* '<Root>/TCTI_BtfTrajGuiQualifier'
                                    * Trajectory
                                    * Guidance Qualifier send by TPLFBT
                                    */
real32_T TCTI_ReqTrajDistYTpl;     /* '<Root>/TCTI_ReqTrajDistYTpl'
                                    * Lateral
                                    * distance of trajectory plan
                                    */
real32_T TCTI_CurTrajDistYTpl;     /* '<Root>/TCTI_CurTrajDistYTpl'
                                    * Current lateral
                                    * distance for trajectory plan
                                    */
real32_T TCTI_ReqTrajCrvTpl;       /* '<Root>/TCTI_ReqTrajCrvTpl'
                                    * Trajectory
                                    * planning setting curvature
                                    */
uint8_T TCTI_StCntrlFcn;           /* '<Root>/TCTI_StCntrlFcn'
                                    * Carries which
                                    * function is allowed to control
                                    */
uint8_T TCTI_StLatCtrlMode;        /* '<Root>/TCTI _StLatCtrlMode'
                                    * Lateral control mode
                                    */
real32_T TCTI_EstSelfSteerGrdnt;   /* '<Root>/TCTI_EstSelfSteerGrdnt'
                                    * Estimated
                                    * self steering gradient of the vehicle
                                    */
real32_T TCTI_ReqTrajCrvCsc;       /* '<Root>/TCTI_ReqTrajCrvCsc'
                                    * CSCLTA_OUTPUT
                                    */
real32_T TCTI_ReqTrajHeadTpl;      /* '<Root>/TCTI_ReqTrajHeadTpl'
                                    * UNDEFINED
                                    */
real32_T TCTI_InclPrevTrajHeadTpl; /* '<Root>/TCTI_InclPrevTrajHeadTpl' */
real32_T TCTI_MaxCrvGrdBuildup;    /* '<Root>/TCTI_MaxCrvGrdBuildup'
                                    * CSCLTA_OUTPUT
                                    */
real32_T TCTI_MaxCrvGrdRed;        /* '<Root>/TCTI_MaxCrvGrdRed'
                                    * CSCLTA_OUTPUT
                                    */
real32_T TCTI_LmtReqTrajCrvGrd;    /* '<Root>/TCTI_LmtReqTrajCrvGrd'
                                    * CSCLTA_OUTPUT
                                    */
real32_T TCTI_MaxTrajCrv;          /* '<Root>/TCTI_MaxTrajCrv'
                                    * CSCLTA_OUTPUT
                                    */
uint8_T TCTI_StVehOdo;             /* '<Root>/TCTI_StVehOdo'
                                    * UNDEFINED
                                    */
real32_T TCTI_VehCurvature;        /* '<Root>/TCTI_VehCrv'
                                    * Vehicle curvature
                                    */
uint8_T TCTI_EnaLmtActCsc;         /* '<Root>/TCTI_EnaLmtActCsc'
                                    * CSCLTA_OUTPUT
                                    */
real32_T TCTI_TimeLmtDur;          /* '<Root>/TCTI_TimeLmtDur'
                                    * CSCLTA_OUTPUT
                                    */
uint8_T TCTI_StLaneLaKmc;          /* '<Root>/TCTI_StLaneLaKmc'
                                    * Side of xDP Intervention
                                    */
uint8_T TCTI_StLcfSys;             /* '<Root>/TCTI_StLcfSys'
                                    * CSCLTA_OUTPUT
                                    */
uint8_T TestFlag_EnableFF;
uint8_T TestFlag_EnableTP;
uint8_T TestFlag_EnableOL;

real32_T Test_CoeffMainPGainLdc;
real32_T Test_CoeffPGainLdc;
real32_T Test_CoeffIGainLdc;
real32_T Test_CoeffDGainLdc;
real32_T Test_TimeDT1Ldc;
real32_T Test_CoeffPT1GainLdc;
real32_T Test_TimePT1Ldc;

real32_T Test_CoeffMainPGainCac;
real32_T Test_CoeffPGainCac;
real32_T Test_CoeffIGainCac;
real32_T Test_CoeffDGainCac;
real32_T Test_TimeDT1Cac;
real32_T Test_CoeffPT1GainCac;
real32_T Test_TimePT1Cac;

real32_T Test_CDCTimeFltCurHeading;
real32_T LQR_e1_gains[9];
real32_T LQR_e1dot_gains[9];
real32_T LQR_e2_gains[9];
real32_T LQR_e2dot_gains[9];
real32_T LQR_Feedforward_gains[9];
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  Functionname: TJATCT_Init                                  */ /*!

                                                                  @brief: TJATCT
                                                                function init
                                                                reset

                                                                  @description:the
                                                                function
                                                                first exec reset
                                                                process

                                                                  @param[in]:void

                                                                  @return: void
                                                                *****************************************************************************/
void TJATCT_Init(void) { TJATCT_initialize(); }

/*****************************************************************************
  Functionname: TJATCT_Exec                                  */ /*!

                                                                  @brief:
                                                                TJATCT Exec main
                                                                function

                                                                  @description:
                                                                TJATCT main
                                                                function

                                                                  @param[in]:reqPorts
                                                                TJATCT
                                                                input
                                                                                         params
                                                                TJATCT
                                                                parameter input
                                                                                         proPorts
                                                                TJATCT
                                                                output
                                                                                         debugInfo
                                                                TJATCT debug
                                                                information

                                                                  @return:void
                                                                *****************************************************************************/
void TJATCT_Exec(const sTJATCTInReq_st* reqPorts,
                 const sTJATCTParam_st* params,
                 sTJATCTOut_st* proPorts,
                 sTJATCTDebug_st* debugInfo) {
    debugInfo->uiVersionNum_nu = TJATCT_VIRSION;
    TJATCT_Input_initialize(reqPorts);
    TJATCT_step();
    // printf("TC : LGC_EnableCtrl_nu = %d\n", LGC_EnableCtrl_nu);
    TJATCT_Output_initialize(proPorts, debugInfo);

    debugInfo->CDC_BtfQualifier = CDC_BtfQualifier;
    debugInfo->CDC_CtrlErrDistY = CDC_CtrlErrDistY;
    debugInfo->CDC_CtrlErrHeading = CDC_CtrlErrHeading;
    debugInfo->CDC_EnaCntrlByTgq = CDC_EnaCntrlByTgq;
    debugInfo->CDC_EnaFreezeByTgq = CDC_EnaFreezeByTgq;
    debugInfo->CDC_EnaResetByTgq = CDC_EnaResetByTgq;
    debugInfo->CDC_EnaWatchdog = CDC_EnaWatchdog;
    debugInfo->CDC_EstCurHeading = CDC_EstCurHeading;
    debugInfo->CDC_EstDeltaTheta = CDC_EstDeltaTheta;
    debugInfo->CDC_FltDeltaTheta = CDC_FltDeltaTheta;
    debugInfo->CDC_FltErrDistYTpl = CDC_FltErrDistYTpl;
    debugInfo->CDC_HldCtrlErrDistY = CDC_HldCtrlErrDistY;
    debugInfo->CDC_HldCtrlErrHeading = CDC_HldCtrlErrHeading;
    debugInfo->CDC_PreErrCtrlHeading = CDC_PreErrCtrlHeading;
    debugInfo->CDC_RawBtfQualifier = CDC_RawBtfQualifier;
    debugInfo->CDC_RawCtrlErrDistY = CDC_RawCtrlErrDistY;
    debugInfo->CDC_RawCtrlErrHeading = CDC_RawCtrlErrHeading;
    debugInfo->CDC_RawDeltaTheta = CDC_RawDeltaTheta;
    debugInfo->CDC_RawErrDistYTpl = CDC_RawErrDistYTpl;
    debugInfo->CDC_RawFltDeltaTheta = CDC_RawFltDeltaTheta;
    debugInfo->CLM_CrvBySteerAngle = CLM_CrvBySteerAngle;
    debugInfo->CLM_EnaDegrReq = CLM_EnaDegrReq;
    debugInfo->CLM_EnaGrdDeltaFCmd = CLM_EnaGrdDeltaFCmd;
    debugInfo->CLM_EnaGrdFbcDc = CLM_EnaGrdFbcDc;
    debugInfo->CLM_EnaGrdFfcCrv = CLM_EnaGrdFfcCrv;
    debugInfo->CLM_EnaSatDeltaFCmd = CLM_EnaSatDeltaFCmd;
    debugInfo->CLM_EnaSatFbcDc = CLM_EnaSatFbcDc;
    debugInfo->CLM_EnaSatFfcCrv = CLM_EnaSatFfcCrv;
    debugInfo->CLM_GrdDeltaFCmd = CLM_GrdDeltaFCmd;
    debugInfo->CLM_GrdFbcDc = CLM_GrdFbcDc;
    debugInfo->CLM_GrdFfcCrv = CLM_GrdFfcCrv;
    debugInfo->CLM_RawEnaDegrReq = CLM_RawEnaDegrReq;
    debugInfo->CLM_RawGrdDeltaFCmd = CLM_RawGrdDeltaFCmd;
    debugInfo->CLM_RawGrdFbcDc = CLM_RawGrdFbcDc;
    debugInfo->CLM_RawGrdFfcCrv = CLM_RawGrdFfcCrv;
    debugInfo->CLM_SatDeltaFCmd = CLM_SatDeltaFCmd;
    debugInfo->CLM_SatFbcDc = CLM_SatFbcDc;
    debugInfo->CLM_SatFfcCrv = CLM_SatFfcCrv;
    debugInfo->CLM_ThdDeltaFCmdGrd = CLM_ThdDeltaFCmdGrd;
    debugInfo->CLM_ThdDeltaFCmdSat = CLM_ThdDeltaFCmdSat;
    debugInfo->CLM_ThdFbcDcGrd = CLM_ThdFbcDcGrd;
    debugInfo->CLM_ThdFbcDcSat = CLM_ThdFbcDcSat;
    debugInfo->CLM_ThdFfcCrvGrd = CLM_ThdFfcCrvGrd;
    debugInfo->CLM_ThdFfcCrvSat = CLM_ThdFfcCrvSat;
    debugInfo->DEV_BtfFfcQualifierPar = DEV_BtfFfcQualifierPar;
    debugInfo->DEV_BtfFfcQualifierRte = DEV_BtfFfcQualifierRte;
    debugInfo->DEV_CoeffDeltaGainFfc = DEV_CoeffDeltaGainFfc;
    debugInfo->DEV_CrvTestSignal = DEV_CrvTestSignal;
    debugInfo->DEV_DeltaFTestSignal = DEV_DeltaFTestSignal;
    debugInfo->DEV_DlySetDeltaF2DotPar = DEV_DlySetDeltaF2DotPar;
    debugInfo->DEV_DlySetDeltaF2DotRte = DEV_DlySetDeltaF2DotRte;
    debugInfo->DEV_DlySetDeltaFDotPar = DEV_DlySetDeltaFDotPar;
    debugInfo->DEV_DlySetDeltaFDotRte = DEV_DlySetDeltaFDotRte;
    debugInfo->DEV_DlySetDeltaFPar = DEV_DlySetDeltaFPar;
    debugInfo->DEV_DlySetDeltaFRte = DEV_DlySetDeltaFRte;
    debugInfo->DEV_EnaCntrlByTgq = DEV_EnaCntrlByTgq;
    debugInfo->DEV_EnaCrvGen = DEV_EnaCrvGen;
    debugInfo->DEV_EnaDeltaFGen = DEV_EnaDeltaFGen;
    debugInfo->DEV_EnaFreezeByTgq = DEV_EnaFreezeByTgq;
    debugInfo->DEV_EnaResetByTgq = DEV_EnaResetByTgq;
    debugInfo->DEV_HldReqDeltaFRte = DEV_HldReqDeltaFRte;
    debugInfo->DEV_ReqDeltaFRte = DEV_ReqDeltaFRte;
    debugInfo->DEV_RstCrvGen = DEV_RstCrvGen;
    debugInfo->DEV_RstDeltaFGen = DEV_RstDeltaFGen;
    debugInfo->DEV_SetDeltaF3DotPar = DEV_SetDeltaF3DotPar;
    debugInfo->DEV_SetDeltaF3DotRte = DEV_SetDeltaF3DotRte;
    debugInfo->DEV_SetDeltaFPar = DEV_SetDeltaFPar;
    debugInfo->DEV_SetDeltaFRte = DEV_SetDeltaFRte;
    debugInfo->DEV_TimeCrvGen = DEV_TimeCrvGen;
    debugInfo->DEV_TimeDeltaFGen = DEV_TimeDeltaFGen;
    debugInfo->DTE_CoeffA0TranferFcn = DTE_CoeffA0TranferFcn;
    debugInfo->DTE_CoeffA1TranferFcn = DTE_CoeffA1TranferFcn;
    debugInfo->DTE_CoeffB0TranferFcn = DTE_CoeffB0TranferFcn;
    debugInfo->DTE_CoeffB1TranferFcn = DTE_CoeffB1TranferFcn;
    debugInfo->DTE_CoeffB2TranferFcn = DTE_CoeffB2TranferFcn;
    debugInfo->DTE_CoeffDenS0LaDmc = DTE_CoeffDenS0LaDmc;
    debugInfo->DTE_CoeffDenS1LaDmc = DTE_CoeffDenS1LaDmc;
    debugInfo->DTE_CoeffDenS2LaDmc = DTE_CoeffDenS2LaDmc;
    debugInfo->DTE_CoeffDenS3LaDmc = DTE_CoeffDenS3LaDmc;
    debugInfo->DTE_CoeffNumS0LaDmc = DTE_CoeffNumS0LaDmc;
    debugInfo->DTE_CoeffNumS1LaDmc = DTE_CoeffNumS1LaDmc;
    debugInfo->DTE_Delta2DotForCrv = DTE_Delta2DotForCrv;
    debugInfo->DTE_Delta2DotLaDmc = DTE_Delta2DotLaDmc;
    debugInfo->DTE_Delta2DotVdyFcn = DTE_Delta2DotVdyFcn;
    debugInfo->DTE_Delta3DotLaDmc = DTE_Delta3DotLaDmc;
    debugInfo->DTE_DeltaByVdyFcn = DTE_DeltaByVdyFcn;
    debugInfo->DTE_DeltaDotForCrv = DTE_DeltaDotForCrv;
    debugInfo->DTE_DeltaDotLaDmc = DTE_DeltaDotLaDmc;
    debugInfo->DTE_DeltaF2DotPar = DTE_DeltaF2DotPar;
    debugInfo->DTE_DeltaF2DotRte = DTE_DeltaF2DotRte;
    debugInfo->DTE_DeltaF3DotPar = DTE_DeltaF3DotPar;
    debugInfo->DTE_DeltaF3DotRte = DTE_DeltaF3DotRte;
    debugInfo->DTE_DeltaFDotPar = DTE_DeltaFDotPar;
    debugInfo->DTE_DeltaFDotRte = DTE_DeltaFDotRte;
    debugInfo->DTE_DeltaFPar = DTE_DeltaFPar;
    debugInfo->DTE_DeltaFRte = DTE_DeltaFRte;
    debugInfo->DTE_DeltaVdyFcn = DTE_DeltaVdyFcn;
    debugInfo->DTE_DlyCurSteerAngle = DTE_DlyCurSteerAngle;
    debugInfo->DTE_DlyDeltaDotVdyFcn = DTE_DlyDeltaDotVdyFcn;
    debugInfo->DTE_DlyDeltaVdyFcn = DTE_DlyDeltaVdyFcn;
    debugInfo->DTE_DlySetCrvDotLaDmc = DTE_DlySetCrvDotLaDmc;
    debugInfo->DTE_DlySetCrvLaDmc = DTE_DlySetCrvLaDmc;
    debugInfo->DTE_DlySetDelta2DotLaDmc = DTE_DlySetDelta2DotLaDmc;
    debugInfo->DTE_DlySetDeltaDotLaDmc = DTE_DlySetDeltaDotLaDmc;
    debugInfo->DTE_DlySetDeltaLaDmc = DTE_DlySetDeltaLaDmc;
    debugInfo->DTE_EnaCtrlByTgq = DTE_EnaCtrlByTgq;
    debugInfo->DTE_EnaFreezeByTgq = DTE_EnaFreezeByTgq;
    debugInfo->DTE_EnaResetByTgq = DTE_EnaResetByTgq;
    debugInfo->DTE_EstCrvByBnkAgl = DTE_EstCrvByBnkAgl;
    debugInfo->DTE_FltDlyCurSteerAngle = DTE_FltDlyCurSteerAngle;
    debugInfo->DTE_HldReqCrvByBnkAgl = DTE_HldReqCrvByBnkAgl;
    debugInfo->DTE_HldReqCrvByDstrb = DTE_HldReqCrvByDstrb;
    debugInfo->DTE_HldReqDeltaByBnkAgl = DTE_HldReqDeltaByBnkAgl;
    debugInfo->DTE_HldReqDeltaByDstrb = DTE_HldReqDeltaByDstrb;
    debugInfo->DTE_KappaAngleLaDmc = DTE_KappaAngleLaDmc;
    debugInfo->DTE_LmtEstCrvByBnkAgl = DTE_LmtEstCrvByBnkAgl;
    debugInfo->DTE_LmtReqCrvByBnkAgl = DTE_LmtReqCrvByBnkAgl;
    debugInfo->DTE_LmtReqCrvByDstrb = DTE_LmtReqCrvByDstrb;
    debugInfo->DTE_LmtReqDeltaByBnkAgl = DTE_LmtReqDeltaByBnkAgl;
    debugInfo->DTE_LmtReqDeltaByDstrb = DTE_LmtReqDeltaByDstrb;
    debugInfo->DTE_LmtVehVelX = DTE_LmtVehVelX;
    debugInfo->DTE_MaxCrvByBnkAgl = DTE_MaxCrvByBnkAgl;
    debugInfo->DTE_MaxDeltaByBnkAgl = DTE_MaxDeltaByBnkAgl;
    debugInfo->DTE_MaxReqCrvByDstrb = DTE_MaxReqCrvByDstrb;
    debugInfo->DTE_MaxReqDeltaByDstrb = DTE_MaxReqDeltaByDstrb;
    debugInfo->DTE_NdlySetCrvLaDmc = DTE_NdlySetCrvLaDmc;
    debugInfo->DTE_NdlySetDeltaLaDmc = DTE_NdlySetDeltaLaDmc;
    debugInfo->DTE_Psi2DotVdyFcn = DTE_Psi2DotVdyFcn;
    debugInfo->DTE_Psi3DotVdyFcn = DTE_Psi3DotVdyFcn;
    debugInfo->DTE_RawCrvLaDmc = DTE_RawCrvLaDmc;
    debugInfo->DTE_RawDeltaDotLaDmc = DTE_RawDeltaDotLaDmc;
    debugInfo->DTE_RawDeltaFDotPar = DTE_RawDeltaFDotPar;
    debugInfo->DTE_RawDeltaFDotRte = DTE_RawDeltaFDotRte;
    debugInfo->DTE_RawFltEstCrvByBnkAgl = DTE_RawFltEstCrvByBnkAgl;
    debugInfo->DTE_RawLmtEstCrvByBnkAgl = DTE_RawLmtEstCrvByBnkAgl;
    debugInfo->DTE_RawReqCrvByBnkAgl = DTE_RawReqCrvByBnkAgl;
    debugInfo->DTE_RawReqCrvByDstrb = DTE_RawReqCrvByDstrb;
    debugInfo->DTE_RawReqDeltaByBnkAgl = DTE_RawReqDeltaByBnkAgl;
    debugInfo->DTE_RawReqDeltaByDstrb = DTE_RawReqDeltaByDstrb;
    debugInfo->DTE_ReqCrvByBnkAgl = DTE_ReqCrvByBnkAgl;
    debugInfo->DTE_ReqCrvByDstrb = DTE_ReqCrvByDstrb;
    debugInfo->DTE_ReqDeltaByBnkAgl = DTE_ReqDeltaByBnkAgl;
    debugInfo->DTE_ReqDeltaByDstrb = DTE_ReqDeltaByDstrb;
    debugInfo->DTE_ResCrvDenLaDmc = DTE_ResCrvDenLaDmc;
    debugInfo->DTE_ResDeltaDenLaDmc = DTE_ResDeltaDenLaDmc;
    debugInfo->DTE_ResDeltaDenPar = DTE_ResDeltaDenPar;
    debugInfo->DTE_ResDeltaDenRte = DTE_ResDeltaDenRte;
    debugInfo->DTE_ResDeltaDenVdyFcn = DTE_ResDeltaDenVdyFcn;
    debugInfo->DTE_SetCrv2DotLaDmc = DTE_SetCrv2DotLaDmc;
    debugInfo->DTE_SetCrvGainLaDmc = DTE_SetCrvGainLaDmc;
    debugInfo->DTE_SetCrvLaDmc = DTE_SetCrvLaDmc;
    debugInfo->DTE_SetDelta3DotLaDmc = DTE_SetDelta3DotLaDmc;
    debugInfo->DTE_SetDeltaGainLaDmc = DTE_SetDeltaGainLaDmc;
    debugInfo->DTE_SetDeltaLaDmc = DTE_SetDeltaLaDmc;
    debugInfo->EST_AngleCurSteer = EST_AngleCurSteer;
    debugInfo->EST_AngleLaDMCSteer = EST_AngleLaDMCSteer;
    debugInfo->EST_AnglePObsDTheta = EST_AnglePObsDTheta;
    debugInfo->EST_AnglePObsDThetaFreeze = EST_AnglePObsDThetaFreeze;
    debugInfo->EST_AnglePObsDThetaLmt0 = EST_AnglePObsDThetaLmt0;
    debugInfo->EST_AnglePObsDThetaLmt0Raw = EST_AnglePObsDThetaLmt0Raw;
    debugInfo->EST_AnglePObsDThetaLmt1 = EST_AnglePObsDThetaLmt1;
    debugInfo->EST_AnglePObsDThetaLmt2 = EST_AnglePObsDThetaLmt2;
    debugInfo->EST_AnglePObsDThetaSat = EST_AnglePObsDThetaSat;
    debugInfo->EST_AnglePObsDThetaSel = EST_AnglePObsDThetaSel;
    debugInfo->EST_AnglePObsDThetaThd = EST_AnglePObsDThetaThd;
    debugInfo->EST_AnglePObsDThetaThd0 = EST_AnglePObsDThetaThd0;
    debugInfo->EST_AngleVDYSteer = EST_AngleVDYSteer;
    debugInfo->EST_BetaDotPobs = EST_BetaDotPobs;
    debugInfo->EST_BetaDotSObs = EST_BetaDotSObs;
    debugInfo->EST_BetaSObs = EST_BetaSObs;
    debugInfo->EST_BtfQualifierByBeta = EST_BtfQualifierByBeta;
    debugInfo->EST_BtfQualifierByEna = EST_BtfQualifierByEna;
    debugInfo->EST_BtfQualifierByHdr = EST_BtfQualifierByHdr;
    debugInfo->EST_CoeffA11StateSpace = EST_CoeffA11StateSpace;
    debugInfo->EST_CoeffA12StateSpace = EST_CoeffA12StateSpace;
    debugInfo->EST_CoeffA21StateSpace = EST_CoeffA21StateSpace;
    debugInfo->EST_CoeffA22StateSpace = EST_CoeffA22StateSpace;
    // debugInfo->    EST_CoeffAXStateSpace[2]    =    EST_CoeffAXStateSpace[2]
    // ;
    debugInfo->EST_CoeffB11StateSpace = EST_CoeffB11StateSpace;
    debugInfo->EST_CoeffB21StateSpace = EST_CoeffB21StateSpace;
    debugInfo->EST_CoeffL11Pobs = EST_CoeffL11Pobs;
    debugInfo->EST_CoeffL11Sobs = EST_CoeffL11Sobs;
    debugInfo->EST_CoeffL12Pobs = EST_CoeffL12Pobs;
    debugInfo->EST_CoeffL13Pobs = EST_CoeffL13Pobs;
    debugInfo->EST_CoeffL21Pobs = EST_CoeffL21Pobs;
    debugInfo->EST_CoeffL21Sobs = EST_CoeffL21Sobs;
    debugInfo->EST_CoeffL22Pobs = EST_CoeffL22Pobs;
    debugInfo->EST_CoeffL23Pobs = EST_CoeffL23Pobs;
    debugInfo->EST_CoeffL31Pobs = EST_CoeffL31Pobs;
    debugInfo->EST_CoeffL32Pobs = EST_CoeffL32Pobs;
    debugInfo->EST_CoeffL33Pobs = EST_CoeffL33Pobs;
    debugInfo->EST_CoeffL41Pobs = EST_CoeffL41Pobs;
    debugInfo->EST_CoeffL42Pobs = EST_CoeffL42Pobs;
    debugInfo->EST_CoeffL43Pobs = EST_CoeffL43Pobs;
    // debugInfo->    EST_CoeffLPobs[12]    =    EST_CoeffLPobs[12]    ;
    // debugInfo->    EST_CoeffLYPObs[4]    =    EST_CoeffLYPObs[4]    ;
    // debugInfo->    EST_CoeffLYStateSpace[2]    =    EST_CoeffLYStateSpace[2]
    // ;
    debugInfo->EST_CrvPiObsCrvFlt = EST_CrvPiObsCrvFlt;
    debugInfo->EST_CrvPlObsIn = EST_CrvPlObsIn;
    debugInfo->EST_CurSteerAngle = EST_CurSteerAngle;
    debugInfo->EST_DThetaDotPobs = EST_DThetaDotPobs;
    debugInfo->EST_DYDotPobs = EST_DYDotPobs;
    debugInfo->EST_DeltaYPlObsIn = EST_DeltaYPlObsIn;
    debugInfo->EST_DistFromCgToGud = EST_DistFromCgToGud;
    debugInfo->EST_DistPObsDY = EST_DistPObsDY;
    debugInfo->EST_DistPObsDYFreeze = EST_DistPObsDYFreeze;
    debugInfo->EST_DistPObsDYGrdnt = EST_DistPObsDYGrdnt;
    debugInfo->EST_DistPObsDYGrdntThd = EST_DistPObsDYGrdntThd;
    debugInfo->EST_DistPObsDYSat = EST_DistPObsDYSat;
    debugInfo->EST_DistPObsDYSel = EST_DistPObsDYSel;
    debugInfo->EST_DistPObsDYThd = EST_DistPObsDYThd;
    debugInfo->EST_DistPobsDYGrdntRaw = EST_DistPobsDYGrdntRaw;
    debugInfo->EST_DistYDevByGrdntLmt1 = EST_DistYDevByGrdntLmt1;
    debugInfo->EST_DistYDevStep = EST_DistYDevStep;
    debugInfo->EST_DistYDevTrajFromCur = EST_DistYDevTrajFromCur;
    debugInfo->EST_DlyCurSteerAngle = EST_DlyCurSteerAngle;
    debugInfo->EST_EnaActvtGrdntLmt1 = EST_EnaActvtGrdntLmt1;
    debugInfo->EST_EnaActvtGrdntLmt2 = EST_EnaActvtGrdntLmt2;
    debugInfo->EST_EnaBetaSatSObs = EST_EnaBetaSatSObs;
    debugInfo->EST_EnaByMeanHdr = EST_EnaByMeanHdr;
    debugInfo->EST_EnaByMulHdrPerc = EST_EnaByMulHdrPerc;
    debugInfo->EST_EnaCntrlByTgq = EST_EnaCntrlByTgq;
    debugInfo->EST_EnaFreezeByTgq = EST_EnaFreezeByTgq;
    debugInfo->EST_EnaLmt2ByDistY = EST_EnaLmt2ByDistY;
    debugInfo->EST_EnaLmtByDistY = EST_EnaLmtByDistY;
    debugInfo->EST_EnaPObsDThetaLmt0 = EST_EnaPObsDThetaLmt0;
    debugInfo->EST_EnaPObsDThetaLmt1 = EST_EnaPObsDThetaLmt1;
    debugInfo->EST_EnaPObsDThetaLmt2 = EST_EnaPObsDThetaLmt2;
    debugInfo->EST_EnaPObsDThetaRst1 = EST_EnaPObsDThetaRst1;
    debugInfo->EST_EnaPObsDThetaRst2 = EST_EnaPObsDThetaRst2;
    debugInfo->EST_EnaPObsDThetaSat = EST_EnaPObsDThetaSat;
    debugInfo->EST_EnaPObsDYGrdnt = EST_EnaPObsDYGrdnt;
    debugInfo->EST_EnaPObsDYGrdntRaw = EST_EnaPObsDYGrdntRaw;
    debugInfo->EST_EnaPObsDYSat = EST_EnaPObsDYSat;
    debugInfo->EST_EnaResetByTgq = EST_EnaResetByTgq;
    debugInfo->EST_ErrVehYawRate = EST_ErrVehYawRate;
    debugInfo->EST_EstBetaPobs = EST_EstBetaPobs;
    debugInfo->EST_EstBetaSObs = EST_EstBetaSObs;
    debugInfo->EST_EstDThetaPobs = EST_EstDThetaPobs;
    debugInfo->EST_EstDYPobs = EST_EstDYPobs;
    debugInfo->EST_EstPsiDotPobs = EST_EstPsiDotPobs;
    debugInfo->EST_EstPsiDotSObs = EST_EstPsiDotSObs;
    debugInfo->EST_FacDThetaWghtHdrSel = EST_FacDThetaWghtHdrSel;
    debugInfo->EST_FacDYWghtHdrSel = EST_FacDYWghtHdrSel;
    debugInfo->EST_FltDThetaDotPObs = EST_FltDThetaDotPObs;
    debugInfo->EST_FltDYDotPObs = EST_FltDYDotPObs;
    debugInfo->EST_HdrPercByDY = EST_HdrPercByDY;
    debugInfo->EST_HdrPercByTheta = EST_HdrPercByTheta;
    debugInfo->EST_HldBetaSObs = EST_HldBetaSObs;
    debugInfo->EST_LmtBetaSObs = EST_LmtBetaSObs;
    debugInfo->EST_LmtHdrPercByDY = EST_LmtHdrPercByDY;
    debugInfo->EST_LmtHdrPercByTheta = EST_LmtHdrPercByTheta;
    debugInfo->EST_LmtVehVelX = EST_LmtVehVelX;
    debugInfo->EST_MeanHdrPerc = EST_MeanHdrPerc;
    debugInfo->EST_ModeSelParHdr = EST_ModeSelParHdr;
    debugInfo->EST_MulHdrPerc = EST_MulHdrPerc;
    debugInfo->EST_Psi2DotPobs = EST_Psi2DotPobs;
    debugInfo->EST_Psi2DotSObs = EST_Psi2DotSObs;
    debugInfo->EST_RatioSteerGear = EST_RatioSteerGear;
    debugInfo->EST_RawBetaSObs = EST_RawBetaSObs;
    debugInfo->EST_RawEstBetaPobs = EST_RawEstBetaPobs;
    debugInfo->EST_RawEstDThetaPobs = EST_RawEstDThetaPobs;
    debugInfo->EST_RawEstDYPobs = EST_RawEstDYPobs;
    debugInfo->EST_RawEstPsiDotPobs = EST_RawEstPsiDotPobs;
    debugInfo->EST_RawFltDThetaDotPObs = EST_RawFltDThetaDotPObs;
    debugInfo->EST_RawFltDYDotPObs = EST_RawFltDYDotPObs;
    debugInfo->EST_RawHdrPercByDY = EST_RawHdrPercByDY;
    debugInfo->EST_RawHdrPercByTheta = EST_RawHdrPercByTheta;
    debugInfo->EST_ThdBetaSatSObs = EST_ThdBetaSatSObs;
    debugInfo->EST_ThdMeanHdrSel = EST_ThdMeanHdrSel;
    debugInfo->EST_ThdMulHdrSel = EST_ThdMulHdrSel;
    debugInfo->FFC_HldReqFfcCrv = FFC_HldReqFfcCrv;
    debugInfo->FFC_ReqFfcCrv = FFC_ReqFfcCrv;
    debugInfo->LGC_ActiveLgcParamSet_nu = LGC_ActiveLgcParamSet_nu;
    debugInfo->LGC_CacIntReset_nu = LGC_CacIntReset_nu;
    debugInfo->LGC_CacPT1Reset_nu = LGC_CacPT1Reset_nu;
    debugInfo->LGC_CdcCmd_rad = LGC_CdcCmd_rad;
    debugInfo->LGC_Cmpn2DotLaDmcCas = LGC_Cmpn2DotLaDmcCas;
    debugInfo->LGC_Cmpn2DotLaDmcCdc = LGC_Cmpn2DotLaDmcCdc;
    debugInfo->LGC_Cmpn2DotLaDmcLdc = LGC_Cmpn2DotLaDmcLdc;
    debugInfo->LGC_CmpnDotLaDmcCas = LGC_CmpnDotLaDmcCas;
    debugInfo->LGC_CmpnDotLaDmcCdc = LGC_CmpnDotLaDmcCdc;
    debugInfo->LGC_CmpnDotLaDmcLdc = LGC_CmpnDotLaDmcLdc;
    debugInfo->LGC_CmpnLaDmcCas = LGC_CmpnLaDmcCas;
    debugInfo->LGC_CmpnLaDmcCdc = LGC_CmpnLaDmcCdc;
    debugInfo->LGC_CmpnLaDmcLdc = LGC_CmpnLaDmcLdc;
    debugInfo->LGC_CoeffDGainCac = LGC_CoeffDGainCac;
    debugInfo->LGC_CoeffDGainLcCac = LGC_CoeffDGainLcCac;
    debugInfo->LGC_CoeffDGainLcLdc = LGC_CoeffDGainLcLdc;
    debugInfo->LGC_CoeffDGainLdc = LGC_CoeffDGainLdc;
    debugInfo->LGC_CoeffDGainOfCac = LGC_CoeffDGainOfCac;
    debugInfo->LGC_CoeffDGainOfLdc = LGC_CoeffDGainOfLdc;
    debugInfo->LGC_CoeffDGainSfCac = LGC_CoeffDGainSfCac;
    debugInfo->LGC_CoeffDGainSfLdc = LGC_CoeffDGainSfLdc;
    debugInfo->LGC_CoeffIGainCac = LGC_CoeffIGainCac;
    debugInfo->LGC_CoeffIGainLcCac = LGC_CoeffIGainLcCac;
    debugInfo->LGC_CoeffIGainLcLdc = LGC_CoeffIGainLcLdc;
    debugInfo->LGC_CoeffIGainLdc = LGC_CoeffIGainLdc;
    debugInfo->LGC_CoeffIGainOfCac = LGC_CoeffIGainOfCac;
    debugInfo->LGC_CoeffIGainOfLdc = LGC_CoeffIGainOfLdc;
    debugInfo->LGC_CoeffIGainSfCac = LGC_CoeffIGainSfCac;
    debugInfo->LGC_CoeffIGainSfLdc = LGC_CoeffIGainSfLdc;
    debugInfo->LGC_CoeffMainPGainCac = LGC_CoeffMainPGainCac;
    debugInfo->LGC_CoeffMainPGainLdc = LGC_CoeffMainPGainLdc;
    debugInfo->LGC_CoeffNumS0LaDmc = LGC_CoeffNumS0LaDmc;
    debugInfo->LGC_CoeffNumS1LaDmc = LGC_CoeffNumS1LaDmc;
    debugInfo->LGC_CoeffPGainByCrvCac = LGC_CoeffPGainByCrvCac;
    debugInfo->LGC_CoeffPGainByCrvLdc = LGC_CoeffPGainByCrvLdc;
    debugInfo->LGC_CoeffPGainCac = LGC_CoeffPGainCac;
    debugInfo->LGC_CoeffPGainLcCac = LGC_CoeffPGainLcCac;
    debugInfo->LGC_CoeffPGainLcLdc = LGC_CoeffPGainLcLdc;
    debugInfo->LGC_CoeffPGainLdc = LGC_CoeffPGainLdc;
    debugInfo->LGC_CoeffPGainOfCac = LGC_CoeffPGainOfCac;
    debugInfo->LGC_CoeffPGainOfLdc = LGC_CoeffPGainOfLdc;
    debugInfo->LGC_CoeffPGainSfCac = LGC_CoeffPGainSfCac;
    debugInfo->LGC_CoeffPGainSfLdc = LGC_CoeffPGainSfLdc;
    debugInfo->LGC_CoeffPT1GainCac = LGC_CoeffPT1GainCac;
    debugInfo->LGC_CoeffPT1GainLcCac = LGC_CoeffPT1GainLcCac;
    debugInfo->LGC_CoeffPT1GainLcLdc = LGC_CoeffPT1GainLcLdc;
    debugInfo->LGC_CoeffPT1GainLdc = LGC_CoeffPT1GainLdc;
    debugInfo->LGC_CoeffPT1GainOfCac = LGC_CoeffPT1GainOfCac;
    debugInfo->LGC_CoeffPT1GainOfLdc = LGC_CoeffPT1GainOfLdc;
    debugInfo->LGC_CoeffPT1GainSfCac = LGC_CoeffPT1GainSfCac;
    debugInfo->LGC_CoeffPT1GainSfLdc = LGC_CoeffPT1GainSfLdc;
    debugInfo->LGC_CoeffPole1LaDmc = LGC_CoeffPole1LaDmc;
    debugInfo->LGC_CoeffPole2LaDmc = LGC_CoeffPole2LaDmc;
    debugInfo->LGC_CrvReqBAC_1pm = LGC_CrvReqBAC_1pm;
    debugInfo->LGC_CrvReqDte_1pm = LGC_CrvReqDte_1pm;
    debugInfo->LGC_CrvReqFfcFrz_1pm = LGC_CrvReqFfcFrz_1pm;
    debugInfo->LGC_CrvReqFfcGrdLimT1_1pm = LGC_CrvReqFfcGrdLimT1_1pm;
    debugInfo->LGC_CrvReqFfcGrdLimT2_1pm = LGC_CrvReqFfcGrdLimT2_1pm;
    debugInfo->LGC_CrvReqFfcGrdLim_1pm = LGC_CrvReqFfcGrdLim_1pm;
    debugInfo->LGC_CtrlCrv_1pm = LGC_CtrlCrv_1pm;
    debugInfo->LGC_CtrlCrv_DE_1pm = LGC_CtrlCrv_DE_1pm;
    debugInfo->LGC_CtrlErrHeadAglCrtd_rad = LGC_CtrlErrHeadAglCrtd_rad;
    debugInfo->LGC_CtrlErrMainPGain = LGC_CtrlErrMainPGain;
    debugInfo->LGC_CtrlErrMainPGainCas = LGC_CtrlErrMainPGainCas;
    debugInfo->LGC_CtrlErrMainPGainCdc = LGC_CtrlErrMainPGainCdc;
    debugInfo->LGC_DeltaByBnkAglComp_deg = LGC_DeltaByBnkAglComp_deg;
    debugInfo->LGC_DeltaFBAC_deg = LGC_DeltaFBAC_deg;
    debugInfo->LGC_DeltaFCmdCdc = LGC_DeltaFCmdCdc;
    debugInfo->LGC_DeltaFCmdDC_deg = LGC_DeltaFCmdDC_deg;
    debugInfo->LGC_DeltaFCmdFFC_deg = LGC_DeltaFCmdFFC_deg;
    debugInfo->LGC_DeltaFCmdUnlimited_deg = LGC_DeltaFCmdUnlimited_deg;
    debugInfo->LGC_DeltaFCmd_deg = LGC_DeltaFCmd_deg;
    debugInfo->LGC_DeltaFCmd_rad = LGC_DeltaFCmd_rad;
    debugInfo->LGC_DeltaFDGainCas = LGC_DeltaFDGainCas;
    debugInfo->LGC_DeltaFDGainCdc = LGC_DeltaFDGainCdc;
    debugInfo->LGC_DeltaFDGainLdc = LGC_DeltaFDGainLdc;
    debugInfo->LGC_DeltaFIGainCas = LGC_DeltaFIGainCas;
    debugInfo->LGC_DeltaFIGainCdc = LGC_DeltaFIGainCdc;
    debugInfo->LGC_DeltaFIGainLdc = LGC_DeltaFIGainLdc;
    debugInfo->LGC_DeltaFPGainCas = LGC_DeltaFPGainCas;
    debugInfo->LGC_DeltaFPGainCdc = LGC_DeltaFPGainCdc;
    debugInfo->LGC_DeltaFPGainLdc = LGC_DeltaFPGainLdc;
    debugInfo->LGC_DeltaFPT1GainCas = LGC_DeltaFPT1GainCas;
    debugInfo->LGC_DeltaFPT1GainCdc = LGC_DeltaFPT1GainCdc;
    debugInfo->LGC_DeltaFPT1GainLdc = LGC_DeltaFPT1GainLdc;
    debugInfo->LGC_EnaActObjFollow = LGC_EnaActObjFollow;
    debugInfo->LGC_EnaActSafetyFcn = LGC_EnaActSafetyFcn;
    debugInfo->LGC_EnaCntrlByTgq = LGC_EnaCntrlByTgq;
    debugInfo->LGC_EnaFreezeByTgq = LGC_EnaFreezeByTgq;
    debugInfo->LGC_EnaModeChangeCas = LGC_EnaModeChangeCas;
    debugInfo->LGC_EnaModeChangeCdc = LGC_EnaModeChangeCdc;
    debugInfo->LGC_EnaModeChangeDtct = LGC_EnaModeChangeDtct;
    debugInfo->LGC_EnaPGainGrdLmtCas = LGC_EnaPGainGrdLmtCas;
    debugInfo->LGC_EnaPGainGrdLmtCdc = LGC_EnaPGainGrdLmtCdc;
    debugInfo->LGC_EnaPGainGrdLmtLdc = LGC_EnaPGainGrdLmtLdc;
    debugInfo->LGC_EnaPGainGrdSignCas = LGC_EnaPGainGrdSignCas;
    debugInfo->LGC_EnaPGainGrdSignCdc = LGC_EnaPGainGrdSignCdc;
    debugInfo->LGC_EnaPGainGrdSignLdc = LGC_EnaPGainGrdSignLdc;
    debugInfo->LGC_EnaPGainGrdThdCas = LGC_EnaPGainGrdThdCas;
    debugInfo->LGC_EnaPGainGrdThdCdc = LGC_EnaPGainGrdThdCdc;
    debugInfo->LGC_EnaPGainGrdThdLdc = LGC_EnaPGainGrdThdLdc;
    debugInfo->LGC_EnaResetByTgq = LGC_EnaResetByTgq;
    debugInfo->LGC_EnaRstByDistY = LGC_EnaRstByDistY;
    debugInfo->LGC_EnaRstByStandStill = LGC_EnaRstByStandStill;
    debugInfo->LGC_EnaRstByTrq = LGC_EnaRstByTrq;
    debugInfo->LGC_EnaRstIntCac = LGC_EnaRstIntCac;
    debugInfo->LGC_EnaRstIntLdc = LGC_EnaRstIntLdc;
    debugInfo->LGC_EnaRstPT1Cac = LGC_EnaRstPT1Cac;
    debugInfo->LGC_EnaRstPT1Ldc = LGC_EnaRstPT1Ldc;
    debugInfo->LGC_EnableCtrl_nu = LGC_EnableCtrl_nu;
    debugInfo->LGC_ErrCourse2DotCas = LGC_ErrCourse2DotCas;
    debugInfo->LGC_ErrCourse2DotCdc = LGC_ErrCourse2DotCdc;
    debugInfo->LGC_ErrCourseDotCas = LGC_ErrCourseDotCas;
    debugInfo->LGC_ErrCourseDotCdc = LGC_ErrCourseDotCdc;
    debugInfo->LGC_ErrCtrlCourseCas = LGC_ErrCtrlCourseCas;
    debugInfo->LGC_ErrCtrlCourseCdc = LGC_ErrCtrlCourseCdc;
    debugInfo->LGC_ErrCtrlDistY = LGC_ErrCtrlDistY;
    debugInfo->LGC_ErrDistY2Dot = LGC_ErrDistY2Dot;
    debugInfo->LGC_ErrDistYDot = LGC_ErrDistYDot;
    debugInfo->LGC_FFCrv_1pm = LGC_FFCrv_1pm;
    debugInfo->LGC_FltErrCourseCas = LGC_FltErrCourseCas;
    debugInfo->LGC_FltErrCourseCdc = LGC_FltErrCourseCdc;
    debugInfo->LGC_FltErrCourseDotCas = LGC_FltErrCourseDotCas;
    debugInfo->LGC_FltErrCourseDotCdc = LGC_FltErrCourseDotCdc;
    debugInfo->LGC_FltPT1YErr_met = LGC_FltPT1YErr_met;
    debugInfo->LGC_FltRawErrDistYDot = LGC_FltRawErrDistYDot;
    debugInfo->LGC_HldReqDeltaF = LGC_HldReqDeltaF;
    debugInfo->LGC_Hold_nu = LGC_Hold_nu;
    debugInfo->LGC_LdcAloneICmd_rad = LGC_LdcAloneICmd_rad;
    debugInfo->LGC_LdcCmd_rad = LGC_LdcCmd_rad;
    debugInfo->LGC_LdcIntReset_nu = LGC_LdcIntReset_nu;
    debugInfo->LGC_LdcPT1Reset_nu = LGC_LdcPT1Reset_nu;
    debugInfo->LGC_LmtCoeffPGainCas = LGC_LmtCoeffPGainCas;
    debugInfo->LGC_LmtCoeffPGainCdc = LGC_LmtCoeffPGainCdc;
    debugInfo->LGC_LmtCoeffPGainLdc = LGC_LmtCoeffPGainLdc;
    debugInfo->LGC_LmtReqDeltaF = LGC_LmtReqDeltaF;
    debugInfo->LGC_LmtSelReqDeltaF = LGC_LmtSelReqDeltaF;
    debugInfo->LGC_MaxReqDeltaF = LGC_MaxReqDeltaF;
    debugInfo->LGC_RawCmpn2DotLaDmcCas = LGC_RawCmpn2DotLaDmcCas;
    debugInfo->LGC_RawCmpn2DotLaDmcCdc = LGC_RawCmpn2DotLaDmcCdc;
    debugInfo->LGC_RawCmpn2DotLaDmcLdc = LGC_RawCmpn2DotLaDmcLdc;
    debugInfo->LGC_RawErrCourseDotCas = LGC_RawErrCourseDotCas;
    debugInfo->LGC_RawErrCourseDotCdc = LGC_RawErrCourseDotCdc;
    debugInfo->LGC_RawErrDistYDot = LGC_RawErrDistYDot;
    debugInfo->LGC_RawFfcDeltaF = LGC_RawFfcDeltaF;
    debugInfo->LGC_RawFltErrCourseCas = LGC_RawFltErrCourseCas;
    debugInfo->LGC_RawFltErrCourseCdc = LGC_RawFltErrCourseCdc;
    debugInfo->LGC_RawFltErrCtrlDistY = LGC_RawFltErrCtrlDistY;
    debugInfo->LGC_RawLmtCoeffPGainCas = LGC_RawLmtCoeffPGainCas;
    debugInfo->LGC_RawLmtCoeffPGainCdc = LGC_RawLmtCoeffPGainCdc;
    debugInfo->LGC_RawLmtCoeffPGainLdc = LGC_RawLmtCoeffPGainLdc;
    debugInfo->LGC_ReqDeltaF = LGC_ReqDeltaF;
    debugInfo->LGC_Reset_nu = LGC_Reset_nu;
    debugInfo->LGC_SafetyFunctionActive_nu = LGC_SafetyFunctionActive_nu;
    debugInfo->LGC_StActParSet = LGC_StActParSet;
    debugInfo->LGC_SumCrvReqFbFrz_1pm = LGC_SumCrvReqFbFrz_1pm;
    debugInfo->LGC_SumCrvReqFbGrdLim_1pm = LGC_SumCrvReqFbGrdLim_1pm;
    debugInfo->LGC_SumCrvReqFbSatLim_1pm = LGC_SumCrvReqFbSatLim_1pm;
    debugInfo->LGC_SumCrvReqFb_1pm = LGC_SumCrvReqFb_1pm;
    debugInfo->LGC_TgtCrv_DENoLatSlp_1pm = LGC_TgtCrv_DENoLatSlp_1pm;
    debugInfo->LGC_TgtCrv_DE_1pm = LGC_TgtCrv_DE_1pm;
    debugInfo->LGC_TgtCrv_NoDE_1pm = LGC_TgtCrv_NoDE_1pm;
    debugInfo->LGC_TimeDT1Cac = LGC_TimeDT1Cac;
    debugInfo->LGC_TimeDT1LcCac = LGC_TimeDT1LcCac;
    debugInfo->LGC_TimeDT1LcLdc = LGC_TimeDT1LcLdc;
    debugInfo->LGC_TimeDT1Ldc = LGC_TimeDT1Ldc;
    debugInfo->LGC_TimeDT1OfCac = LGC_TimeDT1OfCac;
    debugInfo->LGC_TimeDT1OfLdc = LGC_TimeDT1OfLdc;
    debugInfo->LGC_TimeDT1SfCac = LGC_TimeDT1SfCac;
    debugInfo->LGC_TimeDT1SfLdc = LGC_TimeDT1SfLdc;
    debugInfo->LGC_TimeFltErrCourse = LGC_TimeFltErrCourse;
    debugInfo->LGC_TimeFltErrCourseCdc = LGC_TimeFltErrCourseCdc;
    debugInfo->LGC_TimeFltErrDistY = LGC_TimeFltErrDistY;
    debugInfo->LGC_TimePT1Cac = LGC_TimePT1Cac;
    debugInfo->LGC_TimePT1DeltaFCmd = LGC_TimePT1DeltaFCmd;
    debugInfo->LGC_TimePT1LcCac = LGC_TimePT1LcCac;
    debugInfo->LGC_TimePT1LcLdc = LGC_TimePT1LcLdc;
    debugInfo->LGC_TimePT1Ldc = LGC_TimePT1Ldc;
    debugInfo->LGC_TimePT1OfCac = LGC_TimePT1OfCac;
    debugInfo->LGC_TimePT1OfLdc = LGC_TimePT1OfLdc;
    debugInfo->LGC_TimePT1SfCac = LGC_TimePT1SfCac;
    debugInfo->LGC_TimePT1SfLdc = LGC_TimePT1SfLdc;
    debugInfo->TCTI_ActualTrqEPS = TCTI_ActualTrqEPS;
    debugInfo->TCTI_NegReqTrajHeadTpl = TCTI_NegReqTrajHeadTpl;
    debugInfo->TCTI_RoadBankAngle = TCTI_RoadBankAngle;
    debugInfo->TC_EnaUnplauUnitDelay_bool = TC_EnaUnplauUnitDelay_bool;
    debugInfo->TC_Freeze1RSFlipFlop_bool = TC_Freeze1RSFlipFlop_bool;
    debugInfo->TC_Freeze2RSFlipFlop_bool = TC_Freeze2RSFlipFlop_bool;
    debugInfo->TC_FreezeRSFlipFlop_bool = TC_FreezeRSFlipFlop_bool;
    debugInfo->TC_HoldWarnRSFlipFlop_bool = TC_HoldWarnRSFlipFlop_bool;

    debugInfo->LQR_DeltaF_Cmd_rad = LQR_DeltaF_Cmd_rad; /* '<S78>/Add' */
    debugInfo->LQR_DeltaF_feedback_rad =
        LQR_DeltaF_feedback_rad; /* '<S85>/MatrixMultiply' */
    debugInfo->LQR_DeltaF_feedforward_rad =
        LQR_DeltaF_feedforward_rad; /* '<S86>/MATLAB Function' */
    debugInfo->LQR_EnaCntrlByTgq = LQR_EnaCntrlByTgq;   /* '<S87>/Equal1' */
    debugInfo->LQR_EnaFreezeByTgq = LQR_EnaFreezeByTgq; /* '<S87>/Equal' */
    debugInfo->LQR_EnaResetByTgq = LQR_EnaResetByTgq;   /* '<S87>/OR' */
    debugInfo->LQR_I_term_rad = LQR_I_term_rad;         /* '<S93>/Switch2' */
    debugInfo->LQR_MatK_k1 = LQR_MatK_k1; /* '<S85>/Signal Conversion' */
    debugInfo->LQR_MatK_k2 = LQR_MatK_k2; /* '<S85>/Signal Conversion1' */
    debugInfo->LQR_MatK_k3 = LQR_MatK_k3; /* '<S85>/Signal Conversion2' */
    debugInfo->LQR_MatK_k4 = LQR_MatK_k4; /* '<S85>/Signal Conversion3' */
    debugInfo->LQR_ReqDeltaF_Limit_deg =
        LQR_ReqDeltaF_Limit_deg; /* '<S105>/Multiport Switch' */
    debugInfo->LQR_e1_contribution = LQR_e1_contribution; /* '<S85>/Product' */
    debugInfo->LQR_e1dot_contribution =
        LQR_e1dot_contribution;                           /* '<S85>/Product1' */
    debugInfo->LQR_e2_contribution = LQR_e2_contribution; /* '<S85>/Product2' */
    debugInfo->LQR_e2dot_contribution =
        LQR_e2dot_contribution; /* '<S85>/Product3' */
    debugInfo->LQR_heading_error =
        LQR_heading_error; /* '<S75>/Data Type Conversion12' */
    debugInfo->LQR_heading_error_rate =
        LQR_heading_error_rate; /* '<S89>/Subtract' */
    debugInfo->LQR_lateral_error =
        LQR_lateral_error; /* '<S75>/Data Type Conversion11' */
    debugInfo->LQR_lateral_error_rate =
        LQR_lateral_error_rate; /* '<S89>/Product5' */
    debugInfo->LQR_num_iteration = LQR_num_iteration;
    debugInfo->LQR_yawrate_term = LQR_yawrate_term;
    debugInfo->LQR_DeltaF_Lead_Cmd_rad = LQR_DeltaF_Lead_Cmd_rad;
}

/*****************************************************************************
  Functionname: TJATCT_Input_initialize                                  */ /*!

                  @brief: write TJATCT reqPorts input to inner

                  @description: write TJATCT reqPorts input to inner

                  @param[in]:reqPorts   TJATCT input

                  @return:void
                *****************************************************************************/
void TJATCT_Input_initialize(const sTJATCTInReq_st* reqPorts) {
    TCTI_TimeSysCycle = reqPorts->TCTI_TimeSysCycle;
    TCTI_VehicleVelX = reqPorts->TCTI_VehicleVelX;
    TCTI_VehYawRate = reqPorts->TCTI_VehYawRate;
    TCTI_WhlSteerAngleVdy = reqPorts->TCTI_WhlSteerAngleVdy;
    TCTI_SteerAngleLaDmc = reqPorts->TCTI_SteerAngleLaDmc;
    TCTI_EnaReplanCurValues = reqPorts->TCTI_EnaReplanCurValues;
    TCTI_BtfTrajGuiQualifier = reqPorts->TCTI_BtfTrajGuiQualifier;
    TCTI_ReqTrajDistYTpl = reqPorts->TCTI_ReqTrajDistYTpl;
    TCTI_CurTrajDistYTpl = reqPorts->TCTI_CurTrajDistYTpl;
    TCTI_ReqTrajCrvTpl = reqPorts->TCTI_ReqTrajCrvTpl;
    TCTI_StCntrlFcn = reqPorts->TCTI_StCntrlFcn;
    TCTI_StLatCtrlMode = reqPorts->TCTI_StLatCtrlMode;
    TCTI_EstSelfSteerGrdnt = reqPorts->TCTI_EstSelfSteerGrdnt;
    TCTI_ReqTrajCrvCsc = reqPorts->TCTI_ReqTrajCrvCsc;
    TCTI_ReqTrajHeadTpl = reqPorts->TCTI_ReqTrajHeadTpl;
    TCTI_InclPrevTrajHeadTpl = reqPorts->TCTI_InclPrevTrajHeadTpl;
    TCTI_MaxCrvGrdBuildup = reqPorts->TCTI_MaxCrvGrdBuildup;
    TCTI_MaxCrvGrdRed = reqPorts->TCTI_MaxCrvGrdRed;
    TCTI_LmtReqTrajCrvGrd = reqPorts->TCTI_LmtReqTrajCrvGrd;
    TCTI_MaxTrajCrv = reqPorts->TCTI_MaxTrajCrv;
    TCTI_StVehOdo = reqPorts->TCTI_StVehOdo;
    TCTI_VehCurvature = reqPorts->TCTI_VehCurvature;
    TCTI_EnaLmtActCsc = reqPorts->TCTI_EnaLmtActCsc;
    TCTI_TimeLmtDur = reqPorts->TCTI_TimeLmtDur;
    TCTI_StLaneLaKmc = reqPorts->TCTI_StLaneLaKmc;
    TCTI_StLcfSys = reqPorts->TCTI_StLcfSys;

    TestFlag_EnableFF = Cal_TestFlag_EnableFF;
    TestFlag_EnableTP = Cal_TestFlag_EnableTP;
    TestFlag_EnableOL = Cal_TestFlag_EnableOL;
    Test_CoeffMainPGainLdc = Cal_Test_CoeffMainPGainLdc;
    Test_CoeffPGainLdc = Cal_Test_CoeffPGainLdc;
    Test_CoeffIGainLdc = Cal_Test_CoeffIGainLdc;
    Test_CoeffDGainLdc = Cal_Test_CoeffDGainLdc;
    Test_TimeDT1Ldc = Cal_Test_TimeDT1Ldc;
    Test_CoeffPT1GainLdc = Cal_Test_CoeffPT1GainLdc;
    Test_TimePT1Ldc = Cal_Test_TimePT1Ldc;
    Test_CoeffMainPGainCac = Cal_Test_CoeffMainPGainCac;
    Test_CoeffPGainCac = Cal_Test_CoeffPGainCac;
    Test_CoeffIGainCac = Cal_Test_CoeffIGainCac;
    Test_CoeffDGainCac = Cal_Test_CoeffDGainCac;
    Test_TimeDT1Cac = Cal_Test_TimeDT1Cac;
    Test_CoeffPT1GainCac = Cal_Test_CoeffPT1GainCac;
    Test_TimePT1Cac = Cal_Test_TimePT1Cac;
    Test_CDCTimeFltCurHeading = Cal_Test_CDCTimeFltCurHeading;
    memcpy(LQR_e1_gains, Cal_LQR_e1_gains, 9 * sizeof(real32_T));
    memcpy(LQR_e1dot_gains, Cal_LQR_e1dot_gains, 9 * sizeof(real32_T));
    memcpy(LQR_e2_gains, Cal_LQR_e2_gains, 9 * sizeof(real32_T));
    memcpy(LQR_e2dot_gains, Cal_LQR_e2dot_gains, 9 * sizeof(real32_T));
    memcpy(LQR_Feedforward_gains, Cal_LQR_Feedforward_gains,
           9 * sizeof(real32_T));
    // printf("TC : Test_CoeffPGainLdc = %f\n", Test_CoeffPGainLdc);
}
/***************************=
  reqPorts->************************************************** Functionname:
  TJATCT_Output_initialize                                  */ /*!

@brief: write inner result to TJATCT proPorts

@description: write inner result to TJATCT proPorts

@param[in]:proPorts   TJATCT output

@return:void
*****************************************************************************/
void TJATCT_Output_initialize(sTJATCTOut_st* proPorts,
                              sTJATCTDebug_st* debugInfo) {
    proPorts->TJATCT_TimerPlauCheck = CLM_TimerPlauCheck;
    proPorts->TJATCT_DeltaFCmd = CLM_DeltaFCmd;
    proPorts->TJATCT_ReqFfcCrv = CLM_ReqFfcCrv;
    proPorts->TJATCT_SumCtrlCrv = CLM_SumCtrlCrv;
    proPorts->TJATCT_HldVehCrv = CLM_HldVehCrv;
    proPorts->TJATCT_ThdCrvPlauChkUp = CLM_ThdCrvPlauChkUp;
    proPorts->TJATCT_ThdCrvPlauChkLow = CLM_ThdCrvPlauChkLow;
    proPorts->TJATCT_ReqFbcDcCrv = CLM_ReqFbcDcCrv;
    proPorts->TJATCT_LmtReqFfcCrv = CLM_LmtReqFfcCrv;
    proPorts->TJATCT_BtfQulifierTrajCtrl = CLM_BtfQulifierTrajCtrl;
    proPorts->TJATCT_EnaUnplauRequest = CLM_EnaUnplauRequest;
    proPorts->TJATCT_EnaSetDegrReq = CLM_EnaSetDegrReq;
    proPorts->TJATCT_EnaRstDegrReq = CLM_EnaRstDegrReq;
    proPorts->TJATCT_EnaPlausibilityCheck = CLM_EnaPlausibilityCheck;
    proPorts->TJATCT_EnaHldVehCrv = CLM_EnaHldVehCrv;
    proPorts->TJATCT_EnaLmtWarn = CLM_EnaLmtWarn;
    proPorts->TJATCT_EnaPlauCheck = CLM_EnaPlauCheck;
    proPorts->LGC_EnableCtrl_nu = LGC_EnableCtrl_nu;
    proPorts->CDC_CtrlErrDistY = CDC_CtrlErrDistY;
    proPorts->CDC_CtrlErrHeading = CDC_CtrlErrHeading;
    proPorts->data_log_0 = data_log_0;
    proPorts->data_log_1 = data_log_1;
    proPorts->data_log_2 = data_log_2;
    proPorts->data_log_3 = data_log_3;
    proPorts->data_log_4 = data_log_4;
    proPorts->data_log_5 = data_log_5;
    proPorts->data_log_6 = data_log_6;
    proPorts->data_log_7 = data_log_7;
    proPorts->data_log_8 = data_log_8;
    proPorts->data_log_9 = data_log_9;
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */