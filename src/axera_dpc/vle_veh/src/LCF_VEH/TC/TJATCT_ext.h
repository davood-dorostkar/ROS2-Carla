#include "rtwtypes.h"
#ifndef TJATCT_EXT_H
#define TJATCT_EXT_H
#include "Rte_Type.h"  //RENDL
#ifndef Rte_TypeDef_sTJATCTInReq_st
typedef struct {
    real32_T TCTI_TimeSysCycle;        /* '<Root>/TCTI_TimeSysCycle'
                                        * Cycle Time for VEH task
                                          S_LCFRCV_SysCycleTimeVeh_sec
                                        */
    real32_T TCTI_VehicleVelX;         /* '<Root>/TCTI_VehicleVelX'
                                        * Vehicle Speed in miles/hour, computed based on
                                        the wheel speeds
                                          Negative values when vehicle moves backward
                                          S_LCFRCV_VehVelX_mps
                                        */
    real32_T TCTI_VehYawRate;          /* '<Root>/TCTI_VehYawRate'
                                        *
                                          S_LCFRCV_VehYawRate_rps
                                        */
    real32_T TCTI_WhlSteerAngleVdy;    /* '<Root>/TCTI_WhlSteerAngleVdy'
                                        * Offset compensated steering wheel angle
                                          S_LCFRCV_OffCompStWheelAngle_rad
                                        */
    real32_T TCTI_SteerAngleLaDmc;     /* '<Root>/TCTI_SteerAngleLaDmc'
                                        * Offset compensated steer angle at the
                                        front wheels provided by the LatDMC
                                          S_LCFRCV_SteerAngleLatDMC_deg
    
                                        */
    boolean_T TCTI_EnaReplanCurValues; /* '<Root>/TCTI_EnaReplanCurValues'
                                        * S_TPLCEN_ReplanCurValues_nu
                                        */
    uint8_T TCTI_BtfTrajGuiQualifier;  /* '<Root>/TCTI_BtfTrajGuiQualifier'
                                        * Trajectory Guidance Qualifier send by
                                        TPLFBT
 
                                          S_TPLFBT_TrajGuiQualifier_nu
 
                                        */
    real32_T TCTI_ReqTrajDistYTpl;     /* '<Root>/TCTI_ReqTrajDistYTpl'
                                        * Lateral distance of trajectory plan
    
                                          S_TPLFBT_CurDistY_met
    
                                        */
    real32_T
        TCTI_CurTrajDistYTpl;          /* '<Root>/TCTI_CurTrajDistYTpl'
                                        * Current lateral distance for trajectory plan
         
                                          S_TPLFBT_CurDistY_met
                                        */
    real32_T TCTI_ReqTrajCrvTpl;       /* '<Root>/TCTI_ReqTrajCrvTpl'
                                        * Trajectory planning setting curvature
      
                                          S_TPLFBT_TrajTgtCrv_1pm
      
                                        */
    uint8_T TCTI_StCntrlFcn;           /* '<Root>/TCTI_StCntrlFcn'
                                        * Carries which function is allowed to control
          
                                          S_MCTLFC_ControllingFunction_nu
          
                                        */
    uint8_T TCTI_StLatCtrlMode;        /* '<Root>/TCTI _StLatCtrlMode'
                                        * Lateral control mode
       
                                          S_TJASTM_LatCtrlMode_nu
       
                                        */
    real32_T TCTI_EstSelfSteerGrdnt;   /* '<Root>/TCTI_EstSelfSteerGrdnt'
                                        * Estimated self steering gradient of the
                                        vehicle
  
                                          S_LCFRCV_SelfSteerGradEst_rads2pm
  
                                        */
    real32_T TCTI_ReqTrajCrvCsc;       /* '<Root>/TCTI_ReqTrajCrvCsc'
                                        * CSCLTA_OUTPUT
      
                                          S_CSCLTA_TgtTrajCrv_1pm
      
                                        */
    real32_T TCTI_ReqTrajHeadTpl;      /* '<Root>/TCTI_ReqTrajHeadTpl'
                                        *
                                          S_TPLFBT_CurHeading_rad
     
                                        */
    real32_T TCTI_InclPrevTrajHeadTpl; /* '<Root>/TCTI_InclPrevTrajHeadTpl' */
    real32_T TCTI_MaxCrvGrdBuildup;    /* '<Root>/TCTI_MaxCrvGrdBuildup'
                                        * CSCLTA_OUTPUT
   
                                          S_CSCLTA_MaxCrvGrdBuildup_1pms
   
                                        */
    real32_T TCTI_MaxCrvGrdRed;        /* '<Root>/TCTI_MaxCrvGrdRed'
                                        * CSCLTA_OUTPUT
       
                                          S_CSCLTA_MaxCrvGrdRed_1pms
       
                                        */
    real32_T TCTI_LmtReqTrajCrvGrd;    /* '<Root>/TCTI_LmtReqTrajCrvGrd'
                                        * CSCLTA_OUTPUT
   
                                          S_CSCLTA_GrdLimitTgtCrvTGC_1pms
   
                                        */
    real32_T TCTI_MaxTrajCrv;          /* '<Root>/TCTI_MaxTrajCrv'
                                        * CSCLTA_OUTPUT
         
                                          S_CSCLTA_MaxCrvTrajGuiCtrl_1pm
         
                                        */
    uint8_T TCTI_StVehOdo;             /* '<Root>/TCTI_StVehOdo'
                                        *
                                          S_LCFRCV_VehOdoState_nu
            
                                        */
    real32_T TCTI_VehCurvature;        /* '<Root>/TCTI_VehCrv'
                                        * Vehicle curvature
       
                                          S_LCFRCV_VehCrv_1pm
       
                                        */
    uint8_T TCTI_EnaLmtActCsc;         /* '<Root>/TCTI_EnaLmtActCsc'
                                        * CSCLTA_OUTPUT
        
                                          S_CSCLTA_LimiterActivated_nu
        
                                        */
    real32_T TCTI_TimeLmtDur;          /* '<Root>/TCTI_TimeLmtDur'
                                        * CSCLTA_OUTPUT
         
                                          S_CSCLTA_LimiterTimeDuration_sec
         
                                        */
    uint8_T TCTI_StLaneLaKmc;          /* '<Root>/TCTI_StLaneLaKmc'
                                        * Side of xDP Intervention
                                          S_LCFRCV_LaneStatusLaKMC_st
         
                                        */
    uint8_T TCTI_StLcfSys;             /* '<Root>/TCTI_StLcfSys'
                                        * CSCLTA_OUTPUT
                                          S_CSCLTA_SysStateLCF_enum
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
} sTJATCTInReq_st;
#define Rte_TypeDef_sTJATCTInReq_st
#endif

typedef struct { uint8_T tmp; } sTJATCTParam_st;

typedef struct {
    real32_T TJATCT_TimerPlauCheck;        /* '<S122>/Multiport Switch'
                                            * UNDEFINED
                                            */
    real32_T TJATCT_DeltaFCmd;             /* '<S94>/Multiport Switch'
                                            * UNDEFINED
                                            */
    real32_T TJATCT_ReqFfcCrv;             /* '<S94>/Multiport Switch1'
                                            * Control
                                            * signal of the Feedforward Controller after
                                            * safety checks
                                            */
    real32_T TJATCT_SumCtrlCrv;            /* '<S125>/Add1'
                                            * UNDEFINED
                                            */
    real32_T TJATCT_HldVehCrv;             /* '<S122>/Multiport Switch1'
                                            * UNDEFINED
                                            */
    real32_T TJATCT_ThdCrvPlauChkUp;       /* '<S127>/Multiport Switch'
                                            * UNDEFINED
                                            */
    real32_T TJATCT_ThdCrvPlauChkLow;      /* '<S127>/Multiport Switch1'
                                            * UNDEFINED
                                            */
    real32_T TJATCT_ReqFbcDcCrv;           /* '<S94>/Multiport Switch2'
                                            * Control
                                            * signal of the Feedback Controller +
                                            * Disturbance Compensator after safety checks
                                            */
    real32_T TJATCT_LmtReqFfcCrv;          /* '<S94>/Add'
                                            * UNDEFINED
                                            */
    uint8_T TJATCT_BtfQulifierTrajCtrl;    /* '<S96>/Add'
                                            * Trajectory
                                            * controller qualifier. Includes
                                            * relevant    information for the
                                            * Situation Assessment    components.
                                            */
    boolean_T TJATCT_EnaUnplauRequest;     /* '<S93>/Unit_Delay2'
                                            * UNDEFINED
                                            */
    boolean_T TJATCT_EnaSetDegrReq;        /* '<S99>/Signal Copy'
                                            * Set
                                            * condition for Flip-Flop which holds
                                            * degradation request
                                            */
    boolean_T TJATCT_EnaRstDegrReq;        /* '<S97>/OR'
                                            * Reset
                                            * condition for Flip-Flop which holds
                                            * degradation        request
                                            */
    boolean_T TJATCT_EnaPlausibilityCheck; /* '<S123>/AND1'
                                            * UNDEFINED
                                            */
    boolean_T TJATCT_EnaHldVehCrv;         /* '<S126>/FixPt Relational Operator'
                                            * UNDEFINED
                                            */
    boolean_T TJATCT_EnaLmtWarn;           /* '<S128>/AND'
                                            * UNDEFINED
                                            */
    boolean_T TJATCT_EnaPlauCheck;         /* '<S122>/AND'
                                            * UNDEFINED
                                            */
    boolean_T LGC_EnableCtrl_nu;

    real32_T CDC_CtrlErrDistY;

    real32_T CDC_CtrlErrHeading;

    real32_T data_log_0;
    real32_T data_log_1;
    real32_T data_log_2;
    real32_T data_log_3;
    real32_T data_log_4;
    real32_T data_log_5;
    real32_T data_log_6;
    real32_T data_log_7;
    real32_T data_log_8;
    real32_T data_log_9;

} sTJATCTOut_st;

#ifndef Rte_TypeDef_sTJATCTDebug_st
typedef struct {
    uint32_T uiVersionNum_nu;  // uint32 value example
    uint8_T CDC_BtfQualifier;
    real32_T CDC_CtrlErrDistY;
    real32_T CDC_CtrlErrHeading;
    boolean_T CDC_EnaCntrlByTgq;
    boolean_T CDC_EnaFreezeByTgq;
    boolean_T CDC_EnaResetByTgq;
    boolean_T CDC_EnaWatchdog;
    real32_T CDC_EstCurHeading;
    real32_T CDC_EstDeltaTheta;
    real32_T CDC_FltDeltaTheta;
    real32_T CDC_FltErrDistYTpl;
    real32_T CDC_HldCtrlErrDistY;
    real32_T CDC_HldCtrlErrHeading;
    real32_T CDC_PreErrCtrlHeading;
    uint8_T CDC_RawBtfQualifier;
    real32_T CDC_RawCtrlErrDistY;
    real32_T CDC_RawCtrlErrHeading;
    real32_T CDC_RawDeltaTheta;
    real32_T CDC_RawErrDistYTpl;
    real32_T CDC_RawFltDeltaTheta;
    real32_T CLM_CrvBySteerAngle;
    boolean_T CLM_EnaDegrReq;
    boolean_T CLM_EnaGrdDeltaFCmd;
    boolean_T CLM_EnaGrdFbcDc;
    boolean_T CLM_EnaGrdFfcCrv;
    boolean_T CLM_EnaSatDeltaFCmd;
    boolean_T CLM_EnaSatFbcDc;
    boolean_T CLM_EnaSatFfcCrv;
    real32_T CLM_GrdDeltaFCmd;
    real32_T CLM_GrdFbcDc;
    real32_T CLM_GrdFfcCrv;
    boolean_T CLM_RawEnaDegrReq;
    real32_T CLM_RawGrdDeltaFCmd;
    real32_T CLM_RawGrdFbcDc;
    real32_T CLM_RawGrdFfcCrv;
    real32_T CLM_SatDeltaFCmd;
    real32_T CLM_SatFbcDc;
    real32_T CLM_SatFfcCrv;
    real32_T CLM_ThdDeltaFCmdGrd;
    real32_T CLM_ThdDeltaFCmdSat;
    real32_T CLM_ThdFbcDcGrd;
    real32_T CLM_ThdFbcDcSat;
    real32_T CLM_ThdFfcCrvGrd;
    real32_T CLM_ThdFfcCrvSat;
    uint8_T DEV_BtfFfcQualifierPar;
    uint8_T DEV_BtfFfcQualifierRte;
    real32_T DEV_CoeffDeltaGainFfc;
    real32_T DEV_CrvTestSignal;
    real32_T DEV_DeltaFTestSignal;
    real32_T DEV_DlySetDeltaF2DotPar;
    real32_T DEV_DlySetDeltaF2DotRte;
    real32_T DEV_DlySetDeltaFDotPar;
    real32_T DEV_DlySetDeltaFDotRte;
    real32_T DEV_DlySetDeltaFPar;
    real32_T DEV_DlySetDeltaFRte;
    boolean_T DEV_EnaCntrlByTgq;
    uint8_T DEV_EnaCrvGen;
    uint8_T DEV_EnaDeltaFGen;
    boolean_T DEV_EnaFreezeByTgq;
    boolean_T DEV_EnaResetByTgq;
    real32_T DEV_HldReqDeltaFRte;
    real32_T DEV_ReqDeltaFRte;
    boolean_T DEV_RstCrvGen;
    boolean_T DEV_RstDeltaFGen;
    real32_T DEV_SetDeltaF3DotPar;
    real32_T DEV_SetDeltaF3DotRte;
    real32_T DEV_SetDeltaFPar;
    real32_T DEV_SetDeltaFRte;
    real32_T DEV_TimeCrvGen;
    real32_T DEV_TimeDeltaFGen;
    real32_T DTE_CoeffA0TranferFcn;
    real32_T DTE_CoeffA1TranferFcn;
    real32_T DTE_CoeffB0TranferFcn;
    real32_T DTE_CoeffB1TranferFcn;
    real32_T DTE_CoeffB2TranferFcn;
    real32_T DTE_CoeffDenS0LaDmc;
    real32_T DTE_CoeffDenS1LaDmc;
    real32_T DTE_CoeffDenS2LaDmc;
    real32_T DTE_CoeffDenS3LaDmc;
    real32_T DTE_CoeffNumS0LaDmc;
    real32_T DTE_CoeffNumS1LaDmc;
    real32_T DTE_Delta2DotForCrv;
    real32_T DTE_Delta2DotLaDmc;
    real32_T DTE_Delta2DotVdyFcn;
    real32_T DTE_Delta3DotLaDmc;
    real32_T DTE_DeltaByVdyFcn;
    real32_T DTE_DeltaDotForCrv;
    real32_T DTE_DeltaDotLaDmc;
    real32_T DTE_DeltaF2DotPar;
    real32_T DTE_DeltaF2DotRte;
    real32_T DTE_DeltaF3DotPar;
    real32_T DTE_DeltaF3DotRte;
    real32_T DTE_DeltaFDotPar;
    real32_T DTE_DeltaFDotRte;
    real32_T DTE_DeltaFPar;
    real32_T DTE_DeltaFRte;
    real32_T DTE_DeltaVdyFcn;
    real32_T DTE_DlyCurSteerAngle;
    real32_T DTE_DlyDeltaDotVdyFcn;
    real32_T DTE_DlyDeltaVdyFcn;
    real32_T DTE_DlySetCrvDotLaDmc;
    real32_T DTE_DlySetCrvLaDmc;
    real32_T DTE_DlySetDelta2DotLaDmc;
    real32_T DTE_DlySetDeltaDotLaDmc;
    real32_T DTE_DlySetDeltaLaDmc;
    boolean_T DTE_EnaCtrlByTgq;
    boolean_T DTE_EnaFreezeByTgq;
    boolean_T DTE_EnaResetByTgq;
    real32_T DTE_EstCrvByBnkAgl;
    real32_T DTE_FltDlyCurSteerAngle;
    real32_T DTE_HldReqCrvByBnkAgl;
    real32_T DTE_HldReqCrvByDstrb;
    real32_T DTE_HldReqDeltaByBnkAgl;
    real32_T DTE_HldReqDeltaByDstrb;
    real32_T DTE_KappaAngleLaDmc;
    real32_T DTE_LmtEstCrvByBnkAgl;
    real32_T DTE_LmtReqCrvByBnkAgl;
    real32_T DTE_LmtReqCrvByDstrb;
    real32_T DTE_LmtReqDeltaByBnkAgl;
    real32_T DTE_LmtReqDeltaByDstrb;
    real32_T DTE_LmtVehVelX;
    real32_T DTE_MaxCrvByBnkAgl;
    real32_T DTE_MaxDeltaByBnkAgl;
    real32_T DTE_MaxReqCrvByDstrb;
    real32_T DTE_MaxReqDeltaByDstrb;
    real32_T DTE_NdlySetCrvLaDmc;
    real32_T DTE_NdlySetDeltaLaDmc;
    real32_T DTE_Psi2DotVdyFcn;
    real32_T DTE_Psi3DotVdyFcn;
    real32_T DTE_RawCrvLaDmc;
    real32_T DTE_RawDeltaDotLaDmc;
    real32_T DTE_RawDeltaFDotPar;
    real32_T DTE_RawDeltaFDotRte;
    real32_T DTE_RawFltEstCrvByBnkAgl;
    real32_T DTE_RawLmtEstCrvByBnkAgl;
    real32_T DTE_RawReqCrvByBnkAgl;
    real32_T DTE_RawReqCrvByDstrb;
    real32_T DTE_RawReqDeltaByBnkAgl;
    real32_T DTE_RawReqDeltaByDstrb;
    real32_T DTE_ReqCrvByBnkAgl;
    real32_T DTE_ReqCrvByDstrb;
    real32_T DTE_ReqDeltaByBnkAgl;
    real32_T DTE_ReqDeltaByDstrb;
    real32_T DTE_ResCrvDenLaDmc;
    real32_T DTE_ResDeltaDenLaDmc;
    real32_T DTE_ResDeltaDenPar;
    real32_T DTE_ResDeltaDenRte;
    real32_T DTE_ResDeltaDenVdyFcn;
    real32_T DTE_SetCrv2DotLaDmc;
    real32_T DTE_SetCrvGainLaDmc;
    real32_T DTE_SetCrvLaDmc;
    real32_T DTE_SetDelta3DotLaDmc;
    real32_T DTE_SetDeltaGainLaDmc;
    real32_T DTE_SetDeltaLaDmc;
    real32_T EST_AngleCurSteer;
    real32_T EST_AngleLaDMCSteer;
    real32_T EST_AnglePObsDTheta;
    real32_T EST_AnglePObsDThetaFreeze;
    real32_T EST_AnglePObsDThetaLmt0;
    real32_T EST_AnglePObsDThetaLmt0Raw;
    real32_T EST_AnglePObsDThetaLmt1;
    real32_T EST_AnglePObsDThetaLmt2;
    real32_T EST_AnglePObsDThetaSat;
    real32_T EST_AnglePObsDThetaSel;
    real32_T EST_AnglePObsDThetaThd;
    real32_T EST_AnglePObsDThetaThd0;
    real32_T EST_AngleVDYSteer;
    real32_T EST_BetaDotPobs;
    real32_T EST_BetaDotSObs;
    real32_T EST_BetaSObs;
    uint16_T EST_BtfQualifierByBeta;
    uint16_T EST_BtfQualifierByEna;
    uint16_T EST_BtfQualifierByHdr;
    real32_T EST_CoeffA11StateSpace;
    real32_T EST_CoeffA12StateSpace;
    real32_T EST_CoeffA21StateSpace;
    real32_T EST_CoeffA22StateSpace;
    // real32_T	EST_CoeffAXStateSpace[2];
    real32_T EST_CoeffB11StateSpace;
    real32_T EST_CoeffB21StateSpace;
    real32_T EST_CoeffL11Pobs;
    real32_T EST_CoeffL11Sobs;
    real32_T EST_CoeffL12Pobs;
    real32_T EST_CoeffL13Pobs;
    real32_T EST_CoeffL21Pobs;
    real32_T EST_CoeffL21Sobs;
    real32_T EST_CoeffL22Pobs;
    real32_T EST_CoeffL23Pobs;
    real32_T EST_CoeffL31Pobs;
    real32_T EST_CoeffL32Pobs;
    real32_T EST_CoeffL33Pobs;
    real32_T EST_CoeffL41Pobs;
    real32_T EST_CoeffL42Pobs;
    real32_T EST_CoeffL43Pobs;
    // real32_T	EST_CoeffLPobs[12];
    // real32_T	EST_CoeffLYPObs[4];
    // real32_T	EST_CoeffLYStateSpace[2];
    real32_T EST_CrvPiObsCrvFlt;
    real32_T EST_CrvPlObsIn;
    real32_T EST_CurSteerAngle;
    real32_T EST_DThetaDotPobs;
    real32_T EST_DYDotPobs;
    real32_T EST_DeltaYPlObsIn;
    real32_T EST_DistFromCgToGud;
    real32_T EST_DistPObsDY;
    real32_T EST_DistPObsDYFreeze;
    real32_T EST_DistPObsDYGrdnt;
    real32_T EST_DistPObsDYGrdntThd;
    real32_T EST_DistPObsDYSat;
    real32_T EST_DistPObsDYSel;
    real32_T EST_DistPObsDYThd;
    real32_T EST_DistPobsDYGrdntRaw;
    real32_T EST_DistYDevByGrdntLmt1;
    real32_T EST_DistYDevStep;
    real32_T EST_DistYDevTrajFromCur;
    real32_T EST_DlyCurSteerAngle;
    boolean_T EST_EnaActvtGrdntLmt1;
    boolean_T EST_EnaActvtGrdntLmt2;
    boolean_T EST_EnaBetaSatSObs;
    boolean_T EST_EnaByMeanHdr;
    boolean_T EST_EnaByMulHdrPerc;
    boolean_T EST_EnaCntrlByTgq;
    boolean_T EST_EnaFreezeByTgq;
    boolean_T EST_EnaLmt2ByDistY;
    boolean_T EST_EnaLmtByDistY;
    boolean_T EST_EnaPObsDThetaLmt0;
    boolean_T EST_EnaPObsDThetaLmt1;
    boolean_T EST_EnaPObsDThetaLmt2;
    boolean_T EST_EnaPObsDThetaRst1;
    boolean_T EST_EnaPObsDThetaRst2;
    boolean_T EST_EnaPObsDThetaSat;
    boolean_T EST_EnaPObsDYGrdnt;
    boolean_T EST_EnaPObsDYGrdntRaw;
    boolean_T EST_EnaPObsDYSat;
    boolean_T EST_EnaResetByTgq;
    real32_T EST_ErrVehYawRate;
    real32_T EST_EstBetaPobs;
    real32_T EST_EstBetaSObs;
    real32_T EST_EstDThetaPobs;
    real32_T EST_EstDYPobs;
    real32_T EST_EstPsiDotPobs;
    real32_T EST_EstPsiDotSObs;
    real32_T EST_FacDThetaWghtHdrSel;
    real32_T EST_FacDYWghtHdrSel;
    real32_T EST_FltDThetaDotPObs;
    real32_T EST_FltDYDotPObs;
    real32_T EST_HdrPercByDY;
    real32_T EST_HdrPercByTheta;
    real32_T EST_HldBetaSObs;
    real32_T EST_LmtBetaSObs;
    real32_T EST_LmtHdrPercByDY;
    real32_T EST_LmtHdrPercByTheta;
    real32_T EST_LmtVehVelX;
    real32_T EST_MeanHdrPerc;
    uint8_T EST_ModeSelParHdr;
    real32_T EST_MulHdrPerc;
    real32_T EST_Psi2DotPobs;
    real32_T EST_Psi2DotSObs;
    real32_T EST_RatioSteerGear;
    real32_T EST_RawBetaSObs;
    real32_T EST_RawEstBetaPobs;
    real32_T EST_RawEstDThetaPobs;
    real32_T EST_RawEstDYPobs;
    real32_T EST_RawEstPsiDotPobs;
    real32_T EST_RawFltDThetaDotPObs;
    real32_T EST_RawFltDYDotPObs;
    real32_T EST_RawHdrPercByDY;
    real32_T EST_RawHdrPercByTheta;
    real32_T EST_ThdBetaSatSObs;
    real32_T EST_ThdMeanHdrSel;
    real32_T EST_ThdMulHdrSel;
    real32_T FFC_HldReqFfcCrv;
    real32_T FFC_ReqFfcCrv;
    uint8_T LGC_ActiveLgcParamSet_nu;
    boolean_T LGC_CacIntReset_nu;
    boolean_T LGC_CacPT1Reset_nu;
    real32_T LGC_CdcCmd_rad;
    real32_T LGC_Cmpn2DotLaDmcCas;
    real32_T LGC_Cmpn2DotLaDmcCdc;
    real32_T LGC_Cmpn2DotLaDmcLdc;
    real32_T LGC_CmpnDotLaDmcCas;
    real32_T LGC_CmpnDotLaDmcCdc;
    real32_T LGC_CmpnDotLaDmcLdc;
    real32_T LGC_CmpnLaDmcCas;
    real32_T LGC_CmpnLaDmcCdc;
    real32_T LGC_CmpnLaDmcLdc;
    real32_T LGC_CoeffDGainCac;
    real32_T LGC_CoeffDGainLcCac;
    real32_T LGC_CoeffDGainLcLdc;
    real32_T LGC_CoeffDGainLdc;
    real32_T LGC_CoeffDGainOfCac;
    real32_T LGC_CoeffDGainOfLdc;
    real32_T LGC_CoeffDGainSfCac;
    real32_T LGC_CoeffDGainSfLdc;
    real32_T LGC_CoeffIGainCac;
    real32_T LGC_CoeffIGainLcCac;
    real32_T LGC_CoeffIGainLcLdc;
    real32_T LGC_CoeffIGainLdc;
    real32_T LGC_CoeffIGainOfCac;
    real32_T LGC_CoeffIGainOfLdc;
    real32_T LGC_CoeffIGainSfCac;
    real32_T LGC_CoeffIGainSfLdc;
    real32_T LGC_CoeffMainPGainCac;
    real32_T LGC_CoeffMainPGainLdc;
    real32_T LGC_CoeffNumS0LaDmc;
    real32_T LGC_CoeffNumS1LaDmc;
    real32_T LGC_CoeffPGainByCrvCac;
    real32_T LGC_CoeffPGainByCrvLdc;
    real32_T LGC_CoeffPGainCac;
    real32_T LGC_CoeffPGainLcCac;
    real32_T LGC_CoeffPGainLcLdc;
    real32_T LGC_CoeffPGainLdc;
    real32_T LGC_CoeffPGainOfCac;
    real32_T LGC_CoeffPGainOfLdc;
    real32_T LGC_CoeffPGainSfCac;
    real32_T LGC_CoeffPGainSfLdc;
    real32_T LGC_CoeffPT1GainCac;
    real32_T LGC_CoeffPT1GainLcCac;
    real32_T LGC_CoeffPT1GainLcLdc;
    real32_T LGC_CoeffPT1GainLdc;
    real32_T LGC_CoeffPT1GainOfCac;
    real32_T LGC_CoeffPT1GainOfLdc;
    real32_T LGC_CoeffPT1GainSfCac;
    real32_T LGC_CoeffPT1GainSfLdc;
    real32_T LGC_CoeffPole1LaDmc;
    real32_T LGC_CoeffPole2LaDmc;
    real32_T LGC_CrvReqBAC_1pm;
    real32_T LGC_CrvReqDte_1pm;
    real32_T LGC_CrvReqFfcFrz_1pm;
    real32_T LGC_CrvReqFfcGrdLimT1_1pm;
    real32_T LGC_CrvReqFfcGrdLimT2_1pm;
    real32_T LGC_CrvReqFfcGrdLim_1pm;
    real32_T LGC_CtrlCrv_1pm;
    real32_T LGC_CtrlCrv_DE_1pm;
    real32_T LGC_CtrlErrHeadAglCrtd_rad;
    real32_T LGC_CtrlErrMainPGain;
    real32_T LGC_CtrlErrMainPGainCas;
    real32_T LGC_CtrlErrMainPGainCdc;
    real32_T LGC_DeltaByBnkAglComp_deg;
    real32_T LGC_DeltaFBAC_deg;
    real32_T LGC_DeltaFCmdCdc;
    real32_T LGC_DeltaFCmdDC_deg;
    real32_T LGC_DeltaFCmdFFC_deg;
    real32_T LGC_DeltaFCmdUnlimited_deg;
    real32_T LGC_DeltaFCmd_deg;
    real32_T LGC_DeltaFCmd_rad;
    real32_T LGC_DeltaFDGainCas;
    real32_T LGC_DeltaFDGainCdc;
    real32_T LGC_DeltaFDGainLdc;
    real32_T LGC_DeltaFIGainCas;
    real32_T LGC_DeltaFIGainCdc;
    real32_T LGC_DeltaFIGainLdc;
    real32_T LGC_DeltaFPGainCas;
    real32_T LGC_DeltaFPGainCdc;
    real32_T LGC_DeltaFPGainLdc;
    real32_T LGC_DeltaFPT1GainCas;
    real32_T LGC_DeltaFPT1GainCdc;
    real32_T LGC_DeltaFPT1GainLdc;
    uint8_T LGC_EnaActObjFollow;
    uint8_T LGC_EnaActSafetyFcn;
    boolean_T LGC_EnaCntrlByTgq;
    boolean_T LGC_EnaFreezeByTgq;
    boolean_T LGC_EnaModeChangeCas;
    boolean_T LGC_EnaModeChangeCdc;
    boolean_T LGC_EnaModeChangeDtct;
    boolean_T LGC_EnaPGainGrdLmtCas;
    boolean_T LGC_EnaPGainGrdLmtCdc;
    boolean_T LGC_EnaPGainGrdLmtLdc;
    int8_T LGC_EnaPGainGrdSignCas;
    int8_T LGC_EnaPGainGrdSignCdc;
    real32_T LGC_EnaPGainGrdSignLdc;
    boolean_T LGC_EnaPGainGrdThdCas;
    boolean_T LGC_EnaPGainGrdThdCdc;
    boolean_T LGC_EnaPGainGrdThdLdc;
    boolean_T LGC_EnaResetByTgq;
    boolean_T LGC_EnaRstByDistY;
    boolean_T LGC_EnaRstByStandStill;
    boolean_T LGC_EnaRstByTrq;
    boolean_T LGC_EnaRstIntCac;
    boolean_T LGC_EnaRstIntLdc;
    boolean_T LGC_EnaRstPT1Cac;
    boolean_T LGC_EnaRstPT1Ldc;
    boolean_T LGC_EnableCtrl_nu;
    real32_T LGC_ErrCourse2DotCas;
    real32_T LGC_ErrCourse2DotCdc;
    real32_T LGC_ErrCourseDotCas;
    real32_T LGC_ErrCourseDotCdc;
    real32_T LGC_ErrCtrlCourseCas;
    real32_T LGC_ErrCtrlCourseCdc;
    real32_T LGC_ErrCtrlDistY;
    real32_T LGC_ErrDistY2Dot;
    real32_T LGC_ErrDistYDot;
    real32_T LGC_FFCrv_1pm;
    real32_T LGC_FltErrCourseCas;
    real32_T LGC_FltErrCourseCdc;
    real32_T LGC_FltErrCourseDotCas;
    real32_T LGC_FltErrCourseDotCdc;
    real32_T LGC_FltPT1YErr_met;
    real32_T LGC_FltRawErrDistYDot;
    real32_T LGC_HldReqDeltaF;
    boolean_T LGC_Hold_nu;
    real32_T LGC_LdcAloneICmd_rad;
    real32_T LGC_LdcCmd_rad;
    boolean_T LGC_LdcIntReset_nu;
    boolean_T LGC_LdcPT1Reset_nu;
    real32_T LGC_LmtCoeffPGainCas;
    real32_T LGC_LmtCoeffPGainCdc;
    real32_T LGC_LmtCoeffPGainLdc;
    real32_T LGC_LmtReqDeltaF;
    real32_T LGC_LmtSelReqDeltaF;
    real32_T LGC_MaxReqDeltaF;
    real32_T LGC_RawCmpn2DotLaDmcCas;
    real32_T LGC_RawCmpn2DotLaDmcCdc;
    real32_T LGC_RawCmpn2DotLaDmcLdc;
    real32_T LGC_RawErrCourseDotCas;
    real32_T LGC_RawErrCourseDotCdc;
    real32_T LGC_RawErrDistYDot;
    real32_T LGC_RawFfcDeltaF;
    real32_T LGC_RawFltErrCourseCas;
    real32_T LGC_RawFltErrCourseCdc;
    real32_T LGC_RawFltErrCtrlDistY;
    real32_T LGC_RawLmtCoeffPGainCas;
    real32_T LGC_RawLmtCoeffPGainCdc;
    real32_T LGC_RawLmtCoeffPGainLdc;
    real32_T LGC_ReqDeltaF;
    boolean_T LGC_Reset_nu;
    uint8_T LGC_SafetyFunctionActive_nu;
    uint8_T LGC_StActParSet;
    real32_T LGC_SumCrvReqFbFrz_1pm;
    real32_T LGC_SumCrvReqFbGrdLim_1pm;
    real32_T LGC_SumCrvReqFbSatLim_1pm;
    real32_T LGC_SumCrvReqFb_1pm;
    real32_T LGC_TgtCrv_DENoLatSlp_1pm;
    real32_T LGC_TgtCrv_DE_1pm;
    real32_T LGC_TgtCrv_NoDE_1pm;
    real32_T LGC_TimeDT1Cac;
    real32_T LGC_TimeDT1LcCac;
    real32_T LGC_TimeDT1LcLdc;
    real32_T LGC_TimeDT1Ldc;
    real32_T LGC_TimeDT1OfCac;
    real32_T LGC_TimeDT1OfLdc;
    real32_T LGC_TimeDT1SfCac;
    real32_T LGC_TimeDT1SfLdc;
    real32_T LGC_TimeFltErrCourse;
    real32_T LGC_TimeFltErrCourseCdc;
    real32_T LGC_TimeFltErrDistY;
    real32_T LGC_TimePT1Cac;
    real32_T LGC_TimePT1DeltaFCmd;
    real32_T LGC_TimePT1LcCac;
    real32_T LGC_TimePT1LcLdc;
    real32_T LGC_TimePT1Ldc;
    real32_T LGC_TimePT1OfCac;
    real32_T LGC_TimePT1OfLdc;
    real32_T LGC_TimePT1SfCac;
    real32_T LGC_TimePT1SfLdc;
    real32_T TCTI_ActualTrqEPS;
    real32_T TCTI_NegReqTrajHeadTpl;
    real32_T TCTI_RoadBankAngle;
    boolean_T TC_EnaUnplauUnitDelay_bool;
    boolean_T TC_Freeze1RSFlipFlop_bool;
    boolean_T TC_Freeze2RSFlipFlop_bool;
    boolean_T TC_FreezeRSFlipFlop_bool;
    boolean_T TC_HoldWarnRSFlipFlop_bool;
    real32_T LQR_DeltaF_Cmd_rad;         /* '<S78>/Add' */
    real32_T LQR_DeltaF_feedback_rad;    /* '<S85>/MatrixMultiply' */
    real32_T LQR_DeltaF_feedforward_rad; /* '<S86>/MATLAB Function' */
    boolean_T LQR_EnaCntrlByTgq;         /* '<S87>/Equal1' */
    boolean_T LQR_EnaFreezeByTgq;        /* '<S87>/Equal' */
    boolean_T LQR_EnaResetByTgq;         /* '<S87>/OR' */
    real32_T LQR_I_term_rad;             /* '<S93>/Switch2' */
    real32_T LQR_MatK_k1;                /* '<S85>/Signal Conversion' */
    real32_T LQR_MatK_k2;                /* '<S85>/Signal Conversion1' */
    real32_T LQR_MatK_k3;                /* '<S85>/Signal Conversion2' */
    real32_T LQR_MatK_k4;                /* '<S85>/Signal Conversion3' */
    real32_T LQR_ReqDeltaF_Limit_deg;    /* '<S105>/Multiport Switch' */
    real32_T LQR_e1_contribution;        /* '<S85>/Product' */
    real32_T LQR_e1dot_contribution;     /* '<S85>/Product1' */
    real32_T LQR_e2_contribution;        /* '<S85>/Product2' */
    real32_T LQR_e2dot_contribution;     /* '<S85>/Product3' */
    real32_T LQR_heading_error;          /* '<S75>/Data Type Conversion12' */
    real32_T LQR_heading_error_rate;     /* '<S89>/Subtract' */
    real32_T LQR_lateral_error;          /* '<S75>/Data Type Conversion11' */
    real32_T LQR_lateral_error_rate;     /* '<S89>/Product5' */
    real32_T LQR_num_iteration;          /* '<S91>/Data Type Conversion3' */
    real32_T LQR_yawrate_term;           /* '<S85>/Product4' */
    real32_T LQR_DeltaF_Lead_Cmd_rad;    /* '<S78>/MATLAB Function' */

} sTJATCTDebug_st;
#define Rte_TypeDef_sTJATCTDebug_st
#endif

extern void TJATCT_Exec(const sTJATCTInReq_st* reqPorts,
                        const sTJATCTParam_st* params,
                        sTJATCTOut_st* proPorts,
                        sTJATCTDebug_st* debugInfo);
extern void TJATCT_Init(void);

#endif
