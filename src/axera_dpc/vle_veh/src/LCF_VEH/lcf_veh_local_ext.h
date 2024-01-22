// "Copyright [2022] <Copyright Senseauto>" [legal/copyright]
#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_VEH_LCF_VEH_LOCAL_EXT_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_VEH_LCF_VEH_LOCAL_EXT_H_
#ifdef __cplusplus
extern "C" {
#endif
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include <string.h>
#include "TM_Global_Types.h"
#include "HOD_Ext.h"
#include "trajectory_plan_ext.h"
#include "TJATCT_ext.h"
#include "lcd_ext.h"
#include "lck_ext.h"
#include "lck_par.h"
#include "lcd_par.h"
#include "tue_common_libs.h"
/*****************************************************************************
  TYPEDEFS
*****************************************************************************/
#ifndef Rte_TypeDef_LCF_VehSignalHeader_t
    typedef struct
    {
        uint32 uiTimeStamp;
        uint16 uiMeasurementCounter;
        uint16 uiCycleCounter;
        uint8 eSigStatus;
    } LCF_VehSignalHeader_t; /* Common header for all structured data types */
#define Rte_TypeDef_LCF_VehSignalHeader_t
#endif

#ifndef Rte_TypeDef_LCF_VehAlgoCompState_t
    typedef struct
    {
        uint32 uiVersionNumber;
        LCF_VehSignalHeader_t sSigHeader;
        uint32 uiAlgoVersionNumber;
        uint32 uiApplicationNumber;
        uint8 AlgoVersionInfo[50];
        uint8 eCompState;
        uint8 eShedulerSubModeRequest;
        uint32 eErrorCode;
        uint32 eGenAlgoQualifier;
    } LCF_VehAlgoCompState_t;
#define Rte_TypeDef_LCF_VehAlgoCompState_t
#endif

#ifndef Rte_TypeDef_LCF_Veh_BaseCtrlData_t
    typedef struct
    {
        uint32 uiVersionNumber;
        LCF_VehSignalHeader_t sSigHeader;
        uint16 eOpMode;
        // uint8 eSchedulingMode;
        // uint8 eSchedulingSubMode;
    } LCF_Veh_BaseCtrlData_t;
#define Rte_TypeDef_LCF_Veh_BaseCtrlData_t
#endif

#ifndef Rte_TypeDef_LCF_VehTPRCVInReq_t
typedef struct {
    float32 fKappaSumCommand_1pm;    // todo,[-0.15，0.15]
    float32 fEPSManualTrqActVal_Nm;  // the acture manual torque value from EPS,
                                     // [-100,100]
    uint8 uiOdometerState_nu;    // vehicle odometer state,  [0,1], 1: odometer
                                 // state OK, 0: odometer state wrong
    uint32 uiVehSync4LCO_us;     // time stamp of VED signals for latency
                                 // compensation.[0,4294967295]
    uint32 uiSenToVehTStamp_us;  // the time stamp of the signals from sen part,
                                 // [0,4294967295]
    uint8 uiStLaneLaKmc_st;      // Side of xDP Intervention
    uint8 uiStVehOdo_nu;
} LCF_VehTPRCVInReq_t;
#define Rte_TypeDef_LCF_VehTPRCVInReq_t
#endif

#ifndef Rte_TypeDef_LCF_Veh_VehDyn_t
    typedef struct
    {
        LCF_VehSignalHeader_t sSigHeader;
        float32 fEgoVelX_mps;
        float32 fEgoCurve_1pm;
        float32 fEgoAccelX_mps2;
        float32 fEgoYawRate_rps;
        float32 fWhlSteerAngleVdy_rad;
        float32 fEstSelfSteerGrdnt_rads2pm;
        float32 fSteerWheelAgl_rad;
        float32 fManuActuTrqEPS_nm;   /* Actual manual torque of EPS */
        float32 fSteerWhlAglspd_rads; // Steer wheel angle speed
    } LCF_Veh_VehDyn_t;
#define Rte_TypeDef_LCF_Veh_VehDyn_t
#endif

#ifndef Rte_TypeDef_LCF_CSCLTAData_t
    typedef struct
    {
        LCF_VehSignalHeader_t sSigHeader;
        uint8 uiControllingFunction_nu; // function type of lateral controlling,
                                        // [0,7], E_LCF_OFF_nu= 0,E_LCF_TJA_nu=
                                        // 1,E_LCF_LDP_nu= 2,E_LCF_LDPOC_nu=
                                        // 3,E_LCF_RDP_nu= 4,E_LCF_ALCA_nu=
                                        // 5,E_LCF_AOLC_nu= 6, E_LCF_ESA_nu= 7
        float32 fPlanningHorizon_sec;   // max Planning horizon(time) of the
                                        // trajectory, [0，60]
        uint8 uiSysStateLCF_nu;         // lateral control function system state enum
                                        // value, [0,6],E_LCF_SYSSTATE_NOTPRESENT = 0;
                                        // E_LCF_SYSSTATE_DISABLED = 1;
                                        // E_LCF_SYSSTATE_PASSIVE = 2;
                                        // E_LCF_SYSSTATE_REQUEST = 3;
                                        // E_LCF_SYSSTATE_CONTROLLING = 4;
                                        // E_LCF_SYSSTATE_RAMPOUT = 5; E_LCF_SYSSTATE_ERROR
                                        // = 6; \n\n
        float32 fPredTimeCurve_sec;     // todo,[0，60]
        float32 fPredTimeHeadAng_sec;   // todo, [0，60]
        boolean bTriggerReplan;         // trigger replan signal from CSCLTA module,[0，1]
        float32 fLeCridBndPosX0_met;    // the PosX0 value of left corridor bound,
                                        // [-300, 300]
        float32 fLeCridBndPosY0_met;    // the PosY0 value of left corridor bound,
                                        // [-15, 15]
        float32 fLeCridBndHeadAng_rad;  // the heading angle value of left corridor
                                        // bound, [-0.78539816, 0.78539816]
        float32 fLeCridBndCrv_1pm;      // the curve value of left corridor bound,
                                        // [-0.1, 0.1]
        float32 fLeCridBndCrvChng_1pm2; // the curve deviation value of left
                                        // corridor bound, [-0.001, 0.001]
        float32 fLeCridBndLength_met;   // the length value of left corridor bound,
                                        // [0, 150]
        float32 fRiCridBndPosX0_met;    // the PosX0 value of right corridor bound,
                                        // [-300, 300]
        float32 fRiCridBndPosY0_met;    // the PosY0 value of right corridor bound,
                                        // [-15, 15]
        float32 fRiCridBndHeadAng_rad;  // the heading angle value of right corridor
                                        // bound, [-0.78539816, 0.78539816]
        float32 fRiCridBndCrv_1pm;      // the curve value of right corridor bound,
                                        // [-0.1, 0.1]
        float32 fRiCridBndCrvChng_1pm2; // the curve deviation value of right
                                        // corridor bound, [-0.001, 0.001]
        float32 fRiCridBndLength_met;   // the length value of right corridor bound,
                                        // [0, 150]
        float32 fTgtTrajPosX0_met;      // the PosX0 value of target corridor bound,
                                        // [-300, 300]
        float32 fTgtTrajPosY0_met;      // the PosY0 value of target corridor bound,
                                        // [-15, 15]
        float32 fTgtTrajHeadingAng_rad; // the heading angle value of target
                                        // corridor bound, [-0.78539816,
                                        // 0.78539816]
        float32 fTgtTrajCurve_1pm;      // the curve value of target corridor bound,
                                        // [-0.1, 0.1]
        float32 fTgtTrajCrvChng_1pm2;   // the curve deviation value of target
                                        // corridor bound, [-0.001, 0.001]
        float32 fTgtTrajLength_met;     // the length value of target corridor bound,
                                        // [0, 150]
        boolean bLatencyCompActivated;  // the trigger flag for latency compensation
                                        // function, [0,1] 1: latency compensation
                                        // enable, 0: latency compensation disable
        float32 fSensorTimeStamp_sec;   // time stamp of the camera signal from
                                        // camera sensor,[0,4295]
        uint8 uiTrajPlanServQu_nu;      // todo
        float32 fWeightTgtDistY_nu;     // The importance factor of the lateral
                                        // deviation, [0,1]
        float32 fWeightEndTime_nu;      // The importance factor of the time required
                                        // for the planned trajectory, [0,1]
        float32 fDistYToLeTgtArea_met;  // lateral tolerance left boundary value ,
                                        // [0,10]
        float32 fDistYToRiTgtArea_met;  // lateral tolerance right boundary value ,
                                        // [0,10]
        float32 fFTireAclMax_mps2;      // lateral acceleration upper limiting value,
                                        // [-20,20]
        float32 fFTireAclMin_mps2;      // lateral acceleration lower limiting value,
                                        // [-20,20]
        float32 fObstacleVelX_mps;      // the obstacle velocity X in target trajectory
                                        // if it is existed , [-20,150]
        float32 fObstacleAccelX_mps2;   // the obstacle accel X in target trajectory
                                        // if it is existed , [-20,20]
        float32 fObstacleWidth_met;     // the obstacle width in target trajectory if
                                        // it is existed , [0,150]
        float32 fObstacleDistX_met;     // the obstacle distance X in target trajectory
                                        // if it is existed , [-1000,1000]
        float32 fObstacleDistY_met;     // the obstacle distance X in target trajectory
                                        // if it is existed , [-1000,1000]
        float32 fMaxJerkAllowed_mps3;   // Maximum Jerk Allowed in the trajectory
                                        // planning, [0,50]
        uint8 uiTrajGuiQualifier_nu;    // qualifier value of trajectory guidence, The
                                        // qualifier indicates if/how the target
                                        // curvature should be considered, [0,5]
                                        // .E_LCF_TGQ_REQ_OFF= 0, E_LCF_TGQ_REQ = 1,
                                        // E_LCF_TGQ_REQ_FREEZE= 3, E_LCF_TGQ_REQ_FFC
                                        // = 4, E_LCF_TGQ_REQ_REFCHNG= 5
        float32 fMaxCrvGrdBuildup_1pms;
        float32 fMaxCrvGrdRed_1pms;
        float32 fGrdLimitTgtCrvGC_1pms;
        float32 fMaxCrvTrajGuiCtrl_1pm;
        boolean bLimiterActivated_nu;
        float32 fLimiterDuration_sec;
    } LCF_CSCLTAData_t;
#define Rte_TypeDef_LCF_CSCLTAData_t
#endif

#ifndef Rte_TypeDef_LCF_LBPData_t
    typedef struct
    {
        LCF_VehSignalHeader_t sSigHeader;
        uint8 uiLeLnQuality_per;  // quality of left lane, [0, 100]
        uint8 uiRiLnQuality_per;  // quality of right lane, [0, 100]
        uint8 uiLeCrvQuality_per; // quality of left lane curve, [0, 100]
        uint8 uiRiCrvQuality_per; // quality of right lane curve, [0, 100]
    } LCF_LBPData_t;
#define Rte_TypeDef_LCF_LBPData_t
#endif

#ifndef Rte_TypeDef_LCF_TJASAState_t
    typedef struct
    {
        LCF_VehSignalHeader_t sSigHeader;
        uint8 uiLatCtrlMode_nu; // lateral control mode, [0,8]
                                // ,E_TJASTM_LATCTRLMD_PASSIVE=
                                // 0,E_TJASTM_LATCTRLMD_LC=
                                // 1,E_TJASTM_LATCTRLMD_OF=
                                // 2,E_TJASTM_LATCTRLMD_CMB=
                                // 3,E_TJASTM_LATCTRLMD_SALC=
                                // 4,E_TJASTM_LATCTRLMD_LC_RQ=
                                // 5,E_TJASTM_LATCTRLMD_OF_RQ=
                                // 6,E_TJASTM_LATCTRLMD_CMB_RQ= 7,
                                // E_TJASTM_LATCTRLMD_SALC_RQ= 8
    } LCF_TJASAState_t;
#define Rte_TypeDef_LCF_TJASAState_t
#endif

#ifndef Rte_TypeDef_LCF_TJATCTOut_st
    typedef struct
    {
        real32_T TJATCT_TimerPlauCheck;
        real32_T TJATCT_DeltaFCmd;
        real32_T TJATCT_ReqFfcCrv;
        real32_T TJATCT_SumCtrlCrv;
        real32_T TJATCT_HldVehCrv;
        real32_T TJATCT_ThdCrvPlauChkUp;
        real32_T TJATCT_ThdCrvPlauChkLow;
        real32_T TJATCT_ReqFbcDcCrv;
        real32_T TJATCT_LmtReqFfcCrv;
        uint8_T TJATCT_BtfQulifierTrajCtrl;
        boolean_T TJATCT_EnaUnplauRequest;
        boolean_T TJATCT_EnaSetDegrReq;
        boolean_T TJATCT_EnaRstDegrReq;
        boolean_T TJATCT_EnaPlausibilityCheck;
        boolean_T TJATCT_EnaHldVehCrv;
        boolean_T TJATCT_EnaLmtWarn;
        boolean_T TJATCT_EnaPlauCheck;
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
} LCF_TJATCTOut_st;
#define Rte_TypeDef_LCF_TJATCTOut_st
#endif

#ifndef Rte_TypeDef_LCF_LCDOutput_t
    typedef struct
    {
        // LCDState_t uiLCDState;
        uint8 uiLCDState;
        boolean bTorqueReq;  /*!< flag which indicates torque request */
        float32 fTorque;     /*!< requested steering torque [Nm]      */
        float32 fTorqueGrad; /*!< requested steering torque gradient  */
    } LCF_LCDOutput_t;
#define Rte_TypeDef_LCF_LCDOutput_t
#endif

#ifndef Rte_TypeDef_LCF_HODOutput_t
    typedef struct
    {
        UINT8_T HOD_StSysWarning;    /* Level of hands off system warning */
                                     /* 0 NoWarning  */
                                     /* 1 HandsOnRequest */
                                     /* 2 TakeOverRequest */
                                     /* 3 TriggerDegradation */
        boolean HOD_EnaHandsOffCnfm; /* flag of hands off or on */
    } LCF_HODOutput_t;
#define Rte_TypeDef_LCF_HODOutput_t
#endif

#ifndef Rte_TypeDef_LCF_LDPSAData_t
    typedef struct
    {
        uint8 LDWC_DgrSide_St;
        uint8 LDWC_RdyToTrig_B;
        uint8 LDWC_SysOut_St;
        uint8 LDWC_StateLf_St;
        uint8 LDWC_StateRi_St;
        uint8 LDPSC_DgrSide_St;
        uint8 LDPSC_RdyToTrig_B;
        uint8 LDPSC_SysOut_St;
        uint8 LDPSC_StateLf_St;
        uint8 LDPSC_StateRi_St;
        float32 LDPDT_LnPstnLf_Mi;
        float32 LDPDT_LnPstnRi_Mi;
        float32 LDPDT_LnHeadLf_Rad;
        float32 LDPDT_LnHeadRi_Rad;
        float32 LDPDT_LnCltdCurvLf_ReMi;
        float32 LDPDT_LnCltdCurvRi_ReMi;
    } LCF_LDPSAData_t;
#define Rte_TypeDef_LCF_LDPSAData_t
#endif

#ifndef Rte_TypeDef_LCF_SenToVeh_t
    typedef struct
    {
        LCF_LDPSAData_t sLDPSAData;
        LCF_CSCLTAData_t sCSCLTAData; // the input data of lcf veh from MCTLFC(sen)
        LCF_LBPData_t sLBPData_t;     // the input data of lcf veh from LBP(sen)
        LCF_TJASAState_t sTJASAState; // the input data of lcf veh from TJASA(sen)
    } LCF_SenToVeh_t;
#define Rte_TypeDef_LCF_SenToVeh_t
#endif

#ifndef Rte_TypeDef_LCF_DMCToVeh_t
    typedef struct
    {
        LCF_VehSignalHeader_t sSigHeader;
        boolean bSysStOffLatDMC;  // system state of lateral DMC module
                                  // ,[0，1],Indicates TRUE if DMC_LAT_STATUS = 0
                                  // (LCF_LADMC_OFF)
        boolean bSysStReqLatDMC;  // Indicates TRUE if DMC_LAT_STATUS = 6
                                  // (LCF_LADMC_REQ),[0，1]
        float32 fSteerAngleLaDmc; // Offset compensated steer angle at the front
                                  // wheels provided by the LatDMC
    } LCF_DMCToVeh_t;
#define Rte_TypeDef_LCF_DMCToVeh_t
#endif

#ifndef Rte_TypeDef_LCF_VehNVRAMData_t
typedef struct {
    boolean LCFVeh_Nb_SwitchReserved1_nu;   // Power off switch
    boolean LCFVeh_Nb_SwitchReserved2_nu;   // Reserved
    boolean LCFVeh_Nb_SwitchReserved3_nu;   // Reserved
    boolean LCFVeh_Nb_SwitchReserved4_nu;   // Reserved
    boolean LCFVeh_Nb_SwitchReserved5_nu;   // Reserved
    boolean LCFVeh_Nb_SwitchReserved6_nu;   // Reserved
    boolean LCFVeh_Nb_SwitchReserved7_nu;   // Reserved
    boolean LCFVeh_Nb_SwitchReserved8_nu;   // Reserved
    boolean LCFVeh_Nb_SwitchReserved9_nu;   // Reserved
    boolean LCFVeh_Nb_SwitchReserved10_nu;  // Reserved

    uint8 LCFVeh_Nu_SensitivityReserved1_nu;  // sensitivity
    uint8 LCFVeh_Nu_SensitivityReserved2_nu;  // Reserved
    uint8 LCFVeh_Nu_SensitivityReserved3_nu;  // Reserved
    uint8 LCFVeh_Nu_SensitivityReserved4_nu;  // Reserved
    uint8 LCFVeh_Nu_SensitivityReserved5_nu;  // Reserved
} LCF_VehNVRAMData_t;
#define Rte_TypeDef_LCF_VehNVRAMData_t
#endif

#ifndef Rte_TypeDef_LCF_VehCalibration
    typedef struct
    {
        uint8 TestFlag_EnableFF;
        uint8 TestFlag_EnableTP;
        uint8 TestFlag_EnableOL;

    float32 Test_CoeffMainPGainLdc;
    float32 Test_CoeffPGainLdc;
    float32 Test_CoeffIGainLdc;
    float32 Test_CoeffDGainLdc;
    float32 Test_TimeDT1Ldc;
    float32 Test_CoeffPT1GainLdc;
    float32 Test_TimePT1Ldc;

    float32 Test_CoeffMainPGainCac;
    float32 Test_CoeffPGainCac;
    float32 Test_CoeffIGainCac;
    float32 Test_CoeffDGainCac;
    float32 Test_TimeDT1Cac;
    float32 Test_CoeffPT1GainCac;
    float32 Test_TimePT1Cac;

    float32 Test_CDCTimeFltCurHeading;
    float32 LQR_e1_gains[9];
    float32 LQR_e1dot_gains[9];
    float32 LQR_e2_gains[9];
    float32 LQR_e2dot_gains[9];
    float32 LQR_Feedforward_gains[9];
} LCF_VehCalibration;
#define Rte_TypeDef_LCF_VehCalibration
#endif

#ifndef Rte_TypeDef_reqLcfVehPrtList_t
    typedef struct
    {
        LCF_Veh_BaseCtrlData_t sBaseCtrlData; // Control data giving information
                                              // about the current mode
        LCF_Veh_VehDyn_t sLCFVehDyn;          // Vehicle dynamic data (VDY output)
        LCF_SenToVeh_t
            sLcfSVehInputFromSenData; // the input data of lcf veh from lcf sen
        LCF_DMCToVeh_t
            sLcfVehInputFromDMCData;     // the input data of lcf veh from DMC
        LCF_VehTPRCVInReq_t sTPRCVInReq; // the input data of lcf veh from LCFRCV
        LCF_VehNVRAMData_t sNVRAMData;   // NVRAM Data
        LCF_VehCalibration sLCFVehCalibrations;
    } reqLcfVehPrtList_t;
#define Rte_TypeDef_reqLcfVehPrtList_t
#endif

#ifndef Rte_TypeDef_reqLcfVehParams
    typedef struct
    {
        float32 LCFVeh_Kf_fSysCycleTime_sec;
        float32 LCFVeh_Kf_fEgoVehWidth_met;
        float32 LCFVeh_Kf_fEgoVehLength_met;

    float32 LCFVeh_Kf_CrnStiffFr;
    float32 LCFVeh_Kf_CrnStiffRe;
    float32 LCFVeh_Kf_DistToFrontAxle_met;
    float32 LCFVeh_Kf_DistToRearAxle_met;
    float32 LCFVeh_Kf_Mass_kg;
    float32 LCFVeh_Kf_SelfStrGrad;
    float32 LCFVeh_Kf_SteeringRatio_nu;
} reqLcfVehParams;
#define Rte_TypeDef_reqLcfVehParams
#endif

#ifndef Rte_TypeDef_LCF_VehGenericOutputs_t
typedef struct {/* [Satisfies_rte sws 1191] */
    uint32 uiVersionNumber;
    LCF_VehSignalHeader_t sSigHeader;
    LCF_TJATCTOut_st sTCOutput;
    LCF_LCDOutput_t sLCDOutput;
    TRJPLN_TrajectoryPlanOutPro_t sTPOutput;
    TRJPLN_TrajectoryPlanInReq_t sTPInput;
    TRJPLN_TrajectoryPlanDebug_t sTPDebug;
    sTJATCTInReq_st pTCInput;
    sTJATCTDebug_st sTCDebug;
    sHODOutput_t sHODOutput;
    sHODDebug_t sHODDebug;
} LCF_VehGenericOutputs_t;
#define Rte_TypeDef_LCF_VehGenericOutputs_t
#endif

#ifndef Rte_TypeDef_proLcfVehPrtList_t
typedef struct {
    LCF_VehAlgoCompState_t
        pAlgoCompState;  //!< State return values of the algo component
    LCF_VehGenericOutputs_t pLcfVehOutputData;  //!<
    LCF_VehNVRAMData_t sNVRAMData;              // NVRAM Data
} proLcfVehPrtList_t;
#define Rte_TypeDef_proLcfVehPrtList_t
#endif

typedef struct {
    TRJPLN_TrajectoryPlanDebug_t sTPDebug;
    sTJATCTDebug_st sTCDebug;
    sLCKDebug_t sLCKDebug;
    sLCDDebug_t sLCDDebug;
    sHODDebug_t sHODDebug;
} reqLcfVehDebug_t;

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
void LcfVehExec(const reqLcfVehPrtList_t* const reqPorts,
                const reqLcfVehParams* reqParams,
                proLcfVehPrtList_t* const proPorts,
                reqLcfVehDebug_t* proDebugs);
void LcfVehReset(const reqLcfVehParams* reqParams);

#ifdef __cplusplus
}
#endif
#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_
        // SDK_DECISION_SRC_LCF_VEH_LCF_VEH_LOCAL_EXT_H_
