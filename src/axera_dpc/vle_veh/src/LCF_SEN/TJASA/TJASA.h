/**********************************Model
 Property********************************
 *
 * Company             : SENSETIME
 *
 * Tool Version        : Ver2.0
 *
 * Model Name          : TJASA
 *
 * Model Long Name     : Traffic Jam Assist

 *

 * Model Advisor       : Not Check

 *

 * Model Version       : Ver_02

 *

 * Model Author        : WJ

 *

 * Model Reviewer      :

 *

 * Model Review Data   :

 *

 * Model Cycle Time    : 50ms


 ************************************Auto
 Coder**********************************
 *
 * File                             : TJASA.h
 *
 * FileType                         : Code Header File
 *
 * Real-Time Workshop file version  : 9.4 (R2020b) 29-Jul-2020
 *
 * TLC version                      : 9.4 (Aug 20 2020)
 *
 * C source code generated on       : Wed Feb 15 17:30:31 2023
 *
 * Copyright (C) by SenseTime Group Limited. All rights reserved.
 *******************************************************************************/

#ifndef RTW_HEADER_TJASA_h_
#define RTW_HEADER_TJASA_h_
#include <math.h>
#include <string.h>
#ifndef TJASA_COMMON_INCLUDES_
#define TJASA_COMMON_INCLUDES_
#include "Sfun_Set_Bit.h"
#include "rtwtypes.h"
#endif /* TJASA_COMMON_INCLUDES_ */

#include "TJASA_types.h"

/* Macros for accessing real-time model data structure */

/* Block signals (default storage) */
typedef struct {
    E_TJASLC_LaneChangeTrig_nu LCDirection_enum; /* '<S264>/ManeuverState' */
    E_TJASLC_LaneChangeTrig_nu
        SLC_LaneChangeDirection; /* '<S314>/AbortState' */
} B_TJASA_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
    uint8_T is_active_c20_TJASA;  /* '<S8>/StateMachineTJA' */
    uint8_T is_c20_TJASA;         /* '<S8>/StateMachineTJA' */
    uint8_T is_NOT_ACTIVE;        /* '<S8>/StateMachineTJA' */
    uint8_T is_ACTIVE;            /* '<S8>/StateMachineTJA' */
    uint8_T is_active_c19_TJASA;  /* '<S8>/LatCtrlMode' */
    uint8_T is_c19_TJASA;         /* '<S8>/LatCtrlMode' */
    uint8_T is_Controlling;       /* '<S8>/LatCtrlMode' */
    uint8_T is_active_c6_TJASA;   /* '<S264>/ManeuverState' */
    uint8_T is_c6_TJASA;          /* '<S264>/ManeuverState' */
    uint8_T is_ActiveLeft;        /* '<S264>/ManeuverState' */
    uint8_T is_ActiveRight;       /* '<S264>/ManeuverState' */
    uint8_T is_NewEgoLane;        /* '<S264>/ManeuverState' */
    uint8_T is_active_c180_TJASA; /* '<S314>/AbortState' */
    uint8_T is_c180_TJASA;        /* '<S314>/AbortState' */
    uint8_T is_OriginLaneAbort;   /* '<S314>/AbortState' */
} DW_TJASA_T;

/* Block signals (default storage) */
extern B_TJASA_T TJASA_B;

/* Block states (default storage) */
extern DW_TJASA_T TJASA_DW;

/* External data declarations for dependent source files */
extern const Bus_TgtTrajAndCridrBnd_nu
    TJASA_rtZBus_TgtTrajAndCridrBnd_nu; /* Bus_TgtTrajAndCridrBnd_nu ground */

/*
 * Exported Global Signals
 *
 * Note: Exported global signals are block signals with an exported global
 * storage class designation.  Code generation will declare the memory for
 * these signals and export their symbols.
 *
 */
extern real32_T TJACMB_LaneCrvStdDev_nu;        /* '<S12>/Abs'
                                                 * TJACMB lane curve standard deviation
                                                   DT:float32
                                                 */
extern real32_T TJACMB_TraceCrvStdDev_nu;       /* '<S13>/Abs'
                                                 * TJACMB trace curve standard
                                                 deviation       DT:float32
                                                 */
extern real32_T TJATVG_DistYTolLeTgtArea_met;   /* '<S891>/Constant15'
                                                 * TJATVG DistYTolLeTgtArea
                                                   DT:float32
                                                 */
extern real32_T TJATVG_DistYTolRiTgtArea_met;   /* '<S891>/Constant16'
                                                 * TJATVG DistYTolRiTgtArea
                                                   DT:float32
                                                 */
extern real32_T TJATVG_FTireAclMax_mps2;        /* '<S891>/Constant13'
                                                 * TJATVG FTireAclMax
                                                   DT:float32
                                                 */
extern real32_T TJATVG_FTireAclMin_mps2;        /* '<S891>/Constant14'
                                                 * TJATVG FTireAclMin
                                                   DT:float32
                                                 */
extern real32_T TJATVG_WeightTgtDistY_nu;       /* '<S891>/Constant17'
                                                 * TJATVG weighted target distY
                                                   DT:float32
                                                 */
extern real32_T TJATVG_WeightEndTime_nu;        /* '<S891>/Constant18'
                                                 * TJATVG weighted end time
                                                   DT:float32
                                                 */
extern real32_T TJATVG_PredTimeCrv_sec;         /* '<S891>/Constant4'
                                                 * TJATVG predict time of curve
                                                   DT:float32
                                                 */
extern real32_T TJATVG_PredTimeHeadAng_sec;     /* '<S891>/Constant10'
                                                 * TJATVG predict time of heading
                                                   DT:float32
                                                 */
extern real32_T TJATVG_MaxTrqScalLimit_nu;      /* '<S962>/Constant1'
                                                 * TJATVG max toque scal limit
                                                   DT:float32
                                                 */
extern real32_T TJATVG_MaxJerkAllowed_mps3;     /* '<S891>/Constant12'
                                                 * TJATVG max jerk allowed
                                                   DT:float32
                                                 */
extern real32_T TJACMB_CombinedPosY0_met;       /* '<S22>/Add'
                                                 * TJACMB combined PosY0
                                                   DT:float32
                                                 */
extern real32_T TJACMB_CombinedPosX0_met;       /* '<S14>/Max'
                                                 * TJACMB combined PosX0
                                                   DT:float32
                                                 */
extern real32_T TJACMB_CombinedHeading_rad;     /* '<S21>/Add'
                                                 * TJACMB combined Heading
                                                   DT:float32
                                                 */
extern real32_T TJACMB_CombinedCrv_1pm;         /* '<S19>/Switch'
                                                 * TJACMB combined Curve
                                                   DT:float32
                                                 */
extern real32_T TJACMB_CombinedCrvChng_1pm2;    /* '<S20>/Add'
                                                 * TJACMB combined Curve of change
                                                   DT:float32
                                                 */
extern real32_T TJACMB_CombinedLength_met;      /* '<S14>/Min'
                                                 * TJACMB combined length
                                                   DT:float32
                                                 */
extern real32_T TJATTG_LeCridrBndPosX0_met;     /* '<S10>/Signal Conversion12'
                                                 * TJATTG left corridor boundary
                                                 PosX0     DT:float32
                                                 */
extern real32_T TJATTG_LeCridrBndPosY0_met;     /* '<S10>/Signal Conversion13'
                                                 * TJATTG left corridor boundary
                                                 PosY0     DT:float32
                                                 */
extern real32_T TJATTG_LeCridrBndHeadAng_rad;   /* '<S10>/Signal Conversion14'
                                                 * TJATTG left corridor boundary
                                                 heading   DT:float32
                                                 */
extern real32_T TJATTG_LeCridrBndCrv_1pm;       /* '<S10>/Signal Conversion15'
                                                 * TJATTG left corridor boundary curve
                                                   DT:float32
                                                 */
extern real32_T TJATTG_LeCridrBndCrvChng_1pm2;  /* '<S10>/Signal Conversion16'
                                                 * TJATTG left corridor boundary
                                                 curve of change  DT:float32
                                                 */
extern real32_T TJATTG_LeCridrBndLength_met;    /* '<S10>/Signal Conversion17'
                                                 * TJATTG left corridor boundary
                                                 length    DT:float32
                                                 */
extern real32_T TJATTG_RiCridrBndPosX0_met;     /* '<S10>/Signal Conversion18'
                                                 * TJATTG right corridor boundary
                                                 PosX0     DT:float32
                                                 */
extern real32_T TJATTG_RiCridrBndPosY0_met;     /* '<S10>/Signal Conversion19'
                                                 * TJATTG right corridor boundary
                                                 PosY0     DT:float32
                                                 */
extern real32_T TJATTG_RiCridrBndHeadAng_rad;   /* '<S10>/Signal Conversion20'
                                                 * TJATTG right corridor boundary
                                                 heading   DT:float32
                                                 */
extern real32_T TJATTG_RiCridrBndCrv_1pm;       /* '<S10>/Signal Conversion21'
                                                 * TJATTG right corridor boundary
                                                 curve       DT:float32
                                                 */
extern real32_T TJATTG_RiCridrBndCrvChng_1pm2;  /* '<S10>/Signal Conversion22'
                                                 * TJATTG right corridor boundary
                                                 curve of change  DT:float32
                                                 */
extern real32_T TJATTG_RiCridrBndLength_met;    /* '<S10>/Signal Conversion23'
                                                 * TJATTG right corridor boundary
                                                 length    DT:float32
                                                 */
extern real32_T TJATTG_TgtTrajPosX0_met;        /* '<S10>/Signal Conversion24'
                                                 * TJATTG target trajectory PosX0
                                                   DT:float32
                                                 */
extern real32_T TJATTG_TgtTrajPosY0_met;        /* '<S10>/Signal Conversion25'
                                                 * TJATTG target trajectory PosY0
                                                   DT:float32
                                                 */
extern real32_T TJATTG_TgtTrajCrv_1pm;          /* '<S10>/Signal Conversion27'
                                                 * TJATTG target trajectory curve
                                                   DT:float32
                                                 */
extern real32_T TJATVG_PlanningHorizon_sec;     /* '<S890>/Multiport Switch'
                                                 * TJATVG planning horizon time
                                                   DT:float32
                                                 */
extern real32_T TJATTG_TgtTrajHeadAng_rad;      /* '<S10>/Signal Conversion26'
                                                 * TJATTG target trajectory heading
                                                   DT:float32
                                                 */
extern real32_T TJATTG_TgtTrajCrvChng_1pm2;     /* '<S10>/Signal Conversion28'
                                                 * TJATTG target trajectory curve of
                                                 change     DT:float32
                                                 */
extern real32_T TJATTG_TgtTrajLength_met;       /* '<S10>/Signal Conversion29'
                                                 * TJATTG target trajectory length
                                                   DT:float32
                                                 */
extern real32_T TJATVG_StrWhStifLimit_nu;       /* '<S962>/Product'
                                                 * TJATVG steer wheel limit
                                                   DT:float32
                                                 */
extern real32_T TJATVG_TrqRampGrad_1ps;         /* '<S961>/Multiport Switch2'
                                                 * TJATVG torque ramp gradient
                                                   DT:float32
                                                 */
extern real32_T TJATVG_StrWhStifGrad_1ps;       /* '<S961>/Multiport Switch'
                                                 * TJATVG steer wheel stif gradient
                                                   DT:float32
                                                 */
extern real32_T TJATVG_MaxTrqScalGrad_1ps;      /* '<S961>/Multiport Switch1'
                                                 * TJATVG steer wheel scal gradient
                                                   DT:float32
                                                 */
extern real32_T TJATVG_MaxCrvTrajGuiCtl_1pm;    /* '<S892>/Switch'
                                                 * TJATVG max curve trajectory
                                                 control    DT:float32
                                                 */
extern real32_T TJATVG_MaxCrvGrdBuildup_1pms;   /* '<S892>/Switch'
                                                 * TJATVG max curve gradient
                                                 buildup   DT:float32
                                                 */
extern real32_T TJATVG_MaxCrvGrdRed_1pms;       /* '<S892>/Switch'
                                                 * TJATVG max curve gradient red
                                                   DT:float32
                                                 */
extern real32_T TJATVG_MaxCrvGrdTGC_1pms;       /* '<S892>/Switch'
                                                 * TJATVG max curve gradient TGC
                                                   DT:float32
                                                 */
extern real32_T TJATVG_SensorTStamp_sec;        /* '<S889>/Multiport Switch'
                                                 * TJATVG sensor timestamp
                                                   DT:float32
                                                 */
extern real32_T TJATVG_ObstacleVelX_mps;        /* '<S891>/Constant'
                                                 * TJATVG obstacle velX
                                                   DT:float32
                                                 */
extern real32_T TJATVG_ObstacleAclX_mps2;       /* '<S891>/Constant2'
                                                 * TJATVG obstacle AclX
                                                   DT:float32
                                                 */
extern real32_T TJATVG_ObstacleWidth_met;       /* '<S891>/Constant3'
                                                 * TJATVG obstacle width
                                                   DT:float32
                                                 */
extern real32_T TJATVG_ObstacleDistX_met;       /* '<S891>/Constant8'
                                                 * TJATVG obstacle DisX
                                                   DT:float32
                                                 */
extern real32_T TJATVG_ObstacleDistY_met;       /* '<S891>/Constant9'
                                                 * TJATVG obstacle DisY
                                                   DT:float32
                                                 */
extern real32_T TJATVG_LimiterTimeDuration_sec; /* '<S891>/Constant7'
                                                 * TJATVG limiter time duration
                                                   DT:float32
                                                 */
extern int32_T TJASLC_SLCHighLightID_nu;        /* '<S375>/Switch3'
                                                 * TJATVG obstacle DisX
                                                   DT:float32
                                                 */
extern uint16_T TJATTG_TgtCorridorInvalid_btf; /* '<S681>/Data Type Conversion1'
                                                * TJASTM target corridor invalid
                                                bitfield DT:uint16
                                                */
extern uint16_T TJACMB_CombinedInvalid_btf;    /* '<S40>/Data Type Conversion1'
                                                * TJACMB combined invalid bitfield
                                                  DT:uint16
                                                */
extern uint16_T TJASTM_TJAInvalid_btf;         /* '<S572>/Data Type Conversion1'
                                                * TJASTM TJA invalid bitfield
                                                  DT:uint16
                                                */
extern uint16_T
    TJAGEN_SuspendedAndQuit_debug;           /* '<S78>/Data Type Conversion1'
                                              * TJAGEN Suspended And Quit debug
                                              */
extern uint16_T TJAOBF_ObjFollowInvalid_btf; /* '<S200>/Data Type Conversion1'
                                              * TJAOBF object  following invalid
                                              bitfield DT:uint16
                                              */
extern uint16_T TJASLC_TriggerInvalid_btf;   /* '<S525>/Data Type Conversion1'
                                              * SLC trigger invalid bitfield
                                                DT:uint16
                                              */
extern uint16_T
    TJASLC_RiLaneChangeInvalid_btf; /* '<S469>/Data Type Conversion1'
                                     * SLC right lane change invalid bitfield
                                       DT:uint16
                                     */
extern uint16_T
    TJASLC_LeLaneChangeInvalid_btf;           /* '<S468>/Data Type Conversion1'
                                               * SLC left lane change invalid bitfield
                                                 DT:uint16
                                               */
extern uint16_T TJAOBF_ObjInLaneInvalid_btf;  /* '<S218>/Data Type Conversion1'
                                               * TJAOBF object in lane invalid
                                               bitfield  DT:uint16
                                               */
extern uint16_T TJALKA_LaneCenterInvalid_btf; /* '<S136>/Data Type Conversion1'
                                               * TJALKA lane center invalid
                                               bitfield DT:uint16
                                               */
extern uint8_T TJATVG_DeratingLevel_nu;       /* '<S891>/Product'
                                               * TJATVG derating level
                                                 DT:uint8
                                               */
extern uint8_T TJAGEN_StrongReadyInvalid_btf; /* '<S96>/Data Type Conversion1'
                                               * TJAGEN strong ready invalid
                                               bitfield DT:uint8
                                               */
extern uint8_T TJALKA_LnIncoherenceStatus_nu; /* '<S128>/Switch1'
                                               * TJALKA Lane incoherence status
                                                 DT:uint8
                                               */
extern uint8_T TJASLC_LaneChangeInfo;         /* '<S374>/Switch1'
                                               * SLC weak ready flag
                                                 DT:boolean
                                               */
extern uint8_T TJATVG_TrajPlanServQu_nu;      /* '<S891>/Switch'
                                               * TJATVG trajectory plan serv qulifier
                                                 DT:uint8
                                               */
extern uint8_T TJAGEN_CancelStatus_btf;       /* '<S56>/Data Type Conversion1'
                                               * TJAGEN cancel status bitfield
                                                 DT:uint8
                                               */
extern uint8_T TJAGEN_WeakReadyInvalid_btf;   /* '<S100>/Data Type Conversion1'
                                               * TJAGEN weak ready invalid
                                               bitfield   DT:uint8
                                               */
extern uint8_T TJATOW_DriverTakeOverWarning_nu; /* '<S9>/Data Type Conversion1'
                                                 * TJATOW takeover warning
                                                   DT:uint8
                                                 */
extern uint8_T TJASTM_NpilotSysInfo;            /* '<S613>/Switch' */
extern uint8_T TJALKA_LaneIncoherence_btf; /* '<S133>/Data Type Conversion1'
                                            * TJALKA Lane incoherence debug
                                              DT:uint8
                                            */
extern uint8_T TJASTM_PilotAudioPlay;      /* '<S613>/Switch5' */
extern uint8_T
    TJASTM_LatCtrlHandsOffReleaseWarn_nu; /* '<S546>/Data Type Conversion3' */
extern uint8_T
    TJASTM_PilotDisableACCSwitch_nu; /* '<S546>/Data Type Conversion2' */
extern uint8_T
    TJASTM_PilotEnableACCSwitch_nu;          /* '<S546>/Data Type Conversion' */
extern uint8_T TJASLC_LaneChangWarning_nu;   /* '<S375>/Data Type Conversion'
                                              * lane change warning side
                                                0: no warning
                                                1: left warning
                                                2: right warning
                                              */
extern uint8_T TJASLC_SLCAudioPlay_nu;       /* '<S322>/Switch' */
extern uint8_T TJASLC_CancelAbort_btf;       /* '<S294>/Data Type Conversion1'
                                              * SLC cancel abort bitfield
                                                DT:uint8
                                              */
extern uint8_T TJASLC_TurnLtDirctionReq_nu;  /* '<S7>/Data Type Conversion'
                                              * Turn light diercetion reques:
                                                0: norequest 1: left; 2: Right
                                                DT:uint8
                                              */
extern uint8_T TJAOBF_TgtObjDataInvalid_btf; /* '<S255>/Data Type Conversion1'
                                              * TJAOBF object data invalid
                                              bitfield DT:uint8
                                              */
extern uint8_T TJALKA_LnQualityInv_btf;      /* '<S181>/Data Type Conversion1'
                                              * TJALKA lane quality invalid bitfield
                                                DT:uint8
                                              */
extern uint8_T TJATVG_CrvAmplActivated_nu;   /* '<S891>/Constant1'
                                              * TJATVG CrvAmplActivated
                                                DT:uint8
                                              */
extern uint8_T TJATVG_LimiterActivated_nu;   /* '<S891>/Constant5'
                                              * TJATVG limiter Activated
                                                DT:uint8
                                              */
extern uint8_T TJATVG_LimiterType_nu;        /* '<S891>/Constant6'
                                              * TJATVG limiter type
                                                DT:uint8
                                              */
extern boolean_T TJATVG_TriggerReplan_nu;    /* '<S891>/Constant11'
                                              * TJATVG trigger replan flag
                                                DT:boolean
                                              */
extern boolean_T TJATVG_HighStatAccu_bool;   /* '<S893>/Constant1'
                                              * TJATVG  high stationary accuracy
                                              flag   DT:boolean
                                              */
extern boolean_T TJALKA_LanePredictValid_bool; /* '<S165>/Switch'
                                                * TJALKA lane predict valid flag
                                                  DT:boolean
                                                */
extern boolean_T TJALKA_Cancel_bool;           /* '<S101>/AND1'
                                                * TJALKA cancel ready flag
                                                  DT:boolean
                                                */
extern boolean_T
    TJAOBF_ObjLaneValidDuration_bool;            /* '<S222>/AND'
                                                  * TJAOBF object in lane valid duration
                                                  flag            DT:boolean
                                                  */
extern boolean_T TJALKA_StrongReady_bool;        /* '<S101>/AND'
                                                  * TJALKA strong ready flag
                                                    DT:boolean
                                                  */
extern boolean_T TJAGEN_LKAOnlySwitch_bool;      /* '<S48>/AND3'
                                                  * TJAGEN LKA only switch flag
                                                    DT:boolean
                                                  */
extern boolean_T TJAOBF_Cancel_bool;             /* '<S182>/AND3'
                                                  * TJAOBF cancel flag
                                                    DT:boolean
                                                  */
extern boolean_T TJALKA_WeakReady_bool;          /* '<S101>/AND2'
                                                  * TJALKA weak ready flag
                                                    DT:boolean
                                                  */
extern boolean_T TJAGEN_Clearance_bool;          /* '<S47>/AND'
                                                  * TJAGEN clearance flag
                                                    DT:boolean
                                                  */
extern boolean_T TJAGEN_Degradation_bool;        /* '<S50>/AND'
                                                  * TJAGEN degradation flag
                                                    DT:boolean
                                                  */
extern boolean_T TJACMB_ObjectCorridor_bool;     /* '<S18>/AND'
                                                  * TJACMB object corridor flag
                                                    DT:boolean
                                                  */
extern boolean_T TJAOBF_StrongReady_bool;        /* '<S182>/AND1'
                                                  * TJAOBF strong ready flag
                                                    DT:boolean
                                                  */
extern boolean_T TJAOBF_WeakReady_bool;          /* '<S182>/AND2'
                                                  * TJAOBF weak ready flag
                                                    DT:boolean
                                                  */
extern boolean_T TJASLC_Nb_DCLCSwitchNVRAM_nu;   /* '<S267>/Switch2'
                                                  * TJATVG obstacle DisX
                                                    DT:float32
                                                  */
extern boolean_T TJASLC_StrongReady_bool;        /* '<S315>/Switch'
                                                  * SLC strong ready flag
                                                    DT:boolean
                                                  */
extern boolean_T TJASLC_WeakReady_bool;          /* '<S315>/Switch2'
                                                  * SLC weak ready flag
                                                    DT:boolean
                                                  */
extern boolean_T TJASLC_Cancel_bool;             /* '<S315>/Signal Conversion'
                                                  * SLC cancel flag
                                                    DT:boolean
                                                  */
extern boolean_T TJACMB_StrongReady_bool;        /* '<S15>/AND1'
                                                  * TJACMB Strong ready flag
                                                    DT:boolean
                                                  */
extern boolean_T TJACMB_WeakReady_bool;          /* '<S15>/AND2'
                                                  * TJACMB weak ready flag
                                                    DT:boolean
                                                  */
extern boolean_T TJAGEN_FunctionSwitch_bool;     /* '<S48>/OR1'
                                                  * TJAGEN function switch flag
                                                    DT:boolean
                                                  */
extern boolean_T TJAGEN_CodeFunction_bool;       /* '<S48>/OR3'
                                                  * TJAGEN code function flag
                                                    DT:boolean
                                                  */
extern boolean_T TJAGEN_Error_bool;              /* '<S51>/Switch'
                                                  * TJAGEN error flag
                                                    DT:boolean
                                                  */
extern boolean_T SLC_LCPLeft2Active_bool;        /* '<S370>/AND'
                                                  * LCPLeft2Active Flag
                                                  */
extern boolean_T SLC_LCPRight2Active_bool;       /* '<S373>/AND'
                                                  * LCPRight2Active Flag
                                                  */
extern boolean_T SLC_LCPLeft2Passive_bool;       /* '<S371>/AND'
                                                  * LCPLeft2Passive Flag
                                                  */
extern boolean_T SLC_LCPRight2Passive_bool;      /* '<S372>/AND'
                                                  * LCPRight2Passive Flag
                                                  */
extern boolean_T TJAGEN_Abort_bool;              /* '<S49>/NotEqual1'
                                                  * TJAGEN abort flag
                                                    DT:boolean
                                                  */
extern boolean_T GEN_AllStateAvailable_bool;     /* '<S89>/Equal2' */
extern boolean_T TJAGEN_StrongReady_bool;        /* '<S53>/AND'
                                                  * TJAGEN strong ready flag
                                                    DT:boolean
                                                  */
extern boolean_T TJAGEN_Cancel_nu;               /* '<S46>/OR'
                                                  * TJAGEN cancel flag
                                                    DT:boolean
                                                  */
extern boolean_T TJAGEN_WeakReady_bool;          /* '<S54>/AND'
                                                  * TJAGEN weak ready flag
                                                    DT:boolean
                                                  */
extern boolean_T TJAGEN_FunctionQuit_bool;       /* '<S52>/OR'
                                                  * Function Quit Flag
                                                  */
extern boolean_T TJAGEN_SuspendStart_bool;       /* '<S52>/OR4'
                                                  * Suspended start Flag
                                                  */
extern boolean_T TJAGEN_SuspendEnd_bool;         /* '<S52>/OR5'
                                                  * Suspended end Flag
                                                  */
extern boolean_T STM_PrevRAMPOUT_bool;           /* '<S552>/Unit Delay'
                                                  * Previous rampout Flag
                                                  */
extern boolean_T TJATTG_PredictionEnable_bool;   /* '<S743>/Switch'
                                                  * TJATTG prediction enable flag
                                                    DT:boolean
                                                  */
extern boolean_T TJATTG_TransTriggerReplan_bool; /* '<S670>/OR2'
                                                  * TJATTG transition trigger
                                                  replan flag DT:boolean
                                                  */
extern boolean_T TJASLC_TakeOverValid_bool;      /* '<S378>/OR4'
                                                  * SLC take over valid flag
                                                    DT:boolean
                                                  */
extern boolean_T TJATVG_LtcyCompActivated_nu;    /* '<S924>/Switch'
                                                  * TJATVG  latency compensation
                                                  flag    DT:boolean
                                                  */
extern boolean_T TJALKA_SRLaneRelateCheck_bool;  /* '<S101>/AND3'
                                                  * TJALKA strong ready line
                                                  related check  DT:boolean
                                                  */
extern boolean_T TJASTM_DrvTakeOver_bool;        /* '<S553>/AND'
                                                  * TJASTM driver takeover Flag
                                                  */
extern boolean_T STM_SuspendTimeExpired_bool;    /* '<S603>/Switch'
                                                  * STE Flag
                                                  */
extern boolean_T TJAOBF_TgtObjDataValid_bool;    /* '<S5>/Signal Conversion'
                                                  * TJAOBF target date valid flag
                                                    DT:boolean
                                                  */
extern boolean_T TJACMB_Cancel_bool;             /* '<S15>/Constant2'
                                                  * TJACMB cancel flag
                                                    DT:boolean
                                                  */
extern E_TJATVG_TrajGuiQu_nu
    TJATVG_TrajGuiQu_nu; /* '<S888>/Switch'
                          * TJATVG trajectory qualifier
                            DT:Enum: E_TJATVG_TrajGuiQu_nu
                          */
extern E_TJASTM_SysStateTJA_nu
    TJASTM_SysStateTJAIn_nu; /* '<S8>/Signal Conversion'
                              * TJASTM system state
                                DT:Enum: E_TJASTM_SysStateTJA_nu
                              */
extern E_TJASTM_SysStateTJA_nu
    TJASTM_SysStateTJA_nu;                            /* '<S552>/Switch3'
                                                       * TJASTM system state
                                                         DT:Enum: E_TJASTM_SysStateTJA_nu
                                                       */
extern E_TJASTM_SysStateHWA_nu TJASTM_SysStateHWA_nu; /* '<S551>/Switch2' */
extern E_TJASTM_LatCtrlMode_nu
    TJASTM_LatCtrlMode_nu; /* '<S8>/Signal Conversion1'
                            * TJASTM lateral control mode
                              DT:Enum: E_TJASTM_LatCtrlMode_nu
                            */
extern E_TJASLC_ReadyToTrigger_nu
    TJASLC_ReadyToTrigger_nu; /* '<S479>/Switch'
                               * SLC ready to trigger state
                                 DT:Enum: E_TJASLC_ReadyToTrigger_nu
                               */
extern E_TJASLC_ManeuverState_nu
    TJASLC_ManeuverState_nu; /* '<S320>/Switch'
                              * SLC maneuver state
                                DT:Enum: E_TJASLC_ManeuverState_nu
                              */
extern E_TJASLC_LaneChangeTrig_nu
    TJASLC_LCDirection_enum; /* '<S264>/Signal Conversion'
                              * SLC cancel flag
                                DT:boolean
                              */
extern E_TJASLC_LaneChangeTrig_nu
    TJASLC_LaneChangeTrig_nu; /* '<S516>/Switch2'
                               * SLC lane change trigger state
                                 DT:Enum: E_TJASLC_LaneChangeTrig_nu
                               */
extern E_TJALKA_LnBndValid_nu
    TJALKA_LnBndValid_nu; /* '<S151>/Switch'
                           * TJALKA lane boundary valid state
                             DT:Enum: E_TJALKA_LnBndValid_nu
                           */

/* Model entry point functions */
extern void TJASA_initialize(void);
extern void TJASA_step(void);

/* Exported data declaration */

/* Declaration for custom storage class: Global */
extern uint8_T TJASA_SetBit_BS_Param_1[16]; /* Referenced by:
                                             * '<S572>/ex_sfun_set_bit'
                                             * '<S40>/ex_sfun_set_bit'
                                             * '<S78>/ex_sfun_set_bit'
                                             * '<S136>/ex_sfun_set_bit'
                                             * '<S200>/ex_sfun_set_bit'
                                             * '<S218>/ex_sfun_set_bit'
                                             * '<S468>/ex_sfun_set_bit'
                                             * '<S469>/ex_sfun_set_bit'
                                             * '<S525>/ex_sfun_set_bit'
                                             * '<S681>/ex_sfun_set_bit'
                                             */
extern uint8_T TJASA_SetBit_BS_Param_2[8];  /* Referenced by:
                                             * '<S56>/ex_sfun_set_bit'
                                             * '<S96>/ex_sfun_set_bit'
                                             * '<S100>/ex_sfun_set_bit'
                                             * '<S181>/ex_sfun_set_bit'
                                             * '<S255>/ex_sfun_set_bit'
                                             * '<S294>/ex_sfun_set_bit'
                                             */
extern uint8_T TJASA_SetBit_BS_Param_3[5];
/* Referenced by: '<S133>/ex_sfun_set_bit' */

/* ConstVolatile memory section */
/* Declaration for custom storage class: ConstVolatile */
extern const volatile boolean_T
    OF_CheckRearObjects_C_bool; /* Referenced by: '<S203>/Constant3' */

/* 0: No observation of rear objects
   1: Switch on rear object check */
extern const volatile boolean_T
    OF_LcaBsdSigEnabled_C_bool; /* Referenced by:
                                 * '<S203>/Constant'
                                 * '<S203>/Constant2'
                                 */

/* Switch to enable check of LCA/BSD signals for rear object assessment
   (if FALSE MS flag signals will be read) */
extern const volatile real32_T
    OF_MinDurFreeAdjLane_C_sec; /* Referenced by: '<S203>/Constant1' */

/* Minimum duration of an empty adjacent lane to allow initilization of object
 * follow mode */
extern const volatile real32_T
    OF_MinDurFreeSideCollision_C_sec; /* Referenced by: '<S193>/Constant2' */

/* Minimum duration of an empty adjacent lane to allow initilization of object
 * follow mode */
extern const volatile boolean_T
    TJACMB_CMB_Enabled_C_bool; /* Referenced by: '<S15>/Constant1' */

/* Switch to enable combined mode */
extern const volatile boolean_T
    TJACMB_CombinedDataEnable_C_bool; /* Referenced by:
                                       * '<S15>/Constant3'
                                       * '<S18>/Constant1'
                                       * '<S665>/Constant'
                                       * '<S744>/Constant'
                                       * '<S809>/Constant'
                                       */

/* Switch to enable use of combined data */
extern const volatile boolean_T
    TJACMB_EnableOFO_C_bool; /* Referenced by: '<S17>/Constant4' */

/* Switch to enable object following only during combined mode */
extern const volatile boolean_T
    TJACMB_IndVelLimitsEnable_C_bool; /* Referenced by:
                                       * '<S39>/Constant1'
                                       * '<S39>/Constant6'
                                       */

/* TRUE if individual combined mode velocities shall be enabled
   (P_TJACMB_VelXMax_kph, P_TJACMB_VelXMin_kph) */
extern const volatile uint8_T
    TJACMB_LaneQualityHyst_C_perc; /* Referenced by: '<S16>/Constant3' */

/* Minimum ABPR lane quality below which use of lane data in combined shall be
 * disabled (hysteresis) */
extern const volatile uint8_T
    TJACMB_LaneQualityMin_C_perc; /* Referenced by: '<S16>/Constant4' */

/* Minimum ABPR lane quality below which use of lane data in combined shall be
 * disabled */
extern const volatile real32_T
    TJACMB_LnLengthMaxOFO_C_met; /* Referenced by: '<S17>/Constant3' */

/* Maximum ABPR lane length below which OFO mode can be enabled */
extern const volatile boolean_T
    TJACMB_LnQualBothLanes_C_bool; /* Referenced by: '<S16>/Constant5' */

/*  Switch to require both lane qualities invalid for transition to object data
 * during combined mode */
extern const volatile boolean_T
    TJACMB_LnQualCheckEnable_C_bool; /* Referenced by: '<S16>/Constant2' */

/* Switch to enable check of lane qualities for transition to object data during
 * combined mode */
extern const volatile real32_T
    TJACMB_LnQualTurnOffTime_C_sec; /* Referenced by: '<S16>/Constant' */

/* Turn off delay time for minimum lane quality check */
extern const volatile real32_T
    TJACMB_LnQualTurnOnTime_C_sec; /* Referenced by:
                                    * '<S16>/Constant1'
                                    * '<S19>/Constant3'
                                    */

/* Turn on delay time for minimum lane quality check */
extern const volatile real32_T
    TJACMB_LnWeightCrvChng_C_nu; /* Referenced by: '<S18>/Constant5' */

/* Combined mode curvature weight factor
   (1: 100 % weight on lane data, 0: 100 % weight on object data) */
extern const volatile real32_T
    TJACMB_LnWeightCrv_C_nu; /* Referenced by: '<S18>/Constant4' */

/* Combined mode curvature weight factor
   (1: 100 % weight on lane data, 0: 100 % weight on object data) */
extern const volatile real32_T
    TJACMB_LnWeightHead_C_nu; /* Referenced by: '<S18>/Constant3' */

/* Combined mode heading angle weight factor
   (1: 100 % weight on lane data, 0: 100 % weight on object data) */
extern const volatile real32_T
    TJACMB_LnWeightPosY0_C_nu; /* Referenced by: '<S18>/Constant2' */

/* Combined mode lateral position weight factor
   (1: 100 % weight on lane data, 0: 100 % weight on object data) */
extern const volatile real32_T
    TJACMB_TgtObjLengthMaxOFO_C_met; /* Referenced by: '<S17>/Constant1' */

/* Maximum target object length below which OFO mode can be enabled */
extern const volatile real32_T
    TJACMB_VelXMaxOFO_C_kph; /* Referenced by: '<S17>/Constant2' */

/* Maximum ego vehicle velocity below which OFO mode can be enabled */
extern const volatile real32_T
    TJACMB_VelXMax_C_kph; /* Referenced by: '<S39>/Constant7' */

/* Maximum allowed ego vehicle velocity for combined mode */
extern const volatile real32_T
    TJACMB_VelXMin_C_kph; /* Referenced by: '<S39>/Constant3' */

/* Minimum required ego vehicle velocity for combined mode */
extern const volatile real32_T TJAGEN_AccelXHyst_C_mps2; /* Referenced by:
                                                          * '<S88>/Constant3'
                                                          * '<S88>/Constant6'
                                                          */

/* Maximum long acceleration for TJA activation (hysteresis) */
extern const volatile real32_T
    TJAGEN_AccelYHyst_C_mps2; /* Referenced by: '<S88>/Constant1' */

/* Maximum lateral acceleration for TJA activation (hysteresis) */
extern const volatile real32_T
    TJAGEN_AclXMax_C_mps2; /* Referenced by: '<S88>/Constant4' */

/* Maximum long acceleration for TJA activation */
extern const volatile real32_T
    TJAGEN_AclXMin_C_mps2; /* Referenced by: '<S88>/Constant5' */

/* Minimum long deceleration for TJA activation */
extern const volatile real32_T
    TJAGEN_AclYMax_C_mps2; /* Referenced by: '<S88>/Constant2' */

/* Maximum lateral acceleration for TJA activation */
extern const volatile uint8_T
    TJAGEN_ActiveStCtrlSR_C_btm; /* Referenced by: '<S89>/Constant2' */

/* Bitmask for active state check of vehicle safety functions */
extern const volatile uint8_T
    TJAGEN_ActiveStCtrlWarn_C_btm; /* Referenced by: '<S613>/Constant23' */

/* Bitmask for active state check of vehicle safety functions */
extern const volatile real32_T
    TJAGEN_BlockTimeTJA_C_sec; /* Referenced by: '<S97>/Constant2' */

/* TJA blocking time */
extern const volatile boolean_T
    TJAGEN_CheckTJAErrorState_C_bool; /* Referenced by: '<S51>/Constant2' */

/* TJA manual function switch (if HMI not working):
   1: TJA switch on, 0: TJA switched off */
extern const volatile uint8_T TJAGEN_DrvStBrake_C_btm; /* Referenced by:
                                                        * '<S609>/Constant4'
                                                        * '<S62>/Constant4'
                                                        */

/* Bitmask value for driver brake intervention */
extern const volatile uint8_T
    TJAGEN_DrvStHOD_C_btm; /* Referenced by: '<S64>/Constant1' */

/* Bitmask value for turn signal hazard */
extern const volatile uint8_T
    TJAGEN_DrvStHazard_C_btm; /* Referenced by: '<S67>/Constant1' */

/* Bitmask value for turn signal hazard */
extern const volatile uint8_T
    TJAGEN_DrvStInvalidC_C_btm; /* Referenced by: '<S46>/Constant4' */

/* Bitmask value for driver state checks of the cancel condition */
extern const volatile uint8_T
    TJAGEN_DrvStInvalidSR_C_btm; /* Referenced by: '<S90>/Constant2' */

/*  Bitmask value for driver state checks of the strong ready condition */
extern const volatile uint8_T
    TJAGEN_DrvStInvalidWR_C_btm; /* Referenced by: '<S54>/Constant1' */

/* Bitmask value for driver state checks of the weak ready condition */
extern const volatile uint8_T
    TJAGEN_DrvStNotBuckled_C_btm; /* Referenced by: '<S63>/Constant1' */

/* Bitmask value for not buckled up */
extern const volatile real32_T
    TJAGEN_HODMaxTime_sec; /* Referenced by: '<S64>/V_Parameter3' */

/* Max time of HOD */
extern const volatile real32_T
    TJAGEN_HazardMaxTime_sec; /* Referenced by: '<S67>/V_Parameter3' */

/* Max time of turn signal hazard */
extern const volatile boolean_T
    TJAGEN_LKA_Available_C_bool; /* Referenced by:
                                  * '<S48>/Constant'
                                  * '<S48>/Constant3'
                                  * '<S51>/Constant'
                                  */

/* Indicates TRUE if LKA is a coded function of the system */
extern const volatile real32_T
    TJAGEN_ManualTorqueESMaxTime_sec; /* Referenced by: '<S65>/V_Parameter3' */

/* Max time of manual torque intervention */
extern const volatile real32_T
    TJAGEN_ManualTorqueHyst_nm; /* Referenced by:
                                 * '<S65>/V_Parameter2'
                                 * '<S65>/V_Parameter4'
                                 */

/* maximum manual torque hysteresis */
extern const volatile real32_T
    TJAGEN_ManualTorqueMaxTime_sec; /* Referenced by: '<S65>/V_Parameter1' */

/* Max time of manual torque intervention */
extern const volatile real32_T TJAGEN_ManualTorqueMax_nm;
/* Referenced by: '<S65>/V_Parameter11' */

/* maximum manual torque  */
extern const volatile real32_T
    TJAGEN_ManualTorqueMin_nm; /* Referenced by: '<S65>/V_Parameter5' */

/* minimum manual torque  */
extern const volatile uint16_T
    TJAGEN_PrjSpecQuA_C_btm; /* Referenced by: '<S49>/Constant7' */

/* Bitmask to check project specific abort conditions for all TJA modes */
extern const volatile uint16_T
    TJAGEN_PrjSpecQuC_C_btm; /* Referenced by: '<S49>/Constant1' */

/* Bitmask to check project specific cancel conditions for all TJA modes */
extern const volatile uint16_T
    TJAGEN_PrjSpecQuSR_C_btm; /* Referenced by: '<S49>/Constant2' */

/* Bitmask to check project specific strong ready conditions for all TJA modes
 */
extern const volatile uint16_T
    TJAGEN_PrjSpecQuWR_C_btm; /* Referenced by: '<S49>/Constant4' */

/* Bitmask to check project specific weak ready conditions for all TJA modes */
extern const volatile uint8_T
    TJAGEN_QuTrajCtrCancel_C_btm; /* Referenced by: '<S46>/Constant7' */

/* Bitmask check for S_TCTCLM_QuServTrajCtr_nu as cancel condition */
extern const volatile uint8_T
    TJAGEN_QuTrajCtrClearance_C_btm; /* Referenced by: '<S47>/Constant4' */

/* Bitmask check for S_TCTCLM_QuServTrajCtr_nu as clearance condition */
extern const volatile uint16_T
    TJAGEN_QuTrajPlanCancel_C_btm; /* Referenced by: '<S46>/Constant1' */

/* Bitmask check for S_TPLTJC_QuStatusTrajPlan_nu as cancel condition */
extern const volatile uint16_T
    TJAGEN_QuTrajPlanClearance_C_btm; /* Referenced by: '<S47>/Constant2' */

/* Bitmask check for S_TPLTJC_QuStatusTrajPlan_nu as clearance condition */
extern const volatile uint8_T
    TJAGEN_QuTrajPlanMinLnQual_C_perc; /* Referenced by: '<S105>/Constant1' */

/* Minimum required lane quality for function re-activation after specific
 * blocking time for TrajPlanCancelQualifier */
extern const volatile real32_T TJAGEN_RampoutTimeMax_C_sec; /* Referenced by:
                                                             * '<S59>/Constant'
                                                             * '<S599>/Constant'
                                                             */

/*  Maximum rampout time */
extern const volatile real32_T
    TJAGEN_SafetyFuncMaxTime_sec; /* Referenced by:
                                   * '<S71>/V_Parameter1'
                                   * '<S71>/V_Parameter2'
                                   */

/* Max time of Safety function */
extern const volatile boolean_T
    TJAGEN_SetSysStOnLatDMC_C_bool; /* Referenced by: '<S47>/Constant1' */

/* Manual switch to set LatDMC system state ON */
extern const volatile real32_T
    TJAGEN_SteerWAngleGradHystDI_C_degps; /* Referenced by: '<S68>/Constant8' */

/* steering wheel angle grad hysteresis */
extern const volatile real32_T
    TJAGEN_SteerWAngleGradHystSus_C_degps; /* Referenced by: '<S68>/Constant2'
                                            */

/* steering wheel angle grad hysteresis for suspended */
extern const volatile real32_T
    TJAGEN_SteerWAngleGradMaxDI_C_degps; /* Referenced by: '<S68>/Constant7' */

/* steering wheel angle grad maximum */
extern const volatile real32_T
    TJAGEN_SteerWAngleGradMaxSus_C_degps; /* Referenced by: '<S68>/Constant1' */

/* steering wheel angle grad maximum for suspended */
extern const volatile real32_T
    TJAGEN_SteerWAngleHystDI_C_deg; /* Referenced by: '<S69>/Constant6' */

/* steering wheel angle hysterisis */
extern const volatile real32_T
    TJAGEN_SteerWAngleHystSus_C_deg; /* Referenced by: '<S69>/Constant2' */

/* steering wheel angle hysterisis for suspended */
extern const volatile real32_T
    TJAGEN_SteerWAngleMaxDI_C_deg; /* Referenced by: '<S69>/Constant3' */

/* steering wheel angle maximum */
extern const volatile real32_T
    TJAGEN_SteerWAngleMaxSus_C_deg; /* Referenced by: '<S69>/Constant1' */

/* steering wheel angle maximum for suspended */
extern const volatile real32_T
    TJAGEN_SteerWAngleMaxWR_C_deg; /* Referenced by: '<S97>/Constant3' */

/* Maximum steering wheel angle for function activation */
extern const volatile uint8_T TJAGEN_SysStErrorSR_C_btm; /* Referenced by:
                                                          * '<S613>/Constant22'
                                                          * '<S89>/Constant1'
                                                          */

/* Bitmask for system error check of vehicle safety functions (ABS, ESC, etc.)
 * for SR condition */
extern const volatile uint8_T
    TJAGEN_SysStNotAvailableSR_C_btm; /* Referenced by:
                                       * '<S613>/Constant26'
                                       * '<S89>/Constant5'
                                       */

/* Bitmask to check for weak ready conditions if vehicle safety functions are
 * not available */
extern const volatile uint8_T
    TJAGEN_SysStNotAvailableWR_C_btm; /* Referenced by: '<S54>/Constant5' */

/* Bitmask to check for weak ready conditions if vehicle safety functions are
 * not available */
extern const volatile boolean_T
    TJAGEN_TJA_Available_C_bool; /* Referenced by:
                                  * '<S48>/Constant1'
                                  * '<S48>/Constant4'
                                  * '<S51>/Constant1'
                                  */

/* Indicates TRUE if TJA is a coded function of the system */
extern const volatile boolean_T
    TJAGEN_TJA_ManFunctionSwitch_C_bool; /* Referenced by: '<S48>/Constant2' */

/* TJA manual function switch (if HMI not working):
   1: TJA switch on, 0: TJA switched off */
extern const volatile uint8_T
    TJAGEN_VehAclYInvalid_C_btm; /* Referenced by: '<S71>/Constant7' */

/* Bitmask value for VehAclYInvalid */
extern const volatile real32_T
    TJAGEN_VehCrvHystDI_1pm; /* Referenced by: '<S70>/Constant11' */

/* Vehicle curve hysteresis */
extern const volatile real32_T
    TJAGEN_VehCrvMaxDI_1pm; /* Referenced by: '<S70>/Constant12' */

/* Vehicle curve maximum */
extern const volatile real32_T TJAGEN_VehCrvMaxWR_Cr_1pm[6];
/* Referenced by: '<S54>/1-D Lookup Table1' */

/* 1DLookupTable-Cr-Vehicle curve maximum */
extern const volatile uint8_T
    TJAGEN_VehSafetyFuncActive_C_btm; /* Referenced by: '<S71>/Constant1' */

/* Bitmask value for VehSafetyFuncActive */
extern const volatile uint8_T
    TJAGEN_VehSafetyFuncError_C_btm; /* Referenced by: '<S71>/Constant5' */

/* Bitmask value for VehSafetyFuncError */
extern const volatile uint16_T
    TJAGEN_VehStInvalidC_C_btm; /* Referenced by: '<S46>/Constant2' */

/* Bitmask value for vehicle state checks of the cancel condition */
extern const volatile uint16_T
    TJAGEN_VehStInvalidSR_C_btm; /* Referenced by: '<S92>/Constant2' */

/* Bitmask for vehicle state checks of the strong ready condition */
extern const volatile uint16_T
    TJAGEN_VehStInvalidWR_C_btm; /* Referenced by: '<S54>/Constant2' */

/*  Bitmask for vehicle state checks of the weak ready condition */
extern const volatile uint8_T
    TJAGEN_VehStInvalid_C_btm; /* Referenced by: '<S71>/Constant' */

/* Bitmask value for VehStInvalid */
extern const volatile real32_T TJAGEN_VehVelX_Bx_mps[6];
/* Referenced by: '<S54>/1-D Lookup Table1' */

/* 1DLookupTable-Bx-Vehicle velocity X */
extern const volatile real32_T
    TJAGEN_VehYawRateHystDI_C_rps; /* Referenced by: '<S72>/Constant10' */

/* Vehicle yaw rate hysteresis */
extern const volatile real32_T
    TJAGEN_VehYawRateMaxDI_C_rps; /* Referenced by: '<S72>/Constant9' */

/* Vehicle yaw rate maximum */
extern const volatile real32_T TJALKA_BlockTimeTJA_C_sec; /* Referenced by:
                                                           * '<S104>/Constant'
                                                           * '<S185>/Constant1'
                                                           */

/* TJA Blocking time */
extern const volatile boolean_T
    TJALKA_BothSideBrdgEnable_C_bool; /* Referenced by: '<S150>/Constant' */

/* TRUE if both-sided bridging/prediction is enabled */
extern const volatile real32_T TJALKA_CntrCrv_Bx_1pm[11];
/* Referenced by: '<S112>/1-D Lookup Table' */

/* 1DLookupTable-Bx-TJAOBF VehVelX  */
extern const volatile boolean_T
    TJALKA_ConstSiteCheckOn_C_bool; /* Referenced by: '<S106>/Constant' */

/*  Switch to enable construction site check */
extern const volatile real32_T TJALKA_CrvLaneWidthMaxThd_Cr_met[11];
/* Referenced by: '<S112>/1-D Lookup Table' */

/* 1DLookupTable-Cr-Maximum allowed target object clothoid heading angle for OF
 * mode activation (WR condition) */
extern const volatile real32_T
    TJALKA_CrvQualTurnOffTime_C_sec; /* Referenced by: '<S152>/Constant3' */

/* Turn off delay time for lane curvature quality check */
extern const volatile real32_T
    TJALKA_CrvQualTurnOnTime_C_sec; /* Referenced by: '<S152>/Constant2' */

/* Turn on delay time for lane curvature quality check */
extern const volatile uint8_T
    TJALKA_CrvQualityHyst_C_perc; /* Referenced by: '<S152>/Constant' */

/* Minimum lane curvature quality hysteresis */
extern const volatile uint8_T
    TJALKA_CrvQualityMin_C_perc; /* Referenced by: '<S152>/Constant1' */

/* Minimum lane curvature quality */
extern const volatile real32_T
    TJALKA_DistVeh2LnBndHyst_C_met; /* Referenced by: '<S116>/Constant1' */

/* Minimum distance to lane boundary hysteresis */
extern const volatile real32_T
    TJALKA_DistVeh2LnBndMin_C_met; /* Referenced by: '<S116>/Constant' */

/* Minimum distance to lane boundary */
extern const volatile uint8_T
    TJALKA_DrvStInvalidSR_C_btm; /* Referenced by: '<S118>/Constant2' */

/* Bitmask to check if turn signal left/right is engaged */
extern const volatile boolean_T
    TJALKA_InjectLaneError_C_bool; /* Referenced by: '<S153>/Constant5' */

/* Setting to TRUE will inject single error, which invalidates lanes for
 * P_TJALKA_LaneInvalidTime_sec seconds */
extern const volatile boolean_T
    TJALKA_LC_Enabled_C_bool; /* Referenced by: '<S101>/Constant1' */

/* Indicates TRUE, if lane centering mode is enabled. */
extern const volatile real32_T
    TJALKA_LaneInvalidTime_C_sec; /* Referenced by: '<S153>/Constant6' */

/* Lane invalidation time for lane error injection */
extern const volatile uint8_T
    TJALKA_LaneQualityHyst_C_perc; /* Referenced by: '<S154>/Constant' */

/* Minimum lane quality hysteresis */
extern const volatile uint8_T
    TJALKA_LaneQualityMin_C_perc; /* Referenced by: '<S154>/Constant1' */

/* Minimum lane quality */
extern const volatile real32_T
    TJALKA_LaneValidMaxTime_sec; /* Referenced by:
                                  * '<S102>/V_Parameter1'
                                  * '<S102>/V_Parameter3'
                                  */

/* Max time of manual torque intervention */
extern const volatile uint16_T
    TJALKA_LaneValid_C_btm; /* Referenced by: '<S153>/Constant3' */

/* Bitmask for lane validity check (left/right) */
extern const volatile uint16_T
    TJALKA_LaneVirtOrBridged_C_btm; /* Referenced by: '<S153>/Constant2' */

/* Bitmask to check if lanes are bridged or virtual */
extern const volatile real32_T TJALKA_LaneWidthHyst_C_met; /* Referenced by:
                                                            * '<S112>/Constant1'
                                                            * '<S112>/Constant2'
                                                            * '<S270>/Constant2'
                                                            * '<S270>/Constant4'
                                                            * '<S308>/Constant2'
                                                            * '<S308>/Constant4'
                                                            */

/* Hysteresis for lane width check */
extern const volatile real32_T
    TJALKA_LaneWidthMax_C_met; /* Referenced by: '<S308>/Constant1' */

/* Maximum allowed lane width for lane centering */
extern const volatile real32_T TJALKA_LaneWidthMin_C_met; /* Referenced by:
                                                           * '<S112>/Constant3'
                                                           * '<S308>/Constant3'
                                                           */

/* Minimum required lane width for lane centering */
extern const volatile real32_T
    TJALKA_LnIncohMaxTime_C_sec; /* Referenced by:
                                  * '<S111>/Constant2'
                                  * '<S111>/Constant5'
                                  * '<S111>/Constant6'
                                  */

/* Lane incoherence max time */
extern const volatile real32_T
    TJALKA_LnIncoherenceMaxPosY0_C_met; /* Referenced by:
                                         * '<S111>/Constant'
                                         * '<S111>/Constant1'
                                         */

/* Lane incoherence max posy0 */
extern const volatile real32_T
    TJALKA_LnPredMinValidTime_C_sec; /* Referenced by: '<S150>/Constant2' */

/* Minimum required time of valid lanes and active controlling in LC or CMB mode
 * for allowing both-sided lane prediction */
extern const volatile real32_T
    TJALKA_LnPredictionCrvMax_C_1pm; /* Referenced by: '<S150>/Constant1' */

/* Maximum allowed lane curvature for lane prediction activation */
extern const volatile real32_T
    TJALKA_LnPredictionTimeMax_C_sec; /* Referenced by: '<S164>/Constant1' */

/* Maximum allowed lane prediction duration */
extern const volatile real32_T
    TJALKA_LnPredictionTimeMin_C_sec; /* Referenced by: '<S164>/Constant' */

/* Minimum prediction time for rampout prediction (prediction of lanes while
 * outputs torque output is set to zero) */
extern const volatile real32_T
    TJALKA_LnQualTurnOffTime_C_sec; /* Referenced by: '<S154>/Constant3' */

/* Turn off delay time for lane quality check */
extern const volatile real32_T
    TJALKA_LnQualTurnOnTime_C_sec; /* Referenced by: '<S154>/Constant2' */

/* Turn on delay time for lane quality check */
extern const volatile real32_T
    TJALKA_MaxHeadAngActnTJA_C_rad; /* Referenced by: '<S110>/Constant1' */

/*  Maximum allowed ego vehicle heading angle for activation of TJA */
extern const volatile uint16_T
    TJALKA_PrjSpecQuC_C_btm; /* Referenced by: '<S107>/Constant6' */

/* Bitmask to check project specific cancel conditions for lane centering */
extern const volatile uint16_T
    TJALKA_PrjSpecQuSR_C_btm; /* Referenced by: '<S107>/Constant4' */

/* Bitmask to check project specific SR conditions for lane centering */
extern const volatile uint16_T
    TJALKA_PrjSpecQuWR_C_btm; /* Referenced by: '<S107>/Constant3' */

/* Bitmask to check project specific WR conditions for lane centering */
extern const volatile real32_T
    TJALKA_RadiusHyst_C_met; /* Referenced by: '<S114>/Constant1' */

/* Curvature radius hysteresis */
extern const volatile real32_T
    TJALKA_RadiusMin_C_met; /* Referenced by: '<S114>/Constant' */

/* Minimum curvature radius for lane centering controlling */
extern const volatile boolean_T
    TJALKA_RampoutPredictOn_C_bool; /* Referenced by:
                                     * '<S963>/Constant1'
                                     * '<S164>/Constant2'
                                     * '<S686>/Constant1'
                                     */

/* Switch to enable rampout prediction (lane prediction while LatDMC output is
 * set to zero) */
extern const volatile boolean_T
    TJALKA_TransLnChecksOff_C_bool; /* Referenced by: '<S103>/Constant' */

/* TRUE if lane quality checks shall be disabled during SALC --> LC transition.
   That is, lane quality is valid by default if the Parameter is TRUE and
   TakeOverValid flag from SLC module is TRUE. */
extern const volatile boolean_T
    TJALKA_UseUncoupLaneWidth_C_bool; /* Referenced by: '<S112>/Constant4' */

/* Switch to enable use of uncoupled lane width for lane width check */
extern const volatile real32_T
    TJALKA_ValidLengthMinHyst_C_met; /* Referenced by: '<S115>/Constant2' */

/* Minimum required valid length (hysteresis) */
extern const volatile real32_T
    TJALKA_ValidLengthMin_C_met; /* Referenced by: '<S115>/Constant3' */

/* Minimum required valid length */
extern const volatile real32_T
    TJALKA_VehDistMax_C_met; /* Referenced by: '<S126>/Constant3' */

/* Vehicle distance maximum */
extern const volatile real32_T TJALKA_VelXHyst_C_kph; /* Referenced by:
                                                       * '<S39>/Constant10'
                                                       * '<S39>/Constant2'
                                                       * '<S119>/Constant1'
                                                       * '<S119>/Constant2'
                                                       * '<S120>/Constant1'
                                                       * '<S120>/Constant2'
                                                       * '<S192>/Constant2'
                                                       * '<S192>/Constant4'
                                                       * '<S488>/Constant2'
                                                       * '<S488>/Constant4'
                                                       */

/* Longitudinal velocity hysteresis */
extern const volatile real32_T TJALKA_VelXMax_C_kph; /* Referenced by:
                                                      * '<S39>/Constant9'
                                                      * '<S119>/Constant'
                                                      * '<S120>/Constant'
                                                      */

/* Maximum longitudinal velocity allowed for lane centering mode */
extern const volatile real32_T
    TJALKA_VelXMinWR_C_kph; /* Referenced by: '<S120>/Constant3' */

/* Maximum longitudinal velocity allowed for lane centering mode */
extern const volatile real32_T TJALKA_VelXMin_C_kph; /* Referenced by:
                                                      * '<S39>/Constant5'
                                                      * '<S119>/Constant3'
                                                      */

/*  Minimum longitudinal velocity required for lane centering mode */
extern const volatile uint16_T
    TJAOBF_AccObjChange_C_btm; /* Referenced by: '<S206>/Constant4' */

/* Bit mask to check ACC object validity for object in target lane evalution
 * (ignoring lateral position validity) */
extern const volatile uint16_T
    TJAOBF_AccObjLanesInvalid_C_btm; /* Referenced by: '<S206>/Constant2' */

/* Bit mask to check ACC object validity for object in target lane evalution
 * (ignoring lateral position validity) */
extern const volatile uint16_T
    TJAOBF_AccObjectInvalid_C_btm; /* Referenced by: '<S206>/Constant1' */

/* Bit mask to check ACC object validity */
extern const volatile real32_T
    TJAOBF_CheckLineValidMaxTime_sec; /* Referenced by:
                                       * '<S242>/V_Parameter1'
                                       * '<S242>/V_Parameter2'
                                       */

/* Maximum lateral acceleration for TJA activation */
extern const volatile boolean_T
    TJAOBF_ConstSiteCheckOn_C_bool; /* Referenced by: '<S187>/Constant1' */

/* Switch to enable construction site check for OF SR condition */
extern const volatile uint8_T
    TJAOBF_CrvQualityHyst_C_perc; /* Referenced by: '<S242>/Constant4' */

/* Minimum lane curvature quality hysteresis for object-in-lane-evaluation */
extern const volatile uint8_T
    TJAOBF_CrvQualityMin_C_perc; /* Referenced by: '<S242>/Constant' */

/* Minimum lane curvature quality required for object-in-lane-evaluation */
extern const volatile real32_T
    TJAOBF_CutinObValidFreezTm_C_sec; /* Referenced by: '<S210>/Constant' */

/*  Freeze time of Acc object validity status after object cut-in */
extern const volatile real32_T
    TJAOBF_DefaultLaneWidth_C_met; /* Referenced by: '<S228>/Constant4' */

/* Assumption of default lane width for object-in-lane check for maximum
 * distance to lane boundary */
extern const volatile uint8_T
    TJAOBF_DrvStInvalidSR_C_btm; /* Referenced by: '<S194>/Constant2' */

/* Bitmask for turn signal SR condition check */
extern const volatile real32_T
    TJAOBF_EgoCurveMaxSideCollision_C_1pm; /* Referenced by:
                                            * '<S193>/Constant'
                                            * '<S193>/Constant1'
                                            */

/* Maximum allowed ego curvature for OF mode side collision check */
extern const volatile boolean_T
    TJAOBF_LaneCheckEnabled_C_bool; /* Referenced by: '<S224>/Constant1' */

/* Switch to enable object in lane evaluation */
extern const volatile real32_T
    TJAOBF_MaxDiffLnLen2ObjPosX_C_met; /* Referenced by:
                                        * '<S241>/Constant'
                                        * '<S241>/Constant2'
                                        */

/* Maximum longitudinal difference between detected lane length and acc object
 * position x for object  in lane evaluation */
extern const volatile real32_T
    TJAOBF_MaxDurObjBrdg_C_sec; /* Referenced by: '<S186>/Constant2' */

/* Maximum allowed duration of object bridging mode at high velocities */
extern const volatile real32_T
    TJAOBF_MinDurAccObjValid_C_sec; /* Referenced by:
                                     * '<S211>/Constant'
                                     * '<S212>/Constant'
                                     */

/* Minimum duration of ACC object validity for activation of object following
 * and combined mode */
extern const volatile real32_T
    TJAOBF_MinDurLCforOB_C_sec; /* Referenced by: '<S186>/Constant' */

/* Minimum duration of Lane Centering availability to allow object bridging at
 * high velocities */
extern const volatile real32_T
    TJAOBF_MinDurObjLnValidWR_C_sec; /* Referenced by: '<S219>/Constant' */

/* Minimum duration of object in lane validity as WR condition */
extern const volatile boolean_T
    TJAOBF_OF_Enabled_C_bool; /* Referenced by: '<S182>/Constant1' */

/*  Switch to enable object following mode */
extern const volatile boolean_T
    TJAOBF_ObjBrdgEnabled_C_bool; /* Referenced by: '<S186>/Constant1' */

/*  Switch to enable object briding at speeds greated than TJAOBF_VelXMax_kph */
extern const volatile uint16_T
    TJAOBF_PrjSpecQuC_C_btm; /* Referenced by: '<S188>/Constant3' */

/* Bitmask to check project specific cancel conditions for object following */
extern const volatile uint16_T
    TJAOBF_PrjSpecQuSR_C_btm; /* Referenced by: '<S188>/Constant1' */

/* Bitmask to check project specific SR conditions for object following */
extern const volatile uint16_T TJAOBF_PrjSpecQuWR_C_btm; /* Referenced by:
                                                          * '<S188>/Constant2'
                                                          * '<S486>/Constant2'
                                                          */

/* Bitmask to check project specific WR conditions for object following */
extern const volatile real32_T
    TJAOBF_TgtClthCrvMaxHyst_C_1pm; /* Referenced by: '<S251>/Constant' */

/* Maximum allowed target object clothoid curvature for OF mode activation
 * (hysteresis) */
extern const volatile real32_T TJAOBF_TgtClthCrvMaxWR_Cr_rad[6];
/* Referenced by: '<S251>/1-D Lookup Table1' */

/* 1DLookupTable-Cr-Maximum allowed target object clothoid curvature for OF mode
 * activation (WR condition) */
extern const volatile real32_T
    TJAOBF_TgtClthHeadMaxHyst_C_rad; /* Referenced by: '<S252>/Constant' */

/* Maximum allowed target object clothoid heading angle for OF mode activation
 */
extern const volatile real32_T TJAOBF_TgtClthHeadMaxWR_Cr_rad[6];
/* Referenced by: '<S252>/1-D Lookup Table1' */

/* 1DLookupTable-Cr-Maximum allowed target object clothoid heading angle for OF
 * mode activation (WR condition) */
extern const volatile uint16_T
    TJAOBF_TgtClthInvalid_C_btm; /* Referenced by: '<S250>/Constant1' */

/* Bit mask to check validity of ACC object clothoid generation */
extern const volatile real32_T
    TJAOBF_TgtClthLengthMin_C_met; /* Referenced by: '<S253>/Constant3' */

/* Minimum required object trajectory length */
extern const volatile real32_T
    TJAOBF_TgtClthMinValidTime_C_sec; /* Referenced by: '<S250>/Constant' */

/* Minimum validity time of target clothoid features for OF mode activation */
extern const volatile real32_T
    TJAOBF_TgtClthPosYMaxHyst_C_met; /* Referenced by: '<S254>/Constant' */

/* Maximum allowed target object clothoid pos Y0 for OF mode activation
 * (hysteresis) */
extern const volatile real32_T TJAOBF_TgtClthPosYMaxWR_Cr_met[6];
/* Referenced by: '<S254>/1-D Lookup Table1' */

/* 1DLookupTable-Cr-Maximum allowed target object clothoid lateral position Y0
 * for OF mode activation (WR condition) */
extern const volatile real32_T
    TJAOBF_TgtVehDist2LnBndHst_C_met; /* Referenced by:
                                       * '<S228>/Constant1'
                                       * '<S228>/Constant3'
                                       */

/* Target vehicle minimum distance to lane boundary hysteresis
   (Distance defined as vehicle center to lane boundary) */
extern const volatile real32_T
    TJAOBF_TgtVehDist2LnBndMin_C_met; /* Referenced by:
                                       * '<S228>/Constant'
                                       * '<S228>/Constant2'
                                       */

/* Target vehicle minimum distance to lane boundary hysteresis
   (Distance defined as vehicle center to lane boundary) */
extern const volatile real32_T
    TJAOBF_VehVelX_Bx_kph[6]; /* Referenced by:
                               * '<S251>/1-D Lookup Table1'
                               * '<S252>/1-D Lookup Table1'
                               * '<S254>/1-D Lookup Table1'
                               */

/* 1DLookupTable-Bx-TJAOBF VehVelX  */
extern const volatile real32_T TJAOBF_VelXMax_C_kph; /* Referenced by:
                                                      * '<S39>/Constant8'
                                                      * '<S192>/Constant1'
                                                      */

/* Maximum longitudinal velocity allowed for object following mode */
extern const volatile real32_T TJAOBF_VelXMin_C_kph; /* Referenced by:
                                                      * '<S39>/Constant4'
                                                      * '<S192>/Constant3'
                                                      */

/*  Minimum longitudinal velocity required for object following mode */
extern const volatile real32_T
    TJAPARAM_VEH_Wheelbase_C_met; /* Referenced by:
                                   * '<S957>/Constant7'
                                   * '<S363>/Constant4'
                                   */

/* Vehicle wheelbase */
extern const volatile real32_T TJAPARAM_VEH_Width_C_met; /* Referenced by:
                                                          * '<S362>/Constant1'
                                                          * '<S363>/Constant1'
                                                          */

/* Vehicle width */
extern const volatile real32_T
    TJASLC_AbortYHdSignTurnOnTi_sec; /* Referenced by: '<S327>/Constant' */

/* Minimum ego lane length for activation of lane change */
extern const volatile uint16_T
    TJASLC_AdjLaneBridged_C_btm; /* Referenced by: '<S269>/Constant3' */

/* Adjacent Lane Invalid Qualifier Bitmask */
extern const volatile uint16_T
    TJASLC_AdjLaneInvalid_C_btm; /* Referenced by: '<S269>/Constant1' */

/* Adjacent Lane Invalid Qualifier Bitmask */
extern const volatile real32_T
    TJASLC_AdjLaneWidthMax_C_met; /* Referenced by: '<S270>/Constant1' */

/* Maximum adjacent lane width */
extern const volatile real32_T
    TJASLC_AdjLaneWidthMin_C_met; /* Referenced by: '<S270>/Constant3' */

/* Minimum adjacent lane width */
extern const volatile real32_T
    TJASLC_BlockTimeSALC_C_sec; /* Referenced by: '<S484>/Constant1' */

/* SALC specific blocking time */
extern const volatile real32_T
    TJASLC_BlockTmSALCCancle_C_sec; /* Referenced by: '<S484>/Constant2' */

/* Maximum long acceleration for TJA activation (hysteresis) */
extern const volatile boolean_T
    TJASLC_CheckAdjLanes_C_bool; /* Referenced by:
                                  * '<S269>/Constant5'
                                  * '<S270>/Constant5'
                                  */

/* Switch to enable adjacent lane check */
extern const volatile boolean_T
    TJASLC_CheckLaneTypes_C_bool; /* Referenced by: '<S306>/Constant' */

/* Enable check of lane types for lane change */
extern const volatile boolean_T
    TJASLC_CheckRearObjects_C_bool; /* Referenced by: '<S266>/Constant3' */

/* 0: No observation of rear objects
   1: Switch on rear object check */
extern const volatile boolean_T
    TJASLC_DisableAllowGoBack_bool; /* Referenced by:
                                     * '<S274>/Constant'
                                     * '<S279>/Constant'
                                     */

/* Switch to enable check of lane qualities for transition to object data during
 * combined mode */
extern const volatile real32_T TJASLC_DistAllowAbortHd_Mp_nu[12];
/* Referenced by: '<S299>/1-D Lookup Table5' */

/* 1DLookupTable-Bx-Vehicle velocity X */
extern const volatile real32_T TJASLC_DistAllowAbortVelX_Mp_met[12];
/* Referenced by: '<S299>/1-D Lookup Table1' */

/* 1DLookupTable-Bx-Vehicle velocity X */
extern const volatile real32_T
    TJASLC_EgoCrvMaxActivation_C_1pm; /* Referenced by: '<S305>/Constant' */

/* Maximum curvature of ego lane for activation of lane change */
extern const volatile real32_T
    TJASLC_EgoLengthMinActv_C_met; /* Referenced by: '<S304>/Constant' */

/* Minimum ego lane length for activation of lane change */
extern const volatile boolean_T
    TJASLC_EnableFrontObjCancle_bool; /* Referenced by: '<S279>/Constant1' */

/* Debounce time to surpress "blinking" of the signal */
extern const volatile boolean_T
    TJASLC_EnableSLCHMISwitch_nu; /* Referenced by: '<S267>/Constant' */

/* Minimum long deceleration for TJA activation */
extern const volatile real32_T
    TJASLC_GoBackTurnLtDlyTiC_sec; /* Referenced by:
                                    * '<S262>/Constant'
                                    * '<S262>/Constant1'
                                    */

/* Indicates TRUE if TJA is a coded function of the system */
extern const volatile real32_T TJASLC_HeadingAllowAbort_By_rad[12];
/* Referenced by: '<S299>/1-D Lookup Table5' */

/* 1DLookupTable-Bx-Vehicle velocity X */
extern const volatile real32_T
    TJASLC_LCPInitDurationMax_C_sec; /* Referenced by: '<S527>/Constant' */

/* Maximum duration of LCP initialization after which trigger will be reset, if
 * condition didn't allow initialization of lateral movement */
extern const volatile real32_T
    TJASLC_LCPLeft2Active_sec; /* Referenced by: '<S317>/Constant' */

/* LCPLeft2Active time
   origin:2s */
extern const volatile real32_T
    TJASLC_LCPLeft2Passive_sec; /* Referenced by: '<S317>/Constant1' */

/* LCPLeft2Passive time */
extern const volatile real32_T
    TJASLC_LCPRight2Active_sec; /* Referenced by: '<S317>/Constant3' */

/* LCPRight2Active time
   origin:2s */
extern const volatile real32_T
    TJASLC_LCPRight2Passive_sec; /* Referenced by: '<S317>/Constant2' */

/* LCPRight2Passive time */
extern const volatile real32_T
    TJASLC_LCWPassiveDlyTm_C_sec; /* Referenced by: '<S375>/Constant12' */

/* Maximum long acceleration for TJA activation (hysteresis) */
extern const volatile real32_T
    TJASLC_LaneChangInfoTime_sec; /* Referenced by:
                                   * '<S379>/Constant'
                                   * '<S380>/Constant'
                                   */

/* Maximum duration of takeover time for lane centering mode after lane change
 * maneuver */
extern const volatile real32_T
    TJASLC_LaneChangeWarnTimeMax_C_sec; /* Referenced by:
                                         * '<S375>/Constant1'
                                         * '<S375>/Constant2'
                                         * '<S375>/Constant8'
                                         */

/* Maximum duration of warn time for lane change abort situation */
extern const volatile uint16_T
    TJASLC_LaneInvalid_C_btm; /* Referenced by: '<S307>/Constant1' */

/* Lane validity check bitmask */
extern const volatile real32_T
    TJASLC_LaneTypeDebTime_C_sec; /* Referenced by: '<S306>/Constant2' */

/* Lane type check debounce time */
extern const volatile boolean_T
    TJASLC_LcaBsdSigEnabled_C_bool; /* Referenced by:
                                     * '<S266>/Constant'
                                     * '<S266>/Constant2'
                                     */

/* Switch to enable check of LCA/BSD signals for rear object assessment
   (if FALSE MS flag signals will be read) */
extern const volatile real32_T
    TJASLC_LnChngFlagTurnOffTm_C_sec; /* Referenced by: '<S364>/Constant' */

/* Turn off delay time of ABPR lane change flag */
extern const volatile real32_T
    TJASLC_ManeuverTimeMax_C_sec; /* Referenced by: '<S285>/Constant1' */

/* Maximum lane change maneuver time */
extern const volatile boolean_T
    TJASLC_ManualTrigger_C_bool; /* Referenced by: '<S513>/Constant1' */

/* Switch to allow manual triggering of lane change with turn signals only */
extern const volatile real32_T
    TJASLC_MinDurFreeAdjLane_C_sec; /* Referenced by: '<S266>/Constant1' */

/* Minimum duration of an empty adjacent lane to allow initilization of lane
 * change maneuver */
extern const volatile real32_T
    TJASLC_MinDurLCTrigActv_C_sec; /* Referenced by: '<S485>/Constant1' */

/* Minimum duration of controlling in lane centering mode required for SALC
 * trigger activation */
extern const volatile real32_T TJASLC_ObjSafeTime_C_sec; /* Referenced by:
                                                          * '<S266>/Constant7'
                                                          * '<S266>/Constant8'
                                                          * '<S266>/Constant9'
                                                          */

/* Debounce time to surpress "blinking" of the signal */
extern const volatile real32_T TJASLC_ObjUnSafeTime_C_sec; /* Referenced by:
                                                            * '<S266>/Constant4'
                                                            * '<S266>/Constant5'
                                                            * '<S266>/Constant6'
                                                            */

/* Debounce time to surpress "blinking" of the signal */
extern const volatile real32_T
    TJASLC_PosYThdExitAbort_met; /* Referenced by: '<S327>/Constant3' */

/* Minimum ego lane length for activation of lane change */
extern const volatile uint16_T
    TJASLC_PrjSpecQuC_C_btm; /* Referenced by: '<S273>/Constant1' */

/* Bitmask to check project specific cancel conditions for lane change mode */
extern const volatile uint16_T
    TJASLC_PrjSpecQuSR_C_btm; /* Referenced by: '<S486>/Constant1' */

/* Bitmask to check project specific SR conditions for lane change mode */
extern const volatile boolean_T
    TJASLC_SALC_Enabled_C_bool; /* Referenced by:
                                 * '<S118>/Constant'
                                 * '<S194>/Constant1'
                                 * '<S463>/Constant1'
                                 */

/* Switch to enable semi-automatic lane change functionality */
extern const volatile real32_T
    TJASLC_TakeoverTiMaxAbrt_C_sec; /* Referenced by: '<S378>/Constant2' */

/* Turn off delay time of ABPR lane change flag */
extern const volatile real32_T
    TJASLC_TakeoverTimeMax_C_sec; /* Referenced by: '<S378>/Constant' */

/* Maximum duration of takeover time for lane centering mode after lane change
 * maneuver */
extern const volatile real32_T
    TJASLC_TimeMaxInAbortNewEgo_sec; /* Referenced by: '<S327>/Constant2' */

/* Minimum ego lane length for activation of lane change */
extern const volatile real32_T
    TJASLC_TimeMaxInAbort_sec; /* Referenced by: '<S327>/Constant1' */

/* Minimum ego lane length for activation of lane change */
extern const volatile real32_T
    TJASLC_TrigResetBlockTime_C_sec; /* Referenced by: '<S515>/Constant' */

/* Trigger blocking time after reset */
extern const volatile real32_T
    TJASLC_TriggerTurnOnTime_C_sec; /* Referenced by: '<S516>/Constant' */

/* Trigger turn on delay time */
extern const volatile real32_T
    TJASLC_TurnSignalDebTime_C_sec; /* Referenced by: '<S513>/Constant' */

/* Debounce time to surpress "blinking" of the signal */
extern const volatile real32_T
    TJASLC_TurnSignalOffTime_C_sec; /* Referenced by:
                                     * '<S536>/Constant'
                                     * '<S536>/Constant1'
                                     */

/* The time to cancle or abort ALCA, when driver turn off turn lights */
extern const volatile uint8_T
    TJASLC_ValidLaneTypeDD_C_nu; /* Referenced by:
                                  * '<S306>/Constant12'
                                  * '<S306>/Constant6'
                                  */

/* Valid type selector (default: 2 (PAINTED_DASHED)) */
extern const volatile uint8_T
    TJASLC_ValidLaneTypeDS_C_nu; /* Referenced by: '<S306>/Constant11' */

/* Valid type selector (default: 2 (SOLID_DASHED_LINE)) */
extern const volatile uint8_T
    TJASLC_ValidLaneTypeSD_C_nu; /* Referenced by: '<S306>/Constant8' */

/* Valid type selector (default: 2 (PAINTED_DASHED)) */
extern const volatile uint8_T
    TJASLC_ValidLaneTypeTD_C_nu; /* Referenced by:
                                  * '<S306>/Constant10'
                                  * '<S306>/Constant7'
                                  */

/* Valid type selector (default: 2 (PAINTED_DASHED)) */
extern const volatile uint8_T TJASLC_ValidLaneType_C_nu; /* Referenced by:
                                                          * '<S306>/Constant4'
                                                          * '<S306>/Constant9'
                                                          */

/* Valid type selector (default: 2 (PAINTED_DASHED)) */
extern const volatile real32_T TJASLC_VehVelXAllowAbort_Bx_mps[12];
/* Referenced by: '<S299>/1-D Lookup Table1' */

/* 1DLookupTable-Bx-Vehicle velocity X */
extern const volatile real32_T
    TJASLC_VelXMax_C_kph; /* Referenced by: '<S488>/Constant1' */

/* Maximum velocity threshold for SALC activation */
extern const volatile real32_T
    TJASLC_VelXMin_C_kph; /* Referenced by: '<S488>/Constant3' */

/* Minimum velocity threshold for SALC activation */
extern const volatile real32_T
    TJASTM_ACCManeveurTime_C_sec; /* Referenced by: '<S546>/Constant7' */

/* Maximum long acceleration for TJA activation */
extern const volatile real32_T
    TJASTM_ACCOvertime_C_sec; /* Referenced by: '<S546>/Constant' */

/* ACC Active over time check: sec */
extern const volatile boolean_T
    TJASTM_EnableFS_C_bool; /* Referenced by: '<S8>/Constant' */

/* TJA manual function switch (if HMI not working):
   1: TJA switch on, 0: TJA switched off */
extern const volatile boolean_T
    TJASTM_EnableUseACCState_bool; /* Referenced by: '<S555>/Constant2' */

/* TJA manual function switch (if HMI not working):
   1: TJA switch on, 0: TJA switched off */
extern const volatile real32_T
    TJASTM_HansOffReleaseWarn_C_sec; /* Referenced by: '<S546>/Constant1' */

/* Turn on delay time for minimum lane quality check */
extern const volatile real32_T
    TJASTM_SusTakeOverTurnOffTime_C_sec; /* Referenced by: '<S9>/Constant7' */

/* Turn off delay time for suspended take over */
extern const volatile real32_T
    TJASTM_SuspendQuitTimeMax_C_sec; /* Referenced by: '<S600>/Constant' */

/*  Maximum rampout time */
extern const volatile real32_T
    TJATOW_NPilotAudioTime_C_sec; /* Referenced by:
                                   * '<S613>/Constant13'
                                   * '<S613>/Constant14'
                                   * '<S613>/Constant18'
                                   * '<S613>/Constant2'
                                   * '<S613>/Constant20'
                                   * '<S613>/Constant29'
                                   * '<S613>/Constant31'
                                   * '<S613>/Constant33'
                                   * '<S613>/Constant35'
                                   * '<S613>/Constant37'
                                   * '<S613>/Constant39'
                                   * '<S613>/Constant5'
                                   * '<S613>/Constant6'
                                   * '<S613>/Constant7'
                                   * '<S613>/Constant9'
                                   */

/* Turn on delay time for minimum lane quality check */
extern const volatile uint8_T
    TJATOW_NPilotSysInfoDefault_nu; /* Referenced by: '<S613>/Constant17' */

/* Minimum duration of an empty adjacent lane to allow initilization of object
 * follow mode */
extern const volatile uint16_T
    TJATOW_OFSpdInvalid_C_btm; /* Referenced by: '<S648>/Constant5' */

/* Bitmask to check project specific strong ready conditions for all TJA modes
 */
extern const volatile boolean_T
    TJATTG_EnableObjDuringCMB_C_bool; /* Referenced by: '<S765>/Constant' */

/* Switch to enable use of object data during combined mode */
extern const volatile boolean_T
    TJATTG_EnableVirtAdjLane_C_bool; /* Referenced by:
                                      * '<S832>/Constant'
                                      * '<S834>/Constant'
                                      */

/* Enable virtual adjacent lane width for SALC */
extern const volatile real32_T
    TJATTG_LnPredMinTrajLength_C_met; /* Referenced by:
                                       * '<S687>/Constant'
                                       * '<S704>/Constant'
                                       * '<S721>/Constant'
                                       */

/* Minimum trajectory length for lane prediction */
extern const volatile boolean_T
    TJATTG_NewPredEnable_C_bool; /* Referenced by: '<S686>/Constant' */

/* Switch to enable new prediction implementation */
extern const volatile real32_T
    TJATTG_ObjFolVirtLnWdth_C_met; /* Referenced by:
                                    * '<S671>/Constant1'
                                    * '<S672>/Constant1'
                                    * '<S744>/Constant1'
                                    * '<S809>/Constant1'
                                    */

/* TJATTG_APARAM */
extern const volatile real32_T
    TJATTG_PredCrvChngPT1_C_sec; /* Referenced by:
                                  * '<S688>/Constant1'
                                  * '<S705>/Constant1'
                                  * '<S722>/Constant1'
                                  */

/* Curvature change PT1 time constant during prediction (if signal is set to
 * zero) */
extern const volatile real32_T TJATTG_PredCrvPT1_C_sec; /* Referenced by:
                                                         * '<S689>/Constant1'
                                                         * '<S706>/Constant1'
                                                         * '<S723>/Constant1'
                                                         */

/* Curvature PT1 time constant during prediction (if signal is set to zero) */
extern const volatile boolean_T
    TJATTG_PredFreezeCrvChng_C_bool; /* Referenced by:
                                      * '<S688>/Constant'
                                      * '<S705>/Constant'
                                      * '<S722>/Constant'
                                      */

/* TRUE if curvature change signal shall be frozen during prediction */
extern const volatile boolean_T
    TJATTG_PredFreezeCrv_C_bool; /* Referenced by:
                                  * '<S689>/Constant'
                                  * '<S706>/Constant'
                                  * '<S723>/Constant'
                                  */

/* TRUE if curvature signal shall be frozen during prediction */
extern const volatile real32_T
    TJATTG_PredResetTrajLength_C_met; /* Referenced by:
                                       * '<S695>/Constant'
                                       * '<S712>/Constant'
                                       * '<S729>/Constant'
                                       */

/* Remaining saved trajectory length for prediction reset */
extern const volatile real32_T
    TJATTG_TransDurationCMB_C_sec; /* Referenced by: '<S804>/Constant4' */

/* Transition duration for mode switch to combined data */
extern const volatile real32_T
    TJATTG_TransDurationLD_C_sec; /* Referenced by: '<S804>/Constant2' */

/* Transition duration for mode switch to lane data */
extern const volatile real32_T
    TJATTG_TransDurationOD_C_sec; /* Referenced by: '<S804>/Constant3' */

/*  Transition duration for mode switch to object data */
extern const volatile real32_T
    TJATTG_TransDurationPredct_C_sec; /* Referenced by:
                                       * '<S963>/Constant2'
                                       * '<S804>/Constant'
                                       */

/* Transition duration to switch from predicted lane data to new detected lane
 * data */
extern const volatile boolean_T
    TJATTG_TransHandleEnable_C_bool; /* Referenced by: '<S798>/Constant' */

/* Switch to enable mode transition handling */
extern const volatile real32_T
    TJATTG_VirtAdjLaneWidth_C_met; /* Referenced by:
                                    * '<S832>/Constant1'
                                    * '<S834>/Constant1'
                                    */

/* Virtual adjacent lane width */
extern const volatile real32_T
    TJATVG_CrvPlanHorizon_Bx_1pm[7]; /* Referenced by:
                                      * '<S947>/1-D Lookup Table1'
                                      * '<S949>/1-D Lookup Table1'
                                      */

/* 1DLookupTable-Bx-X axis for the Planning Horizon */
extern const volatile real32_T
    TJATVG_DistYToleranceLeftTgtArea_C_met; /* Referenced by:
                                               '<S891>/Constant15' */

/* TJATVG_PARAMETER- Distance Y to tolerance left target area */
extern const volatile real32_T
    TJATVG_DistYToleranceRightTgtArea_C_met; /* Referenced by:
                                                '<S891>/Constant16' */

/* TJATVG_PARAMETER- Distance Y to tolerance right target area */
extern const volatile real32_T
    TJATVG_FTireAccelMax_C_mps2; /* Referenced by: '<S891>/Constant13' */

/* TJATVG_PARAMETER- FTire acceleration maximum */
extern const volatile real32_T
    TJATVG_FTireAccelMin_C_mps2; /* Referenced by: '<S891>/Constant14' */

/* TJATVG_PARAMETER- FTire acceleration minimum */
extern const volatile real32_T
    TJATVG_FactorCrvGrdBuildUp_C_fac; /* Referenced by: '<S957>/Constant8' */

/* TJATVG_PARAMETER-Factor of Curve Gradient Buildup */
extern const volatile real32_T
    TJATVG_FactorCrvGrdRed_C_fac; /* Referenced by: '<S957>/Constant9' */

/* TJATVG_PARAMETER-Factor of Curve Gradient Red */
extern const volatile real32_T
    TJATVG_GrdLimitTgtCrvTGC_C_1pms; /* Referenced by: '<S957>/Constant11' */

/* TJATVG_PARAMETER-Gradient Limit Target Curve TGC */
extern const volatile boolean_T
    TJATVG_HighStatAccu_C_bool; /* Referenced by: '<S893>/Constant1' */

/* Indicates TRUE, if high stationary accuracy is required */
extern const volatile real32_T TJATVG_LWDeratLvlScalFact_Cr_nu[6];
/* Referenced by: '<S891>/1-D Lookup Table' */

/* 1DLookupTable-Cr-Lookup table for lane width derating level scaling factort
 */
extern const volatile real32_T TJATVG_LaneWidth_Bx_met[6];
/* Referenced by: '<S891>/1-D Lookup Table' */

/* 1DLookupTable-Bx-Lane width

   The raw value is [0 0 0 0 0 0], but this BreakpointsForDimension1 of
   1DLookUp table must be strictly monotonically increasing,  so set it to [0 1
   2 3 4 5] Temporarily. */
extern const volatile real32_T
    TJATVG_LimiterMaxCrvGrd_C_1pms; /* Referenced by:
                                     * '<S956>/Constant2'
                                     * '<S956>/Constant3'
                                     * '<S956>/Constant4'
                                     * '<S957>/Constant2'
                                     * '<S957>/Constant3'
                                     * '<S957>/Constant4'
                                     * '<S958>/Constant2'
                                     * '<S958>/Constant3'
                                     * '<S958>/Constant4'
                                     */

/* TJATVG_PARAMETER-Max Curvature Gradient */
extern const volatile real32_T TJATVG_LimiterMaxCrv_C_1pm; /* Referenced by:
                                                            * '<S956>/Constant1'
                                                            * '<S957>/Constant1'
                                                            * '<S958>/Constant1'
                                                            */

/* TJATVG_PARAMETER-Max Curvature */
extern const volatile uint8_T
    TJATVG_MD1DeratingLevel_C_perc; /* Referenced by: '<S891>/Constant22' */

/* MD1DeratingLevel */
extern const volatile uint8_T
    TJATVG_MD2DeratingLevel_C_perc; /* Referenced by: '<S891>/Constant21' */

/* MD2DeratingLevel */
extern const volatile uint8_T
    TJATVG_MD3DeratingLevel_C_perc; /* Referenced by: '<S891>/Constant23' */

/* MD3DeratingLevel */
extern const volatile real32_T
    TJATVG_MaxJerkAllowed_C_mps3; /* Referenced by: '<S891>/Constant12' */

/* Maximum allowed jerk */
extern const volatile real32_T
    TJATVG_MaxSteeringAngle_C_deg; /* Referenced by: '<S957>/Constant6' */

/* TJATVG_PARAMETER-Max Steering Angle */
extern const volatile real32_T
    TJATVG_MaxTrqScalLimit_C_nu; /* Referenced by: '<S962>/Constant1' */

/* TJATVG_APARAM-Max torque scal limit */
extern const volatile real32_T
    TJATVG_MaxTrqScalRampInGrd_C_1ps; /* Referenced by: '<S961>/Constant9' */

/* Maximum torque scaling gradient during ramp-in */
extern const volatile real32_T
    TJATVG_MaxTrqScalRampOutGrd_C_1ps; /* Referenced by: '<S961>/Constant10' */

/* Maximum torque scaling gradient during ramp-out */
extern const volatile real32_T
    TJATVG_MinFactorCrvGrd_C_fac; /* Referenced by: '<S957>/Constant10' */

/* TJATVG_PARAMETER-Min Factor of Curve Gradient  */
extern const volatile boolean_T
    TJATVG_ModeTransTrigReplan_bool; /* Referenced by: '<S888>/Constant' */

/* Switch to enable trigger replan during mode transitions */
extern const volatile real32_T TJATVG_PlanHorizonLChange_Vel_sec[15];
/* Referenced by: '<S948>/1-D Lookup Table' */

/* 1DLookupTable-Cr-Maximum allowed target object clothoid curvature for OF mode
 * activation (WR condition) */
extern const volatile real32_T TJATVG_PlanHorizonObjFolVal_Cr_sec[15];
/* Referenced by: '<S949>/1-D Lookup Table' */

/* 1DLookupTable-Cr-Lookup table for vehicle speed dependend planning horizon
 * for OF */
extern const volatile real32_T
    TJATVG_PlanHorizonScal_Cr_Fac[7]; /* Referenced by:
                                       * '<S947>/1-D Lookup Table1'
                                       * '<S949>/1-D Lookup Table1'
                                       */

/* 1DLookupTable-Cr-Y axis of the Planning Horizon scaling factor */
extern const volatile real32_T TJATVG_PlanningHorizonValLC_Cr_sec[15];
/* Referenced by: '<S947>/1-D Lookup Table' */

/* 1DLookupTable-Cr-Lookup table for vehicle speed dependend planning horizon */
extern const volatile real32_T
    TJATVG_PosYPlanHorizonScal_Cr_Fac[7]; /* Referenced by:
                                           * '<S947>/1-D Lookup Table3'
                                           * '<S949>/1-D Lookup Table3'
                                           */

/* 1DLookupTable-Bx-Vehicle velocity X */
extern const volatile real32_T
    TJATVG_PosYPlanHorizon_Bx_met[7]; /* Referenced by:
                                       * '<S947>/1-D Lookup Table3'
                                       * '<S949>/1-D Lookup Table3'
                                       */

/* 1DLookupTable-Bx-Vehicle velocity X */
extern const volatile real32_T
    TJATVG_PredTimeHeadAng_C_sec; /* Referenced by: '<S891>/Constant10' */

/* TJATVG_PARAMETER-Prediction time of heading angle */
extern const volatile real32_T
    TJATVG_PredictionTimeCrv_C_sec; /* Referenced by: '<S891>/Constant4' */

/* TJATVG_PARAMETER-Prediction time of curve */
extern const volatile real32_T TJATVG_RedFact_Vel_Cr_fac[6];
/* Referenced by: '<S893>/1-D Lookup Table' */

/* 1DLookupTable-Cr-Y_TJATVG_RedFact_Vel_nu */
extern const volatile boolean_T
    TJATVG_SetMaxCrvAndGrdLims_C_bool; /* Referenced by: '<S957>/Constant' */

/* TJATVG_PARAMETER-Set max curve and grandient limit */
extern const volatile real32_T
    TJATVG_StrWhStifAbortGrd_C_1ps; /* Referenced by: '<S961>/Constant6' */

/* Steering Wheel Stiffness Abort Ramp Out Gradient */
extern const volatile real32_T
    TJATVG_StrWhStifLimitPredct_C_fac; /* Referenced by: '<S963>/Constant5' */

/* TJA specific steering wheel stiffness limiter during rampout prediction */
extern const volatile real32_T
    TJATVG_StrWhStifLimit_C_nu; /* Referenced by: '<S962>/Constant6' */

/* TJA specific steering wheel stiffness limiter */
extern const volatile real32_T
    TJATVG_StrWhStifRampInGrd_C_1ps; /* Referenced by:
                                      * '<S961>/Constant4'
                                      * '<S963>/Constant6'
                                      */

/* Steering wheel stiffness gradient during ramp-in */
extern const volatile real32_T
    TJATVG_StrWhStifRampOutGrd_C_1ps; /* Referenced by:
                                       * '<S961>/Constant1'
                                       * '<S963>/Constant7'
                                       */

/* Steering wheel stiffness gradient during ramp-out */
extern const volatile uint8_T
    TJATVG_TrajPlanValServQu_C_nu; /* Referenced by: '<S891>/Constant20' */

/* Trajectory planning service qualifier */
extern const volatile uint8_T
    TJATVG_TrajPlanValSrvQuSALC_C_nu; /* Referenced by: '<S891>/Constant19' */

/* Trajectory planning service qualifier for semi-automatic lane change
   (enables lane cross check) */
extern const volatile boolean_T
    TJATVG_TriggerReplan_C_bool; /* Referenced by: '<S891>/Constant11' */

/* Switch to trigger replanning */
extern const volatile real32_T
    TJATVG_TrqAbortGrad_C_1ps; /* Referenced by: '<S961>/Constant3' */

/* Torque Ramp Abort Ramp Out Gradient */
extern const volatile real32_T
    TJATVG_TrqRampInGrad_C_1ps; /* Referenced by: '<S961>/Constant11' */

/* Torque ramp gradient during ramp-in */
extern const volatile real32_T
    TJATVG_TrqRampOutGrad_C_1ps; /* Referenced by: '<S961>/Constant2' */

/* Torque ramp gradient during ramp-out */
extern const volatile boolean_T
    TJATVG_UseLtcyCompCMB_C_bool; /* Referenced by: '<S924>/Constant2' */

/* Switch to enable use of latency compensation during CMB mode */
extern const volatile boolean_T
    TJATVG_UseLtcyCompLC_C_bool; /* Referenced by: '<S924>/Constant' */

/* Switch to enable use of latency compensation during LC mode */
extern const volatile boolean_T
    TJATVG_UseLtcyCompOF_C_bool; /* Referenced by: '<S924>/Constant1' */

/* Switch to enable use of latency compensation during OF mode */
extern const volatile boolean_T
    TJATVG_UseLtcyCompSALC_C_bool; /* Referenced by: '<S924>/Constant3' */

/* Switch to enable use of latency compensation during SALC mode */
extern const volatile real32_T
    TJATVG_VehVelXLC_Bx_mps[15]; /* Referenced by:
                                  * '<S947>/1-D Lookup Table'
                                  * '<S948>/1-D Lookup Table'
                                  */

/* 1DLookupTable-Bx-TJATVG VehVelX  */
extern const volatile real32_T TJATVG_VehVelXOF_Bx_mps[15];
/* Referenced by: '<S949>/1-D Lookup Table' */

/* 1DLookupTable-Bx-TJATVG VehVelX  */
extern const volatile real32_T TJATVG_VehVelX_RedFact_Bx_mps[6];
/* Referenced by: '<S893>/1-D Lookup Table' */

/* 1DLookupTable-Bx-TJATVG_APARAM for Y_TJATVG_RedFact_Vel_nu

   The raw value is [0 0 0 0 0 0], but this BreakpointsForDimension1 of
   1DLookUp table must be strictly monotonically increasing,  so set it to [0 1
   2 3 4 5] Temporarily. */
extern const volatile real32_T
    TJATVG_WeightEndTime_C_nu; /* Referenced by: '<S891>/Constant18' */

/* TJATVG_PARAMETER- Weight of end time */
extern const volatile real32_T
    TJATVG_WeightTgtDistY_C_nu; /* Referenced by: '<S891>/Constant17' */

/* TJATVG_PARAMETER- Weight of target distance Y  */

/* Declaration for custom storage class: Global */
extern boolean_T CMB_EnableFusion2EdgeRising_bool; /* '<S30>/Unit Delay' */

/* Enable fusion 2 edge rising--Used in TJACMB module
   DT:boolean */
extern boolean_T CMB_EnableFusionEdgeFalling_bool; /* '<S32>/Unit Delay' */

/* Enable fusion edge falling--Used in TJACMB module
   DT:boolean */
extern boolean_T CMB_EnableFusionEdgeRising_bool; /* '<S36>/Unit Delay' */

/* Enable fusion edge rising--Used in TJACMB module
   DT:boolean */
extern real32_T CMB_EnableFusionStopwatch_sec; /* '<S34>/Unit Delay' */

/* Enable fusion stopwatch time--Used in TJACMB module
   DT:float32 */
extern real32_T CMB_EnableFusionTurnOffDelay_sec; /* '<S35>/Unit Delay' */

/* Enable fusion turn off delay time--Used in TJACMB module
   DT:float32 */
extern boolean_T CMB_LaneQualityInvalid_bool; /* '<S16>/Switch' */

/* Debug */
extern boolean_T CMB_LnQualRSFF_bool[2]; /* '<S43>/Unit Delay' */

/* Lane quality RSFlipFlop flag--Used in TJACMB module
   DT:boolean */
extern real32_T CMB_LnQualTurnOffDelay_sec[2]; /* '<S44>/Unit Delay' */

/* Lane quality turn off delay time--Used in TJACMB module
   DT:float32 */
extern real32_T CMB_LnQualTurnOnDelay_sec[2]; /* '<S45>/Unit Delay' */

/* Lane quality turn on delay time--Used in TJACMB module
   DT:float32 */
extern boolean_T CMB_ObjectFollowingOnly_bool; /* '<S17>/AND' */

/* Debug */
extern real32_T CMB_PrevCntrCrv1UnitDelay_1pm; /* '<S12>/Unit Delay' */

/* Previous control lane clothoid curve(Unit delay 1)--Used in TJACMB module
   DT:float32 */
extern real32_T CMB_PrevCntrCrv2UnitDelay_1pm; /* '<S12>/Unit Delay1' */

/* Previous control lane clothoid curve(Unit delay 2)--Used in TJACMB module
   DT:float32 */
extern real32_T CMB_PrevCntrCrv3UnitDelay_1pm; /* '<S12>/Unit Delay2' */

/* Previous control lane clothoid curve(Unit delay 3)--Used in TJACMB module
   DT:float32 */
extern real32_T CMB_PrevCntrCrv4UnitDelay_1pm; /* '<S12>/Unit Delay3' */

/* Previous control lane clothoid curve(Unit delay 4)--Used in TJACMB module
   DT:float32 */
extern real32_T CMB_PrevCntrCrv5UnitDelay_1pm; /* '<S12>/Unit Delay4' */

/* Previous control lane clothoid curve(Unit delay 5)--Used in TJACMB module
   DT:float32 */
extern real32_T CMB_PrevCntrCrv6UnitDelay_1pm; /* '<S12>/Unit Delay5' */

/* Previous control lane clothoid curve(Unit delay 6)--Used in TJACMB module
   DT:float32 */
extern real32_T CMB_PrevCntrCrv7UnitDelay_1pm; /* '<S12>/Unit Delay6' */

/* Previous control lane clothoid curve(Unit delay 7)--Used in TJACMB module
   DT:float32 */
extern real32_T CMB_PrevCntrCrv8UnitDelay_1pm; /* '<S12>/Unit Delay7' */

/* Previous control lane clothoid curve(Unit delay 8)--Used in TJACMB module
   DT:float32 */
extern real32_T CMB_PrevCntrCrv9UnitDelay_1pm; /* '<S12>/Unit Delay8' */

/* Previous control lane clothoid curve(Unit delay 9)--Used in TJACMB module
   DT:float32 */
extern real32_T CMB_PrevCombCrvLFUnitDelay_1pm; /* '<S25>/Unit Delay' */
extern real32_T CMB_PrevTgtCrv1UnitDelay_1pm;   /* '<S13>/Unit Delay' */

/* Previous target clothoid curve(Unit delay 1)--Used in TJACMB module
   DT:float32 */
extern real32_T CMB_PrevTgtCrv2UnitDelay_1pm; /* '<S13>/Unit Delay1' */

/* Previous target clothoid curve(Unit delay 2)--Used in TJACMB module
   DT:float32 */
extern real32_T CMB_PrevTgtCrv3UnitDelay_1pm; /* '<S13>/Unit Delay2' */

/* Previous target clothoid curve(Unit delay 3)--Used in TJACMB module
   DT:float32 */
extern real32_T CMB_PrevTgtCrv4UnitDelay_1pm; /* '<S13>/Unit Delay3' */

/* Previous target clothoid curve(Unit delay 4)--Used in TJACMB module
   DT:float32 */
extern real32_T CMB_PrevTgtCrv5UnitDelay_1pm; /* '<S13>/Unit Delay4' */

/* Previous target clothoid curve(Unit delay 5)--Used in TJACMB module
   DT:float32 */
extern real32_T CMB_PrevTgtCrv6UnitDelay_1pm; /* '<S13>/Unit Delay5' */

/* Previous target clothoid curve(Unit delay 6)--Used in TJACMB module
   DT:float32 */
extern real32_T CMB_PrevTgtCrv7UnitDelay_1pm; /* '<S13>/Unit Delay6' */

/* Previous target clothoid curve(Unit delay 7)--Used in TJACMB module
   DT:float32 */
extern real32_T CMB_PrevTgtCrv8UnitDelay_1pm; /* '<S13>/Unit Delay7' */

/* Previous target clothoid curve(Unit delay 8)--Used in TJACMB module
   DT:float32 */
extern real32_T CMB_PrevTgtCrv9UnitDelay_1pm; /* '<S13>/Unit Delay8' */

/* Previous target clothoid curve(Unit delay 9)--Used in TJACMB module
   DT:float32 */
extern boolean_T CMB_VelXMaxHyst_bool; /* '<S41>/Unit Delay' */

/* Velocity X Max hystereis flag--Used in TJACMB module
   DT:boolean */
extern boolean_T CMB_VelXMinHyst_bool; /* '<S42>/Unit Delay' */

/* Velocity X Min hystereis flag--Used in TJACMB module
   DT:boolean */
extern real32_T CMB_WeightCrv2LowPass_1pm; /* '<S31>/Unit Delay' */

/* Weighted curve 2 low pass filter--Used in TJACMB module
   DT:float32 */
extern real32_T CMB_WeightCrvLowPass_1pm; /* '<S37>/Unit Delay' */

/* Weighted curve low pass filter--Used in TJACMB module
   DT:float32 */
extern boolean_T GEN_AclXMaxHyst_bool; /* '<S94>/Unit Delay' */

/* Acceleration X Max hystereis flag--Used in TJAGEN module
   DT:boolean */
extern boolean_T GEN_AclXMinHyst_bool; /* '<S95>/Unit Delay' */

/* Acceleration X Min hystereis flag--Used in TJAGEN module
   DT:boolean */
extern boolean_T GEN_AclYMaxHyst_bool; /* '<S93>/Unit Delay' */

/* Acceleration Y Max hystereis flag--Used in TJAGEN module
   DT:boolean */
extern real32_T GEN_BlockTimeExpiredTimerRetrigger_sec; /* '<S99>/Unit Delay' */

/* Block Timer Expired timer retrigger--Used in TJAGEN module
   DT:float32 */
extern real32_T GEN_HODTurnOnDelay_sec; /* '<S73>/Unit Delay' */

/* HOD turn on delay time--Used in TJAGEN module
   DT:float32 */
extern real32_T GEN_HazardTurnOnDelay_sec; /* '<S79>/Unit Delay' */

/* turn signal hazard turn on delay time--Used in TJAGEN module
   DT:float32 */
extern real32_T GEN_ManualTorMaxTurnOnDelay_sec; /* '<S76>/Unit Delay' */

/* Manual torque Max turn on delay time--Used in TJAGEN module
   DT:float32 */
extern real32_T GEN_ManualTorMinTurnOnDelay_sec; /* '<S77>/Unit Delay' */

/* Manual torque Min turn on delay time--Used in TJAGEN module
   DT:float32 */
extern boolean_T GEN_ManualTorqueMaxHyst_bool; /* '<S74>/Unit Delay' */

/* Manual torque max hystereis flag--Used in TJAGEN module
   DT:boolean */
extern boolean_T GEN_ManualTorqueMinHyst_bool; /* '<S75>/Unit Delay' */

/* Manual torque max hystereis flag--Used in TJAGEN module
   DT:boolean */
extern E_TJASTM_LatCtrlMode_nu GEN_PrevLatCtrlMode_Enum; /* '<S8>/Unit Delay' */

/* Previous lateal control mode--Used in TJAGEN module
   Enum: E_TJASTM_LatCtrlMode_nu */
extern boolean_T GEN_PrevRampoutNUnitDelay_bool; /* '<S552>/Unit Delay' */

/* Previous rampout flag--Used in TJAGEN module
   DT:boolean */
extern boolean_T GEN_PrevRampoutUnitDelay_bool; /* '<S59>/Unit Delay' */

/* Previous rampout flag--Used in TJAGEN module
   DT:boolean */
extern boolean_T GEN_PrevSus2UnitDelay_bool; /* '<S600>/Unit Delay' */

/* Previous suspended flag--Used in TJAGEN module
   DT:boolean */
extern boolean_T GEN_PrevSusQuitUnitDelay_bool; /* '<S552>/Unit Delay1' */

/* Previous suspended quit flag--Used in TJAGEN module
   DT:boolean */
extern boolean_T GEN_PrevSusUnitDelay_bool; /* '<S599>/Unit Delay' */

/* Previous suspended flag--Used in TJAGEN module
   DT:boolean */
extern boolean_T GEN_RampoutTimeExpiredRSFF_bool; /* '<S60>/Unit Delay' */

/* Rampout Time Expired RSFlipFlop flag--Used in TJACMB module
   DT:boolean */
extern real32_T
    GEN_RampoutTimeExpiredTimerRetrigger_sec; /* '<S61>/Unit Delay' */

/* Rampout Timer Expired timer retrigger--Used in TJAGEN module
   DT:float32 */
extern real32_T GEN_SafeFuncActiveTurnOnDelay_sec; /* '<S85>/Unit Delay' */

/* Safety function active turn on delay time--Used in TJAGEN module
   DT:float32 */
extern real32_T GEN_SafeFuncErrorTurnOnDelay_sec; /* '<S86>/Unit Delay' */

/* Safety function error turn on delay time--Used in TJAGEN module
   DT:float32 */
extern boolean_T GEN_SteerWAngleGradHyst_bool; /* '<S81>/Unit Delay' */

/* Steering angle grad hystereis flag--Used in TJAGEN module
   DT:boolean */
extern boolean_T GEN_SteerWAngleGradSusHyst_bool; /* '<S80>/Unit Delay' */

/* Steering angle grad for suspended hystereis flag--Used in TJAGEN module
   DT:boolean */
extern boolean_T GEN_SteerWAngleHyst_bool; /* '<S82>/Unit Delay' */

/* Steering angle hystereis flag--Used in TJAGEN module
   DT:boolean */
extern boolean_T GEN_SteerWAngleSusHyst_bool; /* '<S83>/Unit Delay' */

/* Steering angle for suspended hystereis flag--Used in TJAGEN module
   DT:boolean */
extern boolean_T GEN_Sus2TimeExpiredRSFF_bool; /* '<S603>/Unit Delay' */

/* Susopended Time Expired RSFlipFlop flag--Used in TJACMB module
   DT:boolean */
extern real32_T GEN_Sus2TimeExpiredTimerRetrigger_sec; /* '<S604>/Unit Delay' */

/* Suspended Timer Expired timer retrigger--Used in TJAGEN module
   DT:float32 */
extern boolean_T GEN_SusTimeExpiredRSFF_bool; /* '<S601>/Unit Delay' */

/* Susopended Time Expired RSFlipFlop flag--Used in TJACMB module
   DT:boolean */
extern real32_T GEN_SusTimeExpiredTimerRetrigger_sec; /* '<S602>/Unit Delay' */

/* Suspended Timer Expired timer retrigger--Used in TJAGEN module
   DT:float32 */
extern boolean_T GEN_VehCrvDIHyst_bool; /* '<S84>/Unit Delay' */

/* vehicle curve hystereis flag--Used in TJAGEN module
   DT:boolean */
extern boolean_T GEN_VehYawRateDIHyst_bool; /* '<S87>/Unit Delay' */

/* vehicle yaw rate hystereis flag--Used in TJAGEN module
   DT:boolean */
extern real32_T
    LKA_BlockTimerExpiredTimerRetrigger_sec; /* '<S121>/Unit Delay' */

/* Block Timer Expired timer retrigger--Used in TJALKA module
   DT:float32 */
extern boolean_T LKA_CrvQualRSFF_bool[2]; /* '<S173>/Unit Delay' */

/* Curve quality RSFlipFlop flag--Used in TJALKA module
   DT:boolean */
extern real32_T LKA_CrvQualTurnOffDelay_sec[2]; /* '<S174>/Unit Delay' */

/* Curve quality turn off delay time--Used in TJALKA module
   DT:float32 */
extern real32_T LKA_CrvQualTurnOnDelay_sec[2]; /* '<S175>/Unit Delay' */

/* Curve quality turn on delay time--Used in TJALKA module
   DT:float32 */
extern boolean_T LKA_DistVeh2LnBndHyst_bool[2]; /* '<S142>/Unit Delay' */

/* Distance of Veh to Lane Boundary hystereis flag--Used in TJALKA module
   DT:boolean */
extern boolean_T LKA_LanePredictValidRSFF_bool; /* '<S165>/Unit Delay' */

/* Lane predict valid RSFlipFlop flag--Used in TJALKA module
   DT:boolean */
extern boolean_T LKA_LaneWidthMaxHyst_bool; /* '<S134>/Unit Delay' */

/* Lane width Max hystereis flag--Used in TJALKA module
   DT:boolean */
extern boolean_T LKA_LaneWidthMinHyst_bool; /* '<S135>/Unit Delay' */

/* Lane width Min hystereis flag--Used in TJALKA module
   DT:boolean */
extern boolean_T LKA_LeLnCrvQualityValid_bool; /* '<S152>/OR' */

/* Debug */
extern real32_T LKA_LeLnIncohTurnOffDelay_sec; /* '<S131>/Unit Delay' */

/* Left lane incoherence turn off delay time--Used in TJALKA module
   DT:float32 */
extern boolean_T LKA_LeLnQualityValid_bool; /* '<S154>/OR' */

/* Debug */
extern real32_T LKA_LnBndValidTurnOnDelay_sec; /* '<S166>/Unit Delay' */

/* Lane boundary valid turn on delay time--Used in TJALKA module
   DT:float32 */
extern boolean_T LKA_LnIncohEdgeRising_bool; /* '<S125>/Unit Delay' */

/* Lane incoherence edge rising--Used in TJALKA module
   DT:boolean */
extern real32_T LKA_LnIncohTurnOffDelay_sec; /* '<S130>/Unit Delay' */

/* Lane incoherence turn off delay time--Used in TJALKA module
   DT:float32 */
extern boolean_T LKA_LnQualRSFF_bool[2]; /* '<S178>/Unit Delay' */

/* Lane quality RSFlipFlop flag--Used in TJALKA module
   DT:boolean */
extern real32_T LKA_LnQualTurnOffDelay_sec[2]; /* '<S179>/Unit Delay' */

/* Lane quality turn off delay time--Used in TJALKA module
   DT:float32 */
extern real32_T LKA_LnQualTurnOnDelay_sec[2]; /* '<S180>/Unit Delay' */

/* Lane quality turn on delay time--Used in TJALKA module
   DT:float32 */
extern boolean_T LKA_LnQualifierEdgeRising_bool; /* '<S176>/Unit Delay' */

/* Lane Qualifier edge rising--Used in TJALKA module
   DT:boolean */
extern real32_T LKA_LnQualifierTurnOffDelay_sec; /* '<S177>/Unit Delay' */

/* Lane Qualifier turn off delay time--Used in TJALKA module
   DT:float32 */
extern boolean_T LKA_OBFValidEdgeRising_bool; /* '<S158>/Unit Delay' */

/* OBF valid edge rising--Used in TJALKA module
   DT:boolean */
extern boolean_T LKA_PredTimeExceededEdgeFalling_bool; /* '<S167>/Unit Delay' */

/* Prediction time exceeded edge falling--Used in TJALKA module
   DT:boolean */
extern real32_T LKA_PredTimeExceededTurnOffDelay_sec; /* '<S168>/Unit Delay' */

/* Prediction time exceeded turn off delay time--Used in TJALKA module
   DT:float32 */
extern real32_T LKA_PrevLeLnPosY0UnitDelay_met;        /* '<S111>/Unit Delay' */
extern E_TJALKA_LnBndValid_nu LKA_PrevLnBndValid_Enum; /* '<S150>/Unit Delay' */

/* Previous lane boundary valid flag--Used in TJALKA module
   DT:Enum: E_TJALKA_LnBndValid_nu */
extern real32_T LKA_PrevRiLnPosY0UnitDelay_met; /* '<S111>/Unit Delay1' */
extern real32_T LKA_PrevVehDistUnitDelay_met;   /* '<S126>/Unit Delay2' */

/* Previous vehicle distance--Used in TJALKA module
   DT:boolean */
extern boolean_T LKA_RadiusHyst_bool; /* '<S137>/Unit Delay' */

/* Radius hystereis flag--Used in TJALKA module
   DT:boolean */
extern real32_T LKA_RiLnIncohTurnOffDelay_sec; /* '<S132>/Unit Delay' */

/* Right lane incoherence turn off delay time--Used in TJALKA module
   DT:float32 */
extern boolean_T LKA_TrajPlanCancelRSFF_bool; /* '<S124>/Unit Delay' */

/* Trajectory plan cancel RSFlipFlop flag--Used in TJALKA module
   DT:boolean */
extern boolean_T
    LKA_TurnSignalLevelHoldEdgeFalling_bool; /* '<S144>/Unit Delay' */

/* Turn signal level hold edge falling--Used in TJALKA module
   DT:boolean */
extern boolean_T LKA_ValidLengthHyst_bool[2]; /* '<S138>/Unit Delay' */

/* Valid length hystereis flag--Used in TJALKA module
   DT:boolean */
extern boolean_T LKA_VelXMaxHystWR_bool; /* '<S148>/Unit Delay' */

/* Velocity X Max hystereis flag--Used in TJALKA module
   DT:boolean */
extern boolean_T LKA_VelXMaxHyst_bool; /* '<S146>/Unit Delay' */

/* Velocity X Max hystereis flag--Used in TJALKA module
   DT:boolean */
extern boolean_T LKA_VelXMinHystWR_bool; /* '<S149>/Unit Delay' */

/* Velocity X Min hystereis flag--Used in TJALKA module
   DT:boolean */
extern boolean_T LKA_VelXMinHyst_bool; /* '<S147>/Unit Delay' */

/* Velocity X Min hystereis flag--Used in TJALKA module
   DT:boolean */
extern boolean_T OBF_AccObjSwitch; /* '<S206>/NOT' */

/* Debug */
extern boolean_T OBF_AccObjSwitchEdgeRising_bool; /* '<S213>/Unit Delay' */

/* Acc object switch edge rising--Used in TJAOBF module
   DT:boolean */
extern real32_T OBF_AccObjSwitchTurnOffDelay_sec; /* '<S214>/Unit Delay' */

/* Acc object switch turn off delay time--Used in TJAOBF module
   DT:float32 */
extern boolean_T OBF_AccObjValid; /* '<S206>/Equal' */

/* Debug */
extern boolean_T OBF_AccObjValidHoldUnitDelay_bool; /* '<S220>/Unit Delay' */

/* Previous AccObjValidHold flag--Used in TJASLC module
   DT:boolean */
extern boolean_T OBF_AccObjValidLaneCheck; /* '<S206>/Equal1' */

/* Debug */
extern real32_T OBF_AccObjValidLaneTurnOnDelay_sec; /* '<S216>/Unit Delay' */

/* Acc object lane valid turn on delay time--Used in TJAOBF module
   DT:float32 */
extern real32_T OBF_AccObjValidTurnOnDelay_sec; /* '<S215>/Unit Delay' */

/* Acc object valid turn on delay time--Used in TJAOBF module
   DT:float32 */
extern boolean_T OBF_AccObjValid_bool; /* '<S215>/AND' */

/* Debug */
extern real32_T OBF_BitfieldValidTurnOnDelay_sec; /* '<S256>/Unit Delay' */

/* Bitfield valid turn on delay time--Used in TJAOBF module
   DT:float32 */
extern real32_T
    OBF_BlockTimeExpiredTimerRetrigger_sec; /* '<S195>/Unit Delay' */

/* Block Timer Expired timer retrigger--Used in TJAOBF module
   DT:float32 */
extern boolean_T OBF_CancelUnitDelay_bool; /* '<S1>/Unit Delay5' */

/* Cancel Unit Delay--Used in TJAOBF module
   DT:boolean */
extern boolean_T OBF_Crv_SRHyst_bool; /* '<S257>/Unit Delay' */

/* Crv_SR hystereis flag--Used in TJAOBF module
   DT:boolean */
extern boolean_T OBF_DistOrEgoLaneInvalid_bool; /* '<S225>/OR5' */

/* Debug */
extern boolean_T OBF_Heading_SRHyst_bool; /* '<S258>/Unit Delay' */

/* Heading_SR hystereis flag--Used in TJAOBF module
   DT:boolean */
extern boolean_T OBF_LaneAttributesValidHyst_bool[2]; /* '<S246>/Unit Delay' */

/* Lane attributes valid hystereis flag--Used in TJAOBF module
   DT:boolean */
extern boolean_T OBF_LaneCheckValid_bool; /* '<S225>/OR2' */

/* Debug */
extern boolean_T OBF_LeftLaneCheckValid_bool; /* '<S224>/AND' */

/* Debug */
extern real32_T OBF_LeftLineValidTurnOnDelay_sec; /* '<S247>/Unit Delay' */

/* Lane boundary valid turn on delay time--Used in TJALKA module
   DT:float32 */
extern boolean_T OBF_MaxDist2BndHyst_bool[2]; /* '<S239>/Unit Delay' */

/* Max distance to boundary hystereis flag--Used in TJAOBF module
   DT:boolean */
extern real32_T OBF_MaxDurObjBrdgTimerRe_sec; /* '<S198>/Unit Delay' */

/* Max duration object bridging--Used in TJAOBF module
   DT:float32 */
extern boolean_T OBF_MinDist2BndHyst_bool[2]; /* '<S238>/Unit Delay' */

/* Min distance to boundary hystereis flag--Used in TJAOBF module
   DT:boolean */
extern boolean_T OBF_MinDist2LeftBndInvalid; /* '<S228>/NOT' */

/* Debug */
extern boolean_T OBF_MinDist2RightBndInvalid; /* '<S228>/NOT1' */

/* Debug */
extern real32_T OBF_MinDurLCforOBTurnOnDelay_sec; /* '<S199>/Unit Delay' */

/* Min duration LC for OBF turn on delay time--Used in TJAOBF module
   DT:float32 */
extern real32_T
    OBF_ObjLaneValidDurationTurnOnDelay_sec; /* '<S222>/Unit Delay' */

/* Obj in lane valid duration turn on delay time--Used in TJAOBF module
   DT:float32 */
extern boolean_T OBF_ObjLaneValidHoldUnitDelay_bool; /* '<S220>/Unit Delay1' */

/* Previous ObjLaneValidHold flag--Used in TJASLC module
   DT:boolean */
extern boolean_T OBF_PosY0_SRHyst_bool; /* '<S259>/Unit Delay' */

/* PosY0_SR hystereis flag--Used in TJAOBF module
   DT:boolean */
extern boolean_T OBF_RightLaneCheckValid_bool; /* '<S224>/AND1' */

/* Debug */
extern real32_T OBF_RightLineValidTurnOnDelay_sec; /* '<S248>/Unit Delay' */

/* Lane boundary valid turn on delay time--Used in TJALKA module
   DT:float32 */
extern boolean_T OBF_StrongReadyUnitDelay_bool; /* '<S1>/Unit Delay3' */

/* Strong ready Unit Delay--Used in TJAOBF module
   DT:boolean */
extern boolean_T OBF_TargetObjDataSR_bool; /* '<S184>/AND1' */

/* Debug */
extern boolean_T OBF_TargetObjDataWR_bool; /* '<S184>/AND' */

/* Debug */
extern boolean_T OBF_TargetOutsideEgoLane_bool; /* '<S229>/Multiport Switch' */

/* Debug */
extern boolean_T OBF_TurnSignalHoldUnitDelay_bool; /* '<S194>/Unit Delay' */

/* Previous turn signal level hold flag--Used in TJAOBF module
   DT:boolean */
extern boolean_T OBF_VelXMaxHyst_bool; /* '<S201>/Unit Delay' */

/* Velocity X Max hystereis flag--Used in TJAOBF module
   DT:boolean */
extern boolean_T OBF_VelXMinHyst_bool; /* '<S202>/Unit Delay' */

/* Velocity X Min hystereis flag--Used in TJAOBF module
   DT:boolean */
extern boolean_T OBF_WeakReadyUnitDelay_bool; /* '<S1>/Unit Delay4' */

/* Weak ready Unit Delay--Used in TJAOBF module
   DT:boolean */
extern boolean_T OF_NoObjectCollision_bool; /* '<S193>/NOT' */

/* Ego vehicle velocity min hystereis flag--Used in TJASLC module
   DT:boolean */
extern boolean_T OF_ObjectDangerLeftRear_bool; /* '<S193>/AND' */

/* Ego vehicle velocity min hystereis flag--Used in TJASLC module
   DT:boolean */
extern boolean_T OF_ObjectDangerRightRear_bool; /* '<S193>/AND1' */

/* Ego vehicle velocity min hystereis flag--Used in TJASLC module
   DT:boolean */
extern real32_T OF_RearObjDecCrvtTurnOffDelay_sec; /* '<S204>/Unit Delay' */

/* Rear obejct detected check turn off delay time--Used in TJAOF module
   DT:float32 */
extern real32_T OF_RearObjDecTurnOffDelay_sec[2]; /* '<S205>/Unit Delay' */

/* Rear obejct detected check turn off delay time--Used in TJAOF module
   DT:float32 */
extern real32_T SLC_AbortNewEgoTime_sec; /* '<S342>/Unit Delay' */

/* Previou reset RSFlipFlop flag 2--Used in TJASLC module
   DT:boolean */
extern E_TJASLC_AbortState_nu
    SLC_AbortState_enum; /* '<S314>/Signal Conversion' */

/* Previou reset RSFlipFlop flag 2--Used in TJASLC module
   DT:boolean */
extern real32_T SLC_AbortTime_sec; /* '<S341>/Unit Delay' */

/* Previou reset RSFlipFlop flag 2--Used in TJASLC module
   DT:boolean */
extern real32_T SLC_AbortYHdSignDelay2_sec; /* '<S343>/Unit Delay' */

/* SLC left lane type check turn off delay time--Used in TJASLC module
   DT:float32 */
extern boolean_T SLC_Abort_bool; /* '<S261>/OR1' */

/* Debug */
extern boolean_T SLC_AdjLnWidthMaxHyst_bool[2]; /* '<S271>/Unit Delay' */

/* Adjacent  lane width max hystereis flag--Used in TJASLC module
   DT:boolean */
extern boolean_T SLC_AdjLnWidthMinHyst_bool[2]; /* '<S272>/Unit Delay' */

/* Adjacent  lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
extern boolean_T SLC_AllowGoBack_bool; /* '<S299>/GreaterThan' */

/* Take over edge rising--Used in TJASLC module
   DT:boolean */
extern boolean_T SLC_Cancel_bool; /* '<S261>/OR' */

/* Debug */
extern real32_T SLC_CenterDistToBoundary_met; /* '<S299>/Switch2' */

/* Rear obejct detected check turn off delay time--Used in TJASLC module
   DT:float32 */
extern boolean_T SLC_EgoVehVelMaxHyst_bool; /* '<S504>/Unit Delay' */

/* Ego vehicle velocity max hystereis flag--Used in TJASLC module
   DT:boolean */
extern boolean_T SLC_EgoVehVelMinHyst_bool; /* '<S505>/Unit Delay' */

/* Ego vehicle velocity min hystereis flag--Used in TJASLC module
   DT:boolean */
extern boolean_T SLC_ExitAbortNewEgo_bool; /* '<S342>/AND' */

/* Previou reset RSFlipFlop flag 2--Used in TJASLC module
   DT:boolean */
extern boolean_T SLC_ExitAbort_bool; /* '<S327>/OR1' */

/* Previou reset RSFlipFlop flag 2--Used in TJASLC module
   DT:boolean */
extern real32_T SLC_FrontSafeTrOnTime_sec; /* '<S512>/Unit Delay' */

/* Lane change detected turn off delay time--Used in TJASLC module
   DT:float32 */
extern real32_T SLC_FrontUnSafeTrOnTime_sec; /* '<S509>/Unit Delay' */

/* Lane change detected turn off delay time--Used in TJASLC module
   DT:float32 */
extern boolean_T SLC_GRCOLeftRSFF_bool; /* '<S360>/Unit Delay' */

/* Previou reset RSFlipFlop flag--Used in TJASLC module
   DT:boolean */
extern boolean_T SLC_GRCORightRSFF_bool; /* '<S361>/Unit Delay' */

/* Previou reset RSFlipFlop flag--Used in TJASLC module
   DT:boolean */
extern real32_T SLC_GoBkLfTurnLtTurnOffDelay_sec; /* '<S301>/Unit Delay' */

/* Turn signal check turn off delay time--Used in TJASLC module
   DT:float32 */
extern real32_T SLC_GoBkRiTurnLtTurnOffDelay_sec; /* '<S302>/Unit Delay' */

/* Turn signal check turn off delay time--Used in TJASLC module
   DT:float32 */
extern boolean_T SLC_IntoAbort_nu; /* '<S324>/OR1' */

/* Previous driver trigger reset left flag--Used in TJASLC module
   DT:boolean */
extern real32_T SLC_LCActiveTurnOnDelay_sec; /* '<S503>/Unit Delay' */

/* Lane centering active turn off delay time--Used in TJASLC module
   DT:float32 */
extern boolean_T SLC_LCM_Cancel_bool; /* '<S321>/OR' */

/* Debug */
extern boolean_T SLC_LCM_End_bool; /* '<S366>/NOT' */

/* Debug */
extern boolean_T SLC_LCM_Start_bool; /* '<S367>/NOT' */

/* Debug */
extern real32_T SLC_LCPLeft2ActiveTurnOnDelay_sec; /* '<S370>/Unit Delay' */

/* LCPLeft2Active turn on delay time--Used in TJASLC module
   DT:float32 */
extern real32_T SLC_LCPLeft2PassiveTurnOnDelay_sec; /* '<S371>/Unit Delay' */

/* LCPLeft2Passive turn on delay time--Used in TJASLC module
   DT:float32 */
extern real32_T SLC_LCPRight2ActiveTurnOnDelay_sec; /* '<S373>/Unit Delay' */

/* LCPRight2Active turn on delay time--Used in TJASLC module
   DT:float32 */
extern real32_T SLC_LCPRight2PassiveTurnOnDelay_sec; /* '<S372>/Unit Delay' */

/* LCPRight2Passive turn on delay time--Used in TJASLC module
   DT:float32 */
extern real32_T SLC_LCWPassiveTurnOnDly_sec; /* '<S432>/Unit Delay' */

/* SLC right lane type check turn off delay time--Used in TJASLC module
   DT:float32 */
extern real32_T SLC_LaneChangCancleTimeDelay_sec; /* '<S394>/Unit Delay' */

/* Take over valid turn off delay time--Used in TJASLC module
   DT:float32 */
extern real32_T SLC_LaneChangEndTimeDelay_sec; /* '<S397>/Unit Delay' */

/* Take over valid turn off delay time--Used in TJASLC module
   DT:float32 */
extern boolean_T SLC_LaneChangeBackDetc_bool; /* '<S326>/Switch2' */

/* Previous driver trigger reset left flag--Used in TJASLC module
   DT:boolean */
extern boolean_T SLC_LaneChangeCancleInfo; /* '<S394>/OR' */

/* Previous reset flag--Used in TJASLC module
   DT:boolean */
extern E_TJASLC_LaneChangeTrig_nu
    SLC_LaneChangeDirectionAbort_enum; /* '<S314>/Signal Conversion1' */

/* Previous driver trigger reset left flag--Used in TJASLC module
   DT:boolean */
extern E_TJASLC_LaneChangeTrig_nu
    SLC_LaneChangeDirectionIn_nu; /* '<S325>/Switch' */

/* Previous driver trigger reset left flag--Used in TJASLC module
   DT:boolean */
extern boolean_T SLC_LaneChangeEdgeRising_bool; /* '<S470>/Unit Delay' */

/* Lane change edge rising--Used in TJASLC module
   DT:boolean */
extern boolean_T SLC_LaneChangeEndInfo; /* '<S397>/OR' */

/* Previous reset flag--Used in TJASLC module
   DT:boolean */
extern boolean_T SLC_LaneChangeOnGoingInfo; /* '<S381>/OR1' */

/* Previous reset flag--Used in TJASLC module
   DT:boolean */
extern boolean_T SLC_LaneChangePendingInfo; /* '<S382>/OR4' */

/* Previous reset flag--Used in TJASLC module
   DT:boolean */
extern boolean_T SLC_LaneChangeRSFF_bool; /* '<S478>/Unit Delay' */

/* Lane change RSFlipFlop flag--Used in TJASLC module
   DT:boolean */
extern boolean_T SLC_LaneChangeWarnEdgeRising_bool; /* '<S409>/Unit Delay' */

/* Take over edge rising--Used in TJASLC module
   DT:boolean */
extern real32_T SLC_LaneChangeWarnTurnOffDelay_sec; /* '<S429>/Unit Delay' */

/* Take over valid turn off delay time--Used in TJASLC module
   DT:float32 */
extern E_TJASLC_LaneChangeWarning_nu
    SLC_LaneChangeWarnUnitDy_Enum; /* '<S375>/Unit Delay' */

/* Previous maneuver state 2--Used in TJASLC module
   DT:Enum: E_TJASLC_ManeuverState_nu */
extern boolean_T SLC_LaneCheckValid_bool; /* '<S364>/AND' */

/* Debug */
extern real32_T SLC_LaneChngDetectedTurnOffDelay_sec; /* '<S368>/Unit Delay' */

/* Lane change detected turn off delay time--Used in TJASLC module
   DT:float32 */
extern boolean_T SLC_LaneWidthMaxHyst_bool; /* '<S312>/Unit Delay' */

/* SLC lane width max hystereis flag--Used in TJASLC module
   DT:boolean */
extern boolean_T SLC_LaneWidthMinHyst_bool; /* '<S313>/Unit Delay' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
extern real32_T SLC_LeLaneTypeTurnOffDelay_sec; /* '<S309>/Unit Delay' */

/* SLC left lane type check turn off delay time--Used in TJASLC module
   DT:float32 */
extern real32_T SLC_LeftSafeTrOnTime_sec; /* '<S510>/Unit Delay' */

/* Lane change detected turn off delay time--Used in TJASLC module
   DT:float32 */
extern real32_T SLC_LeftTurnSignalOffDelay_sec; /* '<S544>/Unit Delay' */

/* Left turn signal turn off dealy time in sec in TJASLC module
   DT:single */
extern real32_T SLC_LeftUnSafeTrOnTime_sec; /* '<S507>/Unit Delay' */

/* Lane change detected turn off delay time--Used in TJASLC module
   DT:float32 */
extern real32_T SLC_LeverEngagedTurnOnDelay2_sec[2]; /* '<S541>/Unit Delay' */

/* Lever left or right engaged turn on delay time 2--Used in TJASLC module
   DT:float32 */
extern real32_T SLC_LeverEngagedTurnOnDelay_sec; /* '<S535>/Unit Delay' */

/* Lever left or right engaged turn on delay time--Used in TJASLC module
   DT:float32 */
extern boolean_T SLC_LeverLeftEngagedRSFF_bool; /* '<S520>/Unit Delay' */

/* Lever left engaged RSFlipFlop flag--Used in TJASLC module
   DT:boolean */
extern boolean_T SLC_LeverLeftEngaged_bool; /* '<S520>/Switch' */

/* Debug */
extern boolean_T SLC_LeverRightEngagedRSFF_bool; /* '<S522>/Unit Delay' */

/* Lever right engaged RSFlipFlop flag--Used in TJASLC module
   DT:boolean */
extern boolean_T SLC_LeverRightEngaged_bool; /* '<S522>/Switch' */

/* Debug */
extern real32_T SLC_LnChngBlockTimeCancleTfDelay_sec; /* '<S493>/Unit Delay' */

/* SLC right lane type check turn off delay time--Used in TJASLC module
   DT:float32 */
extern real32_T SLC_LnChngBlockTimeTurnOffDelay_sec; /* '<S492>/Unit Delay' */

/* Lane change block time expired turn off delay time--Used in TJASLC module
   DT:float32 */
extern boolean_T SLC_LnChngDetectedRSFF_bool; /* '<S495>/Unit Delay' */

/* Lane change detected RSFlipFlop flag--Used in TJASLC module
   DT:boolean */
extern real32_T SLC_LnChngDetectedTurnOffDelay_sec; /* '<S496>/Unit Delay' */

/* Lane change detected turn off delay time--Used in TJASLC module
   DT:float32 */
extern real32_T SLC_ManeuverStateTurnOnDelay_sec; /* '<S448>/Unit Delay' */

/* Maneuver state turn on delay time--Used in TJASLC module
   DT:float32 */
extern boolean_T SLC_ManvStatePassive_bool; /* '<S526>/AND' */

/* Debug */
extern boolean_T SLC_MaxInitDurationExceeded_bool; /* '<S527>/AND' */

/* Debug */
extern boolean_T SLC_MaxManeuTimeRSFF_bool; /* '<S289>/Unit Delay' */

/* max maneuver time expired RSFlipFlop flag--Used in TJASLC module
   DT:boolean */
extern real32_T SLC_MaxManeuTimeRetrigger_sec; /* '<S290>/Unit Delay' */

/* Max maneuver time expired Time retrigger--Used in TJASLC module
   DT:float32 */
extern boolean_T SLC_NewEgoLaneRSFF_bool; /* '<S369>/Unit Delay' */

/* New ego lane RSFlipFlop flag--Used in TJASLC module
   DT:boolean */
extern boolean_T SLC_NewEgoLane_bool; /* '<S369>/Switch' */

/* Debug */
extern boolean_T SLC_OELCNewEgoLaneRSFF_bool; /* '<S303>/Unit Delay' */

/* New ego lane RSFlipFlop flag--Used in TJASLC module
   DT:boolean */
extern E_TJASLC_AbortState_nu SLC_PreAbortState_enum; /* '<S314>/Unit Delay1' */

/* Previou reset RSFlipFlop flag 2--Used in TJASLC module
   DT:boolean */
extern E_TJASLC_LaneChangeTrig_nu
    SLC_PreLaneChangeDirtAbort_enum; /* '<S314>/Unit Delay' */

/* Previous driver trigger reset left flag--Used in TJASLC module
   DT:boolean */
extern boolean_T SLC_PrebNewEgoOverTime_nu; /* '<S379>/Unit Delay' */

/* Previous reset flag--Used in TJASLC module
   DT:boolean */
extern boolean_T
    SLC_PrevDriverTrigResetLeftUnitDelay_bool; /* '<S268>/Unit Delay1' */

/* Previous driver trigger reset left flag--Used in TJASLC module
   DT:boolean */
extern boolean_T
    SLC_PrevDriverTrigResetRightUnitDelay_bool; /* '<S268>/Unit Delay2' */

/* Previous driver trigger reset Right flag--Used in TJASLC module
   DT:boolean */
extern real32_T
    SLC_PrevFrontWheelDist2BoundUnitDelay_met; /* '<S362>/Unit Delay' */

/* Previous front whell distance to boundary--Used in TJASLC module
   DT:float32 */
extern real32_T SLC_PrevFtWhDit2BdSignUnitDelay_met; /* '<S367>/Unit Delay' */

/* Previous front wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
extern E_TJASLC_LaneChangeTrig_nu
    SLC_PrevLaneChangeTrigger_nu; /* '<S516>/Unit Delay1' */

/* Previous driver trigger reset left flag--Used in TJASLC module
   DT:boolean */
extern E_TJASTM_LatCtrlMode_nu
    SLC_PrevLatCtrlMd2_Enum; /* '<S287>/Unit Delay' */

/* Previous lateral control mode 2--Used in TJASLC module
   DT:E_TJASTM_LatCtrlMode_nu */
extern E_TJASTM_LatCtrlMode_nu
    SLC_PrevLatCtrlMd3_Enum; /* '<S285>/Unit Delay' */

/* Previous lateral control mode 3--Used in TJASLC module
   DT:E_TJASTM_LatCtrlMode_nu */
extern E_TJASTM_LatCtrlMode_nu
    SLC_PrevLatCtrlMdSLC2LCC_Enum; /* '<S888>/Unit Delay' */

/* Previous lateral control mode--Used in TJASLC module
   DT:E_TJASTM_LatCtrlMode_nu */
extern E_TJASTM_LatCtrlMode_nu SLC_PrevLatCtrlMd_Enum; /* '<S491>/Unit Delay' */

/* Previous lateral control mode--Used in TJASLC module
   DT:E_TJASTM_LatCtrlMode_nu */
extern E_TJASLC_ManeuverState_nu
    SLC_PrevManeuverState2_Enum; /* '<S526>/Unit Delay' */

/* Previous maneuver state 2--Used in TJASLC module
   DT:Enum: E_TJASLC_ManeuverState_nu */
extern E_TJASLC_ManeuverState_nu
    SLC_PrevManeuverState3_Enum; /* '<S320>/Unit Delay' */

/* Previous maneuver state 3--Used in TJASLC module
   DT:Enum: E_TJASLC_ManeuverState_nu */
extern E_TJASLC_ManeuverState_nu
    SLC_PrevManeuverStateGRCO_Enum; /* '<S264>/Unit Delay1' */

/* Previous maneuver state--Used in TJASLC module
   DT:Enum: E_TJASLC_ManeuverState_nu */
extern E_TJASLC_ManeuverState_nu
    SLC_PrevManeuverStateSLC2LCC_Enum; /* '<S888>/Unit Delay1' */

/* Previous maneuver state 2--Used in TJASLC module
   DT:Enum: E_TJASLC_ManeuverState_nu */
extern E_TJASLC_ManeuverState_nu
    SLC_PrevManeuverState_Enum; /* '<S7>/Unit Delay1' */

/* Previous maneuver state--Used in TJASLC module
   DT:Enum: E_TJASLC_ManeuverState_nu */
extern real32_T SLC_PrevReWhDit2BdSignUnitDelay_met; /* '<S366>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
extern E_TJASLC_LaneChangeWarning_nu
    SLC_PrevReaAbortWarnSide_enum; /* '<S375>/Unit Delay3' */

/* Lane change detected turn off delay time--Used in TJASLC module
   DT:float32 */
extern E_TJASLC_LaneChangeWarning_nu
    SLC_PrevRearCancleWarnSide_enum; /* '<S375>/Unit Delay2' */

/* Lane change detected turn off delay time--Used in TJASLC module
   DT:float32 */
extern real32_T
    SLC_PrevRearWheelDist2BoundUnitDelay_met; /* '<S363>/Unit Delay' */

/* Previous rear whell distance to boundary flag--Used in TJASLC module
   DT:float32 */
extern boolean_T SLC_PrevResetRSFF2_bool; /* '<S523>/Unit Delay' */

/* Previou reset RSFlipFlop flag 2--Used in TJASLC module
   DT:boolean */
extern boolean_T SLC_PrevResetRSFF_bool; /* '<S521>/Unit Delay' */

/* Previou reset RSFlipFlop flag--Used in TJASLC module
   DT:boolean */
extern boolean_T SLC_PrevResetUnitDelay_bool; /* '<S268>/Unit Delay' */

/* Previous reset flag--Used in TJASLC module
   DT:boolean */
extern boolean_T SLC_PrevReset_bool; /* '<S268>/Unit Delay' */

/* Debug */
extern boolean_T SLC_PrevTakeoverValidUnitDelay_bool; /* '<S7>/Unit Delay' */

/* Previous take over valid flag--Used in TJASLC module
   DT:boolean */
extern boolean_T SLC_RearAbortEdgRs_bool; /* '<S411>/Unit Delay' */

/* Lane change detected turn off delay time--Used in TJASLC module
   DT:float32 */
extern real32_T SLC_RearAbortTunOffDly_sec; /* '<S431>/Unit Delay' */

/* Lane change detected turn off delay time--Used in TJASLC module
   DT:float32 */
extern boolean_T SLC_RearCancleEdgRs_bool; /* '<S410>/Unit Delay' */

/* Lane change detected turn off delay time--Used in TJASLC module
   DT:float32 */
extern real32_T SLC_RearCancleTunOffDly_sec; /* '<S430>/Unit Delay' */

/* Lane change detected turn off delay time--Used in TJASLC module
   DT:float32 */
extern real32_T SLC_RearObjDecTurnOffDelay_sec[2]; /* '<S506>/Unit Delay' */

/* Rear obejct detected check turn off delay time--Used in TJASLC module
   DT:float32 */
extern real32_T SLC_ResetTurnOffDelay_sec; /* '<S528>/Unit Delay' */

/* Reset turn off delay time--Used in TJASLC module
   DT:float32 */
extern real32_T SLC_RiLaneTypeTurnOffDelay_sec; /* '<S310>/Unit Delay' */

/* SLC right lane type check turn off delay time--Used in TJASLC module
   DT:float32 */
extern real32_T SLC_RightSafeTrOnTime_sec; /* '<S511>/Unit Delay' */

/* Lane change detected turn off delay time--Used in TJASLC module
   DT:float32 */
extern real32_T SLC_RightTurnSignalOffDelay_sec; /* '<S545>/Unit Delay' */

/* Right turn signal turn off dealy time in sec in TJASLC module
   DT:single */
extern real32_T SLC_RightUnSafeTrOnTime_sec; /* '<S508>/Unit Delay' */

/* Lane change detected turn off delay time--Used in TJASLC module
   DT:float32 */
extern boolean_T SLC_SameLaneChangeDetc_bool; /* '<S326>/Switch' */

/* Previous driver trigger reset left flag--Used in TJASLC module
   DT:boolean */
extern boolean_T SLC_StrongReadyBothSides_bool; /* '<S463>/AND1' */

/* Debug */
extern boolean_T SLC_TakeOverRSFF2_bool; /* '<S445>/Unit Delay' */

/* Adjacent  lane width max hystereis flag--Used in TJASLC module
   DT:boolean */
extern boolean_T SLC_TakeOverValidUnitDelay_bool; /* '<S1>/Unit Delay2' */

/* Take over valid Unit Delay--Used in TJASLC module
   DT:boolean */
extern real32_T SLC_TakeoverAbortTurnOffDelay_sec; /* '<S447>/Unit Delay' */

/* Lane centering active turn off delay time--Used in TJASLC module
   DT:float32 */
extern boolean_T SLC_TakeoverEdgeRising_bool; /* '<S433>/Unit Delay' */

/* Take over edge rising--Used in TJASLC module
   DT:boolean */
extern real32_T SLC_TakeoverValidTurnOffDelay_sec; /* '<S446>/Unit Delay' */

/* Take over valid turn off delay time--Used in TJASLC module
   DT:float32 */
extern boolean_T SLC_TriggerLeft_bool; /* '<S516>/AND' */

/* Debug */
extern boolean_T SLC_TriggerRight_bool; /* '<S516>/AND1' */

/* Debug */
extern boolean_T SLC_TurnSignalOffBlckRSFF_bool; /* '<S499>/Unit Delay' */

/* SLC turn signal off RSFlipFlop flag--Used in TJASLC module
   DT:boolean */
extern boolean_T SLC_TurnSignalOffEF_bool; /* '<S384>/Unit Delay' */

/* Previous reset flag--Used in TJASLC module
   DT:boolean */
extern real32_T SLC_TurnSignalTurnOffDelay_sec[2]; /* '<S519>/Unit Delay' */

/* Turn signal check turn off delay time--Used in TJASLC module
   DT:float32 */
extern real32_T SLC_UnitDelay_LePosY0_met; /* '<S326>/Unit Delay' */

/* Previous driver trigger reset left flag--Used in TJASLC module
   DT:boolean */
extern real32_T SLC_UnitDelay_RiPosY0_met; /* '<S326>/Unit Delay1' */

/* Previous driver trigger reset left flag--Used in TJASLC module
   DT:boolean */
extern boolean_T SLC_VehSpdTooLowInfo; /* '<S383>/OR2' */

/* Previous reset flag--Used in TJASLC module
   DT:boolean */
extern boolean_T SLC_WeakReadyBothSides_bool; /* '<S463>/AND2' */

/* Debug */
extern boolean_T SLC_WeakReadyLeft_bool; /* '<S464>/AND' */

/* Debug */
extern boolean_T SLC_WeakReadyRight_bool; /* '<S465>/AND' */

/* Debug */
extern boolean_T STM_Cancel_bool; /* '<S547>/OR' */

/* Debug */
extern E_TJASTM_LatCtrlMode_nu
    STM_LatCtrlMdUnitDelay_bool; /* '<S1>/Unit Delay' */

/* Lateral control mode Unit Delay--Used in TJASTM module
   DT:Enum: E_TJASTM_LatCtrlMode_nu */
extern boolean_T STM_SuspendEdgeRising_bool; /* '<S605>/Unit Delay' */

/* Suspended state edge rising--Used in TJATTG module
   DT:boolean */
extern E_TJASTM_SysStateTJA_nu
    STM_SysStateUnitDelay_bool; /* '<S1>/Unit Delay1' */

/* System state Unit Delay--Used in TJASTM module
   DT:Enum: E_TJASTM_SysStateTJA_nu */
extern real32_T TJALKA_LeftLlineValidRD_Sec; /* '<S156>/Unit Delay' */

/* SLC left lane type check turn off delay time--Used in TJASLC module
   DT:float32 */
extern real32_T TJALKA_RightLlineValidRD_Sec; /* '<S157>/Unit Delay' */

/* SLC left lane type check turn off delay time--Used in TJASLC module
   DT:float32 */
extern int32_T TJASLC_PrevSLCHighLightID_nu; /* '<S375>/Unit Delay1' */

/* Lane change detected turn off delay time--Used in TJASLC module
   DT:float32 */
extern boolean_T TJASLC_RSLeftLaneChangeWarn_nu; /* '<S376>/Unit Delay' */

/* Previou reset RSFlipFlop flag in lane change warning --Used in TJASLC module
   DT:boolean */
extern boolean_T TJASLC_RSRightLaneChangeWarn_nu; /* '<S377>/Unit Delay' */

/* Previou reset RSFlipFlop flag in lane change warning --Used in TJASLC module
   DT:boolean */
extern real32_T TJASTM_ACCActiveOvertime_sec; /* '<S563>/Unit Delay' */

/* Max maneuver time expired Time retrigger--Used in TJASLC module
   DT:float32 */
extern real32_T TJASTM_ACCOffTurnOffDelay_sec; /* '<S561>/Unit Delay' */

/* Lane change detected turn off delay time--Used in TJASLC module
   DT:float32 */
extern real32_T TJASTM_HandsOffWarnTurnOffDelay_sec; /* '<S562>/Unit Delay' */

/* Reset turn off delay time--Used in TJASLC module
   DT:float32 */
extern boolean_T TJASTM_PrevACCActOvrtm_bool; /* '<S8>/Unit Delay1' */

/* PrevACCActiveOvertime uint delay */
extern E_TJASTM_SysStateTJA_nu
    TJASTM_PrevSysStateTJAIn_envm; /* '<S546>/Unit Delay1' */

/* Previous lane boundary valid flag--Used in TJALKA module
   DT:Enum: E_TJALKA_LnBndValid_nu */
extern real32_T TJATOW_NPilotBrkPAudioPlay_sec; /* '<S659>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
extern boolean_T TJATOW_NPilotBrkPUnitDealy_bool; /* '<S619>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
extern real32_T TJATOW_NPilotDoorAudioPlay_sec; /* '<S650>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
extern boolean_T TJATOW_NPilotDoorOpenUnitDealy_bool; /* '<S622>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
extern real32_T TJATOW_NPilotEPBAudioPlay_sec; /* '<S653>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
extern boolean_T TJATOW_NPilotEPBUnitDealy_bool; /* '<S625>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
extern real32_T TJATOW_NPilotErrorAudioPlay_sec; /* '<S657>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
extern boolean_T TJATOW_NPilotErrorUnitDealy_bool; /* '<S616>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
extern real32_T TJATOW_NPilotHoodAudioPlay_sec; /* '<S652>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
extern boolean_T TJATOW_NPilotHoodUnitDealy_bool; /* '<S624>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
extern real32_T TJATOW_NPilotLaneAudioPlay_sec; /* '<S662>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
extern boolean_T TJATOW_NPilotLineUnitDealy_bool; /* '<S620>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
extern real32_T TJATOW_NPilotNoAudioPlay_sec; /* '<S655>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
extern boolean_T TJATOW_NPilotNoUnitDealy_bool; /* '<S618>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
extern real32_T TJATOW_NPilotOffAudioPlay_sec; /* '<S656>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
extern real32_T TJATOW_NPilotOnAudioPlay_sec; /* '<S649>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
extern real32_T TJATOW_NPilotSafeAudioPlay_sec; /* '<S654>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
extern boolean_T TJATOW_NPilotSafeUnitDealy_bool; /* '<S626>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
extern real32_T TJATOW_NPilotSeatBeltPAudioPlay_sec; /* '<S663>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
extern boolean_T TJATOW_NPilotSeatBeltUnitDealy_bool; /* '<S621>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
extern real32_T TJATOW_NPilotSpdAudioPlay_sec; /* '<S658>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
extern boolean_T TJATOW_NPilotSpdUnitDealy_bool; /* '<S617>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
extern real32_T TJATOW_NPilotTrunkAudioPlay_sec; /* '<S651>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
extern boolean_T TJATOW_NPilotTrunkUnitDealy_bool; /* '<S623>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
extern real32_T TJATOW_PilotOverrideAudioPlay_sec; /* '<S660>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
extern real32_T TJATOW_PilotResumeAudioPlay_sec; /* '<S661>/Unit Delay' */

/* Previous rear wheel distance to boundary sign--Used in TJASLC module
   DT:float32 */
extern boolean_T TJATOW_RSFlipFlop_nu; /* '<S614>/Unit Delay' */

/* Previou reset RSFlipFlop flag in take over warning --Used in TJASLC module
   DT:boolean */
extern real32_T TJATOW_TakeOverTurnOffDelay_sec; /* '<S615>/Unit Delay' */

/* Take Over turn off delay time --Used in TJASTM module */
extern boolean_T TJATOW_TakeOverTurnOnDelay_nu; /* '<S610>/Unit Delay' */

/* Take Over Signal Unit Delay--Used in TJASTM module */
extern real32_T TJATTG_TgtTrajHeadAngUnitDy_rad; /* '<S1>/Unit Delay7' */

/* Previous maneuver state 2--Used in TJASLC module
   DT:Enum: E_TJASLC_ManeuverState_nu */
extern real32_T TJATTG_TgtTrajPosY0UnitDy_met; /* '<S1>/Unit Delay6' */

/* Previous maneuver state 2--Used in TJASLC module
   DT:Enum: E_TJASLC_ManeuverState_nu */
extern boolean_T TTG_CMBEnableUnitDelay_bool; /* '<S795>/Unit Delay' */

/* Combined date enable Unit Delay--Used in TJATTG module
   DT:boolean */
extern boolean_T TTG_CMBObjectCorridorUnitDelay_bool; /* '<S779>/Unit Delay2' */

/* CMB object corridor Unit Delay--Used in TJATTG module
   DT:boolean */
extern boolean_T TTG_CMBObjectCorridor_bool; /* '<S665>/AND' */

/* Debug */
extern boolean_T TTG_CMB_Enable_bool; /* '<S795>/Switch1' */

/* Debug */
extern real32_T TTG_CntrCrvPredictLowPass_1pm; /* '<S702>/Unit Delay' */

/* Control lane curve predict low pass filter--Used in TJATTG module
   DT:float32 */
extern real32_T TTG_CntrLnCrvChngPredictLowPass_1pm2; /* '<S700>/Unit Delay' */

/* Control lane curve of change predict low pass filter--Used in TJATTG module
   DT:float32 */
extern real32_T TTG_CntrLnCrvChngUnitDelay_1pm2; /* '<S690>/Unit Delay' */

/* Control lane curve of change freeze output--Used in TJATTG module
   DT:float32 */
extern real32_T TTG_CntrLnCrvUnitDelay_1pm; /* '<S691>/Unit Delay' */

/* Left lane curve freeze output--Used in TJATTG module
   DT:float32 */
extern real32_T TTG_CntrLnHeadingUnitDelay_rad; /* '<S692>/Unit Delay' */

/* Control lane heading freeze output--Used in TJATTG module
   DT:float32 */
extern real32_T TTG_CntrLnLengthUnitDelay_met; /* '<S695>/Unit Delay' */

/* Control lane lenght freeze output--Used in TJATTG module
   DT:float32 */
extern real32_T TTG_CntrLnPosY0UnitDelay_met; /* '<S693>/Unit Delay' */

/* Control lane posY0 freeze output--Used in TJATTG module
   DT:float32 */
extern boolean_T
    TTG_CntrLnPredictEnable2EdgeRising_bool; /* '<S699>/Unit Delay' */

/* Control lane predict enable 2 flag edge rising--Used in TJATTG module
   DT:boolean */
extern boolean_T
    TTG_CntrLnPredictEnableEdgeRising_bool; /* '<S701>/Unit Delay' */

/* Control lane predict enable flag edge rising--Used in TJATTG module
   DT:boolean */
extern boolean_T TTG_CntrLnResetEdgeRising_bool; /* '<S703>/Unit Delay' */

/* Control lane reset flag edge rising--Used in TJATTG module
   DT:boolean */
extern boolean_T TTG_Enable_bool; /* '<S798>/AND' */

/* Debug */
extern boolean_T TTG_LDEnableUnitDelay_bool; /* '<S796>/Unit Delay' */

/* Lane date enable Unit Delay--Used in TJATTG module
   DT:boolean */
extern boolean_T TTG_LD_Enable_bool; /* '<S796>/Switch1' */

/* Debug */
extern boolean_T TTG_LD_PredictFinish_bool; /* '<S766>/AND' */

/* Debug */
extern boolean_T TTG_LanePredictEdgeRising_bool; /* '<S742>/Unit Delay' */

/* Lane predict edge rising--Used in TJATTG module
   DT:boolean */
extern boolean_T TTG_LaneUpdate_bool; /* '<S764>/OR3' */

/* Debug */
extern real32_T TTG_LeCorridorCrvChngUnitDelay_1pm2; /* '<S751>/Unit Delay4' */

/* Left corridor curve of change unit delay--Used in TJATTG module
   DT:float32 */
extern real32_T TTG_LeCorridorCrvUnitDelay_1pm; /* '<S751>/Unit Delay3' */

/* Left corridor curve Unit Delay--Used in TJATTG module
   DT:float32 */
extern real32_T TTG_LeCorridorHeadingUnitDelay_rad; /* '<S751>/Unit Delay2' */

/* Left corridor heading Unit Delay--Used in TJATTG module
   DT:float32 */
extern real32_T TTG_LeCorridorLengthUnitDelay_met; /* '<S751>/Unit Delay5' */

/* Left corridor lenght Unit Delay--Used in TJATTG module
   DT:float32 */
extern real32_T TTG_LeCorridorPosX0UnitDelay_met; /* '<S751>/Unit Delay' */

/* Left corridor posX0 Unit Delay--Used in TJATTG module
   DT:float32 */
extern real32_T TTG_LeCorridorPosY0UnitDelay_met; /* '<S751>/Unit Delay1' */

/* Left corridor posY0 Unit Delay--Used in TJATTG module
   DT:float32 */
extern real32_T TTG_LeLnCrvChngPredictLowPass_1pm2; /* '<S717>/Unit Delay' */

/* Left lane curve of change predict low pass filter--Used in TJATTG module
   DT:float32 */
extern real32_T TTG_LeLnCrvChngUnitDelay_1pm2; /* '<S707>/Unit Delay' */

/* Left lane curve of change freeze output--Used in TJATTG module
   DT:float32 */
extern real32_T TTG_LeLnCrvPredictLowPass_1pm; /* '<S719>/Unit Delay' */

/* Left lane curve predict low pass filter--Used in TJATTG module
   DT:float32 */
extern real32_T TTG_LeLnCrvUnitDelay_1pm; /* '<S708>/Unit Delay' */

/* Left lane curve freeze output--Used in TJATTG module
   DT:float32 */
extern real32_T TTG_LeLnHeadingUnitDelay_rad; /* '<S709>/Unit Delay' */

/* Left lane heading freeze output--Used in TJATTG module
   DT:float32 */
extern real32_T TTG_LeLnLengthUnitDelay_met; /* '<S712>/Unit Delay' */

/* Left lane lenght freeze output--Used in TJATTG module
   DT:float32 */
extern real32_T TTG_LeLnPosY0UnitDelay_met; /* '<S710>/Unit Delay' */

/* Left lane posY0 freeze output--Used in TJATTG module
   DT:float32 */
extern boolean_T
    TTG_LeLnPredictEnable2EdgeRising_bool; /* '<S716>/Unit Delay' */

/* Left lane predict enable 2 flag edge rising--Used in TJATTG module
   DT:boolean */
extern boolean_T TTG_LeLnPredictEnableEdgeRising_bool; /* '<S718>/Unit Delay' */

/* Left lane predict enable flag edge rising--Used in TJATTG module
   DT:boolean */
extern boolean_T TTG_LeLnResetEdgeRising_bool; /* '<S720>/Unit Delay' */

/* Left lane reset flag edge rising--Used in TJATTG module
   DT:boolean */
extern boolean_T TTG_ODEnableUnitDelay_bool; /* '<S794>/Unit Delay' */

/* Object date enable Unit Delay--Used in TJATTG module
   DT:boolean */
extern boolean_T TTG_OD_Enable_bool; /* '<S794>/Switch1' */

/* Debug */
extern boolean_T TTG_ObjectCorridorEdgeFalling_bool; /* '<S677>/Unit Delay' */

/* Object corridor flag edge falling--Used in TJATTG module
   DT:boolean */
extern boolean_T TTG_ObjectCorridorEdgeRising_bool; /* '<S678>/Unit Delay' */

/* Object corridor flag edge rising--Used in TJATTG module
   DT:boolean */
extern boolean_T TTG_ObjectUpdate_bool; /* '<S765>/OR2' */

/* Debug */
extern real32_T TTG_OdoPosXDelayRe_met; /* '<S740>/Unit Delay1' */

/* Odometrie position X Delay_RE--Used in TJATTG module
   DT:float32 */
extern real32_T TTG_OdoPosYDelayRe_met; /* '<S739>/Unit Delay1' */

/* Odometrie position Y Delay_RE--Used in TJATTG module
   DT:float32 */
extern real32_T TTG_OdoYawDelayRe_rad; /* '<S738>/Unit Delay1' */

/* Odometrie yaw angle Delay_RE--Used in TJATTG module
   DT:float32 */
extern boolean_T TTG_PredictEnableEdgeRising_bool; /* '<S741>/Unit Delay' */

/* Lane predict enable edge rising--Used in TJATTG module
   DT:boolean */
extern boolean_T TTG_PredictEnableRSFF_bool; /* '<S743>/Unit Delay' */

/* Lane prediction enable RSFlipFlop flag--Used in TJATTG module
   DT:boolean */
extern boolean_T TTG_PredictEnableUnitDelay_bool; /* '<S797>/Unit Delay' */

/* Predict enable Unit Delay--Used in TJATTG module
   DT:boolean */
extern boolean_T TTG_Predict_Enable_bool; /* '<S797>/Switch1' */

/* Debug */
extern boolean_T TTG_PredictionEnableEdgeFalling_bool; /* '<S777>/Unit Delay' */

/* Prediction enable flag edge falling--Used in TJATTG module
   DT:boolean */
extern boolean_T TTG_PrevLnLengResetUnitDelay_bool; /* '<S667>/Unit Delay' */
extern boolean_T TTG_Reset_bool;                    /* '<S798>/OR1' */

/* Debug */
extern real32_T TTG_RiCorridorCrvChngUnitDelay_1pm2; /* '<S816>/Unit Delay4' */

/* Right corridor curve of change unit delay--Used in TJATTG module
   DT:float32 */
extern real32_T TTG_RiCorridorCrvUnitDelay_1pm; /* '<S816>/Unit Delay3' */

/* Right corridor curve Unit Delay--Used in TJATTG module
   DT:float32 */
extern real32_T TTG_RiCorridorHeadingUnitDelay_rad; /* '<S816>/Unit Delay2' */

/* Right corridor heading Unit Delay--Used in TJATTG module
   DT:float32 */
extern real32_T TTG_RiCorridorLengthUnitDelay_met; /* '<S816>/Unit Delay5' */

/* Right corridor lenght Unit Delay--Used in TJATTG module
   DT:float32 */
extern real32_T TTG_RiCorridorPosX0UnitDelay_met; /* '<S816>/Unit Delay' */

/* Right corridor posX0 Unit Delay--Used in TJATTG module
   DT:float32 */
extern real32_T TTG_RiCorridorPosY0UnitDelay_met; /* '<S816>/Unit Delay1' */

/* Right corridor posY0 Unit Delay--Used in TJATTG module
   DT:float32 */
extern real32_T TTG_RiCrvPredictLowPass_1pm; /* '<S736>/Unit Delay' */

/* Right lane curve predict low pass filter--Used in TJATTG module
   DT:float32 */
extern real32_T TTG_RiLnCrvChngPredictLowPass_1pm2; /* '<S734>/Unit Delay' */

/* Right lane curve of change predict low pass filter--Used in TJATTG module
   DT:float32 */
extern real32_T TTG_RiLnCrvChngUnitDelay_1pm2; /* '<S724>/Unit Delay' */

/* Left lane curve of change freeze output--Used in TJATTG module
   DT:float32 */
extern real32_T TTG_RiLnCrvUnitDelay_1pm; /* '<S725>/Unit Delay' */

/* Left lane curve freeze output--Used in TJATTG module
   DT:float32 */
extern real32_T TTG_RiLnHeadingUnitDelay_rad; /* '<S726>/Unit Delay' */

/* Left lane heading freeze output--Used in TJATTG module
   DT:float32 */
extern real32_T TTG_RiLnLengthUnitDelay_met; /* '<S729>/Unit Delay' */

/* Left lane lenght freeze output--Used in TJATTG module
   DT:float32 */
extern real32_T TTG_RiLnPosY0UnitDelay_met; /* '<S727>/Unit Delay' */

/* Left lane posY0 freeze output--Used in TJATTG module
   DT:float32 */
extern boolean_T
    TTG_RiLnPredictEnable2EdgeRising_bool; /* '<S733>/Unit Delay' */

/* Right lane predict enable 2 flag edge rising--Used in TJATTG module
   DT:boolean */
extern boolean_T TTG_RiLnPredictEnableEdgeRising_bool; /* '<S735>/Unit Delay' */

/* Right lane predict enable flag edge rising--Used in TJATTG module
   DT:boolean */
extern boolean_T TTG_RiLnResetEdgeRising_bool; /* '<S737>/Unit Delay' */

/* Left lane reset flag edge rising--Used in TJATTG module
   DT:boolean */
extern E_TJASTM_LatCtrlMode_nu
    TTG_STMLatCtrlMode2_Enum; /* '<S803>/Unit Delay' */

/* Previous STM lateral control mode 2--Used in TJATTG module
   DT:Enum: E_TJASTM_LatCtrlMode_nu */
extern E_TJASTM_LatCtrlMode_nu
    TTG_STMLatCtrlMode_Enum; /* '<S779>/Unit Delay1' */

/* Previous STM lateral control mode--Used in TJATTG module
   DT:Enum: E_TJASTM_LatCtrlMode_nu */
extern E_TJASTM_SysStateTJA_nu
    TTG_STMSystemState_Enum; /* '<S779>/Unit Delay' */

/* Previous STM system state--Used in TJATTG module
   DT:Enum: E_TJASTM_SysStateTJA_nu */
extern real32_T TTG_TgtCorridorCrvChngUnitDelay_1pm2; /* '<S867>/Unit Delay4' */

/* Target corridor curve of change unit delay--Used in TJATTG module
   DT:float32 */
extern real32_T TTG_TgtCorridorCrvUnitDelay_1pm; /* '<S867>/Unit Delay3' */

/* Target corridor curve Unit Delay--Used in TJATTG module
   DT:float32 */
extern real32_T TTG_TgtCorridorHeadingUnitDelay_rad; /* '<S867>/Unit Delay2' */

/* Target corridor heading Unit Delay--Used in TJATTG module
   DT:float32 */
extern real32_T TTG_TgtCorridorLengthUnitDelay_met; /* '<S867>/Unit Delay5' */

/* Target corridor lenght Unit Delay--Used in TJATTG module
   DT:float32 */
extern real32_T TTG_TgtCorridorPosX0UnitDelay_met; /* '<S867>/Unit Delay' */

/* Target corridor posX0 Unit Delay--Used in TJATTG module
   DT:float32 */
extern real32_T TTG_TgtCorridorPosY0UnitDelay_met; /* '<S867>/Unit Delay1' */

/* Target corridor posY0 Unit Delay--Used in TJATTG module
   DT:float32 */
extern Bus_TgtTrajAndCridrBnd_nu
    TTG_TgtTrajAndCridrBndUnitDelay_bus; /* '<S676>/Unit Delay1' */

/* Target trajectory and corridor boundary unit delay--Used in TJATTG module
   DT:Bus: Bus_TgtTrajAndCridrBnd_nu */
extern real32_T TTG_TransitionFactorAStopwatch_sec; /* '<S801>/Unit Delay' */

/* Transition factor A stopwatch time--Used in TJATTG module
   DT:float32 */
extern real32_T
    TTG_TransitionTimeTurnOffDelayWithRst_sec; /* '<S808>/Unit Delay' */

/* Transition time Turn Off Delay With Rst--Used in TJATTG module
   DT:float32 */
extern real32_T TTG_TransitionTimeUnitDelay_sec; /* '<S804>/Unit Delay' */

/* Transition time Unit Delay--Used in TJATTG module
   DT:float32 */
extern E_TJASLC_AbortState_nu
    TVG_AbortStateUnitDy_Enum; /* '<S888>/Unit Delay2' */

/* Previous driver trigger reset left flag--Used in TJASLC module
   DT:boolean */
extern E_TJASTM_LatCtrlMode_nu TVG_LatCtrlMode_Enum; /* '<S920>/Unit Delay' */

/* Rampout lateral control mode--Used in TJATVG module
   DT:Enum: E_TJASTM_LatCtrlMode_nu */
extern boolean_T TVG_LatMovStartEdgeRising_bool; /* '<S915>/Unit Delay' */

/* move start edge rising--Used in TJATVG module
   DT:boolean */
extern boolean_T TVG_LatMovStartRSFF_bool; /* '<S918>/Unit Delay' */

/* Lateral moving start RSFlipFlop flag--Used in TJATVG module
   DT:boolean */
extern real32_T TVG_PredictionEnableTurnOffDelay_sec; /* '<S965>/Unit Delay' */

/* Prediction enable turn off delay time--Used in TJATVG module
   DT:float32 */
extern boolean_T TVG_PredictionEnableUnitDelay_bool; /* '<S963>/Unit Delay' */

/* Prediction enable Unit Delay--Used in TJATVG module
   DT:boolean */
extern E_TJASTM_LatCtrlMode_nu
    TVG_PrevLatCtlModeUnitDelay_bool; /* '<S890>/Unit Delay' */

/* Previous lateral control mode--Used in TJATVG module
   Enum: E_TJASTM_LatCtrlMode_nu */

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S101>/OR4' : Unused code path elimination
 * Block '<S101>/OR5' : Unused code path elimination
 * Block '<S182>/OR4' : Unused code path elimination
 * Block '<S182>/OR5' : Unused code path elimination
 * Block '<S275>/Constant' : Unused code path elimination
 * Block '<S261>/Equal' : Unused code path elimination
 * Block '<S279>/NOT1' : Unused code path elimination
 * Block '<S299>/Scope1' : Unused code path elimination
 * Block '<S299>/Scope3' : Unused code path elimination
 * Block '<S314>/Scope' : Unused code path elimination
 * Block '<S314>/Scope1' : Unused code path elimination
 * Block '<S314>/Scope2' : Unused code path elimination
 * Block '<S314>/Scope3' : Unused code path elimination
 * Block '<S314>/Scope4' : Unused code path elimination
 * Block '<S314>/Scope5' : Unused code path elimination
 * Block '<S314>/Scope6' : Unused code path elimination
 * Block '<S694>/Constant' : Unused code path elimination
 * Block '<S694>/Product1' : Unused code path elimination
 * Block '<S694>/Product2' : Unused code path elimination
 * Block '<S694>/Product4' : Unused code path elimination
 * Block '<S697>/Constant1' : Unused code path elimination
 * Block '<S697>/Product3' : Unused code path elimination
 * Block '<S697>/Product4' : Unused code path elimination
 * Block '<S697>/Product5' : Unused code path elimination
 * Block '<S698>/Constant1' : Unused code path elimination
 * Block '<S698>/Product3' : Unused code path elimination
 * Block '<S698>/Product4' : Unused code path elimination
 * Block '<S698>/Product5' : Unused code path elimination
 * Block '<S711>/Constant' : Unused code path elimination
 * Block '<S711>/Product1' : Unused code path elimination
 * Block '<S711>/Product2' : Unused code path elimination
 * Block '<S711>/Product4' : Unused code path elimination
 * Block '<S720>/AND' : Unused code path elimination
 * Block '<S720>/NOT' : Unused code path elimination
 * Block '<S714>/Constant1' : Unused code path elimination
 * Block '<S714>/Product3' : Unused code path elimination
 * Block '<S714>/Product4' : Unused code path elimination
 * Block '<S714>/Product5' : Unused code path elimination
 * Block '<S715>/Constant1' : Unused code path elimination
 * Block '<S715>/Product3' : Unused code path elimination
 * Block '<S715>/Product4' : Unused code path elimination
 * Block '<S715>/Product5' : Unused code path elimination
 * Block '<S728>/Constant' : Unused code path elimination
 * Block '<S728>/Product1' : Unused code path elimination
 * Block '<S728>/Product2' : Unused code path elimination
 * Block '<S728>/Product4' : Unused code path elimination
 * Block '<S731>/Constant1' : Unused code path elimination
 * Block '<S731>/Product3' : Unused code path elimination
 * Block '<S731>/Product4' : Unused code path elimination
 * Block '<S731>/Product5' : Unused code path elimination
 * Block '<S732>/Constant1' : Unused code path elimination
 * Block '<S732>/Product3' : Unused code path elimination
 * Block '<S732>/Product4' : Unused code path elimination
 * Block '<S732>/Product5' : Unused code path elimination
 * Block '<S888>/AND4' : Unused code path elimination
 * Block '<S895>/Constant' : Unused code path elimination
 * Block '<S897>/Constant' : Unused code path elimination
 * Block '<S911>/Constant' : Unused code path elimination
 * Block '<S888>/Equal3' : Unused code path elimination
 * Block '<S888>/Equal4' : Unused code path elimination
 * Block '<S888>/Equal7' : Unused code path elimination
 * Block '<S948>/Constant' : Unused code path elimination
 * Block '<S29>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S15>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S46>/Signal Conversion5' : Eliminate redundant signal conversion
 * block Block '<S52>/Signal Conversion16' : Eliminate redundant signal
 * conversion block Block '<S53>/Signal Conversion16' : Eliminate redundant
 * signal conversion block Block '<S54>/Signal Conversion1' : Eliminate
 * redundant signal conversion block Block '<S111>/Signal Conversion' :
 * Eliminate redundant signal conversion block Block '<S101>/Signal Conversion1'
 * : Eliminate redundant signal conversion block Block '<S102>/Signal
 * Conversion2' : Eliminate redundant signal conversion block Block
 * '<S186>/Signal Conversion3' : Eliminate redundant signal conversion block
 * Block '<S182>/Signal Conversion7' : Eliminate redundant signal conversion
 * block Block '<S207>/Signal Conversion8' : Eliminate redundant signal
 * conversion block Block '<S231>/Signal Conversion' : Eliminate redundant
 * signal conversion block Block '<S231>/Signal Conversion1' : Eliminate
 * redundant signal conversion block Block '<S235>/Signal Conversion1' :
 * Eliminate redundant signal conversion block Block '<S235>/Signal Conversion2'
 * : Eliminate redundant signal conversion block Block '<S184>/Signal
 * Conversion2' : Eliminate redundant signal conversion block Block
 * '<S261>/Signal Conversion9' : Eliminate redundant signal conversion block
 * Block '<S262>/Signal Conversion' : Eliminate redundant signal conversion
 * block Block '<S262>/Signal Conversion1' : Eliminate redundant signal
 * conversion block Block '<S327>/Signal Conversion' : Eliminate redundant
 * signal conversion block Block '<S318>/Signal Conversion' : Eliminate
 * redundant signal conversion block Block '<S318>/Signal Conversion1' :
 * Eliminate redundant signal conversion block Block '<S461>/Signal
 * Conversion16' : Eliminate redundant signal conversion block Block
 * '<S461>/Signal Conversion17' : Eliminate redundant signal conversion block
 * Block '<S487>/Signal Conversion' : Eliminate redundant signal conversion
 * block Block '<S514>/Signal Conversion16' : Eliminate redundant signal
 * conversion block Block '<S8>/Signal Conversion3' : Eliminate redundant signal
 * conversion block Block '<S10>/Signal Conversion' : Eliminate redundant signal
 * conversion block Block '<S10>/Signal Conversion1' : Eliminate redundant
 * signal conversion block Block '<S10>/Signal Conversion10' : Eliminate
 * redundant signal conversion block Block '<S10>/Signal Conversion11' :
 * Eliminate redundant signal conversion block Block '<S10>/Signal Conversion2'
 * : Eliminate redundant signal conversion block Block '<S10>/Signal
 * Conversion3' : Eliminate redundant signal conversion block Block
 * '<S10>/Signal Conversion4' : Eliminate redundant signal conversion block
 * Block '<S10>/Signal Conversion5' : Eliminate redundant signal conversion
 * block Block '<S10>/Signal Conversion6' : Eliminate redundant signal
 * conversion block Block '<S10>/Signal Conversion7' : Eliminate redundant
 * signal conversion block Block '<S10>/Signal Conversion8' : Eliminate
 * redundant signal conversion block Block '<S10>/Signal Conversion9' :
 * Eliminate redundant signal conversion block Block '<S666>/Signal
 * Conversion17' : Eliminate redundant signal conversion block Block
 * '<S687>/Signal Conversion1' : Eliminate redundant signal conversion block
 * Block '<S687>/Signal Conversion10' : Eliminate redundant signal conversion
 * block Block '<S687>/Signal Conversion11' : Eliminate redundant signal
 * conversion block Block '<S687>/Signal Conversion12' : Eliminate redundant
 * signal conversion block Block '<S687>/Signal Conversion13' : Eliminate
 * redundant signal conversion block Block '<S687>/Signal Conversion14' :
 * Eliminate redundant signal conversion block Block '<S687>/Signal
 * Conversion15' : Eliminate redundant signal conversion block Block
 * '<S687>/Signal Conversion16' : Eliminate redundant signal conversion block
 * Block '<S687>/Signal Conversion2' : Eliminate redundant signal conversion
 * block Block '<S687>/Signal Conversion3' : Eliminate redundant signal
 * conversion block Block '<S687>/Signal Conversion4' : Eliminate redundant
 * signal conversion block Block '<S687>/Signal Conversion5' : Eliminate
 * redundant signal conversion block Block '<S687>/Signal Conversion6' :
 * Eliminate redundant signal conversion block Block '<S687>/Signal Conversion7'
 * : Eliminate redundant signal conversion block Block '<S687>/Signal
 * Conversion8' : Eliminate redundant signal conversion block Block
 * '<S687>/Signal Conversion9' : Eliminate redundant signal conversion block
 * Block '<S704>/Signal Conversion1' : Eliminate redundant signal conversion
 * block Block '<S704>/Signal Conversion10' : Eliminate redundant signal
 * conversion block Block '<S704>/Signal Conversion11' : Eliminate redundant
 * signal conversion block Block '<S704>/Signal Conversion12' : Eliminate
 * redundant signal conversion block Block '<S704>/Signal Conversion13' :
 * Eliminate redundant signal conversion block Block '<S704>/Signal
 * Conversion14' : Eliminate redundant signal conversion block Block
 * '<S704>/Signal Conversion15' : Eliminate redundant signal conversion block
 * Block '<S704>/Signal Conversion16' : Eliminate redundant signal conversion
 * block Block '<S704>/Signal Conversion2' : Eliminate redundant signal
 * conversion block Block '<S704>/Signal Conversion3' : Eliminate redundant
 * signal conversion block Block '<S704>/Signal Conversion4' : Eliminate
 * redundant signal conversion block Block '<S704>/Signal Conversion5' :
 * Eliminate redundant signal conversion block Block '<S704>/Signal Conversion6'
 * : Eliminate redundant signal conversion block Block '<S704>/Signal
 * Conversion7' : Eliminate redundant signal conversion block Block
 * '<S704>/Signal Conversion8' : Eliminate redundant signal conversion block
 * Block '<S704>/Signal Conversion9' : Eliminate redundant signal conversion
 * block Block '<S721>/Signal Conversion1' : Eliminate redundant signal
 * conversion block Block '<S721>/Signal Conversion10' : Eliminate redundant
 * signal conversion block Block '<S721>/Signal Conversion11' : Eliminate
 * redundant signal conversion block Block '<S721>/Signal Conversion12' :
 * Eliminate redundant signal conversion block Block '<S721>/Signal
 * Conversion13' : Eliminate redundant signal conversion block Block
 * '<S721>/Signal Conversion14' : Eliminate redundant signal conversion block
 * Block '<S721>/Signal Conversion15' : Eliminate redundant signal conversion
 * block Block '<S721>/Signal Conversion16' : Eliminate redundant signal
 * conversion block Block '<S721>/Signal Conversion2' : Eliminate redundant
 * signal conversion block Block '<S721>/Signal Conversion3' : Eliminate
 * redundant signal conversion block Block '<S721>/Signal Conversion4' :
 * Eliminate redundant signal conversion block Block '<S721>/Signal Conversion5'
 * : Eliminate redundant signal conversion block Block '<S721>/Signal
 * Conversion6' : Eliminate redundant signal conversion block Block
 * '<S721>/Signal Conversion7' : Eliminate redundant signal conversion block
 * Block '<S721>/Signal Conversion8' : Eliminate redundant signal conversion
 * block Block '<S721>/Signal Conversion9' : Eliminate redundant signal
 * conversion block Block '<S685>/Signal Conversion1' : Eliminate redundant
 * signal conversion block Block '<S685>/Signal Conversion2' : Eliminate
 * redundant signal conversion block Block '<S685>/Signal Conversion3' :
 * Eliminate redundant signal conversion block Block '<S685>/Signal Conversion4'
 * : Eliminate redundant signal conversion block Block '<S748>/Signal
 * Conversion1' : Eliminate redundant signal conversion block Block
 * '<S748>/Signal Conversion10' : Eliminate redundant signal conversion block
 * Block '<S748>/Signal Conversion11' : Eliminate redundant signal conversion
 * block Block '<S748>/Signal Conversion12' : Eliminate redundant signal
 * conversion block Block '<S748>/Signal Conversion13' : Eliminate redundant
 * signal conversion block Block '<S748>/Signal Conversion14' : Eliminate
 * redundant signal conversion block Block '<S748>/Signal Conversion15' :
 * Eliminate redundant signal conversion block Block '<S748>/Signal
 * Conversion16' : Eliminate redundant signal conversion block Block
 * '<S748>/Signal Conversion17' : Eliminate redundant signal conversion block
 * Block '<S748>/Signal Conversion18' : Eliminate redundant signal conversion
 * block Block '<S748>/Signal Conversion19' : Eliminate redundant signal
 * conversion block Block '<S748>/Signal Conversion2' : Eliminate redundant
 * signal conversion block Block '<S748>/Signal Conversion20' : Eliminate
 * redundant signal conversion block Block '<S748>/Signal Conversion21' :
 * Eliminate redundant signal conversion block Block '<S748>/Signal
 * Conversion22' : Eliminate redundant signal conversion block Block
 * '<S748>/Signal Conversion23' : Eliminate redundant signal conversion block
 * Block '<S748>/Signal Conversion24' : Eliminate redundant signal conversion
 * block Block '<S748>/Signal Conversion25' : Eliminate redundant signal
 * conversion block Block '<S748>/Signal Conversion26' : Eliminate redundant
 * signal conversion block Block '<S748>/Signal Conversion27' : Eliminate
 * redundant signal conversion block Block '<S748>/Signal Conversion28' :
 * Eliminate redundant signal conversion block Block '<S748>/Signal
 * Conversion29' : Eliminate redundant signal conversion block Block
 * '<S748>/Signal Conversion3' : Eliminate redundant signal conversion block
 * Block '<S748>/Signal Conversion30' : Eliminate redundant signal conversion
 * block Block '<S748>/Signal Conversion4' : Eliminate redundant signal
 * conversion block Block '<S748>/Signal Conversion5' : Eliminate redundant
 * signal conversion block Block '<S748>/Signal Conversion6' : Eliminate
 * redundant signal conversion block Block '<S748>/Signal Conversion7' :
 * Eliminate redundant signal conversion block Block '<S748>/Signal Conversion8'
 * : Eliminate redundant signal conversion block Block '<S748>/Signal
 * Conversion9' : Eliminate redundant signal conversion block Block
 * '<S780>/Signal Conversion1' : Eliminate redundant signal conversion block
 * Block '<S780>/Signal Conversion2' : Eliminate redundant signal conversion
 * block Block '<S780>/Signal Conversion3' : Eliminate redundant signal
 * conversion block Block '<S780>/Signal Conversion7' : Eliminate redundant
 * signal conversion block Block '<S798>/Signal Conversion1' : Eliminate
 * redundant signal conversion block Block '<S798>/Signal Conversion2' :
 * Eliminate redundant signal conversion block Block '<S798>/Signal Conversion3'
 * : Eliminate redundant signal conversion block Block '<S798>/Signal
 * Conversion4' : Eliminate redundant signal conversion block Block
 * '<S805>/Signal Conversion1' : Eliminate redundant signal conversion block
 * Block '<S671>/Signal Conversion' : Eliminate redundant signal conversion
 * block Block '<S671>/Signal Conversion1' : Eliminate redundant signal
 * conversion block Block '<S671>/Signal Conversion2' : Eliminate redundant
 * signal conversion block Block '<S671>/Signal Conversion3' : Eliminate
 * redundant signal conversion block Block '<S671>/Signal Conversion4' :
 * Eliminate redundant signal conversion block Block '<S672>/Signal Conversion'
 * : Eliminate redundant signal conversion block Block '<S672>/Signal
 * Conversion1' : Eliminate redundant signal conversion block Block
 * '<S672>/Signal Conversion2' : Eliminate redundant signal conversion block
 * Block '<S672>/Signal Conversion3' : Eliminate redundant signal conversion
 * block Block '<S672>/Signal Conversion4' : Eliminate redundant signal
 * conversion block Block '<S813>/Signal Conversion1' : Eliminate redundant
 * signal conversion block Block '<S813>/Signal Conversion10' : Eliminate
 * redundant signal conversion block Block '<S813>/Signal Conversion11' :
 * Eliminate redundant signal conversion block Block '<S813>/Signal
 * Conversion12' : Eliminate redundant signal conversion block Block
 * '<S813>/Signal Conversion13' : Eliminate redundant signal conversion block
 * Block '<S813>/Signal Conversion14' : Eliminate redundant signal conversion
 * block Block '<S813>/Signal Conversion15' : Eliminate redundant signal
 * conversion block Block '<S813>/Signal Conversion16' : Eliminate redundant
 * signal conversion block Block '<S813>/Signal Conversion17' : Eliminate
 * redundant signal conversion block Block '<S813>/Signal Conversion18' :
 * Eliminate redundant signal conversion block Block '<S813>/Signal
 * Conversion19' : Eliminate redundant signal conversion block Block
 * '<S813>/Signal Conversion2' : Eliminate redundant signal conversion block
 * Block '<S813>/Signal Conversion20' : Eliminate redundant signal conversion
 * block Block '<S813>/Signal Conversion21' : Eliminate redundant signal
 * conversion block Block '<S813>/Signal Conversion22' : Eliminate redundant
 * signal conversion block Block '<S813>/Signal Conversion23' : Eliminate
 * redundant signal conversion block Block '<S813>/Signal Conversion24' :
 * Eliminate redundant signal conversion block Block '<S813>/Signal
 * Conversion25' : Eliminate redundant signal conversion block Block
 * '<S813>/Signal Conversion26' : Eliminate redundant signal conversion block
 * Block '<S813>/Signal Conversion27' : Eliminate redundant signal conversion
 * block Block '<S813>/Signal Conversion28' : Eliminate redundant signal
 * conversion block Block '<S813>/Signal Conversion29' : Eliminate redundant
 * signal conversion block Block '<S813>/Signal Conversion3' : Eliminate
 * redundant signal conversion block Block '<S813>/Signal Conversion30' :
 * Eliminate redundant signal conversion block Block '<S813>/Signal Conversion4'
 * : Eliminate redundant signal conversion block Block '<S813>/Signal
 * Conversion5' : Eliminate redundant signal conversion block Block
 * '<S813>/Signal Conversion6' : Eliminate redundant signal conversion block
 * Block '<S813>/Signal Conversion7' : Eliminate redundant signal conversion
 * block Block '<S813>/Signal Conversion8' : Eliminate redundant signal
 * conversion block Block '<S813>/Signal Conversion9' : Eliminate redundant
 * signal conversion block Block '<S830>/Signal Conversion' : Eliminate
 * redundant signal conversion block Block '<S830>/Signal Conversion1' :
 * Eliminate redundant signal conversion block Block '<S830>/Signal
 * Conversion10' : Eliminate redundant signal conversion block Block
 * '<S830>/Signal Conversion11' : Eliminate redundant signal conversion block
 * Block '<S830>/Signal Conversion12' : Eliminate redundant signal conversion
 * block Block '<S830>/Signal Conversion13' : Eliminate redundant signal
 * conversion block Block '<S830>/Signal Conversion14' : Eliminate redundant
 * signal conversion block Block '<S830>/Signal Conversion15' : Eliminate
 * redundant signal conversion block Block '<S830>/Signal Conversion16' :
 * Eliminate redundant signal conversion block Block '<S830>/Signal
 * Conversion17' : Eliminate redundant signal conversion block Block
 * '<S830>/Signal Conversion2' : Eliminate redundant signal conversion block
 * Block '<S830>/Signal Conversion3' : Eliminate redundant signal conversion
 * block Block '<S830>/Signal Conversion4' : Eliminate redundant signal
 * conversion block Block '<S830>/Signal Conversion5' : Eliminate redundant
 * signal conversion block Block '<S830>/Signal Conversion6' : Eliminate
 * redundant signal conversion block Block '<S830>/Signal Conversion7' :
 * Eliminate redundant signal conversion block Block '<S830>/Signal Conversion8'
 * : Eliminate redundant signal conversion block Block '<S830>/Signal
 * Conversion9' : Eliminate redundant signal conversion block Block
 * '<S831>/Signal Conversion10' : Eliminate redundant signal conversion block
 * Block '<S831>/Signal Conversion11' : Eliminate redundant signal conversion
 * block Block '<S831>/Signal Conversion12' : Eliminate redundant signal
 * conversion block Block '<S831>/Signal Conversion13' : Eliminate redundant
 * signal conversion block Block '<S831>/Signal Conversion14' : Eliminate
 * redundant signal conversion block Block '<S831>/Signal Conversion15' :
 * Eliminate redundant signal conversion block Block '<S831>/Signal
 * Conversion16' : Eliminate redundant signal conversion block Block
 * '<S831>/Signal Conversion17' : Eliminate redundant signal conversion block
 * Block '<S831>/Signal Conversion18' : Eliminate redundant signal conversion
 * block Block '<S831>/Signal Conversion19' : Eliminate redundant signal
 * conversion block Block '<S831>/Signal Conversion20' : Eliminate redundant
 * signal conversion block Block '<S831>/Signal Conversion3' : Eliminate
 * redundant signal conversion block Block '<S831>/Signal Conversion4' :
 * Eliminate redundant signal conversion block Block '<S831>/Signal Conversion5'
 * : Eliminate redundant signal conversion block Block '<S831>/Signal
 * Conversion6' : Eliminate redundant signal conversion block Block
 * '<S831>/Signal Conversion7' : Eliminate redundant signal conversion block
 * Block '<S831>/Signal Conversion8' : Eliminate redundant signal conversion
 * block Block '<S831>/Signal Conversion9' : Eliminate redundant signal
 * conversion block Block '<S832>/Signal Conversion' : Eliminate redundant
 * signal conversion block Block '<S832>/Signal Conversion1' : Eliminate
 * redundant signal conversion block Block '<S832>/Signal Conversion10' :
 * Eliminate redundant signal conversion block Block '<S832>/Signal
 * Conversion11' : Eliminate redundant signal conversion block Block
 * '<S832>/Signal Conversion2' : Eliminate redundant signal conversion block
 * Block '<S832>/Signal Conversion3' : Eliminate redundant signal conversion
 * block Block '<S832>/Signal Conversion4' : Eliminate redundant signal
 * conversion block Block '<S832>/Signal Conversion5' : Eliminate redundant
 * signal conversion block Block '<S832>/Signal Conversion6' : Eliminate
 * redundant signal conversion block Block '<S832>/Signal Conversion7' :
 * Eliminate redundant signal conversion block Block '<S832>/Signal Conversion8'
 * : Eliminate redundant signal conversion block Block '<S832>/Signal
 * Conversion9' : Eliminate redundant signal conversion block Block
 * '<S834>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S834>/Signal Conversion1' : Eliminate redundant signal conversion
 * block Block '<S834>/Signal Conversion10' : Eliminate redundant signal
 * conversion block Block '<S834>/Signal Conversion11' : Eliminate redundant
 * signal conversion block Block '<S834>/Signal Conversion2' : Eliminate
 * redundant signal conversion block Block '<S834>/Signal Conversion3' :
 * Eliminate redundant signal conversion block Block '<S834>/Signal Conversion4'
 * : Eliminate redundant signal conversion block Block '<S834>/Signal
 * Conversion5' : Eliminate redundant signal conversion block Block
 * '<S834>/Signal Conversion6' : Eliminate redundant signal conversion block
 * Block '<S834>/Signal Conversion7' : Eliminate redundant signal conversion
 * block Block '<S834>/Signal Conversion8' : Eliminate redundant signal
 * conversion block Block '<S834>/Signal Conversion9' : Eliminate redundant
 * signal conversion block Block '<S674>/Signal Conversion1' : Eliminate
 * redundant signal conversion block Block '<S674>/Signal Conversion2' :
 * Eliminate redundant signal conversion block Block '<S674>/Signal Conversion3'
 * : Eliminate redundant signal conversion block Block '<S864>/Signal
 * Conversion' : Eliminate redundant signal conversion block Block
 * '<S864>/Signal Conversion1' : Eliminate redundant signal conversion block
 * Block '<S864>/Signal Conversion10' : Eliminate redundant signal conversion
 * block Block '<S864>/Signal Conversion11' : Eliminate redundant signal
 * conversion block Block '<S864>/Signal Conversion12' : Eliminate redundant
 * signal conversion block Block '<S864>/Signal Conversion13' : Eliminate
 * redundant signal conversion block Block '<S864>/Signal Conversion14' :
 * Eliminate redundant signal conversion block Block '<S864>/Signal
 * Conversion15' : Eliminate redundant signal conversion block Block
 * '<S864>/Signal Conversion16' : Eliminate redundant signal conversion block
 * Block '<S864>/Signal Conversion17' : Eliminate redundant signal conversion
 * block Block '<S864>/Signal Conversion18' : Eliminate redundant signal
 * conversion block Block '<S864>/Signal Conversion19' : Eliminate redundant
 * signal conversion block Block '<S864>/Signal Conversion2' : Eliminate
 * redundant signal conversion block Block '<S864>/Signal Conversion20' :
 * Eliminate redundant signal conversion block Block '<S864>/Signal
 * Conversion21' : Eliminate redundant signal conversion block Block
 * '<S864>/Signal Conversion22' : Eliminate redundant signal conversion block
 * Block '<S864>/Signal Conversion23' : Eliminate redundant signal conversion
 * block Block '<S864>/Signal Conversion24' : Eliminate redundant signal
 * conversion block Block '<S864>/Signal Conversion25' : Eliminate redundant
 * signal conversion block Block '<S864>/Signal Conversion26' : Eliminate
 * redundant signal conversion block Block '<S864>/Signal Conversion27' :
 * Eliminate redundant signal conversion block Block '<S864>/Signal
 * Conversion28' : Eliminate redundant signal conversion block Block
 * '<S864>/Signal Conversion29' : Eliminate redundant signal conversion block
 * Block '<S864>/Signal Conversion3' : Eliminate redundant signal conversion
 * block Block '<S864>/Signal Conversion4' : Eliminate redundant signal
 * conversion block Block '<S864>/Signal Conversion5' : Eliminate redundant
 * signal conversion block Block '<S864>/Signal Conversion6' : Eliminate
 * redundant signal conversion block Block '<S864>/Signal Conversion7' :
 * Eliminate redundant signal conversion block Block '<S864>/Signal Conversion8'
 * : Eliminate redundant signal conversion block Block '<S864>/Signal
 * Conversion9' : Eliminate redundant signal conversion block Block
 * '<S883>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S912>/Signal Conversion' : Eliminate redundant signal conversion
 * block Block '<S892>/Signal Conversion' : Eliminate redundant signal
 * conversion block Block '<S892>/Signal Conversion1' : Eliminate redundant
 * signal conversion block Block '<S892>/Signal Conversion2' : Eliminate
 * redundant signal conversion block Block '<S892>/Signal Conversion3' :
 * Eliminate redundant signal conversion block Block '<S19>/Constant' : Unused
 * code path elimination Block '<S31>/Add' : Unused code path elimination Block
 * '<S31>/Constant1' : Unused code path elimination Block '<S31>/Divide' :
 * Unused code path elimination Block '<S31>/Product' : Unused code path
 * elimination Block '<S31>/Product1' : Unused code path elimination Block
 * '<S31>/Subtract' : Unused code path elimination Block '<S28>/Max' : Unused
 * code path elimination Block '<S59>/Constant1' : Unused code path elimination
 * Block '<S97>/Constant1' : Unused code path elimination
 * Block '<S104>/Constant1' : Unused code path elimination
 * Block '<S124>/Constant1' : Unused code path elimination
 * Block '<S128>/Constant10' : Unused code path elimination
 * Block '<S128>/Constant8' : Unused code path elimination
 * Block '<S128>/Constant9' : Unused code path elimination
 * Block '<S128>/Switch2' : Unused code path elimination
 * Block '<S128>/Switch3' : Unused code path elimination
 * Block '<S185>/Constant2' : Unused code path elimination
 * Block '<S186>/Constant3' : Unused code path elimination
 * Block '<S285>/Constant' : Unused code path elimination
 * Block '<S375>/Constant4' : Unused code path elimination
 * Block '<S599>/Constant1' : Unused code path elimination
 * Block '<S600>/Constant1' : Unused code path elimination
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('TJASA_model/TJASA')    - opens subsystem TJASA_model/TJASA
 * hilite_system('TJASA_model/TJASA/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'TJASA_model'
 * '<S1>'   : 'TJASA_model/TJASA'
 * '<S2>'   : 'TJASA_model/TJASA/TJACMB'
 * '<S3>'   : 'TJASA_model/TJASA/TJAGEN'
 * '<S4>'   : 'TJASA_model/TJASA/TJALKA'
 * '<S5>'   : 'TJASA_model/TJASA/TJAOBF'
 * '<S6>'   : 'TJASA_model/TJASA/TJAOBS'
 * '<S7>'   : 'TJASA_model/TJASA/TJASLC'
 * '<S8>'   : 'TJASA_model/TJASA/TJASTM'
 * '<S9>'   : 'TJASA_model/TJASA/TJATOW'
 * '<S10>'  : 'TJASA_model/TJASA/TJATTG'
 * '<S11>'  : 'TJASA_model/TJASA/TJATVG'
 * '<S12>'  : 'TJASA_model/TJASA/TJACMB/CalculateStdDeviation'
 * '<S13>'  : 'TJASA_model/TJASA/TJACMB/CalculateStdDeviation1'
 * '<S14>'  : 'TJASA_model/TJASA/TJACMB/CalculatedCombinedClothoid'
 * '<S15>'  : 'TJASA_model/TJASA/TJACMB/CombineConditionsCheck'
 * '<S16>'  : 'TJASA_model/TJASA/TJACMB/LaneQualityCheck'
 * '<S17>'  : 'TJASA_model/TJASA/TJACMB/ObjectFollowingOnly'
 * '<S18>'  : 'TJASA_model/TJASA/TJACMB/WeightFactors'
 * '<S19>'  : 'TJASA_model/TJASA/TJACMB/CalculatedCombinedClothoid/CombinedCrv'
 * '<S20>'  :
 * 'TJASA_model/TJASA/TJACMB/CalculatedCombinedClothoid/CombinedCrvChng'
 * '<S21>'  : 'TJASA_model/TJASA/TJACMB/CalculatedCombinedClothoid/CombinedHead'
 * '<S22>'  :
 * 'TJASA_model/TJASA/TJACMB/CalculatedCombinedClothoid/CombinedPosX0'
 * '<S23>'  :
 * 'TJASA_model/TJASA/TJACMB/CalculatedCombinedClothoid/CombinedCrv/AdaptiveLnCrvWeight'
 * '<S24>'  :
 * 'TJASA_model/TJASA/TJACMB/CalculatedCombinedClothoid/CombinedCrv/LinearWeighting'
 * '<S25>'  :
 * 'TJASA_model/TJASA/TJACMB/CalculatedCombinedClothoid/CombinedCrv/LowpassFilter'
 * '<S26>'  :
 * 'TJASA_model/TJASA/TJACMB/CalculatedCombinedClothoid/CombinedCrv/Weight'
 * '<S27>'  :
 * 'TJASA_model/TJASA/TJACMB/CalculatedCombinedClothoid/CombinedCrv/LinearWeighting/DivDflt'
 * '<S28>'  :
 * 'TJASA_model/TJASA/TJACMB/CalculatedCombinedClothoid/CombinedCrv/LowpassFilter/LosPassFilter'
 * '<S29>'  :
 * 'TJASA_model/TJASA/TJACMB/CalculatedCombinedClothoid/CombinedCrv/LowpassFilter/LosPassTransition'
 * '<S30>'  :
 * 'TJASA_model/TJASA/TJACMB/CalculatedCombinedClothoid/CombinedCrv/LowpassFilter/LosPassFilter/EdgeRising'
 * '<S31>'  :
 * 'TJASA_model/TJASA/TJACMB/CalculatedCombinedClothoid/CombinedCrv/LowpassFilter/LosPassFilter/LowPassFilter'
 * '<S32>'  :
 * 'TJASA_model/TJASA/TJACMB/CalculatedCombinedClothoid/CombinedCrv/LowpassFilter/LosPassTransition/EdgeFalling'
 * '<S33>'  :
 * 'TJASA_model/TJASA/TJACMB/CalculatedCombinedClothoid/CombinedCrv/LowpassFilter/LosPassTransition/LosPass'
 * '<S34>'  :
 * 'TJASA_model/TJASA/TJACMB/CalculatedCombinedClothoid/CombinedCrv/LowpassFilter/LosPassTransition/STOPWATCH_RE'
 * '<S35>'  :
 * 'TJASA_model/TJASA/TJACMB/CalculatedCombinedClothoid/CombinedCrv/LowpassFilter/LosPassTransition/TurnOffDelay'
 * '<S36>'  :
 * 'TJASA_model/TJASA/TJACMB/CalculatedCombinedClothoid/CombinedCrv/LowpassFilter/LosPassTransition/LosPass/EdgeRising'
 * '<S37>'  :
 * 'TJASA_model/TJASA/TJACMB/CalculatedCombinedClothoid/CombinedCrv/LowpassFilter/LosPassTransition/LosPass/LowPassFilter'
 * '<S38>'  : 'TJASA_model/TJASA/TJACMB/CombineConditionsCheck/MappingUint16'
 * '<S39>'  : 'TJASA_model/TJASA/TJACMB/CombineConditionsCheck/VelocityCheck'
 * '<S40>'  :
 * 'TJASA_model/TJASA/TJACMB/CombineConditionsCheck/MappingUint16/Set_bit'
 * '<S41>'  :
 * 'TJASA_model/TJASA/TJACMB/CombineConditionsCheck/VelocityCheck/Hysteresis'
 * '<S42>'  :
 * 'TJASA_model/TJASA/TJACMB/CombineConditionsCheck/VelocityCheck/Hysteresis1'
 * '<S43>'  : 'TJASA_model/TJASA/TJACMB/LaneQualityCheck/RSFlipFlop'
 * '<S44>'  : 'TJASA_model/TJASA/TJACMB/LaneQualityCheck/TurnOffDelay'
 * '<S45>'  : 'TJASA_model/TJASA/TJACMB/LaneQualityCheck/TurnOnDelay'
 * '<S46>'  : 'TJASA_model/TJASA/TJAGEN/CancelConditions'
 * '<S47>'  : 'TJASA_model/TJASA/TJAGEN/Clearance'
 * '<S48>'  : 'TJASA_model/TJASA/TJAGEN/CodedFunctionAndFunctionSwitch'
 * '<S49>'  : 'TJASA_model/TJASA/TJAGEN/CustomSpecificQualifierCheck'
 * '<S50>'  : 'TJASA_model/TJASA/TJAGEN/DegradationCheck'
 * '<S51>'  : 'TJASA_model/TJASA/TJAGEN/ErrorState'
 * '<S52>'  : 'TJASA_model/TJASA/TJAGEN/FunctionSuspendedAndQuit'
 * '<S53>'  : 'TJASA_model/TJASA/TJAGEN/StrongReadyConditions'
 * '<S54>'  : 'TJASA_model/TJASA/TJAGEN/WeakReadyConditions'
 * '<S55>'  : 'TJASA_model/TJASA/TJAGEN/CancelConditions/MappingUint8'
 * '<S56>'  : 'TJASA_model/TJASA/TJAGEN/CancelConditions/MappingUint8/Set_bit'
 * '<S57>'  : 'TJASA_model/TJASA/TJAGEN/Clearance/Enumerated Constant'
 * '<S58>'  : 'TJASA_model/TJASA/TJAGEN/DegradationCheck/Enumerated Constant'
 * '<S59>'  : 'TJASA_model/TJASA/TJAGEN/DegradationCheck/RampoutWatchDog'
 * '<S60>'  :
 * 'TJASA_model/TJASA/TJAGEN/DegradationCheck/RampoutWatchDog/RSFlipFlop'
 * '<S61>'  :
 * 'TJASA_model/TJASA/TJAGEN/DegradationCheck/RampoutWatchDog/TimerRetrigger'
 * '<S62>'  :
 * 'TJASA_model/TJASA/TJAGEN/FunctionSuspendedAndQuit/BrakePadelInvalid'
 * '<S63>'  :
 * 'TJASA_model/TJASA/TJAGEN/FunctionSuspendedAndQuit/DrvNotBuckledUp'
 * '<S64>'  : 'TJASA_model/TJASA/TJAGEN/FunctionSuspendedAndQuit/HODInvalid'
 * '<S65>'  :
 * 'TJASA_model/TJASA/TJAGEN/FunctionSuspendedAndQuit/ManualTorqueSuspended'
 * '<S66>'  : 'TJASA_model/TJASA/TJAGEN/FunctionSuspendedAndQuit/MappingUint16'
 * '<S67>'  :
 * 'TJASA_model/TJASA/TJAGEN/FunctionSuspendedAndQuit/SignalHazardInvalid'
 * '<S68>'  :
 * 'TJASA_model/TJASA/TJAGEN/FunctionSuspendedAndQuit/SteerWAngleGradlInvalid'
 * '<S69>'  :
 * 'TJASA_model/TJASA/TJAGEN/FunctionSuspendedAndQuit/SteerWAnglelInvalid'
 * '<S70>'  : 'TJASA_model/TJASA/TJAGEN/FunctionSuspendedAndQuit/VehCrvlInvalid'
 * '<S71>'  : 'TJASA_model/TJASA/TJAGEN/FunctionSuspendedAndQuit/VehStlInvalid'
 * '<S72>'  :
 * 'TJASA_model/TJASA/TJAGEN/FunctionSuspendedAndQuit/VehYawRatelInvalid'
 * '<S73>'  :
 * 'TJASA_model/TJASA/TJAGEN/FunctionSuspendedAndQuit/HODInvalid/RiseDelay1'
 * '<S74>'  :
 * 'TJASA_model/TJASA/TJAGEN/FunctionSuspendedAndQuit/ManualTorqueSuspended/Hysteresis'
 * '<S75>'  :
 * 'TJASA_model/TJASA/TJAGEN/FunctionSuspendedAndQuit/ManualTorqueSuspended/Hysteresis1'
 * '<S76>'  :
 * 'TJASA_model/TJASA/TJAGEN/FunctionSuspendedAndQuit/ManualTorqueSuspended/RiseDelay'
 * '<S77>'  :
 * 'TJASA_model/TJASA/TJAGEN/FunctionSuspendedAndQuit/ManualTorqueSuspended/RiseDelay1'
 * '<S78>'  :
 * 'TJASA_model/TJASA/TJAGEN/FunctionSuspendedAndQuit/MappingUint16/Set_bit'
 * '<S79>'  :
 * 'TJASA_model/TJASA/TJAGEN/FunctionSuspendedAndQuit/SignalHazardInvalid/RiseDelay1'
 * '<S80>'  :
 * 'TJASA_model/TJASA/TJAGEN/FunctionSuspendedAndQuit/SteerWAngleGradlInvalid/Hysteresis1'
 * '<S81>'  :
 * 'TJASA_model/TJASA/TJAGEN/FunctionSuspendedAndQuit/SteerWAngleGradlInvalid/Hysteresis2'
 * '<S82>'  :
 * 'TJASA_model/TJASA/TJAGEN/FunctionSuspendedAndQuit/SteerWAnglelInvalid/Hysteresis1'
 * '<S83>'  :
 * 'TJASA_model/TJASA/TJAGEN/FunctionSuspendedAndQuit/SteerWAnglelInvalid/Hysteresis2'
 * '<S84>'  :
 * 'TJASA_model/TJASA/TJAGEN/FunctionSuspendedAndQuit/VehCrvlInvalid/Hysteresis4'
 * '<S85>'  :
 * 'TJASA_model/TJASA/TJAGEN/FunctionSuspendedAndQuit/VehStlInvalid/RiseDelay2'
 * '<S86>'  :
 * 'TJASA_model/TJASA/TJAGEN/FunctionSuspendedAndQuit/VehStlInvalid/RiseDelay3'
 * '<S87>'  :
 * 'TJASA_model/TJASA/TJAGEN/FunctionSuspendedAndQuit/VehYawRatelInvalid/Hysteresis3'
 * '<S88>'  :
 * 'TJASA_model/TJASA/TJAGEN/StrongReadyConditions/CheckLongAndLatAclValidity'
 * '<S89>'  :
 * 'TJASA_model/TJASA/TJAGEN/StrongReadyConditions/CheckStatesOfVehicleSafetyFunctions'
 * '<S90>'  : 'TJASA_model/TJASA/TJAGEN/StrongReadyConditions/DriverStateCheck'
 * '<S91>'  : 'TJASA_model/TJASA/TJAGEN/StrongReadyConditions/MappingUint8'
 * '<S92>'  : 'TJASA_model/TJASA/TJAGEN/StrongReadyConditions/VehicleStateCheck'
 * '<S93>'  :
 * 'TJASA_model/TJASA/TJAGEN/StrongReadyConditions/CheckLongAndLatAclValidity/Hysteresis'
 * '<S94>'  :
 * 'TJASA_model/TJASA/TJAGEN/StrongReadyConditions/CheckLongAndLatAclValidity/Hysteresis1'
 * '<S95>'  :
 * 'TJASA_model/TJASA/TJAGEN/StrongReadyConditions/CheckLongAndLatAclValidity/Hysteresis2'
 * '<S96>'  :
 * 'TJASA_model/TJASA/TJAGEN/StrongReadyConditions/MappingUint8/Set_bit'
 * '<S97>'  :
 * 'TJASA_model/TJASA/TJAGEN/WeakReadyConditions/BlockingTimeConditionCheck'
 * '<S98>'  : 'TJASA_model/TJASA/TJAGEN/WeakReadyConditions/MappingUint8'
 * '<S99>'  :
 * 'TJASA_model/TJASA/TJAGEN/WeakReadyConditions/BlockingTimeConditionCheck/TimerRetrigger'
 * '<S100>' :
 * 'TJASA_model/TJASA/TJAGEN/WeakReadyConditions/MappingUint8/Set_bit'
 * '<S101>' : 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions'
 * '<S102>' : 'TJASA_model/TJASA/TJALKA/LaneValidation'
 * '<S103>' : 'TJASA_model/TJASA/TJALKA/TakeOverCheck'
 * '<S104>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/BlockingTimeConditionsCheck'
 * '<S105>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/CheckTrajPlanStatusQualifier'
 * '<S106>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/ConstructionSiteCheck'
 * '<S107>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/CustomSpecificQualifiersCheck'
 * '<S108>' : 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/Enumerated
 * Constant'
 * '<S109>' : 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/Enumerated
 * Constant1'
 * '<S110>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/LaneInCaptureRangeCheck'
 * '<S111>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/LaneIncoherenceWidthInvalid'
 * '<S112>' : 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/LaneWidthCheck'
 * '<S113>' : 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/MappingUint16'
 * '<S114>' : 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/MinRadiusCheck'
 * '<S115>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/MinValidLengthCheck'
 * '<S116>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/MinimumDistanceToLaneBoundaryCheck'
 * '<S117>' : 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/NoLaneChange'
 * '<S118>' : 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/TurnSignalCheck'
 * '<S119>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/VehicleVelocityCheck_LaneCentering'
 * '<S120>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/VehicleVelocityWRCheck_LaneCentering'
 * '<S121>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/BlockingTimeConditionsCheck/TimerRetrigger'
 * '<S122>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/CheckTrajPlanStatusQualifier/Enumerated
 * Constant'
 * '<S123>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/CheckTrajPlanStatusQualifier/Enumerated
 * Constant1'
 * '<S124>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/CheckTrajPlanStatusQualifier/RSFlipFlop'
 * '<S125>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/LaneIncoherenceWidthInvalid/EdgeRising'
 * '<S126>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/LaneIncoherenceWidthInvalid/EgoVehicleDistanceChk'
 * '<S127>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/LaneIncoherenceWidthInvalid/Enumerated
 * Constant'
 * '<S128>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/LaneIncoherenceWidthInvalid/LnIncoherenceStatus'
 * '<S129>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/LaneIncoherenceWidthInvalid/MappingUint'
 * '<S130>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/LaneIncoherenceWidthInvalid/TurnOffDelay'
 * '<S131>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/LaneIncoherenceWidthInvalid/TurnOffDelay1'
 * '<S132>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/LaneIncoherenceWidthInvalid/TurnOffDelay2'
 * '<S133>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/LaneIncoherenceWidthInvalid/MappingUint/Set_bit'
 * '<S134>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/LaneWidthCheck/Hysteresis'
 * '<S135>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/LaneWidthCheck/Hysteresis1'
 * '<S136>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/MappingUint16/Set_bit'
 * '<S137>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/MinRadiusCheck/Hysteresis'
 * '<S138>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/MinValidLengthCheck/Hysteresis1'
 * '<S139>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/MinimumDistanceToLaneBoundaryCheck/Enumerated
 * Constant4'
 * '<S140>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/MinimumDistanceToLaneBoundaryCheck/Enumerated
 * Constant5'
 * '<S141>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/MinimumDistanceToLaneBoundaryCheck/Enumerated
 * Constant6'
 * '<S142>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/MinimumDistanceToLaneBoundaryCheck/Hysteresis'
 * '<S143>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/NoLaneChange/Enumerated
 * Constant'
 * '<S144>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/TurnSignalCheck/EdgeFalling'
 * '<S145>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/TurnSignalCheck/Enumerated
 * Constant'
 * '<S146>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/VehicleVelocityCheck_LaneCentering/Hysteresis'
 * '<S147>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/VehicleVelocityCheck_LaneCentering/Hysteresis1'
 * '<S148>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/VehicleVelocityWRCheck_LaneCentering/Hysteresis'
 * '<S149>' :
 * 'TJASA_model/TJASA/TJALKA/LaneCenteringConditions/VehicleVelocityWRCheck_LaneCentering/Hysteresis1'
 * '<S150>' : 'TJASA_model/TJASA/TJALKA/LaneValidation/BothSideBridging'
 * '<S151>' : 'TJASA_model/TJASA/TJALKA/LaneValidation/LaneBoundaryQualifier'
 * '<S152>' : 'TJASA_model/TJASA/TJALKA/LaneValidation/LaneCrvQualityCheck'
 * '<S153>' : 'TJASA_model/TJASA/TJALKA/LaneValidation/LaneQualifierCheck'
 * '<S154>' : 'TJASA_model/TJASA/TJALKA/LaneValidation/LaneQualityCheck'
 * '<S155>' : 'TJASA_model/TJASA/TJALKA/LaneValidation/MappingUint1'
 * '<S156>' : 'TJASA_model/TJASA/TJALKA/LaneValidation/RiseDelay1'
 * '<S157>' : 'TJASA_model/TJASA/TJALKA/LaneValidation/RiseDelay2'
 * '<S158>' :
 * 'TJASA_model/TJASA/TJALKA/LaneValidation/BothSideBridging/EdgeRising'
 * '<S159>' :
 * 'TJASA_model/TJASA/TJALKA/LaneValidation/BothSideBridging/Enumerated
 * Constant'
 * '<S160>' :
 * 'TJASA_model/TJASA/TJALKA/LaneValidation/BothSideBridging/Enumerated
 * Constant1'
 * '<S161>' :
 * 'TJASA_model/TJASA/TJALKA/LaneValidation/BothSideBridging/Enumerated
 * Constant2'
 * '<S162>' :
 * 'TJASA_model/TJASA/TJALKA/LaneValidation/BothSideBridging/Enumerated
 * Constant3'
 * '<S163>' :
 * 'TJASA_model/TJASA/TJALKA/LaneValidation/BothSideBridging/Enumerated
 * Constant4'
 * '<S164>' :
 * 'TJASA_model/TJASA/TJALKA/LaneValidation/BothSideBridging/PredictionTime'
 * '<S165>' :
 * 'TJASA_model/TJASA/TJALKA/LaneValidation/BothSideBridging/RSFlipFlop'
 * '<S166>' :
 * 'TJASA_model/TJASA/TJALKA/LaneValidation/BothSideBridging/TurnOnDelay'
 * '<S167>' :
 * 'TJASA_model/TJASA/TJALKA/LaneValidation/BothSideBridging/PredictionTime/EdgeFalling'
 * '<S168>' :
 * 'TJASA_model/TJASA/TJALKA/LaneValidation/BothSideBridging/PredictionTime/TurnOffDelay'
 * '<S169>' :
 * 'TJASA_model/TJASA/TJALKA/LaneValidation/LaneBoundaryQualifier/Enumerated
 * Constant'
 * '<S170>' :
 * 'TJASA_model/TJASA/TJALKA/LaneValidation/LaneBoundaryQualifier/Enumerated
 * Constant1'
 * '<S171>' :
 * 'TJASA_model/TJASA/TJALKA/LaneValidation/LaneBoundaryQualifier/Enumerated
 * Constant2'
 * '<S172>' :
 * 'TJASA_model/TJASA/TJALKA/LaneValidation/LaneBoundaryQualifier/Enumerated
 * Constant3'
 * '<S173>' :
 * 'TJASA_model/TJASA/TJALKA/LaneValidation/LaneCrvQualityCheck/RSFlipFlop'
 * '<S174>' :
 * 'TJASA_model/TJASA/TJALKA/LaneValidation/LaneCrvQualityCheck/TurnOffDelay'
 * '<S175>' :
 * 'TJASA_model/TJASA/TJALKA/LaneValidation/LaneCrvQualityCheck/TurnOnDelay'
 * '<S176>' :
 * 'TJASA_model/TJASA/TJALKA/LaneValidation/LaneQualifierCheck/EdgeRising'
 * '<S177>' :
 * 'TJASA_model/TJASA/TJALKA/LaneValidation/LaneQualifierCheck/TurnOffDelay'
 * '<S178>' :
 * 'TJASA_model/TJASA/TJALKA/LaneValidation/LaneQualityCheck/RSFlipFlop'
 * '<S179>' :
 * 'TJASA_model/TJASA/TJALKA/LaneValidation/LaneQualityCheck/TurnOffDelay'
 * '<S180>' :
 * 'TJASA_model/TJASA/TJALKA/LaneValidation/LaneQualityCheck/TurnOnDelay'
 * '<S181>' : 'TJASA_model/TJASA/TJALKA/LaneValidation/MappingUint1/Set_bit'
 * '<S182>' : 'TJASA_model/TJASA/TJAOBF/ObjectFollowingConditions'
 * '<S183>' : 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck'
 * '<S184>' : 'TJASA_model/TJASA/TJAOBF/TargetClothoidValidityCheck'
 * '<S185>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectFollowingConditions/BlockTimeConditionCheck'
 * '<S186>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectFollowingConditions/CheckForObjectBridingValidity'
 * '<S187>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectFollowingConditions/ConstructionSiteCheck'
 * '<S188>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectFollowingConditions/CustomSpecificQualifierCheck'
 * '<S189>' : 'TJASA_model/TJASA/TJAOBF/ObjectFollowingConditions/Enumerated
 * Constant'
 * '<S190>' : 'TJASA_model/TJASA/TJAOBF/ObjectFollowingConditions/Enumerated
 * Constant1'
 * '<S191>' : 'TJASA_model/TJASA/TJAOBF/ObjectFollowingConditions/MappingUint16'
 * '<S192>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectFollowingConditions/MaxVehVelXCheck_ObjectFollowing'
 * '<S193>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectFollowingConditions/SideCollisionCheck'
 * '<S194>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectFollowingConditions/TurnSignalCheck'
 * '<S195>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectFollowingConditions/BlockTimeConditionCheck/TimerRetrigger'
 * '<S196>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectFollowingConditions/CheckForObjectBridingValidity/Enumerated
 * Constant'
 * '<S197>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectFollowingConditions/CheckForObjectBridingValidity/Enumerated
 * Constant1'
 * '<S198>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectFollowingConditions/CheckForObjectBridingValidity/Timer_Re'
 * '<S199>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectFollowingConditions/CheckForObjectBridingValidity/TurnOnDelay'
 * '<S200>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectFollowingConditions/MappingUint16/Set_bit'
 * '<S201>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectFollowingConditions/MaxVehVelXCheck_ObjectFollowing/Hysteresis'
 * '<S202>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectFollowingConditions/MaxVehVelXCheck_ObjectFollowing/Hysteresis1'
 * '<S203>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectFollowingConditions/SideCollisionCheck/SideCollisionEvaluate'
 * '<S204>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectFollowingConditions/SideCollisionCheck/TurnOffDelay'
 * '<S205>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectFollowingConditions/SideCollisionCheck/SideCollisionEvaluate/TurnOffDelay'
 * '<S206>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/AccObjectValidityCheck'
 * '<S207>' : 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/CreateDebugSignal'
 * '<S208>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/GenerateValidityQualifier'
 * '<S209>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/ObjectInEgoLaneEvaluation'
 * '<S210>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/AccObjectValidityCheck/AccObjSwitch'
 * '<S211>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/AccObjectValidityCheck/AccObjValid'
 * '<S212>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/AccObjectValidityCheck/AccObjValidLaneCheck'
 * '<S213>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/AccObjectValidityCheck/AccObjSwitch/EdgeRising'
 * '<S214>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/AccObjectValidityCheck/AccObjSwitch/TurnOffDelay'
 * '<S215>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/AccObjectValidityCheck/AccObjValid/TurnOnDelay'
 * '<S216>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/AccObjectValidityCheck/AccObjValidLaneCheck/TurnOnDelay'
 * '<S217>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/CreateDebugSignal/MappingUint2'
 * '<S218>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/CreateDebugSignal/MappingUint2/Set_bit'
 * '<S219>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/GenerateValidityQualifier/MinDurationObjInLaneValidity'
 * '<S220>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/GenerateValidityQualifier/QualifierFreezeDuringAccObjChange'
 * '<S221>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/GenerateValidityQualifier/Validity_Debug'
 * '<S222>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/GenerateValidityQualifier/MinDurationObjInLaneValidity/TurnOnDelay'
 * '<S223>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/ObjectInEgoLaneEvaluation/Distance2BoundaryCheck'
 * '<S224>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/ObjectInEgoLaneEvaluation/LaneCheckPreConditions'
 * '<S225>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/ObjectInEgoLaneEvaluation/Distance2BoundaryCheck/CaseAnalysis'
 * '<S226>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/ObjectInEgoLaneEvaluation/Distance2BoundaryCheck/CheckDist2LeftBounday'
 * '<S227>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/ObjectInEgoLaneEvaluation/Distance2BoundaryCheck/CheckDist2RightBounday'
 * '<S228>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/ObjectInEgoLaneEvaluation/Distance2BoundaryCheck/MinimumDistanceToLaneBoundaryCheck'
 * '<S229>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/ObjectInEgoLaneEvaluation/Distance2BoundaryCheck/TargetObjectInEgoLaneCheck'
 * '<S230>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/ObjectInEgoLaneEvaluation/Distance2BoundaryCheck/CheckDist2LeftBounday/CalculateTargetObjectDist2LeftBoundary'
 * '<S231>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/ObjectInEgoLaneEvaluation/Distance2BoundaryCheck/CheckDist2LeftBounday/EvaluateLeftLaneClothoidAtPosX'
 * '<S232>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/ObjectInEgoLaneEvaluation/Distance2BoundaryCheck/CheckDist2LeftBounday/EvaluateLeftLaneClothoidAtPosX/ApproxAtan'
 * '<S233>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/ObjectInEgoLaneEvaluation/Distance2BoundaryCheck/CheckDist2LeftBounday/EvaluateLeftLaneClothoidAtPosX/AvoidUndefinedTangent'
 * '<S234>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/ObjectInEgoLaneEvaluation/Distance2BoundaryCheck/CheckDist2RightBounday/CalculateTargetObjectDist2RightBoundary'
 * '<S235>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/ObjectInEgoLaneEvaluation/Distance2BoundaryCheck/CheckDist2RightBounday/EvaluateRightLaneClothoidAtPosX'
 * '<S236>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/ObjectInEgoLaneEvaluation/Distance2BoundaryCheck/CheckDist2RightBounday/EvaluateRightLaneClothoidAtPosX/ApproxAtan'
 * '<S237>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/ObjectInEgoLaneEvaluation/Distance2BoundaryCheck/CheckDist2RightBounday/EvaluateRightLaneClothoidAtPosX/AvoidUndefinedTangent'
 * '<S238>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/ObjectInEgoLaneEvaluation/Distance2BoundaryCheck/MinimumDistanceToLaneBoundaryCheck/Hysteresis'
 * '<S239>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/ObjectInEgoLaneEvaluation/Distance2BoundaryCheck/MinimumDistanceToLaneBoundaryCheck/Hysteresis1'
 * '<S240>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/ObjectInEgoLaneEvaluation/Distance2BoundaryCheck/TargetObjectInEgoLaneCheck/SwitchCreator'
 * '<S241>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/ObjectInEgoLaneEvaluation/LaneCheckPreConditions/CheckObjectPosXValidity'
 * '<S242>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/ObjectInEgoLaneEvaluation/LaneCheckPreConditions/LaneAttributesValid'
 * '<S243>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/ObjectInEgoLaneEvaluation/LaneCheckPreConditions/LaneAttributesValid/Enumerated
 * Constant'
 * '<S244>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/ObjectInEgoLaneEvaluation/LaneCheckPreConditions/LaneAttributesValid/Enumerated
 * Constant1'
 * '<S245>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/ObjectInEgoLaneEvaluation/LaneCheckPreConditions/LaneAttributesValid/Enumerated
 * Constant2'
 * '<S246>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/ObjectInEgoLaneEvaluation/LaneCheckPreConditions/LaneAttributesValid/Hysteresis'
 * '<S247>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/ObjectInEgoLaneEvaluation/LaneCheckPreConditions/LaneAttributesValid/RiseDelay'
 * '<S248>' :
 * 'TJASA_model/TJASA/TJAOBF/ObjectInLaneCheck/ObjectInEgoLaneEvaluation/LaneCheckPreConditions/LaneAttributesValid/RiseDelay1'
 * '<S249>' :
 * 'TJASA_model/TJASA/TJAOBF/TargetClothoidValidityCheck/MappingUint8'
 * '<S250>' :
 * 'TJASA_model/TJASA/TJAOBF/TargetClothoidValidityCheck/TargetObjBitfieldCheck'
 * '<S251>' :
 * 'TJASA_model/TJASA/TJAOBF/TargetClothoidValidityCheck/TargetObjCurvatureCheck'
 * '<S252>' :
 * 'TJASA_model/TJASA/TJAOBF/TargetClothoidValidityCheck/TargetObjHeadingCheck'
 * '<S253>' :
 * 'TJASA_model/TJASA/TJAOBF/TargetClothoidValidityCheck/TargetObjLengthCheck'
 * '<S254>' :
 * 'TJASA_model/TJASA/TJAOBF/TargetClothoidValidityCheck/TargetObjPosYCheck'
 * '<S255>' :
 * 'TJASA_model/TJASA/TJAOBF/TargetClothoidValidityCheck/MappingUint8/Set_bit'
 * '<S256>' :
 * 'TJASA_model/TJASA/TJAOBF/TargetClothoidValidityCheck/TargetObjBitfieldCheck/TurnOnDelay'
 * '<S257>' :
 * 'TJASA_model/TJASA/TJAOBF/TargetClothoidValidityCheck/TargetObjCurvatureCheck/Hysteresis'
 * '<S258>' :
 * 'TJASA_model/TJASA/TJAOBF/TargetClothoidValidityCheck/TargetObjHeadingCheck/Hysteresis'
 * '<S259>' :
 * 'TJASA_model/TJASA/TJAOBF/TargetClothoidValidityCheck/TargetObjPosYCheck/Hysteresis'
 * '<S260>' : 'TJASA_model/TJASA/TJASLC/AdjacentLaneValidation'
 * '<S261>' : 'TJASA_model/TJASA/TJASLC/CancelAndAbortConditionCheck'
 * '<S262>' : 'TJASA_model/TJASA/TJASLC/Dtrn_AllowGoBack'
 * '<S263>' : 'TJASA_model/TJASA/TJASLC/EgoLaneValidation'
 * '<S264>' : 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine'
 * '<S265>' : 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck'
 * '<S266>' : 'TJASA_model/TJASA/TJASLC/RearObjectValidation'
 * '<S267>' : 'TJASA_model/TJASA/TJASLC/SLCNvRam'
 * '<S268>' : 'TJASA_model/TJASA/TJASLC/TriggerEvaluation'
 * '<S269>' :
 * 'TJASA_model/TJASA/TJASLC/AdjacentLaneValidation/AdjacentLaneValidityCheck'
 * '<S270>' :
 * 'TJASA_model/TJASA/TJASLC/AdjacentLaneValidation/AdjacentLaneWidthCheck'
 * '<S271>' :
 * 'TJASA_model/TJASA/TJASLC/AdjacentLaneValidation/AdjacentLaneWidthCheck/Hysteresis'
 * '<S272>' :
 * 'TJASA_model/TJASA/TJASLC/AdjacentLaneValidation/AdjacentLaneWidthCheck/Hysteresis1'
 * '<S273>' :
 * 'TJASA_model/TJASA/TJASLC/CancelAndAbortConditionCheck/CustomSpecificQualifiersCheck'
 * '<S274>' :
 * 'TJASA_model/TJASA/TJASLC/CancelAndAbortConditionCheck/DriverTriggerResetCheck'
 * '<S275>' : 'TJASA_model/TJASA/TJASLC/CancelAndAbortConditionCheck/Enumerated
 * Constant'
 * '<S276>' :
 * 'TJASA_model/TJASA/TJASLC/CancelAndAbortConditionCheck/FunctionDegradationCheck'
 * '<S277>' :
 * 'TJASA_model/TJASA/TJASLC/CancelAndAbortConditionCheck/LaneCenteringReadyToControlChcek'
 * '<S278>' :
 * 'TJASA_model/TJASA/TJASLC/CancelAndAbortConditionCheck/MappingUint8'
 * '<S279>' :
 * 'TJASA_model/TJASA/TJASLC/CancelAndAbortConditionCheck/RearObjectCheck'
 * '<S280>' :
 * 'TJASA_model/TJASA/TJASLC/CancelAndAbortConditionCheck/TakeOverCheck'
 * '<S281>' :
 * 'TJASA_model/TJASA/TJASLC/CancelAndAbortConditionCheck/DriverTriggerResetCheck/Enumerated
 * Constant'
 * '<S282>' :
 * 'TJASA_model/TJASA/TJASLC/CancelAndAbortConditionCheck/DriverTriggerResetCheck/Enumerated
 * Constant1'
 * '<S283>' :
 * 'TJASA_model/TJASA/TJASLC/CancelAndAbortConditionCheck/DriverTriggerResetCheck/Enumerated
 * Constant2'
 * '<S284>' :
 * 'TJASA_model/TJASA/TJASLC/CancelAndAbortConditionCheck/DriverTriggerResetCheck/Enumerated
 * Constant3'
 * '<S285>' :
 * 'TJASA_model/TJASA/TJASLC/CancelAndAbortConditionCheck/FunctionDegradationCheck/DegradationWatchDog'
 * '<S286>' :
 * 'TJASA_model/TJASA/TJASLC/CancelAndAbortConditionCheck/FunctionDegradationCheck/TJA_Rampout'
 * '<S287>' :
 * 'TJASA_model/TJASA/TJASLC/CancelAndAbortConditionCheck/FunctionDegradationCheck/TakeoverCheck'
 * '<S288>' :
 * 'TJASA_model/TJASA/TJASLC/CancelAndAbortConditionCheck/FunctionDegradationCheck/DegradationWatchDog/Enumerated
 * Constant'
 * '<S289>' :
 * 'TJASA_model/TJASA/TJASLC/CancelAndAbortConditionCheck/FunctionDegradationCheck/DegradationWatchDog/RSFlipFlop'
 * '<S290>' :
 * 'TJASA_model/TJASA/TJASLC/CancelAndAbortConditionCheck/FunctionDegradationCheck/DegradationWatchDog/TimerRetrigger'
 * '<S291>' :
 * 'TJASA_model/TJASA/TJASLC/CancelAndAbortConditionCheck/FunctionDegradationCheck/TJA_Rampout/Enumerated
 * Constant'
 * '<S292>' :
 * 'TJASA_model/TJASA/TJASLC/CancelAndAbortConditionCheck/FunctionDegradationCheck/TakeoverCheck/Enumerated
 * Constant'
 * '<S293>' :
 * 'TJASA_model/TJASA/TJASLC/CancelAndAbortConditionCheck/FunctionDegradationCheck/TakeoverCheck/Enumerated
 * Constant1'
 * '<S294>' :
 * 'TJASA_model/TJASA/TJASLC/CancelAndAbortConditionCheck/MappingUint8/Set_bit'
 * '<S295>' :
 * 'TJASA_model/TJASA/TJASLC/CancelAndAbortConditionCheck/RearObjectCheck/Enumerated
 * Constant'
 * '<S296>' :
 * 'TJASA_model/TJASA/TJASLC/CancelAndAbortConditionCheck/RearObjectCheck/Enumerated
 * Constant1'
 * '<S297>' :
 * 'TJASA_model/TJASA/TJASLC/CancelAndAbortConditionCheck/RearObjectCheck/Enumerated
 * Constant2'
 * '<S298>' :
 * 'TJASA_model/TJASA/TJASLC/CancelAndAbortConditionCheck/RearObjectCheck/Enumerated
 * Constant3'
 * '<S299>' : 'TJASA_model/TJASA/TJASLC/Dtrn_AllowGoBack/DtrmAllowAbort'
 * '<S300>' : 'TJASA_model/TJASA/TJASLC/Dtrn_AllowGoBack/ObserveEgoLaneChange'
 * '<S301>' : 'TJASA_model/TJASA/TJASLC/Dtrn_AllowGoBack/TurnOffDelay'
 * '<S302>' : 'TJASA_model/TJASA/TJASLC/Dtrn_AllowGoBack/TurnOffDelay1'
 * '<S303>' :
 * 'TJASA_model/TJASA/TJASLC/Dtrn_AllowGoBack/ObserveEgoLaneChange/RSFlipFlop'
 * '<S304>' :
 * 'TJASA_model/TJASA/TJASLC/EgoLaneValidation/EgoLaneClthLengthCheck'
 * '<S305>' : 'TJASA_model/TJASA/TJASLC/EgoLaneValidation/EgoLaneCurvatureCheck'
 * '<S306>' : 'TJASA_model/TJASA/TJASLC/EgoLaneValidation/EgoLaneTypeCheck'
 * '<S307>' : 'TJASA_model/TJASA/TJASLC/EgoLaneValidation/EgoLaneValidityCheck'
 * '<S308>' : 'TJASA_model/TJASA/TJASLC/EgoLaneValidation/EgoLaneWidthCheck'
 * '<S309>' :
 * 'TJASA_model/TJASA/TJASLC/EgoLaneValidation/EgoLaneTypeCheck/TurnOffDelay'
 * '<S310>' :
 * 'TJASA_model/TJASA/TJASLC/EgoLaneValidation/EgoLaneTypeCheck/TurnOffDelay1'
 * '<S311>' :
 * 'TJASA_model/TJASA/TJASLC/EgoLaneValidation/EgoLaneValidityCheck/Enumerated
 * Constant'
 * '<S312>' :
 * 'TJASA_model/TJASA/TJASLC/EgoLaneValidation/EgoLaneWidthCheck/Hysteresis'
 * '<S313>' :
 * 'TJASA_model/TJASA/TJASLC/EgoLaneValidation/EgoLaneWidthCheck/Hysteresis1'
 * '<S314>' : 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/AbortState'
 * '<S315>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/GenerateReadyConditionOutputs'
 * '<S316>' : 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LCM_StateCheck'
 * '<S317>' : 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LCPStTransitonChk'
 * '<S318>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover'
 * '<S319>' : 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/ManeuverState'
 * '<S320>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/ManeuverStateFreezeRampout'
 * '<S321>' : 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/PassiveConditions'
 * '<S322>' : 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/SLCAudioPlay'
 * '<S323>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/AbortState/AbortState'
 * '<S324>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/AbortState/DtrmInToAbort'
 * '<S325>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/AbortState/DtrmLaneChangeDirection'
 * '<S326>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/AbortState/LaneChangeDetection'
 * '<S327>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/AbortState/Subsystem'
 * '<S328>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/AbortState/DtrmInToAbort/Enumerated
 * Constant'
 * '<S329>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/AbortState/DtrmInToAbort/Enumerated
 * Constant1'
 * '<S330>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/AbortState/DtrmLaneChangeDirection/Enumerated
 * Constant'
 * '<S331>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/AbortState/DtrmLaneChangeDirection/Enumerated
 * Constant1'
 * '<S332>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/AbortState/DtrmLaneChangeDirection/Enumerated
 * Constant2'
 * '<S333>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/AbortState/LaneChangeDetection/Enumerated
 * Constant1'
 * '<S334>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/AbortState/LaneChangeDetection/Enumerated
 * Constant2'
 * '<S335>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/AbortState/LaneChangeDetection/Enumerated
 * Constant3'
 * '<S336>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/AbortState/LaneChangeDetection/Enumerated
 * Constant4'
 * '<S337>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/AbortState/Subsystem/Enumerated
 * Constant'
 * '<S338>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/AbortState/Subsystem/Enumerated
 * Constant1'
 * '<S339>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/AbortState/Subsystem/Enumerated
 * Constant2'
 * '<S340>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/AbortState/Subsystem/Enumerated
 * Constant3'
 * '<S341>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/AbortState/Subsystem/TurnOnDelay'
 * '<S342>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/AbortState/Subsystem/TurnOnDelay1'
 * '<S343>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/AbortState/Subsystem/TurnOnDelay2'
 * '<S344>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/GenerateReadyConditionOutputs/Enumerated
 * Constant1'
 * '<S345>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/GenerateReadyConditionOutputs/Enumerated
 * Constant10'
 * '<S346>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/GenerateReadyConditionOutputs/Enumerated
 * Constant11'
 * '<S347>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/GenerateReadyConditionOutputs/Enumerated
 * Constant12'
 * '<S348>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/GenerateReadyConditionOutputs/Enumerated
 * Constant13'
 * '<S349>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/GenerateReadyConditionOutputs/Enumerated
 * Constant14'
 * '<S350>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/GenerateReadyConditionOutputs/Enumerated
 * Constant15'
 * '<S351>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/GenerateReadyConditionOutputs/Enumerated
 * Constant16'
 * '<S352>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/GenerateReadyConditionOutputs/Enumerated
 * Constant2'
 * '<S353>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/GenerateReadyConditionOutputs/Enumerated
 * Constant3'
 * '<S354>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/GenerateReadyConditionOutputs/Enumerated
 * Constant4'
 * '<S355>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/GenerateReadyConditionOutputs/Enumerated
 * Constant5'
 * '<S356>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/GenerateReadyConditionOutputs/Enumerated
 * Constant6'
 * '<S357>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/GenerateReadyConditionOutputs/Enumerated
 * Constant7'
 * '<S358>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/GenerateReadyConditionOutputs/Enumerated
 * Constant8'
 * '<S359>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/GenerateReadyConditionOutputs/Enumerated
 * Constant9'
 * '<S360>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/GenerateReadyConditionOutputs/RSFlipFlop1'
 * '<S361>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/GenerateReadyConditionOutputs/RSFlipFlop2'
 * '<S362>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LCM_StateCheck/CalculateFrontWheelDist2LaneBoundary'
 * '<S363>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LCM_StateCheck/CalculateRearWheelDist2LaneBoundary'
 * '<S364>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LCM_StateCheck/LaneCheckValidity'
 * '<S365>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LCM_StateCheck/ObserveEgoLaneChange'
 * '<S366>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LCM_StateCheck/ObserveLCMEnd'
 * '<S367>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LCM_StateCheck/ObserveLCMStart'
 * '<S368>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LCM_StateCheck/LaneCheckValidity/TurnOffDelay'
 * '<S369>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LCM_StateCheck/ObserveEgoLaneChange/RSFlipFlop'
 * '<S370>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LCPStTransitonChk/TurnOnDelay'
 * '<S371>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LCPStTransitonChk/TurnOnDelay1'
 * '<S372>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LCPStTransitonChk/TurnOnDelay2'
 * '<S373>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LCPStTransitonChk/TurnOnDelay3'
 * '<S374>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/DtrnLaneChangeInfo'
 * '<S375>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/LaneChangeWarning'
 * '<S376>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/RSFlipFlop'
 * '<S377>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/RSFlipFlop1'
 * '<S378>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/TakeOverDetermination'
 * '<S379>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/DtrnLaneChangeInfo/DtrnLaneChangCANCLE'
 * '<S380>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/DtrnLaneChangeInfo/DtrnLaneChangEnd'
 * '<S381>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/DtrnLaneChangeInfo/DtrnLaneChangOnGoing'
 * '<S382>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/DtrnLaneChangeInfo/DtrnLaneChangPending'
 * '<S383>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/DtrnLaneChangeInfo/DtrnVehSpdTooLow'
 * '<S384>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/DtrnLaneChangeInfo/DtrnLaneChangCANCLE/EdgeFalling'
 * '<S385>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/DtrnLaneChangeInfo/DtrnLaneChangCANCLE/Enumerated
 * Constant1'
 * '<S386>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/DtrnLaneChangeInfo/DtrnLaneChangCANCLE/Enumerated
 * Constant2'
 * '<S387>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/DtrnLaneChangeInfo/DtrnLaneChangCANCLE/Enumerated
 * Constant3'
 * '<S388>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/DtrnLaneChangeInfo/DtrnLaneChangCANCLE/Enumerated
 * Constant4'
 * '<S389>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/DtrnLaneChangeInfo/DtrnLaneChangCANCLE/Enumerated
 * Constant5'
 * '<S390>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/DtrnLaneChangeInfo/DtrnLaneChangCANCLE/Enumerated
 * Constant6'
 * '<S391>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/DtrnLaneChangeInfo/DtrnLaneChangCANCLE/Enumerated
 * Constant7'
 * '<S392>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/DtrnLaneChangeInfo/DtrnLaneChangCANCLE/Enumerated
 * Constant8'
 * '<S393>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/DtrnLaneChangeInfo/DtrnLaneChangCANCLE/Enumerated
 * Constant9'
 * '<S394>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/DtrnLaneChangeInfo/DtrnLaneChangCANCLE/TurnOffDelay'
 * '<S395>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/DtrnLaneChangeInfo/DtrnLaneChangEnd/Enumerated
 * Constant1'
 * '<S396>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/DtrnLaneChangeInfo/DtrnLaneChangEnd/Enumerated
 * Constant7'
 * '<S397>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/DtrnLaneChangeInfo/DtrnLaneChangEnd/TurnOffDelay'
 * '<S398>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/DtrnLaneChangeInfo/DtrnLaneChangOnGoing/Enumerated
 * Constant1'
 * '<S399>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/DtrnLaneChangeInfo/DtrnLaneChangOnGoing/Enumerated
 * Constant2'
 * '<S400>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/DtrnLaneChangeInfo/DtrnLaneChangOnGoing/Enumerated
 * Constant3'
 * '<S401>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/DtrnLaneChangeInfo/DtrnLaneChangOnGoing/Enumerated
 * Constant7'
 * '<S402>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/DtrnLaneChangeInfo/DtrnLaneChangPending/Enumerated
 * Constant1'
 * '<S403>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/DtrnLaneChangeInfo/DtrnLaneChangPending/Enumerated
 * Constant2'
 * '<S404>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/DtrnLaneChangeInfo/DtrnLaneChangPending/Enumerated
 * Constant7'
 * '<S405>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/DtrnLaneChangeInfo/DtrnLaneChangPending/Enumerated
 * Constant9'
 * '<S406>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/DtrnLaneChangeInfo/DtrnVehSpdTooLow/Enumerated
 * Constant1'
 * '<S407>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/DtrnLaneChangeInfo/DtrnVehSpdTooLow/Enumerated
 * Constant2'
 * '<S408>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/DtrnLaneChangeInfo/DtrnVehSpdTooLow/Enumerated
 * Constant7'
 * '<S409>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/LaneChangeWarning/EdgeRising1'
 * '<S410>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/LaneChangeWarning/EdgeRising2'
 * '<S411>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/LaneChangeWarning/EdgeRising3'
 * '<S412>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/LaneChangeWarning/Enumerated
 * Constant1'
 * '<S413>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/LaneChangeWarning/Enumerated
 * Constant10'
 * '<S414>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/LaneChangeWarning/Enumerated
 * Constant16'
 * '<S415>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/LaneChangeWarning/Enumerated
 * Constant17'
 * '<S416>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/LaneChangeWarning/Enumerated
 * Constant18'
 * '<S417>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/LaneChangeWarning/Enumerated
 * Constant19'
 * '<S418>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/LaneChangeWarning/Enumerated
 * Constant2'
 * '<S419>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/LaneChangeWarning/Enumerated
 * Constant22'
 * '<S420>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/LaneChangeWarning/Enumerated
 * Constant23'
 * '<S421>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/LaneChangeWarning/Enumerated
 * Constant24'
 * '<S422>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/LaneChangeWarning/Enumerated
 * Constant25'
 * '<S423>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/LaneChangeWarning/Enumerated
 * Constant26'
 * '<S424>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/LaneChangeWarning/Enumerated
 * Constant27'
 * '<S425>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/LaneChangeWarning/Enumerated
 * Constant28'
 * '<S426>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/LaneChangeWarning/Enumerated
 * Constant29'
 * '<S427>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/LaneChangeWarning/Enumerated
 * Constant8'
 * '<S428>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/LaneChangeWarning/Enumerated
 * Constant9'
 * '<S429>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/LaneChangeWarning/TimerRetrigger'
 * '<S430>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/LaneChangeWarning/TurnOffDelay'
 * '<S431>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/LaneChangeWarning/TurnOffDelay1'
 * '<S432>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/LaneChangeWarning/TurnOnDelay'
 * '<S433>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/TakeOverDetermination/EdgeRising'
 * '<S434>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/TakeOverDetermination/Enumerated
 * Constant'
 * '<S435>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/TakeOverDetermination/Enumerated
 * Constant1'
 * '<S436>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/TakeOverDetermination/Enumerated
 * Constant2'
 * '<S437>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/TakeOverDetermination/Enumerated
 * Constant20'
 * '<S438>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/TakeOverDetermination/Enumerated
 * Constant21'
 * '<S439>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/TakeOverDetermination/Enumerated
 * Constant3'
 * '<S440>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/TakeOverDetermination/Enumerated
 * Constant4'
 * '<S441>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/TakeOverDetermination/Enumerated
 * Constant5'
 * '<S442>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/TakeOverDetermination/Enumerated
 * Constant6'
 * '<S443>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/TakeOverDetermination/Enumerated
 * Constant7'
 * '<S444>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/TakeOverDetermination/Enumerated
 * Constant9'
 * '<S445>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/TakeOverDetermination/RSFlipFlop1'
 * '<S446>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/TakeOverDetermination/TurnOffDelay'
 * '<S447>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/TakeOverDetermination/TurnOffDelay1'
 * '<S448>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/LaneCenteringTakeover/TakeOverDetermination/TurnOnDelay'
 * '<S449>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/ManeuverStateFreezeRampout/Enumerated
 * Constant'
 * '<S450>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/PassiveConditions/Enumerated
 * Constant'
 * '<S451>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/PassiveConditions/Enumerated
 * Constant1'
 * '<S452>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/PassiveConditions/Enumerated
 * Constant2'
 * '<S453>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/PassiveConditions/Enumerated
 * Constant3'
 * '<S454>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/PassiveConditions/Enumerated
 * Constant4'
 * '<S455>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/SLCAudioPlay/Enumerated
 * Constant'
 * '<S456>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/SLCAudioPlay/Enumerated
 * Constant1'
 * '<S457>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/SLCAudioPlay/Enumerated
 * Constant2'
 * '<S458>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/SLCAudioPlay/Enumerated
 * Constant3'
 * '<S459>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/SLCAudioPlay/Enumerated
 * Constant4'
 * '<S460>' :
 * 'TJASA_model/TJASA/TJASLC/ManeuverStateMachine/SLCAudioPlay/Enumerated
 * Constant5'
 * '<S461>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/DebugSys'
 * '<S462>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTrigger1'
 * '<S463>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTriggerBothSidesConditions'
 * '<S464>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTriggerLeftCheck'
 * '<S465>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTriggerRightCheck'
 * '<S466>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/DebugSys/MappingUint16'
 * '<S467>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/DebugSys/MappingUint16_2'
 * '<S468>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/DebugSys/MappingUint16/Set_bit'
 * '<S469>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/DebugSys/MappingUint16_2/Set_bit'
 * '<S470>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTrigger1/EdgeRising1'
 * '<S471>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTrigger1/Enumerated
 * Constant1'
 * '<S472>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTrigger1/Enumerated
 * Constant10'
 * '<S473>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTrigger1/Enumerated
 * Constant11'
 * '<S474>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTrigger1/Enumerated
 * Constant4'
 * '<S475>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTrigger1/Enumerated
 * Constant5'
 * '<S476>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTrigger1/Enumerated
 * Constant6'
 * '<S477>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTrigger1/Enumerated
 * Constant9'
 * '<S478>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTrigger1/RSFlipFlop'
 * '<S479>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTrigger1/ReadyToTrigger'
 * '<S480>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTrigger1/ReadyToTrigger/Enumerated
 * Constant'
 * '<S481>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTrigger1/ReadyToTrigger/Enumerated
 * Constant1'
 * '<S482>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTrigger1/ReadyToTrigger/Enumerated
 * Constant2'
 * '<S483>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTrigger1/ReadyToTrigger/Enumerated
 * Constant3'
 * '<S484>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTriggerBothSidesConditions/BlockTimeConditionCheck'
 * '<S485>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTriggerBothSidesConditions/CheckLaneCenteringActiveState'
 * '<S486>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTriggerBothSidesConditions/CustomSpecificQualifierCheck'
 * '<S487>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTriggerBothSidesConditions/EgoLaneCheck'
 * '<S488>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTriggerBothSidesConditions/EgoVehicleVelocityCheck'
 * '<S489>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTriggerBothSidesConditions/Enumerated
 * Constant'
 * '<S490>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTriggerBothSidesConditions/BlockTimeConditionCheck/ManualLaneChangeBlocking'
 * '<S491>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTriggerBothSidesConditions/BlockTimeConditionCheck/SALC_Finish'
 * '<S492>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTriggerBothSidesConditions/BlockTimeConditionCheck/TurnOffDelay'
 * '<S493>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTriggerBothSidesConditions/BlockTimeConditionCheck/TurnOffDelay1'
 * '<S494>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTriggerBothSidesConditions/BlockTimeConditionCheck/TurnSignalOffBlocking'
 * '<S495>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTriggerBothSidesConditions/BlockTimeConditionCheck/ManualLaneChangeBlocking/RSFlipFlop'
 * '<S496>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTriggerBothSidesConditions/BlockTimeConditionCheck/ManualLaneChangeBlocking/TurnOffDelay'
 * '<S497>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTriggerBothSidesConditions/BlockTimeConditionCheck/SALC_Finish/Enumerated
 * Constant'
 * '<S498>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTriggerBothSidesConditions/BlockTimeConditionCheck/TurnSignalOffBlocking/Enumerated
 * Constant'
 * '<S499>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTriggerBothSidesConditions/BlockTimeConditionCheck/TurnSignalOffBlocking/RSFlipFlop'
 * '<S500>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTriggerBothSidesConditions/CheckLaneCenteringActiveState/Enumerated
 * Constant'
 * '<S501>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTriggerBothSidesConditions/CheckLaneCenteringActiveState/Enumerated
 * Constant1'
 * '<S502>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTriggerBothSidesConditions/CheckLaneCenteringActiveState/Enumerated
 * Constant2'
 * '<S503>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTriggerBothSidesConditions/CheckLaneCenteringActiveState/TurnOnDelay'
 * '<S504>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTriggerBothSidesConditions/EgoVehicleVelocityCheck/Hysteresis'
 * '<S505>' :
 * 'TJASA_model/TJASA/TJASLC/ReadyToTrigger_StrongWeakReadyCheck/ReadyToTriggerBothSidesConditions/EgoVehicleVelocityCheck/Hysteresis1'
 * '<S506>' : 'TJASA_model/TJASA/TJASLC/RearObjectValidation/TurnOffDelay'
 * '<S507>' : 'TJASA_model/TJASA/TJASLC/RearObjectValidation/TurnOnDelay'
 * '<S508>' : 'TJASA_model/TJASA/TJASLC/RearObjectValidation/TurnOnDelay1'
 * '<S509>' : 'TJASA_model/TJASA/TJASLC/RearObjectValidation/TurnOnDelay2'
 * '<S510>' : 'TJASA_model/TJASA/TJASLC/RearObjectValidation/TurnOnDelay3'
 * '<S511>' : 'TJASA_model/TJASA/TJASLC/RearObjectValidation/TurnOnDelay4'
 * '<S512>' : 'TJASA_model/TJASA/TJASLC/RearObjectValidation/TurnOnDelay5'
 * '<S513>' :
 * 'TJASA_model/TJASA/TJASLC/TriggerEvaluation/CheckTriggerConditions'
 * '<S514>' : 'TJASA_model/TJASA/TJASLC/TriggerEvaluation/TriggerDebug'
 * '<S515>' :
 * 'TJASA_model/TJASA/TJASLC/TriggerEvaluation/TriggerResetConditions'
 * '<S516>' : 'TJASA_model/TJASA/TJASLC/TriggerEvaluation/TriggerSetting'
 * '<S517>' :
 * 'TJASA_model/TJASA/TJASLC/TriggerEvaluation/CheckTriggerConditions/LeverLeftEngaged'
 * '<S518>' :
 * 'TJASA_model/TJASA/TJASLC/TriggerEvaluation/CheckTriggerConditions/LeverRightEngaged'
 * '<S519>' :
 * 'TJASA_model/TJASA/TJASLC/TriggerEvaluation/CheckTriggerConditions/TurnOffDelay'
 * '<S520>' :
 * 'TJASA_model/TJASA/TJASLC/TriggerEvaluation/CheckTriggerConditions/LeverLeftEngaged/RSFlipFlop'
 * '<S521>' :
 * 'TJASA_model/TJASA/TJASLC/TriggerEvaluation/CheckTriggerConditions/LeverLeftEngaged/RSFlipFlop1'
 * '<S522>' :
 * 'TJASA_model/TJASA/TJASLC/TriggerEvaluation/CheckTriggerConditions/LeverRightEngaged/RSFlipFlop'
 * '<S523>' :
 * 'TJASA_model/TJASA/TJASLC/TriggerEvaluation/CheckTriggerConditions/LeverRightEngaged/RSFlipFlop1'
 * '<S524>' :
 * 'TJASA_model/TJASA/TJASLC/TriggerEvaluation/TriggerDebug/MappingUint16'
 * '<S525>' :
 * 'TJASA_model/TJASA/TJASLC/TriggerEvaluation/TriggerDebug/MappingUint16/Set_bit'
 * '<S526>' :
 * 'TJASA_model/TJASA/TJASLC/TriggerEvaluation/TriggerResetConditions/DegradationCheck'
 * '<S527>' :
 * 'TJASA_model/TJASA/TJASLC/TriggerEvaluation/TriggerResetConditions/LCPInitializationTimeMaxCheck'
 * '<S528>' :
 * 'TJASA_model/TJASA/TJASLC/TriggerEvaluation/TriggerResetConditions/TurnOffDelay'
 * '<S529>' :
 * 'TJASA_model/TJASA/TJASLC/TriggerEvaluation/TriggerResetConditions/DegradationCheck/Enumerated
 * Constant'
 * '<S530>' :
 * 'TJASA_model/TJASA/TJASLC/TriggerEvaluation/TriggerResetConditions/DegradationCheck/Enumerated
 * Constant1'
 * '<S531>' :
 * 'TJASA_model/TJASA/TJASLC/TriggerEvaluation/TriggerResetConditions/DegradationCheck/Enumerated
 * Constant2'
 * '<S532>' :
 * 'TJASA_model/TJASA/TJASLC/TriggerEvaluation/TriggerResetConditions/LCPInitializationTimeMaxCheck/Enumerated
 * Constant'
 * '<S533>' :
 * 'TJASA_model/TJASA/TJASLC/TriggerEvaluation/TriggerResetConditions/LCPInitializationTimeMaxCheck/Enumerated
 * Constant1'
 * '<S534>' :
 * 'TJASA_model/TJASA/TJASLC/TriggerEvaluation/TriggerResetConditions/LCPInitializationTimeMaxCheck/Enumerated
 * Constant2'
 * '<S535>' :
 * 'TJASA_model/TJASA/TJASLC/TriggerEvaluation/TriggerResetConditions/LCPInitializationTimeMaxCheck/TurnOnDelay'
 * '<S536>' :
 * 'TJASA_model/TJASA/TJASLC/TriggerEvaluation/TriggerSetting/CheckTriggerResetByDriver'
 * '<S537>' :
 * 'TJASA_model/TJASA/TJASLC/TriggerEvaluation/TriggerSetting/Enumerated
 * Constant'
 * '<S538>' :
 * 'TJASA_model/TJASA/TJASLC/TriggerEvaluation/TriggerSetting/Enumerated
 * Constant1'
 * '<S539>' :
 * 'TJASA_model/TJASA/TJASLC/TriggerEvaluation/TriggerSetting/Enumerated
 * Constant2'
 * '<S540>' :
 * 'TJASA_model/TJASA/TJASLC/TriggerEvaluation/TriggerSetting/Enumerated
 * Constant3'
 * '<S541>' :
 * 'TJASA_model/TJASA/TJASLC/TriggerEvaluation/TriggerSetting/TurnOnDelay'
 * '<S542>' :
 * 'TJASA_model/TJASA/TJASLC/TriggerEvaluation/TriggerSetting/CheckTriggerResetByDriver/Enumerated
 * Constant1'
 * '<S543>' :
 * 'TJASA_model/TJASA/TJASLC/TriggerEvaluation/TriggerSetting/CheckTriggerResetByDriver/Enumerated
 * Constant3'
 * '<S544>' :
 * 'TJASA_model/TJASA/TJASLC/TriggerEvaluation/TriggerSetting/CheckTriggerResetByDriver/TurnOnDelay'
 * '<S545>' :
 * 'TJASA_model/TJASA/TJASLC/TriggerEvaluation/TriggerSetting/CheckTriggerResetByDriver/TurnOnDelay1'
 * '<S546>' : 'TJASA_model/TJASA/TJASTM/ACCActiveOvertime'
 * '<S547>' : 'TJASA_model/TJASA/TJASTM/CancelConditions'
 * '<S548>' : 'TJASA_model/TJASA/TJASTM/Enumerated Constant2'
 * '<S549>' : 'TJASA_model/TJASA/TJASTM/LatCtrlMode'
 * '<S550>' : 'TJASA_model/TJASA/TJASTM/MappingUint16_2'
 * '<S551>' : 'TJASA_model/TJASA/TJASTM/PilotSysState'
 * '<S552>' : 'TJASA_model/TJASA/TJASTM/RAMPOUTStChk'
 * '<S553>' : 'TJASA_model/TJASA/TJASTM/STMTakeOverWarning'
 * '<S554>' : 'TJASA_model/TJASA/TJASTM/StateMachineTJA'
 * '<S555>' : 'TJASA_model/TJASA/TJASTM/Subsystem'
 * '<S556>' : 'TJASA_model/TJASA/TJASTM/ACCActiveOvertime/Enumerated Constant'
 * '<S557>' : 'TJASA_model/TJASA/TJASTM/ACCActiveOvertime/Enumerated Constant1'
 * '<S558>' : 'TJASA_model/TJASA/TJASTM/ACCActiveOvertime/Enumerated Constant2'
 * '<S559>' : 'TJASA_model/TJASA/TJASTM/ACCActiveOvertime/Enumerated Constant3'
 * '<S560>' : 'TJASA_model/TJASA/TJASTM/ACCActiveOvertime/Enumerated Constant4'
 * '<S561>' : 'TJASA_model/TJASA/TJASTM/ACCActiveOvertime/TurnOffDelay1'
 * '<S562>' : 'TJASA_model/TJASA/TJASTM/ACCActiveOvertime/TurnOffDelay2'
 * '<S563>' : 'TJASA_model/TJASA/TJASTM/ACCActiveOvertime/TurnOnDelay'
 * '<S564>' : 'TJASA_model/TJASA/TJASTM/CancelConditions/CMB_CancelConditions'
 * '<S565>' : 'TJASA_model/TJASA/TJASTM/CancelConditions/LC_CancelConditions'
 * '<S566>' : 'TJASA_model/TJASA/TJASTM/CancelConditions/OF_CancelConditions'
 * '<S567>' : 'TJASA_model/TJASA/TJASTM/CancelConditions/SLC_CancelConditions'
 * '<S568>' :
 * 'TJASA_model/TJASA/TJASTM/CancelConditions/CMB_CancelConditions/Enumerated
 * Constant'
 * '<S569>' :
 * 'TJASA_model/TJASA/TJASTM/CancelConditions/LC_CancelConditions/Enumerated
 * Constant'
 * '<S570>' :
 * 'TJASA_model/TJASA/TJASTM/CancelConditions/OF_CancelConditions/Enumerated
 * Constant'
 * '<S571>' :
 * 'TJASA_model/TJASA/TJASTM/CancelConditions/SLC_CancelConditions/Enumerated
 * Constant'
 * '<S572>' : 'TJASA_model/TJASA/TJASTM/MappingUint16_2/Set_bit'
 * '<S573>' : 'TJASA_model/TJASA/TJASTM/PilotSysState/Enumerated Constant'
 * '<S574>' : 'TJASA_model/TJASA/TJASTM/PilotSysState/Enumerated Constant1'
 * '<S575>' : 'TJASA_model/TJASA/TJASTM/PilotSysState/Enumerated Constant10'
 * '<S576>' : 'TJASA_model/TJASA/TJASTM/PilotSysState/Enumerated Constant11'
 * '<S577>' : 'TJASA_model/TJASA/TJASTM/PilotSysState/Enumerated Constant12'
 * '<S578>' : 'TJASA_model/TJASA/TJASTM/PilotSysState/Enumerated Constant13'
 * '<S579>' : 'TJASA_model/TJASA/TJASTM/PilotSysState/Enumerated Constant14'
 * '<S580>' : 'TJASA_model/TJASA/TJASTM/PilotSysState/Enumerated Constant15'
 * '<S581>' : 'TJASA_model/TJASA/TJASTM/PilotSysState/Enumerated Constant16'
 * '<S582>' : 'TJASA_model/TJASA/TJASTM/PilotSysState/Enumerated Constant17'
 * '<S583>' : 'TJASA_model/TJASA/TJASTM/PilotSysState/Enumerated Constant18'
 * '<S584>' : 'TJASA_model/TJASA/TJASTM/PilotSysState/Enumerated Constant2'
 * '<S585>' : 'TJASA_model/TJASA/TJASTM/PilotSysState/Enumerated Constant3'
 * '<S586>' : 'TJASA_model/TJASA/TJASTM/PilotSysState/Enumerated Constant4'
 * '<S587>' : 'TJASA_model/TJASA/TJASTM/PilotSysState/Enumerated Constant5'
 * '<S588>' : 'TJASA_model/TJASA/TJASTM/PilotSysState/Enumerated Constant6'
 * '<S589>' : 'TJASA_model/TJASA/TJASTM/PilotSysState/Enumerated Constant7'
 * '<S590>' : 'TJASA_model/TJASA/TJASTM/PilotSysState/Enumerated Constant8'
 * '<S591>' : 'TJASA_model/TJASA/TJASTM/PilotSysState/Enumerated Constant9'
 * '<S592>' : 'TJASA_model/TJASA/TJASTM/RAMPOUTStChk/Enumerated Constant'
 * '<S593>' : 'TJASA_model/TJASA/TJASTM/RAMPOUTStChk/Enumerated Constant1'
 * '<S594>' : 'TJASA_model/TJASA/TJASTM/RAMPOUTStChk/Enumerated Constant2'
 * '<S595>' : 'TJASA_model/TJASA/TJASTM/RAMPOUTStChk/Enumerated Constant3'
 * '<S596>' : 'TJASA_model/TJASA/TJASTM/RAMPOUTStChk/Enumerated Constant4'
 * '<S597>' : 'TJASA_model/TJASA/TJASTM/RAMPOUTStChk/Enumerated Constant5'
 * '<S598>' : 'TJASA_model/TJASA/TJASTM/RAMPOUTStChk/Enumerated Constant6'
 * '<S599>' : 'TJASA_model/TJASA/TJASTM/RAMPOUTStChk/RampoutWatchDog'
 * '<S600>' : 'TJASA_model/TJASA/TJASTM/RAMPOUTStChk/SuspendedTimeExpired'
 * '<S601>' : 'TJASA_model/TJASA/TJASTM/RAMPOUTStChk/RampoutWatchDog/RSFlipFlop'
 * '<S602>' :
 * 'TJASA_model/TJASA/TJASTM/RAMPOUTStChk/RampoutWatchDog/TimerRetrigger'
 * '<S603>' :
 * 'TJASA_model/TJASA/TJASTM/RAMPOUTStChk/SuspendedTimeExpired/RSFlipFlop'
 * '<S604>' :
 * 'TJASA_model/TJASA/TJASTM/RAMPOUTStChk/SuspendedTimeExpired/TimerRetrigger'
 * '<S605>' : 'TJASA_model/TJASA/TJASTM/STMTakeOverWarning/EdgeRising'
 * '<S606>' : 'TJASA_model/TJASA/TJASTM/STMTakeOverWarning/Enumerated Constant'
 * '<S607>' : 'TJASA_model/TJASA/TJASTM/STMTakeOverWarning/Enumerated Constant1'
 * '<S608>' : 'TJASA_model/TJASA/TJASTM/STMTakeOverWarning/Enumerated Constant2'
 * '<S609>' : 'TJASA_model/TJASA/TJATOW/BrakePadelInvalid'
 * '<S610>' : 'TJASA_model/TJASA/TJATOW/EdgeRising'
 * '<S611>' : 'TJASA_model/TJASA/TJATOW/Enumerated Constant1'
 * '<S612>' : 'TJASA_model/TJASA/TJATOW/Enumerated Constant2'
 * '<S613>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals'
 * '<S614>' : 'TJASA_model/TJASA/TJATOW/RSFlipFlop1'
 * '<S615>' : 'TJASA_model/TJASA/TJATOW/TurnOffDelay1'
 * '<S616>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/EdgeRising'
 * '<S617>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/EdgeRising1'
 * '<S618>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/EdgeRising10'
 * '<S619>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/EdgeRising2'
 * '<S620>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/EdgeRising3'
 * '<S621>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/EdgeRising4'
 * '<S622>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/EdgeRising5'
 * '<S623>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/EdgeRising6'
 * '<S624>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/EdgeRising7'
 * '<S625>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/EdgeRising8'
 * '<S626>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/EdgeRising9'
 * '<S627>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/Enumerated Constant1'
 * '<S628>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/Enumerated Constant10'
 * '<S629>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/Enumerated Constant11'
 * '<S630>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/Enumerated Constant12'
 * '<S631>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/Enumerated Constant13'
 * '<S632>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/Enumerated Constant14'
 * '<S633>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/Enumerated Constant15'
 * '<S634>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/Enumerated Constant16'
 * '<S635>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/Enumerated Constant17'
 * '<S636>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/Enumerated Constant18'
 * '<S637>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/Enumerated Constant19'
 * '<S638>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/Enumerated Constant2'
 * '<S639>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/Enumerated Constant20'
 * '<S640>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/Enumerated Constant21'
 * '<S641>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/Enumerated Constant3'
 * '<S642>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/Enumerated Constant4'
 * '<S643>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/Enumerated Constant5'
 * '<S644>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/Enumerated Constant6'
 * '<S645>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/Enumerated Constant7'
 * '<S646>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/Enumerated Constant8'
 * '<S647>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/Enumerated Constant9'
 * '<S648>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/SpdInvalid'
 * '<S649>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/TurnOffDelay1'
 * '<S650>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/TurnOffDelay10'
 * '<S651>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/TurnOffDelay11'
 * '<S652>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/TurnOffDelay12'
 * '<S653>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/TurnOffDelay13'
 * '<S654>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/TurnOffDelay14'
 * '<S655>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/TurnOffDelay15'
 * '<S656>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/TurnOffDelay2'
 * '<S657>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/TurnOffDelay3'
 * '<S658>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/TurnOffDelay4'
 * '<S659>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/TurnOffDelay5'
 * '<S660>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/TurnOffDelay6'
 * '<S661>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/TurnOffDelay7'
 * '<S662>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/TurnOffDelay8'
 * '<S663>' : 'TJASA_model/TJASA/TJATOW/HMIRelatedSignals/TurnOffDelay9'
 * '<S664>' : 'TJASA_model/TJASA/TJATTG/TJATTG'
 * '<S665>' : 'TJASA_model/TJASA/TJATTG/TJATTG/CombinedCorridorControl'
 * '<S666>' : 'TJASA_model/TJASA/TJATTG/TJATTG/DebugSys'
 * '<S667>' : 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction'
 * '<S668>' : 'TJASA_model/TJASA/TJATTG/TJATTG/LeftCorridorBoundaryGeneration'
 * '<S669>' : 'TJASA_model/TJASA/TJATTG/TJATTG/ModeControl'
 * '<S670>' : 'TJASA_model/TJASA/TJATTG/TJATTG/ModeTransitionHandling'
 * '<S671>' : 'TJASA_model/TJASA/TJATTG/TJATTG/ObjectLeftClothoid'
 * '<S672>' : 'TJASA_model/TJASA/TJATTG/TJATTG/ObjectRightClothoid'
 * '<S673>' : 'TJASA_model/TJASA/TJATTG/TJATTG/RightCorridorBoundaryGeneration'
 * '<S674>' : 'TJASA_model/TJASA/TJATTG/TJATTG/SALC_CorridorShift'
 * '<S675>' : 'TJASA_model/TJASA/TJATTG/TJATTG/TargetCorridorGeneration'
 * '<S676>' : 'TJASA_model/TJASA/TJATTG/TJATTG/TargetCorridorStateHandling'
 * '<S677>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/CombinedCorridorControl/EdgeFalling'
 * '<S678>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/CombinedCorridorControl/EdgeRising'
 * '<S679>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/CombinedCorridorControl/Enumerated Constant'
 * '<S680>' : 'TJASA_model/TJASA/TJATTG/TJATTG/DebugSys/MappingUint16'
 * '<S681>' : 'TJASA_model/TJASA/TJATTG/TJATTG/DebugSys/MappingUint16/Set_bit'
 * '<S682>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictLaneCenterCorridor'
 * '<S683>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictLeftLaneBoundary'
 * '<S684>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictRightLaneBoundary'
 * '<S685>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictVehicleOdometrie'
 * '<S686>' : 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictionControl'
 * '<S687>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictLaneCenterCorridor/Subsystem'
 * '<S688>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictLaneCenterCorridor/Subsystem/CurvatureChngPredict'
 * '<S689>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictLaneCenterCorridor/Subsystem/CurvaturePredict'
 * '<S690>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictLaneCenterCorridor/Subsystem/Freeze1'
 * '<S691>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictLaneCenterCorridor/Subsystem/Freeze2'
 * '<S692>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictLaneCenterCorridor/Subsystem/Freeze3'
 * '<S693>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictLaneCenterCorridor/Subsystem/Freeze4'
 * '<S694>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictLaneCenterCorridor/Subsystem/Heading'
 * '<S695>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictLaneCenterCorridor/Subsystem/LengthPredict'
 * '<S696>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictLaneCenterCorridor/Subsystem/PosY0'
 * '<S697>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictLaneCenterCorridor/Subsystem/X_Yaw'
 * '<S698>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictLaneCenterCorridor/Subsystem/Y_Yaw'
 * '<S699>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictLaneCenterCorridor/Subsystem/CurvatureChngPredict/EdgeRising'
 * '<S700>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictLaneCenterCorridor/Subsystem/CurvatureChngPredict/LowPassFilter'
 * '<S701>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictLaneCenterCorridor/Subsystem/CurvaturePredict/EdgeRising'
 * '<S702>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictLaneCenterCorridor/Subsystem/CurvaturePredict/LowPassFilter'
 * '<S703>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictLaneCenterCorridor/Subsystem/LengthPredict/EdgeRising'
 * '<S704>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictLeftLaneBoundary/Subsystem'
 * '<S705>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictLeftLaneBoundary/Subsystem/CurvatureChngPredict'
 * '<S706>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictLeftLaneBoundary/Subsystem/CurvaturePredict'
 * '<S707>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictLeftLaneBoundary/Subsystem/Freeze1'
 * '<S708>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictLeftLaneBoundary/Subsystem/Freeze2'
 * '<S709>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictLeftLaneBoundary/Subsystem/Freeze3'
 * '<S710>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictLeftLaneBoundary/Subsystem/Freeze4'
 * '<S711>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictLeftLaneBoundary/Subsystem/Heading'
 * '<S712>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictLeftLaneBoundary/Subsystem/LengthPredict'
 * '<S713>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictLeftLaneBoundary/Subsystem/PosY0'
 * '<S714>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictLeftLaneBoundary/Subsystem/X_Yaw'
 * '<S715>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictLeftLaneBoundary/Subsystem/Y_Yaw'
 * '<S716>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictLeftLaneBoundary/Subsystem/CurvatureChngPredict/EdgeRising'
 * '<S717>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictLeftLaneBoundary/Subsystem/CurvatureChngPredict/LowPassFilter'
 * '<S718>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictLeftLaneBoundary/Subsystem/CurvaturePredict/EdgeRising'
 * '<S719>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictLeftLaneBoundary/Subsystem/CurvaturePredict/LowPassFilter'
 * '<S720>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictLeftLaneBoundary/Subsystem/LengthPredict/EdgeRising'
 * '<S721>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictRightLaneBoundary/Subsystem'
 * '<S722>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictRightLaneBoundary/Subsystem/CurvatureChngPredict'
 * '<S723>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictRightLaneBoundary/Subsystem/CurvaturePredict'
 * '<S724>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictRightLaneBoundary/Subsystem/Freeze1'
 * '<S725>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictRightLaneBoundary/Subsystem/Freeze2'
 * '<S726>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictRightLaneBoundary/Subsystem/Freeze3'
 * '<S727>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictRightLaneBoundary/Subsystem/Freeze4'
 * '<S728>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictRightLaneBoundary/Subsystem/Heading'
 * '<S729>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictRightLaneBoundary/Subsystem/LengthPredict'
 * '<S730>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictRightLaneBoundary/Subsystem/PosY0'
 * '<S731>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictRightLaneBoundary/Subsystem/X_Yaw'
 * '<S732>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictRightLaneBoundary/Subsystem/Y_Yaw'
 * '<S733>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictRightLaneBoundary/Subsystem/CurvatureChngPredict/EdgeRising'
 * '<S734>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictRightLaneBoundary/Subsystem/CurvatureChngPredict/LowPassFilter'
 * '<S735>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictRightLaneBoundary/Subsystem/CurvaturePredict/EdgeRising'
 * '<S736>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictRightLaneBoundary/Subsystem/CurvaturePredict/LowPassFilter'
 * '<S737>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictRightLaneBoundary/Subsystem/LengthPredict/EdgeRising'
 * '<S738>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictVehicleOdometrie/DELAY_RE'
 * '<S739>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictVehicleOdometrie/DELAY_RE2'
 * '<S740>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictVehicleOdometrie/DELAY_RE3'
 * '<S741>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictVehicleOdometrie/EdgeRising'
 * '<S742>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictionControl/EdgeRising'
 * '<S743>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LanePrediction/PredictionControl/RSFlipFlop'
 * '<S744>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LeftCorridorBoundaryGeneration/CalculatedCombinedCorridor'
 * '<S745>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LeftCorridorBoundaryGeneration/ModeTransitionFilter'
 * '<S746>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LeftCorridorBoundaryGeneration/TargetCorridorSelection'
 * '<S747>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LeftCorridorBoundaryGeneration/ModeTransitionFilter/IV_Selection'
 * '<S748>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LeftCorridorBoundaryGeneration/ModeTransitionFilter/TransitionFilter'
 * '<S749>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LeftCorridorBoundaryGeneration/ModeTransitionFilter/TransitionFilter/CrvChng_Filtering'
 * '<S750>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LeftCorridorBoundaryGeneration/ModeTransitionFilter/TransitionFilter/Crv_Filtering'
 * '<S751>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LeftCorridorBoundaryGeneration/ModeTransitionFilter/TransitionFilter/Freeze_IV'
 * '<S752>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LeftCorridorBoundaryGeneration/ModeTransitionFilter/TransitionFilter/Heading_Filtering'
 * '<S753>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LeftCorridorBoundaryGeneration/ModeTransitionFilter/TransitionFilter/Length_Filtering'
 * '<S754>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LeftCorridorBoundaryGeneration/ModeTransitionFilter/TransitionFilter/PosX0_Filtering'
 * '<S755>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LeftCorridorBoundaryGeneration/ModeTransitionFilter/TransitionFilter/PosY0_Filtering'
 * '<S756>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LeftCorridorBoundaryGeneration/ModeTransitionFilter/TransitionFilter/CrvChng_Filtering/Filter'
 * '<S757>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LeftCorridorBoundaryGeneration/ModeTransitionFilter/TransitionFilter/Crv_Filtering/Filter'
 * '<S758>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LeftCorridorBoundaryGeneration/ModeTransitionFilter/TransitionFilter/Heading_Filtering/Filter'
 * '<S759>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LeftCorridorBoundaryGeneration/ModeTransitionFilter/TransitionFilter/Length_Filtering/Filter'
 * '<S760>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LeftCorridorBoundaryGeneration/ModeTransitionFilter/TransitionFilter/PosX0_Filtering/Filter'
 * '<S761>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LeftCorridorBoundaryGeneration/ModeTransitionFilter/TransitionFilter/PosY0_Filtering/Filter'
 * '<S762>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LeftCorridorBoundaryGeneration/TargetCorridorSelection/Else'
 * '<S763>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/LeftCorridorBoundaryGeneration/TargetCorridorSelection/Switch'
 * '<S764>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeControl/CheckLaneUpdateConditions'
 * '<S765>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeControl/CheckObjectUpdateConditions'
 * '<S766>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeControl/CheckPredictionConditions'
 * '<S767>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeControl/CheckLaneUpdateConditions/Enumerated
 * Constant'
 * '<S768>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeControl/CheckLaneUpdateConditions/Enumerated
 * Constant1'
 * '<S769>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeControl/CheckLaneUpdateConditions/Enumerated
 * Constant2'
 * '<S770>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeControl/CheckLaneUpdateConditions/Enumerated
 * Constant3'
 * '<S771>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeControl/CheckLaneUpdateConditions/Enumerated
 * Constant4'
 * '<S772>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeControl/CheckLaneUpdateConditions/Enumerated
 * Constant5'
 * '<S773>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeControl/CheckObjectUpdateConditions/Enumerated
 * Constant'
 * '<S774>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeControl/CheckObjectUpdateConditions/Enumerated
 * Constant1'
 * '<S775>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeControl/CheckObjectUpdateConditions/Enumerated
 * Constant2'
 * '<S776>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeControl/CheckObjectUpdateConditions/Enumerated
 * Constant3'
 * '<S777>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeControl/CheckPredictionConditions/EdgeFalling'
 * '<S778>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeControl/CheckPredictionConditions/Enumerated
 * Constant1'
 * '<S779>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeTransitionHandling/ObserveModeTransition'
 * '<S780>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeTransitionHandling/ObserveModeTransition/CalculateTransitionFactorA'
 * '<S781>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeTransitionHandling/ObserveModeTransition/Enumerated
 * Constant'
 * '<S782>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeTransitionHandling/ObserveModeTransition/Enumerated
 * Constant1'
 * '<S783>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeTransitionHandling/ObserveModeTransition/Enumerated
 * Constant10'
 * '<S784>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeTransitionHandling/ObserveModeTransition/Enumerated
 * Constant11'
 * '<S785>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeTransitionHandling/ObserveModeTransition/Enumerated
 * Constant12'
 * '<S786>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeTransitionHandling/ObserveModeTransition/Enumerated
 * Constant2'
 * '<S787>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeTransitionHandling/ObserveModeTransition/Enumerated
 * Constant3'
 * '<S788>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeTransitionHandling/ObserveModeTransition/Enumerated
 * Constant4'
 * '<S789>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeTransitionHandling/ObserveModeTransition/Enumerated
 * Constant5'
 * '<S790>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeTransitionHandling/ObserveModeTransition/Enumerated
 * Constant6'
 * '<S791>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeTransitionHandling/ObserveModeTransition/Enumerated
 * Constant7'
 * '<S792>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeTransitionHandling/ObserveModeTransition/Enumerated
 * Constant8'
 * '<S793>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeTransitionHandling/ObserveModeTransition/Enumerated
 * Constant9'
 * '<S794>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeTransitionHandling/ObserveModeTransition/SetTransitionSpecificEnabler'
 * '<S795>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeTransitionHandling/ObserveModeTransition/SetTransitionSpecificEnabler1'
 * '<S796>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeTransitionHandling/ObserveModeTransition/SetTransitionSpecificEnabler2'
 * '<S797>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeTransitionHandling/ObserveModeTransition/SetTransitionSpecificEnabler3'
 * '<S798>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeTransitionHandling/ObserveModeTransition/SetTransitionTime'
 * '<S799>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeTransitionHandling/ObserveModeTransition/CalculateTransitionFactorA/DivDflt'
 * '<S800>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeTransitionHandling/ObserveModeTransition/CalculateTransitionFactorA/Normalize'
 * '<S801>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeTransitionHandling/ObserveModeTransition/CalculateTransitionFactorA/STOPWATCH_RE'
 * '<S802>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeTransitionHandling/ObserveModeTransition/SetTransitionTime/Passive'
 * '<S803>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeTransitionHandling/ObserveModeTransition/SetTransitionTime/SALC_ManeuverStart'
 * '<S804>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeTransitionHandling/ObserveModeTransition/SetTransitionTime/SetTransitionTime'
 * '<S805>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeTransitionHandling/ObserveModeTransition/SetTransitionTime/TurnOffDelayTimeCUSTOM'
 * '<S806>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeTransitionHandling/ObserveModeTransition/SetTransitionTime/Passive/Enumerated
 * Constant'
 * '<S807>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeTransitionHandling/ObserveModeTransition/SetTransitionTime/SALC_ManeuverStart/Enumerated
 * Constant'
 * '<S808>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/ModeTransitionHandling/ObserveModeTransition/SetTransitionTime/TurnOffDelayTimeCUSTOM/TurnOffDelayWithRst'
 * '<S809>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/RightCorridorBoundaryGeneration/CalculatedCombinedCorridor'
 * '<S810>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/RightCorridorBoundaryGeneration/ModeTransitionFilter'
 * '<S811>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/RightCorridorBoundaryGeneration/TargetCorridorSelection'
 * '<S812>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/RightCorridorBoundaryGeneration/ModeTransitionFilter/IV_Selection'
 * '<S813>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/RightCorridorBoundaryGeneration/ModeTransitionFilter/TransitionFilter'
 * '<S814>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/RightCorridorBoundaryGeneration/ModeTransitionFilter/TransitionFilter/CrvChng_Filtering'
 * '<S815>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/RightCorridorBoundaryGeneration/ModeTransitionFilter/TransitionFilter/Crv_Filtering'
 * '<S816>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/RightCorridorBoundaryGeneration/ModeTransitionFilter/TransitionFilter/Freeze_IV'
 * '<S817>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/RightCorridorBoundaryGeneration/ModeTransitionFilter/TransitionFilter/Heading_Filtering'
 * '<S818>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/RightCorridorBoundaryGeneration/ModeTransitionFilter/TransitionFilter/Length_Filtering'
 * '<S819>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/RightCorridorBoundaryGeneration/ModeTransitionFilter/TransitionFilter/PosX0_Filtering'
 * '<S820>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/RightCorridorBoundaryGeneration/ModeTransitionFilter/TransitionFilter/PosY0_Filtering'
 * '<S821>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/RightCorridorBoundaryGeneration/ModeTransitionFilter/TransitionFilter/CrvChng_Filtering/Filter'
 * '<S822>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/RightCorridorBoundaryGeneration/ModeTransitionFilter/TransitionFilter/Crv_Filtering/Filter'
 * '<S823>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/RightCorridorBoundaryGeneration/ModeTransitionFilter/TransitionFilter/Heading_Filtering/Filter'
 * '<S824>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/RightCorridorBoundaryGeneration/ModeTransitionFilter/TransitionFilter/Length_Filtering/Filter'
 * '<S825>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/RightCorridorBoundaryGeneration/ModeTransitionFilter/TransitionFilter/PosX0_Filtering/Filter'
 * '<S826>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/RightCorridorBoundaryGeneration/ModeTransitionFilter/TransitionFilter/PosY0_Filtering/Filter'
 * '<S827>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/RightCorridorBoundaryGeneration/TargetCorridorSelection/Else'
 * '<S828>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/RightCorridorBoundaryGeneration/TargetCorridorSelection/Switch'
 * '<S829>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/SALC_CorridorShift/CrrdrCenterBusSwitch'
 * '<S830>' : 'TJASA_model/TJASA/TJATTG/TJATTG/SALC_CorridorShift/Egolane'
 * '<S831>' : 'TJASA_model/TJASA/TJATTG/TJATTG/SALC_CorridorShift/Else'
 * '<S832>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/SALC_CorridorShift/LeftAdjcentLane'
 * '<S833>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/SALC_CorridorShift/LeftCrrdrBoundaryBusSwitch'
 * '<S834>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/SALC_CorridorShift/RightAdjcentLane'
 * '<S835>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/SALC_CorridorShift/RightCrrdrBoundaryBusSwitch'
 * '<S836>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/SALC_CorridorShift/TargetCorridorSelection'
 * '<S837>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/SALC_CorridorShift/Egolane/PosY0InLaneIncoherence'
 * '<S838>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/SALC_CorridorShift/TargetCorridorSelection/CheckSystemStatesForAdjacentLaneSelection'
 * '<S839>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/SALC_CorridorShift/TargetCorridorSelection/CheckSystemStatesForEgoLaneSelection'
 * '<S840>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/SALC_CorridorShift/TargetCorridorSelection/CreateSwitch'
 * '<S841>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/SALC_CorridorShift/TargetCorridorSelection/CheckSystemStatesForAdjacentLaneSelection/Enumerated
 * Constant1'
 * '<S842>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/SALC_CorridorShift/TargetCorridorSelection/CheckSystemStatesForAdjacentLaneSelection/Enumerated
 * Constant2'
 * '<S843>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/SALC_CorridorShift/TargetCorridorSelection/CheckSystemStatesForAdjacentLaneSelection/Enumerated
 * Constant3'
 * '<S844>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/SALC_CorridorShift/TargetCorridorSelection/CheckSystemStatesForAdjacentLaneSelection/Enumerated
 * Constant4'
 * '<S845>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/SALC_CorridorShift/TargetCorridorSelection/CheckSystemStatesForAdjacentLaneSelection/Enumerated
 * Constant5'
 * '<S846>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/SALC_CorridorShift/TargetCorridorSelection/CheckSystemStatesForAdjacentLaneSelection/Enumerated
 * Constant6'
 * '<S847>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/SALC_CorridorShift/TargetCorridorSelection/CheckSystemStatesForAdjacentLaneSelection/Enumerated
 * Constant7'
 * '<S848>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/SALC_CorridorShift/TargetCorridorSelection/CheckSystemStatesForAdjacentLaneSelection/Enumerated
 * Constant8'
 * '<S849>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/SALC_CorridorShift/TargetCorridorSelection/CheckSystemStatesForAdjacentLaneSelection/Enumerated
 * Constant9'
 * '<S850>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/SALC_CorridorShift/TargetCorridorSelection/CheckSystemStatesForEgoLaneSelection/Enumerated
 * Constant'
 * '<S851>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/SALC_CorridorShift/TargetCorridorSelection/CheckSystemStatesForEgoLaneSelection/Enumerated
 * Constant1'
 * '<S852>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/SALC_CorridorShift/TargetCorridorSelection/CheckSystemStatesForEgoLaneSelection/Enumerated
 * Constant10'
 * '<S853>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/SALC_CorridorShift/TargetCorridorSelection/CheckSystemStatesForEgoLaneSelection/Enumerated
 * Constant2'
 * '<S854>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/SALC_CorridorShift/TargetCorridorSelection/CheckSystemStatesForEgoLaneSelection/Enumerated
 * Constant3'
 * '<S855>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/SALC_CorridorShift/TargetCorridorSelection/CheckSystemStatesForEgoLaneSelection/Enumerated
 * Constant4'
 * '<S856>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/SALC_CorridorShift/TargetCorridorSelection/CheckSystemStatesForEgoLaneSelection/Enumerated
 * Constant5'
 * '<S857>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/SALC_CorridorShift/TargetCorridorSelection/CheckSystemStatesForEgoLaneSelection/Enumerated
 * Constant6'
 * '<S858>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/SALC_CorridorShift/TargetCorridorSelection/CheckSystemStatesForEgoLaneSelection/Enumerated
 * Constant7'
 * '<S859>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/SALC_CorridorShift/TargetCorridorSelection/CheckSystemStatesForEgoLaneSelection/Enumerated
 * Constant8'
 * '<S860>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/SALC_CorridorShift/TargetCorridorSelection/CheckSystemStatesForEgoLaneSelection/Enumerated
 * Constant9'
 * '<S861>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/TargetCorridorGeneration/ModeTransitionFilter'
 * '<S862>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/TargetCorridorGeneration/TargetCorridorSelection'
 * '<S863>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/TargetCorridorGeneration/ModeTransitionFilter/IV_Selection'
 * '<S864>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/TargetCorridorGeneration/ModeTransitionFilter/TransitionFilter'
 * '<S865>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/TargetCorridorGeneration/ModeTransitionFilter/TransitionFilter/CrvChng_Filtering'
 * '<S866>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/TargetCorridorGeneration/ModeTransitionFilter/TransitionFilter/Crv_Filtering'
 * '<S867>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/TargetCorridorGeneration/ModeTransitionFilter/TransitionFilter/Freeze_IV'
 * '<S868>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/TargetCorridorGeneration/ModeTransitionFilter/TransitionFilter/Heading_Filtering'
 * '<S869>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/TargetCorridorGeneration/ModeTransitionFilter/TransitionFilter/Length_Filtering'
 * '<S870>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/TargetCorridorGeneration/ModeTransitionFilter/TransitionFilter/PosX0_Filtering'
 * '<S871>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/TargetCorridorGeneration/ModeTransitionFilter/TransitionFilter/PosY0_Filtering'
 * '<S872>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/TargetCorridorGeneration/ModeTransitionFilter/TransitionFilter/CrvChng_Filtering/Filter'
 * '<S873>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/TargetCorridorGeneration/ModeTransitionFilter/TransitionFilter/Crv_Filtering/Filter'
 * '<S874>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/TargetCorridorGeneration/ModeTransitionFilter/TransitionFilter/Heading_Filtering/Filter'
 * '<S875>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/TargetCorridorGeneration/ModeTransitionFilter/TransitionFilter/Length_Filtering/Filter'
 * '<S876>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/TargetCorridorGeneration/ModeTransitionFilter/TransitionFilter/PosX0_Filtering/Filter'
 * '<S877>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/TargetCorridorGeneration/ModeTransitionFilter/TransitionFilter/PosY0_Filtering/Filter'
 * '<S878>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/TargetCorridorGeneration/TargetCorridorSelection/Else'
 * '<S879>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/TargetCorridorGeneration/TargetCorridorSelection/Switch'
 * '<S880>' : 'TJASA_model/TJASA/TJATTG/TJATTG/TargetCorridorStateHandling/Else'
 * '<S881>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/TargetCorridorStateHandling/Enumerated
 * Constant1'
 * '<S882>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/TargetCorridorStateHandling/Enumerated
 * Constant2'
 * '<S883>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/TargetCorridorStateHandling/Rampout'
 * '<S884>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/TargetCorridorStateHandling/RequestOrControlling'
 * '<S885>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/TargetCorridorStateHandling/Else/LaneCenter'
 * '<S886>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/TargetCorridorStateHandling/Else/LeftBoundary'
 * '<S887>' :
 * 'TJASA_model/TJASA/TJATTG/TJATTG/TargetCorridorStateHandling/Else/RightBoundary'
 * '<S888>' : 'TJASA_model/TJASA/TJATVG/DetermineTrajGuiQualifier'
 * '<S889>' : 'TJASA_model/TJASA/TJATVG/GenerateTimeStamp'
 * '<S890>' : 'TJASA_model/TJASA/TJATVG/PlanningHorizon'
 * '<S891>' : 'TJASA_model/TJASA/TJATVG/SetConstOutputs'
 * '<S892>' : 'TJASA_model/TJASA/TJATVG/TargetCurvatureGradients'
 * '<S893>' : 'TJASA_model/TJASA/TJATVG/TorqueLimitationAndGradients'
 * '<S894>' : 'TJASA_model/TJASA/TJATVG/DetermineTrajGuiQualifier/Enumerated
 * Constant1'
 * '<S895>' : 'TJASA_model/TJASA/TJATVG/DetermineTrajGuiQualifier/Enumerated
 * Constant10'
 * '<S896>' : 'TJASA_model/TJASA/TJATVG/DetermineTrajGuiQualifier/Enumerated
 * Constant11'
 * '<S897>' : 'TJASA_model/TJASA/TJATVG/DetermineTrajGuiQualifier/Enumerated
 * Constant12'
 * '<S898>' : 'TJASA_model/TJASA/TJATVG/DetermineTrajGuiQualifier/Enumerated
 * Constant13'
 * '<S899>' : 'TJASA_model/TJASA/TJATVG/DetermineTrajGuiQualifier/Enumerated
 * Constant14'
 * '<S900>' : 'TJASA_model/TJASA/TJATVG/DetermineTrajGuiQualifier/Enumerated
 * Constant15'
 * '<S901>' : 'TJASA_model/TJASA/TJATVG/DetermineTrajGuiQualifier/Enumerated
 * Constant16'
 * '<S902>' : 'TJASA_model/TJASA/TJATVG/DetermineTrajGuiQualifier/Enumerated
 * Constant17'
 * '<S903>' : 'TJASA_model/TJASA/TJATVG/DetermineTrajGuiQualifier/Enumerated
 * Constant18'
 * '<S904>' : 'TJASA_model/TJASA/TJATVG/DetermineTrajGuiQualifier/Enumerated
 * Constant2'
 * '<S905>' : 'TJASA_model/TJASA/TJATVG/DetermineTrajGuiQualifier/Enumerated
 * Constant3'
 * '<S906>' : 'TJASA_model/TJASA/TJATVG/DetermineTrajGuiQualifier/Enumerated
 * Constant4'
 * '<S907>' : 'TJASA_model/TJASA/TJATVG/DetermineTrajGuiQualifier/Enumerated
 * Constant5'
 * '<S908>' : 'TJASA_model/TJASA/TJATVG/DetermineTrajGuiQualifier/Enumerated
 * Constant6'
 * '<S909>' : 'TJASA_model/TJASA/TJATVG/DetermineTrajGuiQualifier/Enumerated
 * Constant7'
 * '<S910>' : 'TJASA_model/TJASA/TJATVG/DetermineTrajGuiQualifier/Enumerated
 * Constant8'
 * '<S911>' : 'TJASA_model/TJASA/TJATVG/DetermineTrajGuiQualifier/Enumerated
 * Constant9'
 * '<S912>' :
 * 'TJASA_model/TJASA/TJATVG/DetermineTrajGuiQualifier/ObserveModeTransitions'
 * '<S913>' :
 * 'TJASA_model/TJASA/TJATVG/DetermineTrajGuiQualifier/VDy_Bugfix_Issue'
 * '<S914>' :
 * 'TJASA_model/TJASA/TJATVG/DetermineTrajGuiQualifier/ObserveModeTransitions/SALC_NewTargetLane'
 * '<S915>' :
 * 'TJASA_model/TJASA/TJATVG/DetermineTrajGuiQualifier/ObserveModeTransitions/SALC_NewTargetLane/EdgeRising'
 * '<S916>' :
 * 'TJASA_model/TJASA/TJATVG/DetermineTrajGuiQualifier/ObserveModeTransitions/SALC_NewTargetLane/Enumerated
 * Constant12'
 * '<S917>' :
 * 'TJASA_model/TJASA/TJATVG/DetermineTrajGuiQualifier/ObserveModeTransitions/SALC_NewTargetLane/Enumerated
 * Constant5'
 * '<S918>' :
 * 'TJASA_model/TJASA/TJATVG/DetermineTrajGuiQualifier/ObserveModeTransitions/SALC_NewTargetLane/RSFlipFlop'
 * '<S919>' : 'TJASA_model/TJASA/TJATVG/GenerateTimeStamp/ClothoidSelection'
 * '<S920>' : 'TJASA_model/TJASA/TJATVG/GenerateTimeStamp/ClothoidSelection1'
 * '<S921>' : 'TJASA_model/TJASA/TJATVG/GenerateTimeStamp/CreateSwitch'
 * '<S922>' : 'TJASA_model/TJASA/TJATVG/GenerateTimeStamp/Enumerated Constant6'
 * '<S923>' : 'TJASA_model/TJASA/TJATVG/GenerateTimeStamp/Enumerated Constant7'
 * '<S924>' :
 * 'TJASA_model/TJASA/TJATVG/GenerateTimeStamp/LatencyCompensationSwitch'
 * '<S925>' :
 * 'TJASA_model/TJASA/TJATVG/GenerateTimeStamp/ClothoidSelection/Enumerated
 * Constant5'
 * '<S926>' :
 * 'TJASA_model/TJASA/TJATVG/GenerateTimeStamp/ClothoidSelection/Enumerated
 * Constant6'
 * '<S927>' :
 * 'TJASA_model/TJASA/TJATVG/GenerateTimeStamp/ClothoidSelection1/Enumerated
 * Constant5'
 * '<S928>' :
 * 'TJASA_model/TJASA/TJATVG/GenerateTimeStamp/LatencyCompensationSwitch/Enumerated
 * Constant1'
 * '<S929>' :
 * 'TJASA_model/TJASA/TJATVG/GenerateTimeStamp/LatencyCompensationSwitch/Enumerated
 * Constant2'
 * '<S930>' :
 * 'TJASA_model/TJASA/TJATVG/GenerateTimeStamp/LatencyCompensationSwitch/Enumerated
 * Constant3'
 * '<S931>' :
 * 'TJASA_model/TJASA/TJATVG/GenerateTimeStamp/LatencyCompensationSwitch/Enumerated
 * Constant4'
 * '<S932>' :
 * 'TJASA_model/TJASA/TJATVG/GenerateTimeStamp/LatencyCompensationSwitch/Enumerated
 * Constant5'
 * '<S933>' :
 * 'TJASA_model/TJASA/TJATVG/GenerateTimeStamp/LatencyCompensationSwitch/Enumerated
 * Constant6'
 * '<S934>' :
 * 'TJASA_model/TJASA/TJATVG/GenerateTimeStamp/LatencyCompensationSwitch/Enumerated
 * Constant7'
 * '<S935>' :
 * 'TJASA_model/TJASA/TJATVG/GenerateTimeStamp/LatencyCompensationSwitch/Enumerated
 * Constant8'
 * '<S936>' : 'TJASA_model/TJASA/TJATVG/PlanningHorizon/Else'
 * '<S937>' : 'TJASA_model/TJASA/TJATVG/PlanningHorizon/Enumerated Constant'
 * '<S938>' : 'TJASA_model/TJASA/TJATVG/PlanningHorizon/Enumerated Constant1'
 * '<S939>' : 'TJASA_model/TJASA/TJATVG/PlanningHorizon/Enumerated Constant2'
 * '<S940>' : 'TJASA_model/TJASA/TJATVG/PlanningHorizon/Enumerated Constant3'
 * '<S941>' : 'TJASA_model/TJASA/TJATVG/PlanningHorizon/Enumerated Constant4'
 * '<S942>' : 'TJASA_model/TJASA/TJATVG/PlanningHorizon/Enumerated Constant5'
 * '<S943>' : 'TJASA_model/TJASA/TJATVG/PlanningHorizon/Enumerated Constant6'
 * '<S944>' : 'TJASA_model/TJASA/TJATVG/PlanningHorizon/Enumerated Constant7'
 * '<S945>' : 'TJASA_model/TJASA/TJATVG/PlanningHorizon/Enumerated Constant8'
 * '<S946>' : 'TJASA_model/TJASA/TJATVG/PlanningHorizon/Index_Generator'
 * '<S947>' : 'TJASA_model/TJASA/TJATVG/PlanningHorizon/LaneCentering'
 * '<S948>' : 'TJASA_model/TJASA/TJATVG/PlanningHorizon/LaneChange'
 * '<S949>' : 'TJASA_model/TJASA/TJATVG/PlanningHorizon/ObjectFollowing'
 * '<S950>' : 'TJASA_model/TJASA/TJATVG/SetConstOutputs/SALC_ActiveCheck'
 * '<S951>' :
 * 'TJASA_model/TJASA/TJATVG/SetConstOutputs/SALC_ActiveCheck/Enumerated
 * Constant1'
 * '<S952>' :
 * 'TJASA_model/TJASA/TJATVG/SetConstOutputs/SALC_ActiveCheck/Enumerated
 * Constant2'
 * '<S953>' : 'TJASA_model/TJASA/TJATVG/TargetCurvatureGradients/Enumerated
 * Constant1'
 * '<S954>' : 'TJASA_model/TJASA/TJATVG/TargetCurvatureGradients/Enumerated
 * Constant2'
 * '<S955>' : 'TJASA_model/TJASA/TJATVG/TargetCurvatureGradients/Enumerated
 * Constant3'
 * '<S956>' :
 * 'TJASA_model/TJASA/TJATVG/TargetCurvatureGradients/TargetCurvatureGradient_ELSE'
 * '<S957>' :
 * 'TJASA_model/TJASA/TJATVG/TargetCurvatureGradients/TargetCurvatureGradient_LCorCB'
 * '<S958>' :
 * 'TJASA_model/TJASA/TJATVG/TargetCurvatureGradients/TargetCurvatureGradient_OF'
 * '<S959>' : 'TJASA_model/TJASA/TJATVG/TorqueLimitationAndGradients/Enumerated
 * Constant1'
 * '<S960>' : 'TJASA_model/TJASA/TJATVG/TorqueLimitationAndGradients/Enumerated
 * Constant3'
 * '<S961>' : 'TJASA_model/TJASA/TJATVG/TorqueLimitationAndGradients/Gradients'
 * '<S962>' : 'TJASA_model/TJASA/TJATVG/TorqueLimitationAndGradients/Limiter'
 * '<S963>' :
 * 'TJASA_model/TJASA/TJATVG/TorqueLimitationAndGradients/PredictionRampoutHandling'
 * '<S964>' :
 * 'TJASA_model/TJASA/TJATVG/TorqueLimitationAndGradients/Gradients/IndexGenerator'
 * '<S965>' :
 * 'TJASA_model/TJASA/TJATVG/TorqueLimitationAndGradients/PredictionRampoutHandling/TurnOffDelay'
 */

/*-
 * Requirements for '<Root>': TJASA
 */
#endif /* RTW_HEADER_TJASA_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
