#ifndef TJASA_EXT_H
#define TJASA_EXT_H

#include "TJASA.h"

typedef struct
{
  boolean_T
      ABPR_LaneChangeDetected_bool;     /* '<Root>/Inport11',   Lane change
                                           detected flag--from ABPR module */
  real32_T ABPR_LaneWidth_met;          /* '<Root>/Inport5',    Lane width--from ABPR
                                           module */
  boolean_T ABPR_ConstructionSite_bool; /* '<Root>/Inport13',   Construction
                                           site flag--from ABPR */
  real32_T ABPR_UncoupledLaneWidth_met; /* '<Root>/Inport22',   Uncoupled Lane
                                           Width--from ABPR module */
  real32_T ABPR_ABDTimeStamp_sec;       /* '<Root>/Inport91',   ABD Time Stamp--from
                                           ABPR module */
  uint8_T ABPR_LeftLaneType_enum;       /* '<Root>/Inport54',   Left lane type--from
                                           ABPR */
  real32_T ABPR_LeLnClthPosX0_met;      /* '<Root>/Inport23',   Left Lane Clothoid
                                           PosX0--from ABPR module */
  real32_T ABPR_LeLnClthPosY0_met;      /* '<Root>/Inport1',    Left lane clothoid
                                           posY0--from ABPR module */
  uint16_T ABPR_LeLnInvalidQu_btf;      /* '<Root>/Inport7',    Left lane invalid
                                           qualifier--from ABPR */
  uint8_T ABPR_LeLnQuality_perc;        /* '<Root>/Inport15',   Left lane
                                           quality--from ABPR module */
  uint8_T ABPR_LeCrvQuality_perc;       /* '<Root>/Inport17',   Left lane curve
                                           quality--from ABPR module */
  real32_T ABPR_LeLnClthHeading_rad;    /* '<Root>/Inport24',   Left Lane
                                           Clothoid Heading--from ABPR module */
  real32_T ABPR_LeLnClthCrv_1pm;        /* '<Root>/Inport25',   Left lane clothoid
                                           curvature--from ABPR module */
  real32_T ABPR_LeLnClthCrvChng_1pm2;   /* '<Root>/Inport26',   Left Lane
                                           Clothoid Curve of Change--from ABPR
                                           module */
  real32_T ABPR_LeLnClthLength_met;     /* '<Root>/Inport27',   Left lane clothoid
                                           length--from ABPR module */
  real32_T
      ABPR_LeAdjLnClthPosX0_met; /* '<Root>/Inport80',   Left Adjacent Lane
                                    Clothoid PosX0--from ABPR module */
  real32_T
      ABPR_LeAdjLnClthPosY0_met;         /* '<Root>/Inport40',   Left adjacent lane
                                            clothoid posY0--from ABPR module */
  uint16_T ABPR_LeAdjLnInvalidQu_btf;    /* '<Root>/Inport43',   Left adjacent
                                            lane invalid quality--from ABPR
                                            module */
  real32_T ABPR_LeAdjLnClthHeading_rad;  /* '<Root>/Inport81',   Left Adjacent
                                            Lane Clothoid Heading--from ABPR
                                            module */
  real32_T ABPR_LeAdjLnClthCrv_1pm;      /* '<Root>/Inport82',   Left Adjacent Lane
                                            Clothoid Curve--from ABPR module */
  real32_T ABPR_LeAdjLnClthCrvChng_1pm2; /* '<Root>/Inport83',   Left Adjacent
                                            Lane Clothoid Curve of
                                            Change--from ABPR module */
  real32_T ABPR_LeAdjLnClthLength_met;   /* '<Root>/Inport84',   Left Adjacent
                                            Lane Clothoid Length--from ABPR
                                            module */
  uint8_T ABPR_RightLaneType_enum;       /* '<Root>/Inport55',   Right lane
                                            type--from ABPR */
  real32_T ABPR_RiLnClthPosX0_met;       /* '<Root>/Inport28',   Right Lane Clothoid
                                            PosX0--from ABPR module */
  real32_T ABPR_RiLnClthPosY0_met;       /* '<Root>/Inport2',    Right lane clothoid
                                            posY0--from ABPR module */
  uint16_T ABPR_RiLnInvalidQu_btf;       /* '<Root>/Inport6',    Right lane invalid
                                            qualifier--from ABPR */
  uint8_T ABPR_RiLnQuality_perc;         /* '<Root>/Inport16',   Right lane
                                            quality--from ABPR module */
  uint8_T ABPR_RiCrvQuality_perc;        /* '<Root>/Inport18',   Right lane curve
                                            quality--from ABPR module */
  real32_T ABPR_RiLnClthHeading_rad;     /* '<Root>/Inport29',   Right Lane
                                            Clothoid Heading--from ABPR module */
  real32_T ABPR_RiLnClthCrv_1pm;         /* '<Root>/Inport30',   Right lane clothoid
                                            curvature--from ABPR module */
  real32_T ABPR_RiLnClthCrvChng_1pm2;    /* '<Root>/Inport31',   Right Lane
                                            Clothoid Curve of Change--from ABPR
                                            module */
  real32_T ABPR_RiLnClthLength_met;      /* '<Root>/Inport20',   Right lane
                                            clothoid length--from ABPR module */
  real32_T
      ABPR_RiAdjLnClthPosX0_met; /* '<Root>/Inport89',   Right Adjacent Lane
                                    Clothoid PosX0--from ABPR module */
  real32_T
      ABPR_RiAdjLnClthPosY0_met;          /* '<Root>/Inport41',   Right adjacent lane
                                             clothoid posY0--from ABPR module */
  uint16_T ABPR_RiAdjLnInvalidQu_btf;     /* '<Root>/Inport42',   Right adjacent
                                             lane invalid quality--from ABPR
                                             module */
  real32_T ABPR_RiAdjLnClthHeading_rad;   /* '<Root>/Inport85',   Right Adjacent
                                             Lane Clothoid Heading--from ABPR
                                             module */
  real32_T ABPR_RiAdjLnClthCrv_1pm;       /* '<Root>/Inport86',   Right Adjacent
                                             Lane Clothoid Curve--from ABPR module
                                             */
  real32_T ABPR_RiAdjLnClthCrvChng_1pm2;  /* '<Root>/Inport87',   Right
                                             Adjacent Lane Clothoid Curve of
                                             Change--from ABPR module */
  real32_T ABPR_RiAdjLnClthLength_met;    /* '<Root>/Inport88',   Right Adjacent
                                             Lane Clothoid Length--from ABPR
                                             module */
  real32_T ABPR_CntrLnClthPosX0_met;      /* '<Root>/Inport58',   Control Lane
                                             Clothoid PosX0--from ABPR module */
  real32_T ABPR_CntrLnClthPosY0_met;      /* '<Root>/Inport3',    Control Lane
                                             Clothoid PosY0--from ABPR module */
  real32_T ABPR_CntrLnClthHeading_rad;    /* '<Root>/Inport4',    Control lane
                                             clothoid heading--from ABPR module
                                             */
  real32_T ABPR_CntrLnClthCrv_1pm;        /* '<Root>/Inport9',    Control Lane
                                             Clothoid Curve--from ABPR module */
  real32_T ABPR_CntrLnClthCrvChng_1pm2;   /* '<Root>/Inport59',   Control Lane
                                             Clothoid Curve of Change--from ABPR
                                             module */
  real32_T ABPR_CntrLnClthLength_met;     /* '<Root>/Inport21',   Control Lane
                                             Clothoid Length--from ABPR module */
  uint16_T ODPFOH_TgtObjClothoidInv_btf;  /* '<Root>/Inport32',   Target object
                                             clothoid invalid qualifier--from
                                             ODPR module */
  real32_T ODPFOH_TgtObjPosX0_met;        /* '<Root>/Inport60',   Target object
                                             Clothoid PosX0--from ODPR module */
  real32_T ODPFOH_TgtObjPosY0_met;        /* '<Root>/Inport36',   Target object
                                             Clothoid PosY0--from ODPR module */
  real32_T ODPFOH_TgtObjHeadAng_rad;      /* '<Root>/Inport37',   Target object
                                             Clothoid Heading--from ODPR module */
  real32_T ODPFOH_TgtObjCrv_1pm;          /* '<Root>/Inport38',   Target object
                                             Clothoid Curve--from ODPR module */
  real32_T ODPFOH_TgtObjCrvChng_1pm2;     /* '<Root>/Inport61',   Target object
                                             Clothoid Curve of Change--from ODPR
                                             module */
  real32_T ODPFOH_TgtObjLength_met;       /* '<Root>/Inport39',   Target object
                                             Clothoid Length--from ODPR module */
  real32_T ODPFOP_AccFRObjTStamp_sec;     /* '<Root>/Inport92',   Acc object time
                                             stamp--from ODPR module */
  uint16_T ODPFOP_AccObjInvBitfield_btf;  /* '<Root>/Inport33',   Acc object
                                             invalid qualifier--from ODPR
                                             module */
  real32_T ODPFOP_AccObjPosX_met;         /* '<Root>/Inport34',   Acc obejct
                                             PosX--from ODPR module */
  real32_T ODPFOP_AccObjPosY_met;         /* '<Root>/Inport35',   Acc obejct
                                             PosY--from ODPR module */
  boolean_T LCA_ActiveLeft_bool;          /* '<Root>/Inport46',   Active left
                                             flag--from LCA */
  boolean_T LCA_ActiveRight_bool;         /* '<Root>/Inport47',   Active right
                                             flag--from LCA */
  boolean_T LCA_WarningLeft_bool;         /* '<Root>/Inport48',   Warning left
                                             flag--from LCA */
  boolean_T LCA_WarningRight_bool;        /* '<Root>/Inport49',   Warning right
                                             flag--from LCA */
  boolean_T BSD_ActiveLeft_bool;          /* '<Root>/Inport50',   Active left
                                             flag--from BSD */
  boolean_T BSD_ActiveRight_bool;         /* '<Root>/Inport51',   Active right
                                             flag--from BSD */
  boolean_T BSD_WarningLeft_bool;         /* '<Root>/Inport52',   Warning left
                                             flag--from BSD */
  boolean_T BSD_WarningRight_bool;        /* '<Root>/Inport53',   Warning right
                                             flag--from BSD */
  uint16_T CUSTOM_PrjSpecQu_btf;          /* '<Root>/Inport12',   Specific
                                             qualifier--from CUSTOM */
  boolean_T S_ODPSOP_MSFlag_RearLeft_nu;  /* '<Root>/Inport56',   MS flag_rear
                                             left--from ODPSOP */
  boolean_T S_ODPSOP_MSFlag_RearRight_nu; /* '<Root>/Inport57',   MS flag_rear
                                             right--from ODPSOP */
  uint16_T TRJPLN_QuStatusTrajPlan_nu;    /* '<Root>/Inport62',   Trajectory plan
                                             qualifier--from TRJPLN module */
  uint8_T TRJCTR_QuServTrajCtr_nu;        /* '<Root>/Inport63',   Trajectory control
                                             qualifier--from TRJCTR module */
  E_MCTLCF_ControllingFunction_nu
      MDCTR_ControllingFunction_nu;          /* '<Root>/Inport64',    Controlling
                                                function--from MDCTR module */
  real32_T LCFRCV_SysCycleTimeSen_sec;       /* '<Root>/Inport10',   System cycle
                                                time--from LCFRCV */
  real32_T VDy_VehVelocity_kph;              /* '<Root>/Inport8',    Vehicle velocity--from
                                                VDy module */
  boolean_T LCFRCV_TurnSignalLeverHold_bool; /* '<Root>/Inport19',   Turn
                                                signal level hold flag--from
                                                LCFRCV */
  boolean_T LCFRCV_TurnSignalLeft_bool;      /* '<Root>/Inport44',   Turn signal
                                                left flag--from LCFRCV */
  boolean_T LCFRCV_TurnSignalRight_bool;     /* '<Root>/Inport45',   Turn signal
                                                right flag--from LCFRCV */
  uint8_T LCFRCV_DrivingMode_nu;             /* '<Root>/Inport93',   Driving mode--from
                                                LCFRCV */
  boolean_T LCFRCV_LKASwitch_nu;             /* '<Root>/Inport65',   LKA switch flag--from
                                                LCFRCV */
  boolean_T LCFRCV_TJASwitch_nu;             /* '<Root>/Inport66',   TJA switch flag--from
                                                LCFRCV */
  boolean_T LCFRCV_ErrorStateTJA_bool;       /* '<Root>/Inport67',   Error state of
                                                TJA--from LCFRCV */
  boolean_T LCFRCV_ErrorStateLKA_bool;       /* '<Root>/Inport68',   Error state of
                                                LKA--from LCFRCV */
  real32_T VDy_VehAclX_mps2;                 /* '<Root>/Inport69',   Vehicle acceleration
                                                X--from VDy module */
  real32_T VDy_VehAclY_mps2;                 /* '<Root>/Inport70',   Vehicle acceleration
                                                Y--from VDy module */
  boolean_T LCFRCV_SysStOnLatDMC_bool;       /* '<Root>/Inport14',   System state on
                                                latDMC--from LCFRCV */
  real32_T LCFRCV_SteerWAngle_deg;           /* '<Root>/Inport77',   steer wheel
                                                angle--from LCRCV */
  real32_T VDy_VehCrv_1pm;                   /* '<Root>/Inport76',   Vehicle curve--from VDy
                                                module */
  real32_T VDy_VehYawRate_rps;               /* '<Root>/Inport78',   Vehicle yawrate--from
                                                VDy module */
  real32_T VDy_VehVelX_mps;                  /* '<Root>/Inport79',   Vehicle velocity X--from
                                                VDy module */
  boolean_T LCFRCV_VehStopped_nu;            /* '<Root>/Inport90',   Vehicle stopped
                                                flag--from LCFRCV */
  uint8_T VDPDRV_ActiveStCtrl_btf;           /* '<Root>/Inport71',   Acitve state
                                                control--from VDPDRV module */
  uint8_T VDPDRV_SysStError_btf;             /* '<Root>/Inport72',   System state
                                                error--from VDPDRV module */
  uint16_T VDPDRV_VehStInvalid_btf;          /* '<Root>/Inport73',   Vehicle state
                                                invalid bitfield--from VDPDRV module */
  uint8_T VDPDRV_DrvStInvalid_btf;           /* '<Root>/Inport74',   Driver state
                                                invalid bitfield--from VDPDRV module */
  uint8_T
      VDPDRV_SysStNotAvailable_btf;      /* '<Root>/Inport75',   System state not
                                            avaliable bitfield--from VDPDRV module
                                            */
  real32_T VDy_DashboardVelocity_kph;    /* '<Root>/Inport94',   Dashbord
                                            velocity--from VDy module */
  real32_T LCFRCV_ManualTorque_nm;       /* '<Root>/Inport95',   Manual torque --
                                            from LCFRCV */
  real32_T LCFRCV_SteerWAngleGrad_degps; /* '<Root>/Inport96',   Steering
                                            wheel angle grad -- from LCFRCV */
  boolean_T LCFRCV_SLCHMISwitch_bool;    /* '<Root>/Inport97',   SLC HMI
                                            switch--from LCFRCV */
  boolean_T LCFRCV_TJAAudioSwitch_bool;  /* '<Root>/Inport98',   TJA audio
                                            switch--from LCFRCV */
  boolean_T
      LCCRA_LeftSafeFlag_bool; /* '<Root>/Inport99'
                                * Lane change detected flag--from ABPR module
                                */
  boolean_T
      LCCRA_RightSafeFlag_bool;              /* '<Root>/Inport100'
                                              * Lane change detected flag--from ABPR module
                                              */
  boolean_T LCCRA_LeftFrontSafeFlag_bool;    /* '<Root>/Inport101'
                                              * Lane change detected flag--from
                                              * ABPR module
                                              */
  boolean_T LCCRA_RightFrontSafeFlag_bool;   /* '<Root>/Inport102'
                                              * Lane change detected
                                              * flag--from ABPR module
                                              */
  boolean_T LCCRA_LeftRearSafeFlag_bool;     /* '<Root>/Inport103'
                                              * Lane change detected flag--from
                                              * ABPR module
                                              */
  boolean_T LCCRA_RightRearSafeFlag_bool;    /* '<Root>/Inport104'
                                              * Lane change detected flag--from
                                              * ABPR module
                                              */
  boolean_T LCFRCV_PilotOnLeverSwitch_bool;  /* '<Root>/Inport105'
                                              * Lane change detected
                                              * flag--from ABPR module
                                              */
  boolean_T LCFRCV_PilotOffLeverSwitch_bool; /* '<Root>/Inport106'
                                              * Lane change detected
                                              * flag--from ABPR module
                                              */
  uint8_T VLCVEH_ACCStatus_nu;               /* '<Root>/Inport107'
                                              * Lane change detected flag--from ABPR module
                                              */
  boolean_T LCFSEN_Nb_DCLCSwitchNVRAM_nu;
  int32_T LCCRA_LeftHighLightID_nu;
  int32_T LCCRA_RightHighLightID_nu;
  uint8_T LCFRCV_TurnLightReqSt_nu;
  boolean_T LCFRCV_DoorOpen_bool;    /* '<Root>/Inport112'
                                      * Active left flag--from LCA
                                      */
  boolean_T LCFRCV_HoodOpen_bool;    /* '<Root>/Inport113'
                                      * Active left flag--from LCA
                                      */
  boolean_T LCFRCV_TrunkUnLock_bool; /* '<Root>/Inport114'
                                      * Active left flag--from LCA
                                      */
  boolean_T LCFRCV_bPGear_bool;
  boolean_T LCCRA_inFrontSafeFlag_bool;
  int32_T LCFRCV_inFrontDangerObjID_nu;
  boolean_T EMFID_bRCA_bool;
  boolean_T EMFID_bRCC_bool;
  boolean_T EMFID_bFCA_bool;
  boolean_T EMFID_bFCB_bool;
  boolean_T LCFRCV_bPilotOnOff_bool;
  uint8_T ADCS4_AVM_Sts;
  uint8_T ADCS11_Parking_WorkSts;
} sTJASAInReq_t;

typedef struct
{
  uint8_T uTemp_nu; /* Not used, just for the unified interface */
} sTJASAParam_t;

typedef struct
{
  E_TJASTM_SysStateTJA_nu TJASTM_SysStateTJA_nu; /* '<S8>/StateMachineTJA'
                                          * TJASTM system state
                                            DT:Enum: E_TJASTM_SysStateTJA_nu
                                          */
  E_TJASTM_LatCtrlMode_nu
      TJASTM_LatCtrlMode_nu;                 /* '<S8>/LatCtrlMode'
                                          * TJASTM lateral control mode
                                            DT:Enum: E_TJASTM_LatCtrlMode_nu
                                          */
  real32_T TJATTG_LeCridrBndPosX0_met;       /* '<S9>/Signal Conversion12'
                                              * TJATTG left corridor boundary PosX0
                                                DT:float32
                                              */
  real32_T TJATTG_LeCridrBndPosY0_met;       /* '<S9>/Signal Conversion13'
                                              * TJATTG left corridor boundary PosY0
                                                DT:float32
                                              */
  real32_T TJATTG_LeCridrBndHeadAng_rad;     /* '<S9>/Signal Conversion14'
                                              * TJATTG left corridor boundary
                                              heading     DT:float32
                                              */
  real32_T TJATTG_LeCridrBndCrv_1pm;         /* '<S9>/Signal Conversion15'
                                              * TJATTG left corridor boundary curve
                                                DT:float32
                                              */
  real32_T TJATTG_LeCridrBndCrvChng_1pm2;    /* '<S9>/Signal Conversion16'
                                              * TJATTG left corridor boundary
                                              curve of change
                                                DT:float32
                                              */
  real32_T TJATTG_LeCridrBndLength_met;      /* '<S9>/Signal Conversion17'
                                              * TJATTG left corridor boundary
                                              length      DT:float32
                                              */
  real32_T TJATTG_RiCridrBndPosX0_met;       /* '<S9>/Signal Conversion18'
                                              * TJATTG right corridor boundary PosX0
                                                DT:float32
                                              */
  real32_T TJATTG_RiCridrBndPosY0_met;       /* '<S9>/Signal Conversion19'
                                              * TJATTG right corridor boundary PosY0
                                                DT:float32
                                              */
  real32_T TJATTG_RiCridrBndHeadAng_rad;     /* '<S9>/Signal Conversion20'
                                              * TJATTG right corridor boundary
                                              heading     DT:float32
                                              */
  real32_T TJATTG_RiCridrBndCrv_1pm;         /* '<S9>/Signal Conversion21'
                                              * TJATTG right corridor boundary curve
                                                DT:float32
                                              */
  real32_T TJATTG_RiCridrBndCrvChng_1pm2;    /* '<S9>/Signal Conversion22'
                                              * TJATTG right corridor boundary
                                              curve of change
                                                DT:float32
                                              */
  real32_T TJATTG_RiCridrBndLength_met;      /* '<S9>/Signal Conversion23'
                                              * TJATTG right corridor boundary
                                              length      DT:float32
                                              */
  real32_T TJATTG_TgtTrajPosX0_met;          /* '<S9>/Signal Conversion24'
                                              * TJATTG target trajectory PosX0
                                                DT:float32
                                              */
  real32_T TJATTG_TgtTrajPosY0_met;          /* '<S9>/Signal Conversion25'
                                              * TJATTG target trajectory PosY0
                                                DT:float32
                                              */
  real32_T TJATTG_TgtTrajHeadAng_rad;        /* '<S9>/Signal Conversion26'
                                              * TJATTG target trajectory heading
                                                DT:float32
                                              */
  real32_T TJATTG_TgtTrajCrv_1pm;            /* '<S9>/Signal Conversion27'
                                              * TJATTG target trajectory curve
                                                DT:float32
                                              */
  real32_T TJATTG_TgtTrajCrvChng_1pm2;       /* '<S9>/Signal Conversion28'
                                              * TJATTG target trajectory curve of
                                              change       DT:float32
                                              */
  real32_T TJATTG_TgtTrajLength_met;         /* '<S9>/Signal Conversion29'
                                              * TJATTG target trajectory length
                                                DT:float32
                                              */
  real32_T TJATVG_DistYTolLeTgtArea_met;     /* '<S570>/Constant15'
                                              * TJATVG DistYTolLeTgtArea
                                                DT:float32
                                              */
  real32_T TJATVG_DistYTolRiTgtArea_met;     /* '<S570>/Constant16'
                                              * TJATVG DistYTolRiTgtArea
                                                DT:float32
                                              */
  real32_T TJATVG_FTireAclMax_mps2;          /* '<S570>/Constant13'
                                              * TJATVG FTireAclMax
                                                DT:float32
                                              */
  real32_T TJATVG_FTireAclMin_mps2;          /* '<S570>/Constant14'
                                              * TJATVG FTireAclMin
                                                DT:float32
                                              */
  real32_T TJATVG_WeightTgtDistY_nu;         /* '<S570>/Constant17'
                                              * TJATVG weighted target distY
                                                DT:float32
                                              */
  real32_T TJATVG_WeightEndTime_nu;          /* '<S570>/Constant18'
                                              * TJATVG weighted end time
                                                DT:float32
                                              */
  real32_T TJATVG_PredTimeCrv_sec;           /* '<S570>/Constant4'
                                              * TJATVG predict time of curve
                                                DT:float32
                                              */
  real32_T TJATVG_PredTimeHeadAng_sec;       /* '<S570>/Constant10'
                                              * TJATVG predict time of heading
                                                DT:float32
                                              */
  real32_T TJATVG_MaxCrvTrajGuiCtl_1pm;      /* '<S571>/Switch'
                                              * TJATVG max curve trajectory control
                                                DT:float32
                                              */
  real32_T TJATVG_MaxCrvGrdBuildup_1pms;     /* '<S571>/Switch'
                                              * TJATVG max curve gradient buildup
                                                DT:float32
                                              */
  real32_T TJATVG_MaxCrvGrdRed_1pms;         /* '<S571>/Switch'
                                              * TJATVG max curve gradient red
                                                DT:float32
                                              */
  real32_T TJATVG_MaxCrvGrdTGC_1pms;         /* '<S571>/Switch'
                                              * TJATVG max curve gradient TGC
                                                DT:float32
                                              */
  real32_T TJATVG_PlanningHorizon_sec;       /* '<S569>/Multiport Switch'
                                              * TJATVG planning horizon time
                                                DT:float32
                                              */
  real32_T TJATVG_TrqRampGrad_1ps;           /* '<S629>/Multiport Switch2'
                                              * TJATVG torque ramp gradient
                                                DT:float32
                                              */
  real32_T TJATVG_StrWhStifLimit_nu;         /* '<S630>/Product'
                                              * TJATVG steer wheel limit
                                                DT:float32
                                              */
  real32_T TJATVG_MaxTrqScalLimit_nu;        /* '<S630>/Constant1'
                                              * TJATVG max toque scal limit
                                                DT:float32
                                              */
  real32_T TJATVG_StrWhStifGrad_1ps;         /* '<S629>/Multiport Switch'
                                              * TJATVG steer wheel stif gradient
                                                DT:float32
                                              */
  real32_T TJATVG_MaxTrqScalGrad_1ps;        /* '<S629>/Multiport Switch1'
                                              * TJATVG steer wheel scal gradient
                                                DT:float32
                                              */
  real32_T TJATVG_MaxJerkAllowed_mps3;       /* '<S570>/Constant12'
                                              * TJATVG max jerk allowed
                                                DT:float32
                                              */
  real32_T TJATVG_SensorTStamp_sec;          /* '<S568>/Multiport Switch'
                                              * TJATVG sensor timestamp
                                                DT:float32
                                              */
  real32_T TJATVG_ObstacleVelX_mps;          /* '<S570>/Constant'
                                              * TJATVG obstacle velX
                                                DT:float32
                                              */
  real32_T TJATVG_ObstacleAclX_mps2;         /* '<S570>/Constant2'
                                              * TJATVG obstacle AclX
                                                DT:float32
                                              */
  real32_T TJATVG_ObstacleWidth_met;         /* '<S570>/Constant3'
                                              * TJATVG obstacle width
                                                DT:float32
                                              */
  real32_T TJATVG_ObstacleDistX_met;         /* '<S570>/Constant8'
                                              * TJATVG obstacle DisX
                                                DT:float32
                                              */
  real32_T TJATVG_ObstacleDistY_met;         /* '<S570>/Constant9'
                                              * TJATVG obstacle DisY
                                                DT:float32
                                              */
  real32_T TJATVG_LimiterTimeDuration_sec;   /* '<S570>/Constant7'
                                              * TJATVG limiter time duration
                                                DT:float32
                                              */
  uint16_T TJATTG_TgtCorridorInvalid_btf;    /* '<S367>/Data Type Conversion1'
                                              * TJASTM target corridor invalid
                                              bitfield
                                                DT:uint16
                                              */
  uint8_T TJATVG_TrajPlanServQu_nu;          /* '<S570>/Switch'
                                              * TJATVG trajectory plan serv qulifier
                                                DT:uint8
                                              */
  uint8_T TJATVG_DeratingLevel_nu;           /* '<S570>/Product'
                                              * TJATVG derating level
                                                DT:uint8
                                              */
  uint8_T TJATVG_CrvAmplActivated_nu;        /* '<S570>/Constant1'
                                              * TJATVG CrvAmplActivated
                                                DT:uint8
                                              */
  uint8_T TJATVG_LimiterActivated_nu;        /* '<S570>/Constant5'
                                              * TJATVG limiter Activated
                                                DT:uint8
                                              */
  uint8_T TJATVG_LimiterType_nu;             /* '<S570>/Constant6'
                                              * TJATVG limiter type
                                                DT:uint8
                                              */
  boolean_T TJATTG_TransTriggerReplan_bool;  /* '<S356>/OR2'
                                              * TJATTG transition trigger
                                              replan flag  DT:boolean
                                              */
  boolean_T TJATVG_TriggerReplan_nu;         /* '<S570>/Constant11'
                                              * TJATVG trigger replan flag
                                                DT:boolean
                                              */
  boolean_T TJATVG_HighStatAccu_bool;        /* '<S572>/Constant1'
                                              * TJATVG  high stationary accuracy flag
                                                DT:boolean
                                              */
  boolean_T TJATVG_LtcyCompActivated_nu;     /* '<S591>/Switch'
                                              * TJATVG  latency compensation flag
                                                DT:boolean
                                              */
  E_TJATVG_TrajGuiQu_nu TJATVG_TrajGuiQu_nu; /* '<S567>/Switch'
                                              * TJATVG trajectory qualifier
                                                DT:Enum: E_TJATVG_TrajGuiQu_nu
                                              */
  uint8_T TJASTM_SysStateHWA_nu;
  uint8_T TJASTM_NpilotSysInfo;
  uint8_T TJASTM_PilotAudioPlay;
  uint8_T TJASTM_LatCtrlHandsOffReleaseWarn_nu;
  uint8_T TJASLC_LaneChangWarning_nu; /* '<S298>/Data Type Conversion'
* lane change warning side
 0: no warning
 1: left warning
 2: right warning
*/
  uint8_T TJASLC_SLCAudioPlay_nu;
  uint8_T TJASTM_PilotEnableACCSwitch_nu;
  uint8_T TJASTM_PilotDisableACCSwitch_nu;
  boolean_T TJASLC_Nb_DCLCSwitchNVRAM_nu;
  int32_T TJASLC_SLCHighLightID_nu;
  uint8_T TJASLC_LaneChangeInfo; /* 0:None; 1: Vechicle Speed too low; 2: Lane
                                    Change Start; 3:Lane Change End; 4:Lane
                                    Change Cancle; 5:Lane Change Pending; 6:
                                    Lane Change OnGoing */
  uint8_T TJASLC_TurnLtDirctionReq_nu;
} sTJASAOutPro_t;

#ifndef Rte_TypeDef_sTJASADebug_t
#define Rte_TypeDef_sTJASADebug_t
typedef struct
{
  real32_T TJACMB_CombinedPosX0_met;    /* '<S13>/Max'
                                    * TJACMB combined PosX0
                                      DT:float32
                                    */
  real32_T TJACMB_CombinedPosY0_met;    /* '<S21>/Add'
                                         * TJACMB combined PosY0
                                           DT:float32
                                         */
  real32_T TJACMB_CombinedHeading_rad;  /* '<S20>/Add'
                                         * TJACMB combined Heading
                                           DT:float32
                                         */
  real32_T TJACMB_CombinedCrv_1pm;      /* '<S18>/Switch'
                                         * TJACMB combined Curve
                                           DT:float32
                                         */
  real32_T TJACMB_CombinedCrvChng_1pm2; /* '<S19>/Add'
                                         * TJACMB combined Curve of change
                                           DT:float32
                                         */
  real32_T TJACMB_CombinedLength_met;   /* '<S13>/Min'
                                         * TJACMB combined length
                                           DT:float32
                                         */
  real32_T TJACMB_LaneCrvStdDev_nu;     /* '<S11>/Sqrt'
                                         * TJACMB lane curve standard deviation
                                           DT:float32
                                         */
  real32_T TJACMB_TraceCrvStdDev_nu;    /* '<S12>/Sqrt'
                                         * TJACMB trace curve standard deviation
                                           DT:float32
                                         */
  E_TJASLC_ReadyToTrigger_nu
      TJASLC_ReadyToTrigger_nu; /* '<S283>/Switch'
                                 * SLC ready to trigger state
                                   DT:Enum: E_TJASLC_ReadyToTrigger_nu
                                 */
  E_TJASLC_ManeuverState_nu
      TJASLC_ManeuverState_nu; /* '<S256>/Switch'
                                * SLC maneuver state
                                  DT:Enum: E_TJASLC_ManeuverState_nu
                                */
  E_TJASLC_LaneChangeTrig_nu
      TJASLC_LaneChangeTrig_nu; /* '<S309>/Switch2'
                                 * SLC lane change trigger state
                                   DT:Enum: E_TJASLC_LaneChangeTrig_nu
                                 */
  E_TJALKA_LnBndValid_nu
      TJALKA_LnBndValid_nu;                   /* '<S109>/Switch'
                                               * TJALKA lane boundary valid state
                                                 DT:Enum: E_TJALKA_LnBndValid_nu
                                               */
  boolean_T TJACMB_Cancel_bool;               /* '<S14>/Constant2'
                                           * TJACMB cancel flag
                                             DT:boolean
                                           */
  boolean_T TJALKA_LanePredictValid_bool;     /* '<S121>/Multiport Switch'
                                           * TJALKA lane predict valid flag
                                             DT:boolean
                                           */
  boolean_T TJALKA_Cancel_bool;               /* '<S75>/AND1'
                                               * TJALKA cancel ready flag
                                                 DT:boolean
                                               */
  boolean_T TJALKA_StrongReady_bool;          /* '<S75>/AND'
                                               * TJALKA strong ready flag
                                                 DT:boolean
                                               */
  boolean_T TJALKA_WeakReady_bool;            /* '<S75>/AND2'
                                               * TJALKA weak ready flag
                                                 DT:boolean
                                               */
  boolean_T TJAOBF_ObjLaneValidDuration_bool; /* '<S174>/AND'
                                               * TJAOBF object in lane valid
                                               duration flag
                                                 DT:boolean
                                               */
  boolean_T TJAOBF_TgtObjDataValid_bool;      /* '<S140>/AND1'
                                               * TJAOBF target date valid flag
                                                 DT:boolean
                                               */
  boolean_T TJAGEN_LKAOnlySwitch_bool;        /* '<S49>/AND3'
                                               * TJAGEN LKA only switch flag
                                                 DT:boolean
                                               */
  boolean_T TJAOBF_StrongReady_bool;          /* '<S138>/AND1'
                                               * TJAOBF strong ready flag
                                                 DT:boolean
                                               */
  boolean_T TJAOBF_Cancel_bool;               /* '<S138>/AND3'
                                               * TJAOBF cancel flag
                                                 DT:boolean
                                               */
  boolean_T TJAOBF_WeakReady_bool;            /* '<S138>/AND2'
                                               * TJAOBF weak ready flag
                                                 DT:boolean
                                               */
  boolean_T TJAGEN_Clearance_bool;            /* '<S48>/AND'
                                               * TJAGEN clearance flag
                                                 DT:boolean
                                               */
  boolean_T TJAGEN_Degradation_bool;          /* '<S51>/AND'
                                               * TJAGEN degradation flag
                                                 DT:boolean
                                               */
  boolean_T TJASLC_Cancel_bool;               /* '<S211>/AND'
                                               * SLC cancel flag
                                                 DT:boolean
                                               */
  boolean_T TJASLC_StrongReady_bool;          /* '<S252>/Switch'
                                               * SLC strong ready flag
                                                 DT:boolean
                                               */
  boolean_T TJASLC_WeakReady_bool;            /* '<S252>/Switch2'
                                               * SLC weak ready flag
                                                 DT:boolean
                                               */
  boolean_T TJASLC_TakeOverValid_bool;        /* '<S270>/OR'
                                               * SLC take over valid flag
                                                 DT:boolean
                                               */
  boolean_T TJACMB_ObjectCorridor_bool;       /* '<S17>/AND'
                                               * TJACMB object corridor flag
                                                 DT:boolean
                                               */
  boolean_T TJACMB_StrongReady_bool;          /* '<S14>/AND1'
                                               * TJACMB Strong ready flag
                                                 DT:boolean
                                               */
  boolean_T TJACMB_WeakReady_bool;            /* '<S14>/AND2'
                                               * TJACMB weak ready flag
                                                 DT:boolean
                                               */
  boolean_T TJAGEN_FunctionSwitch_bool;       /* '<S49>/OR1'
                                               * TJAGEN function switch flag
                                                 DT:boolean
                                               */
  boolean_T TJAGEN_CodeFunction_bool;         /* '<S49>/OR2'
                                               * TJAGEN code function flag
                                                 DT:boolean
                                               */
  boolean_T TJAGEN_Error_bool;                /* '<S52>/Switch'
                                               * TJAGEN error flag
                                                 DT:boolean
                                               */
  boolean_T TJAGEN_Abort_bool;                /* '<S50>/NotEqual1'
                                               * TJAGEN abort flag
                                                 DT:boolean
                                               */
  boolean_T TJAGEN_Cancel_nu;                 /* '<S47>/OR'
                                               * TJAGEN cancel flag
                                                 DT:boolean
                                               */
  boolean_T TJAGEN_StrongReady_bool;          /* '<S53>/AND'
                                               * TJAGEN strong ready flag
                                                 DT:boolean
                                               */
  boolean_T TJAGEN_WeakReady_bool;            /* '<S54>/AND'
                                               * TJAGEN weak ready flag
                                                 DT:boolean
                                               */
  boolean_T TJATTG_PredictionEnable_bool;     /* '<S429>/Multiport Switch'
                                               * TJATTG prediction enable flag
                                                 DT:boolean
                                               */
  uint8_T TJAGEN_CancelStatus_btf;            /* '<S56>/Data Type Conversion1'
                                           * TJAGEN cancel status bitfield
                                             DT:uint8
                                           */
  uint8_T TJAGEN_WeakReadyInvalid_btf;        /* '<S74>/Data Type Conversion1'
                                               * TJAGEN weak ready invalid bitfield
                                                 DT:uint8
                                               */
  uint16_T TJALKA_LaneCenterInvalid_btf;      /* '<S99>/Data Type Conversion1'
                                              * TJALKA lane center invalid bitfield
                                                DT:uint16
                                              */
  uint16_T TJAOBF_ObjInLaneInvalid_btf;       /* '<S170>/Data Type Conversion1'
                                               * TJAOBF object in lane invalid
                                               bitfield    DT:uint16
                                               */
  uint16_T TJAOBF_ObjFollowInvalid_btf;       /* '<S155>/Data Type Conversion1'
                                               * TJAOBF object  following  invalid
                                               bitfield
                                                 DT:uint16
                                               */
  uint16_T TJASLC_LeLaneChangeInvalid_btf;    /* '<S281>/Data Type Conversion1'
                                               * SLC left lane change invalid
                                               bitfield DT:uint16
                                               */
  uint16_T TJASLC_RiLaneChangeInvalid_btf;    /* '<S282>/Data Type Conversion1'
                                               * SLC right lane change invalid
                                               bitfield
                                                 DT:uint16
                                               */
  uint16_T TJASLC_TriggerInvalid_btf;         /* '<S318>/Data Type Conversion1'
                                               * SLC trigger invalid bitfield
                                                 DT:uint16
                                               */
  uint16_T TJACMB_CombinedInvalid_btf;        /* '<S41>/Data Type Conversion1'
                                               * TJACMB combined invalid bitfield
                                                 DT:uint16
                                               */
  uint16_T TJASTM_TJAInvalid_btf;             /* '<S349>/Data Type Conversion1'
                                               * TJASTM TJA invalid bitfield
                                                 DT:uint16
                                               */
  uint8_T TJALKA_LnQualityInv_btf;            /* '<S137>/Data Type Conversion1'
                                               * TJALKA lane quality invalid bitfield
                                                 DT:uint8
                                               */
  uint8_T TJAOBF_TgtObjDataInvalid_btf;       /* '<S205>/Data Type Conversion1'
                                               * TJAOBF object data invalid bitfield
                                                 DT:uint8
                                               */
  uint8_T TJASLC_CancelAbort_btf;             /* '<S240>/Data Type Conversion1'
                                               * SLC cancel abort bitfield
                                                 DT:uint8
                                               */
  uint8_T TJAGEN_StrongReadyInvalid_btf;      /* '<S70>/Data Type Conversion1'
                                               * TJAGEN strong ready invalid
                                               bitfield   DT:uint8
                                               */
  boolean_T LKA_LeLnCrvQualityValid_bool;     /* '<S110>/OR' */

  /* Debug */
  boolean_T LKA_LeLnQualityValid_bool; /* '<S112>/OR' */

  /* Debug */
  boolean_T SLC_StrongReadyBothSides_bool;
  boolean_T SLC_WeakReadyLeft_bool;
  boolean_T SLC_WeakReadyBothSides_bool;
  boolean_T SLC_WeakReadyRight_bool;
  boolean_T SLC_TriggerRight_bool;
  boolean_T SLC_TriggerLeft_bool;
  boolean_T SLC_LeverLeftEngaged_bool;
  boolean_T SLC_LeverRightEngaged_bool;
  boolean_T SLC_MaxInitDurationExceeded_bool;
  boolean_T SLC_ManvStatePassive_bool;
  boolean_T SLC_Abort_bool;
  boolean_T SLC_Cancel_bool;
  boolean_T SLC_LCM_End_bool;
  boolean_T SLC_LCM_Start_bool;
  boolean_T SLC_LaneCheckValid_bool;
  boolean_T SLC_NewEgoLane_bool;
  boolean_T SLC_LCM_Cancel_bool;
  boolean_T STM_Cancel_bool;
  boolean_T SLC_PrevReset_bool;
  boolean_T OBF_AccObjValidLaneCheck;
  boolean_T OBF_AccObjValid;
  boolean_T OBF_LeftLaneCheckValid_bool;
  boolean_T OBF_AccObjValid_bool;
  boolean_T OBF_AccObjSwitch;
  boolean_T OBF_MinDist2LeftBndInvalid;
  boolean_T OBF_TargetOutsideEgoLane_bool;
  boolean_T OBF_RightLaneCheckValid_bool;
  boolean_T OBF_MinDist2RightBndInvalid;
  boolean_T OBF_LaneCheckValid_bool;
  boolean_T OBF_DistOrEgoLaneInvalid_bool;
  boolean_T OBF_TargetObjDataWR_bool;
  boolean_T OBF_TargetObjDataSR_bool;
  boolean_T CMB_ObjectFollowingOnly_bool;
  boolean_T CMB_LaneQualityInvalid_bool;
  boolean_T TTG_ObjectUpdate_bool;
  boolean_T TTG_LaneUpdate_bool;
  boolean_T TTG_CMBObjectCorridor_bool;
  boolean_T TTG_Enable_bool;
  boolean_T TTG_LD_PredictFinish_bool;
  boolean_T TTG_LD_Enable_bool;
  boolean_T TTG_CMB_Enable_bool;
  boolean_T TTG_OD_Enable_bool;
  boolean_T TTG_Reset_bool;
  boolean_T TTG_Predict_Enable_bool;

  boolean_T TJAGEN_FunctionQuit_bool;     /* '<S54>/OR'
                                           * Function Quit Flag
                                           */
  uint16_T TJAGEN_SuspendedAndQuit_debug; /* '<S80>/Data Type Conversion1'
                                           * TJAGEN Suspended And Quit debug
                                           */
  boolean_T TJAGEN_SuspendEnd_bool;       /* '<S54>/OR5'
                                           * Suspended end Flag
                                           */
  boolean_T TJAGEN_SuspendStart_bool;     /* '<S54>/OR4'
                                           * Suspended start Flag
                                           */
  E_TJASTM_SysStateTJA_nu
      TJASTM_SysStateTJAIn_nu;             /* '<S8>/StateMachineTJA'
                                               * TJASTM system state
                                                 DT:Enum: E_TJASTM_SysStateTJA_nu
                                               */
  boolean_T STM_SuspendTimeExpired_bool;   /* '<S400>/Switch'
                                            * STE Flag
                                            */
  boolean_T STM_PrevRAMPOUT_bool;          /* '<S382>/Unit Delay'
                                            * Previous rampout Flag
                                            */
  boolean_T SLC_LCPLeft2Active_bool;       /* '<S305>/AND'
                                            * LCPLeft2Active Flag
                                            */
  boolean_T SLC_LCPRight2Active_bool;      /* '<S308>/AND'
                                            * LCPRight2Active Flag
                                            */
  boolean_T SLC_LCPRight2Passive_bool;     /* '<S307>/AND'
                                            * LCPRight2Passive Flag
                                            */
  boolean_T SLC_LCPLeft2Passive_bool;      /* '<S306>/AND'
                                            * LCPLeft2Passive Flag
                                            */
  boolean_T TJASTM_DrvTakeOver_bool;       /* '<S404>/OR'
                                            * TJASTM driver takeover Flag
                                            */
  uint8_T TJATOW_DriverTakeOverWarning_nu; /* '<S9>/Max'
                                               * TJATOW takeover warning
                                                 DT:uint8
                                               */
  uint8_T TJALKA_LaneIncoherence_btf;      /* '<S134>/Data Type Conversion1'
                                               * TJALKA Lane incoherence debug
                                                 DT:uint8
                                               */
  uint8_T TJALKA_LnIncoherenceStatus_nu;   /* '<S129>/Switch1'
                                               * TJALKA Lane incoherence status
                                                 DT:uint8
                                               */
  boolean_T SLC_VehSpdTooLowInfo;
  boolean_T SLC_LaneChangeOnGoingInfo;
  boolean_T SLC_LaneChangeEndInfo;
  boolean_T SLC_LaneChangePendingInfo;
  boolean_T SLC_LaneChangeCancleInfo;

  boolean_T SLC_AllowGoBack_bool;
  real32_T SLC_CenterDistToBoundary_met;
  boolean_T SLC_IntoAbort_nu;
  E_TJASLC_LaneChangeTrig_nu SLC_LaneChangeDirectionIn_nu;
  boolean_T SLC_SameLaneChangeDetc_bool;
  boolean_T SLC_LaneChangeBackDetc_bool;
  boolean_T SLC_ExitAbort_bool;
  boolean_T SLC_ExitAbortNewEgo_bool;
  E_TJASLC_AbortState_nu SLC_AbortState_enum;
  E_TJASLC_LaneChangeTrig_nu SLC_LaneChangeDirectionAbort_enum;
  boolean_T TJASLC_Nb_DCLCSwitchNVRAM_nu;
  int32_T TJASLC_SLCHighLightID_nu;
  uint8_T TJASLC_LaneChangeInfo; /* 0:None; 1: Vechicle Speed too low; 2: Lane
                                    Change Start; 3:Lane Change End; 4:Lane
                                    Change Cancle; 5:Lane Change Pending; 6:
                                    Lane Change OnGoing */
  uint8_T TJASLC_TurnLtDirctionReq_nu;
} sTJASADebug_t;
#endif
extern void LCF_TJASA_Reset(void);
extern void LCF_TJASA_Exec(const sTJASAInReq_t *reqPorts,
                           const sTJASAParam_t *param,
                           sTJASAOutPro_t *proPorts,
                           sTJASADebug_t *debug);

#endif