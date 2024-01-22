/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
#include "TJASA_ext.h"

#define ASW_QM_CORE2_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/* Imported (extern) block signals */
boolean_T ABPR_LaneChangeDetected_bool; /* '<Root>/Inport11',   Lane change
                                           detected flag--from ABPR module */
real32_T
    ABPR_LaneWidth_met; /* '<Root>/Inport5',    Lane width--from ABPR module */
boolean_T ABPR_ConstructionSite_bool; /* '<Root>/Inport13',   Construction site
                                         flag--from ABPR */
real32_T ABPR_UncoupledLaneWidth_met; /* '<Root>/Inport22',   Uncoupled Lane
                                         Width--from ABPR module */
real32_T ABPR_ABDTimeStamp_sec; /* '<Root>/Inport91',   ABD Time Stamp--from
                                   ABPR module */
uint8_T
    ABPR_LeftLaneType_enum; /* '<Root>/Inport54',   Left lane type--from ABPR */
real32_T ABPR_LeLnClthPosX0_met; /* '<Root>/Inport23',   Left Lane Clothoid
                                    PosX0--from ABPR module */
real32_T ABPR_LeLnClthPosY0_met; /* '<Root>/Inport1',    Left lane clothoid
                                    posY0--from ABPR module */
uint16_T ABPR_LeLnInvalidQu_btf; /* '<Root>/Inport7',    Left lane invalid
                                    qualifier--from ABPR */
uint8_T ABPR_LeLnQuality_perc;   /* '<Root>/Inport15',   Left lane quality--from
                                    ABPR module */
uint8_T ABPR_LeCrvQuality_perc;  /* '<Root>/Inport17',   Left lane curve
                                    quality--from ABPR module */
real32_T ABPR_LeLnClthHeading_rad;    /* '<Root>/Inport24',   Left Lane Clothoid
                                         Heading--from ABPR module */
real32_T ABPR_LeLnClthCrv_1pm;        /* '<Root>/Inport25',   Left lane clothoid
                                         curvature--from ABPR module */
real32_T ABPR_LeLnClthCrvChng_1pm2;   /* '<Root>/Inport26',   Left Lane Clothoid
                                         Curve of Change--from ABPR module */
real32_T ABPR_LeLnClthLength_met;     /* '<Root>/Inport27',   Left lane clothoid
                                         length--from ABPR module */
real32_T ABPR_LeAdjLnClthPosX0_met;   /* '<Root>/Inport80',   Left Adjacent Lane
                                         Clothoid PosX0--from ABPR module */
real32_T ABPR_LeAdjLnClthPosY0_met;   /* '<Root>/Inport40',   Left adjacent lane
                                         clothoid posY0--from ABPR module */
uint16_T ABPR_LeAdjLnInvalidQu_btf;   /* '<Root>/Inport43',   Left adjacent lane
                                         invalid quality--from ABPR module */
real32_T ABPR_LeAdjLnClthHeading_rad; /* '<Root>/Inport81',   Left Adjacent Lane
                                         Clothoid Heading--from ABPR module */
real32_T ABPR_LeAdjLnClthCrv_1pm;     /* '<Root>/Inport82',   Left Adjacent Lane
                                         Clothoid Curve--from ABPR module */
real32_T ABPR_LeAdjLnClthCrvChng_1pm2; /* '<Root>/Inport83',   Left Adjacent
                                          Lane Clothoid Curve of Change--from
                                          ABPR module */
real32_T ABPR_LeAdjLnClthLength_met; /* '<Root>/Inport84',   Left Adjacent Lane
                                        Clothoid Length--from ABPR module */
uint8_T ABPR_RightLaneType_enum; /* '<Root>/Inport55',   Right lane type--from
                                    ABPR */
real32_T ABPR_RiLnClthPosX0_met; /* '<Root>/Inport28',   Right Lane Clothoid
                                    PosX0--from ABPR module */
real32_T ABPR_RiLnClthPosY0_met; /* '<Root>/Inport2',    Right lane clothoid
                                    posY0--from ABPR module */
uint16_T ABPR_RiLnInvalidQu_btf; /* '<Root>/Inport6',    Right lane invalid
                                    qualifier--from ABPR */
uint8_T ABPR_RiLnQuality_perc;  /* '<Root>/Inport16',   Right lane quality--from
                                   ABPR module */
uint8_T ABPR_RiCrvQuality_perc; /* '<Root>/Inport18',   Right lane curve
                                   quality--from ABPR module */
real32_T ABPR_RiLnClthHeading_rad;  /* '<Root>/Inport29',   Right Lane Clothoid
                                       Heading--from ABPR module */
real32_T ABPR_RiLnClthCrv_1pm;      /* '<Root>/Inport30',   Right lane clothoid
                                       curvature--from ABPR module */
real32_T ABPR_RiLnClthCrvChng_1pm2; /* '<Root>/Inport31',   Right Lane Clothoid
                                       Curve of Change--from ABPR module */
real32_T ABPR_RiLnClthLength_met;   /* '<Root>/Inport20',   Right lane clothoid
                                       length--from ABPR module */
real32_T ABPR_RiAdjLnClthPosX0_met; /* '<Root>/Inport89',   Right Adjacent Lane
                                       Clothoid PosX0--from ABPR module */
real32_T ABPR_RiAdjLnClthPosY0_met; /* '<Root>/Inport41',   Right adjacent lane
                                       clothoid posY0--from ABPR module */
uint16_T ABPR_RiAdjLnInvalidQu_btf; /* '<Root>/Inport42',   Right adjacent lane
                                       invalid quality--from ABPR module */
real32_T
    ABPR_RiAdjLnClthHeading_rad;  /* '<Root>/Inport85',   Right Adjacent Lane
                                     Clothoid Heading--from ABPR module */
real32_T ABPR_RiAdjLnClthCrv_1pm; /* '<Root>/Inport86',   Right Adjacent Lane
                                     Clothoid Curve--from ABPR module */
real32_T ABPR_RiAdjLnClthCrvChng_1pm2; /* '<Root>/Inport87',   Right Adjacent
                                          Lane Clothoid Curve of Change--from
                                          ABPR module */
real32_T ABPR_RiAdjLnClthLength_met; /* '<Root>/Inport88',   Right Adjacent Lane
                                        Clothoid Length--from ABPR module */
real32_T ABPR_CntrLnClthPosX0_met; /* '<Root>/Inport58',   Control Lane Clothoid
                                      PosX0--from ABPR module */
real32_T ABPR_CntrLnClthPosY0_met; /* '<Root>/Inport3',    Control Lane Clothoid
                                      PosY0--from ABPR module */
real32_T ABPR_CntrLnClthHeading_rad; /* '<Root>/Inport4',    Control lane
                                        clothoid heading--from ABPR module */
real32_T ABPR_CntrLnClthCrv_1pm; /* '<Root>/Inport9',    Control Lane Clothoid
                                    Curve--from ABPR module */
real32_T ABPR_CntrLnClthCrvChng_1pm2;  /* '<Root>/Inport59',   Control Lane
                                          Clothoid Curve of Change--from ABPR
                                          module */
real32_T ABPR_CntrLnClthLength_met;    /* '<Root>/Inport21',   Control Lane
                                          Clothoid Length--from ABPR module */
uint16_T ODPFOH_TgtObjClothoidInv_btf; /* '<Root>/Inport32',   Target object
                                          clothoid invalid qualifier--from ODPR
                                          module */
real32_T ODPFOH_TgtObjPosX0_met; /* '<Root>/Inport60',   Target object Clothoid
                                    PosX0--from ODPR module */
real32_T ODPFOH_TgtObjPosY0_met; /* '<Root>/Inport36',   Target object Clothoid
                                    PosY0--from ODPR module */
real32_T ODPFOH_TgtObjHeadAng_rad; /* '<Root>/Inport37',   Target object
                                      Clothoid Heading--from ODPR module */
real32_T ODPFOH_TgtObjCrv_1pm; /* '<Root>/Inport38',   Target object Clothoid
                                  Curve--from ODPR module */
real32_T ODPFOH_TgtObjCrvChng_1pm2; /* '<Root>/Inport61',   Target object
                                       Clothoid Curve of Change--from ODPR
                                       module */
real32_T ODPFOH_TgtObjLength_met; /* '<Root>/Inport39',   Target object Clothoid
                                     Length--from ODPR module */
real32_T ODPFOP_AccFRObjTStamp_sec;    /* '<Root>/Inport92',   Acc object time
                                          stamp--from ODPR module */
uint16_T ODPFOP_AccObjInvBitfield_btf; /* '<Root>/Inport33',   Acc object
                                          invalid qualifier--from ODPR module */
real32_T ODPFOP_AccObjPosX_met; /* '<Root>/Inport34',   Acc obejct PosX--from
                                   ODPR module */
real32_T ODPFOP_AccObjPosY_met; /* '<Root>/Inport35',   Acc obejct PosY--from
                                   ODPR module */
boolean_T
    LCA_ActiveLeft_bool; /* '<Root>/Inport46',   Active left flag--from LCA */
boolean_T
    LCA_ActiveRight_bool; /* '<Root>/Inport47',   Active right flag--from LCA */
boolean_T
    LCA_WarningLeft_bool; /* '<Root>/Inport48',   Warning left flag--from LCA */
boolean_T LCA_WarningRight_bool; /* '<Root>/Inport49',   Warning right
                                    flag--from LCA */
boolean_T
    BSD_ActiveLeft_bool; /* '<Root>/Inport50',   Active left flag--from BSD */
boolean_T
    BSD_ActiveRight_bool; /* '<Root>/Inport51',   Active right flag--from BSD */
boolean_T
    BSD_WarningLeft_bool; /* '<Root>/Inport52',   Warning left flag--from BSD */
boolean_T BSD_WarningRight_bool; /* '<Root>/Inport53',   Warning right
                                    flag--from BSD */
uint16_T CUSTOM_PrjSpecQu_btf; /* '<Root>/Inport12',   Specific qualifier--from
                                  CUSTOM */
boolean_T S_ODPSOP_MSFlag_RearLeft_nu;  /* '<Root>/Inport56',   MS flag_rear
                                           left--from ODPSOP */
boolean_T S_ODPSOP_MSFlag_RearRight_nu; /* '<Root>/Inport57',   MS flag_rear
                                           right--from ODPSOP */
uint16_T TRJPLN_QuStatusTrajPlan_nu;    /* '<Root>/Inport62',   Trajectory plan
                                           qualifier--from TRJPLN module */
uint8_T TRJCTR_QuServTrajCtr_nu; /* '<Root>/Inport63',   Trajectory control
                                    qualifier--from TRJCTR module */
E_MCTLCF_ControllingFunction_nu
    MDCTR_ControllingFunction_nu;     /* '<Root>/Inport64',    Controlling
                                         function--from MDCTR module */
real32_T LCFRCV_TSysCycleTimeSen_sec; /* '<Root>/Inport10',   System cycle
                                        time--from LCFRCV */
real32_T VDy_VehVelocity_kph; /* '<Root>/Inport8',    Vehicle velocity--from VDy
                                 module */
boolean_T LCFRCV_TurnSignalLeverHold_bool; /* '<Root>/Inport19',   Turn signal
                                              level hold flag--from LCFRCV */
boolean_T LCFRCV_TurnSignalLeft_bool;  /* '<Root>/Inport44',   Turn signal left
                                          flag--from LCFRCV */
boolean_T LCFRCV_TurnSignalRight_bool; /* '<Root>/Inport45',   Turn signal right
                                          flag--from LCFRCV */
uint8_T
    LCFRCV_DrivingMode_nu; /* '<Root>/Inport93',   Driving mode--from LCFRCV */
boolean_T
    LCFRCV_LKASwitch_nu; /* '<Root>/Inport65',   LKA switch flag--from LCFRCV */
boolean_T
    LCFRCV_TJASwitch_nu; /* '<Root>/Inport66',   TJA switch flag--from LCFRCV */
boolean_T LCFRCV_ErrorStateTJA_bool; /* '<Root>/Inport67',   Error state of
                                        TJA--from LCFRCV */
boolean_T LCFRCV_ErrorStateLKA_bool; /* '<Root>/Inport68',   Error state of
                                        LKA--from LCFRCV */
real32_T VDy_VehAclX_mps2; /* '<Root>/Inport69',   Vehicle acceleration X--from
                              VDy module */
real32_T VDy_VehAclY_mps2; /* '<Root>/Inport70',   Vehicle acceleration Y--from
                              VDy module */
boolean_T LCFRCV_SysStOnLatDMC_bool; /* '<Root>/Inport14',   System state on
                                        latDMC--from LCFRCV */
real32_T LCFRCV_SteerWAngle_deg; /* '<Root>/Inport77',   steer wheel angle--from
                                    LCRCV */
real32_T
    VDy_VehCrv_1pm; /* '<Root>/Inport76',   Vehicle curve--from VDy module */
real32_T VDy_VehYawRate_rps; /* '<Root>/Inport78',   Vehicle yawrate--from VDy
                                module */
real32_T VDy_VehVelX_mps; /* '<Root>/Inport79',   Vehicle velocity X--from VDy
                             module */
boolean_T LCFRCV_VehStopped_nu;  /* '<Root>/Inport90',   Vehicle stopped
                                    flag--from LCFRCV */
uint8_T VDPDRV_ActiveStCtrl_btf; /* '<Root>/Inport71',   Acitve state
                                    control--from VDPDRV module */
uint8_T VDPDRV_SysStError_btf; /* '<Root>/Inport72',   System state error--from
                                  VDPDRV module */
uint16_T VDPDRV_VehStInvalid_btf; /* '<Root>/Inport73',   Vehicle state invalid
                                     bitfield--from VDPDRV module */
uint8_T VDPDRV_DrvStInvalid_btf;  /* '<Root>/Inport74',   Driver state invalid
                                     bitfield--from VDPDRV module */
uint8_T
    VDPDRV_SysStNotAvailable_btf; /* '<Root>/Inport75',   System state not
                                     avaliable bitfield--from VDPDRV module */

real32_T VDy_DashboardVelocity_kph; /* '<Root>/Inport94',   Dashbord
                                       velocity--from VDy module */
real32_T LCFRCV_ManualTorque_nm; /* '<Root>/Inport95',   Manual torque -- from
                                    LCFRCV */
real32_T LCFRCV_SteerWAngleGrad_degps; /* '<Root>/Inport96',   Steering wheel
                                          angle grad -- from LCFRCV */
boolean_T LCFRCV_SLCHMISwitch_bool; /* '<Root>/Inport97',   SLC HMI switch--from
                                       LCFRCV */
boolean_T LCFRCV_TJAAudioSwitch_bool; /* '<Root>/Inport98',   TJA audio
                                         switch--from LCFRCV */

boolean_T
    LCCRA_InLeftSafeFlag_bool; /* '<Root>/Inport99'
                                * Lane change detected flag--from ABPR module
                                */
boolean_T
    LCCRA_InRightSafeFlag_bool;            /* '<Root>/Inport100'
                                            * Lane change detected flag--from ABPR module
                                            */
boolean_T LCCRA_InLeftFrontSafeFlag_bool;  /* '<Root>/Inport101'
                                            * Lane change detected flag--from
                                            * ABPR module
                                            */
boolean_T LCCRA_InRightFrontSafeFlag_bool; /* '<Root>/Inport102'
                                            * Lane change detected
                                            * flag--from ABPR module
                                            */
boolean_T LCCRA_InLeftRearSafeFlag_bool;   /* '<Root>/Inport103'
                                            * Lane change detected flag--from ABPR
                                            * module
                                            */
boolean_T LCCRA_InRightRearSafeFlag_bool;  /* '<Root>/Inport104'
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
boolean_T LCFSEN_Nb_DCLCSwitchNVRAM_nu;    /* '<Root>/Inport108'
                                            * Right lane clothoid
                                            * curvature--from ABPR module
                                            */
int32_T LCCRA_InLeftHighLightID_nu;        /* '<Root>/Inport109'
                                            * Right lane clothoid curvature--from ABPR
                                            * module
                                            */
int32_T LCCRA_InRightHighLightID_nu;       /* '<Root>/Inport110'
                                            * Right lane clothoid curvature--from
                                            * ABPR module
                                            */
uint8_T LCFRCV_TurnLightReqSt_nu;          /* '<Root>/Inport111'
                                            * TJA switch flag--from LCFRCV
                                            */
boolean_T LCFRCV_DoorOpen_bool;            /* '<Root>/Inport112'
                                            * Active left flag--from LCA
                                            */
boolean_T LCFRCV_HoodOpen_bool;            /* '<Root>/Inport113'
                                            * Active left flag--from LCA
                                            */
boolean_T LCFRCV_TrunkUnLock_bool;         /* '<Root>/Inport114'
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
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  Functionname:     LCF_TJASA_Reset                                  */ /*!

                  @brief:           TJASA function reset

                  @description:     All global variables related to TJASA are
                reset in
                this function
                                    when TAJSA executes for the first time, or
                system
                exception needs
                                    to be reset

                  @param[in]:       void

                  @return:          void
                *****************************************************************************/
void LCF_TJASA_Reset(void) { TJASA_initialize(); }

/*****************************************************************************
  Functionname:     LCF_TJASA_Exec                                  */ /*!

                    @brief:           Execution entry of TJASA function

                    @description:     MCTLFC main function

                    @param[in]:       reqPorts   MCTLFC input
                                      params     MCTLFC parameter input
                                      proPorts   MCTLFC output
                                      debugInfo  MCTLFC debug information

                    @return:void
                  *****************************************************************************/
void LCF_TJASA_Exec(const sTJASAInReq_t* reqPorts,
                    const sTJASAParam_t* param,
                    sTJASAOutPro_t* proPorts,
                    sTJASADebug_t* debug) {
    // printf("--------------------TJASA--------------------\n");
    /* TJASA input wrapper */
    ABPR_LaneChangeDetected_bool = reqPorts->ABPR_LaneChangeDetected_bool;
    ABPR_LaneWidth_met = reqPorts->ABPR_LaneWidth_met;
    ABPR_ConstructionSite_bool = reqPorts->ABPR_ConstructionSite_bool;
    ABPR_UncoupledLaneWidth_met = reqPorts->ABPR_UncoupledLaneWidth_met;
    ABPR_ABDTimeStamp_sec = reqPorts->ABPR_ABDTimeStamp_sec;
    ABPR_LeftLaneType_enum = reqPorts->ABPR_LeftLaneType_enum;
    ABPR_LeLnClthPosX0_met = reqPorts->ABPR_LeLnClthPosX0_met;
    ABPR_LeLnClthPosY0_met = reqPorts->ABPR_LeLnClthPosY0_met;
    ABPR_LeLnInvalidQu_btf = reqPorts->ABPR_LeLnInvalidQu_btf;
    ABPR_LeLnQuality_perc = reqPorts->ABPR_LeLnQuality_perc;
    ABPR_LeCrvQuality_perc = reqPorts->ABPR_LeCrvQuality_perc;
    ABPR_LeLnClthHeading_rad = reqPorts->ABPR_LeLnClthHeading_rad;
    ABPR_LeLnClthCrv_1pm = reqPorts->ABPR_LeLnClthCrv_1pm;
    ABPR_LeLnClthCrvChng_1pm2 = reqPorts->ABPR_LeLnClthCrvChng_1pm2;
    ABPR_LeLnClthLength_met = reqPorts->ABPR_LeLnClthLength_met;
    ABPR_LeAdjLnClthPosX0_met = reqPorts->ABPR_LeAdjLnClthPosX0_met;
    ABPR_LeAdjLnClthPosY0_met = reqPorts->ABPR_LeAdjLnClthPosY0_met;
    ABPR_LeAdjLnInvalidQu_btf = reqPorts->ABPR_LeAdjLnInvalidQu_btf;
    ABPR_LeAdjLnClthHeading_rad = reqPorts->ABPR_LeAdjLnClthHeading_rad;
    ABPR_LeAdjLnClthCrv_1pm = reqPorts->ABPR_LeAdjLnClthCrv_1pm;
    ABPR_LeAdjLnClthCrvChng_1pm2 = reqPorts->ABPR_LeAdjLnClthCrvChng_1pm2;
    ABPR_LeAdjLnClthLength_met = reqPorts->ABPR_LeAdjLnClthLength_met;
    ABPR_RightLaneType_enum = reqPorts->ABPR_RightLaneType_enum;
    ABPR_RiLnClthPosX0_met = reqPorts->ABPR_RiLnClthPosX0_met;
    ABPR_RiLnClthPosY0_met = reqPorts->ABPR_RiLnClthPosY0_met;
    ABPR_RiLnInvalidQu_btf = reqPorts->ABPR_RiLnInvalidQu_btf;
    ABPR_RiLnQuality_perc = reqPorts->ABPR_RiLnQuality_perc;
    ABPR_RiCrvQuality_perc = reqPorts->ABPR_RiCrvQuality_perc;
    ABPR_RiLnClthHeading_rad = reqPorts->ABPR_RiLnClthHeading_rad;
    ABPR_RiLnClthCrv_1pm = reqPorts->ABPR_RiLnClthCrv_1pm;
    ABPR_RiLnClthCrvChng_1pm2 = reqPorts->ABPR_RiLnClthCrvChng_1pm2;
    ABPR_RiLnClthLength_met = reqPorts->ABPR_RiLnClthLength_met;
    ABPR_RiAdjLnClthPosX0_met = reqPorts->ABPR_RiAdjLnClthPosX0_met;
    ABPR_RiAdjLnClthPosY0_met = reqPorts->ABPR_RiAdjLnClthPosY0_met;
    ABPR_RiAdjLnInvalidQu_btf = reqPorts->ABPR_RiAdjLnInvalidQu_btf;
    ABPR_RiAdjLnClthHeading_rad = reqPorts->ABPR_RiAdjLnClthHeading_rad;
    ABPR_RiAdjLnClthCrv_1pm = reqPorts->ABPR_RiAdjLnClthCrv_1pm;
    ABPR_RiAdjLnClthCrvChng_1pm2 = reqPorts->ABPR_RiAdjLnClthCrvChng_1pm2;
    ABPR_RiAdjLnClthLength_met = reqPorts->ABPR_RiAdjLnClthLength_met;
    ABPR_CntrLnClthPosX0_met = reqPorts->ABPR_CntrLnClthPosX0_met;
    ABPR_CntrLnClthPosY0_met = reqPorts->ABPR_CntrLnClthPosY0_met;
    ABPR_CntrLnClthHeading_rad = reqPorts->ABPR_CntrLnClthHeading_rad;
    ABPR_CntrLnClthCrv_1pm = reqPorts->ABPR_CntrLnClthCrv_1pm;
    ABPR_CntrLnClthCrvChng_1pm2 = reqPorts->ABPR_CntrLnClthCrvChng_1pm2;
    ABPR_CntrLnClthLength_met = reqPorts->ABPR_CntrLnClthLength_met;
    ODPFOH_TgtObjClothoidInv_btf = reqPorts->ODPFOH_TgtObjClothoidInv_btf;
    ODPFOH_TgtObjPosX0_met = reqPorts->ODPFOH_TgtObjPosX0_met;
    ODPFOH_TgtObjPosY0_met = reqPorts->ODPFOH_TgtObjPosY0_met;
    ODPFOH_TgtObjHeadAng_rad = reqPorts->ODPFOH_TgtObjHeadAng_rad;
    ODPFOH_TgtObjCrv_1pm = reqPorts->ODPFOH_TgtObjCrv_1pm;
    ODPFOH_TgtObjCrvChng_1pm2 = reqPorts->ODPFOH_TgtObjCrvChng_1pm2;
    ODPFOH_TgtObjLength_met = reqPorts->ODPFOH_TgtObjLength_met;
    ODPFOP_AccFRObjTStamp_sec = reqPorts->ODPFOP_AccFRObjTStamp_sec;
    ODPFOP_AccObjInvBitfield_btf = reqPorts->ODPFOP_AccObjInvBitfield_btf;
    ODPFOP_AccObjPosX_met = reqPorts->ODPFOP_AccObjPosX_met;
    ODPFOP_AccObjPosY_met = reqPorts->ODPFOP_AccObjPosY_met;
    LCA_ActiveLeft_bool = reqPorts->LCA_ActiveLeft_bool;
    LCA_ActiveRight_bool = reqPorts->LCA_ActiveRight_bool;
    LCA_WarningLeft_bool = reqPorts->LCA_WarningLeft_bool;
    LCA_WarningRight_bool = reqPorts->LCA_WarningRight_bool;
    BSD_ActiveLeft_bool = reqPorts->BSD_ActiveLeft_bool;
    BSD_ActiveRight_bool = reqPorts->BSD_ActiveRight_bool;
    BSD_WarningLeft_bool = reqPorts->BSD_WarningLeft_bool;
    BSD_WarningRight_bool = reqPorts->BSD_WarningRight_bool;
    CUSTOM_PrjSpecQu_btf = reqPorts->CUSTOM_PrjSpecQu_btf;
    S_ODPSOP_MSFlag_RearLeft_nu = reqPorts->S_ODPSOP_MSFlag_RearLeft_nu;
    S_ODPSOP_MSFlag_RearRight_nu = reqPorts->S_ODPSOP_MSFlag_RearRight_nu;
    TRJPLN_QuStatusTrajPlan_nu = reqPorts->TRJPLN_QuStatusTrajPlan_nu;
    TRJCTR_QuServTrajCtr_nu = reqPorts->TRJCTR_QuServTrajCtr_nu;
    MDCTR_ControllingFunction_nu = reqPorts->MDCTR_ControllingFunction_nu;
    LCFRCV_TSysCycleTimeSen_sec = reqPorts->LCFRCV_SysCycleTimeSen_sec;
    VDy_VehVelocity_kph = reqPorts->VDy_VehVelocity_kph;
    LCFRCV_TurnSignalLeverHold_bool = reqPorts->LCFRCV_TurnSignalLeverHold_bool;
    LCFRCV_TurnSignalLeft_bool = reqPorts->LCFRCV_TurnSignalLeft_bool;
    LCFRCV_TurnSignalRight_bool = reqPorts->LCFRCV_TurnSignalRight_bool;
    LCFRCV_DrivingMode_nu = reqPorts->LCFRCV_DrivingMode_nu;
    LCFRCV_LKASwitch_nu = reqPorts->LCFRCV_LKASwitch_nu;
    LCFRCV_TJASwitch_nu = reqPorts->LCFRCV_TJASwitch_nu;
    LCFRCV_ErrorStateTJA_bool = reqPorts->LCFRCV_ErrorStateTJA_bool;
    LCFRCV_ErrorStateLKA_bool = reqPorts->LCFRCV_ErrorStateLKA_bool;
    VDy_VehAclX_mps2 = reqPorts->VDy_VehAclX_mps2;
    VDy_VehAclY_mps2 = reqPorts->VDy_VehAclY_mps2;
    LCFRCV_SysStOnLatDMC_bool = reqPorts->LCFRCV_SysStOnLatDMC_bool;
    LCFRCV_SteerWAngle_deg = reqPorts->LCFRCV_SteerWAngle_deg;
    VDy_VehCrv_1pm = reqPorts->VDy_VehCrv_1pm;
    VDy_VehYawRate_rps = reqPorts->VDy_VehYawRate_rps;
    VDy_VehVelX_mps = reqPorts->VDy_VehVelX_mps;
    LCFRCV_VehStopped_nu = reqPorts->LCFRCV_VehStopped_nu;
    VDPDRV_ActiveStCtrl_btf = reqPorts->VDPDRV_ActiveStCtrl_btf;
    VDPDRV_SysStError_btf = reqPorts->VDPDRV_SysStError_btf;
    VDPDRV_VehStInvalid_btf = reqPorts->VDPDRV_VehStInvalid_btf;
    VDPDRV_DrvStInvalid_btf = reqPorts->VDPDRV_DrvStInvalid_btf;
    VDPDRV_SysStNotAvailable_btf = reqPorts->VDPDRV_SysStNotAvailable_btf;

    VDy_DashboardVelocity_kph = reqPorts->VDy_DashboardVelocity_kph;
    LCFRCV_ManualTorque_nm = reqPorts->LCFRCV_ManualTorque_nm;
    LCFRCV_SteerWAngleGrad_degps = reqPorts->LCFRCV_SteerWAngleGrad_degps;
    LCFRCV_SLCHMISwitch_bool = reqPorts->LCFRCV_SLCHMISwitch_bool;
    LCFRCV_TJAAudioSwitch_bool = reqPorts->LCFRCV_TJAAudioSwitch_bool;

    LCCRA_InLeftSafeFlag_bool = reqPorts->LCCRA_LeftSafeFlag_bool;
    LCCRA_InRightSafeFlag_bool = reqPorts->LCCRA_RightSafeFlag_bool;
    LCCRA_InLeftFrontSafeFlag_bool = reqPorts->LCCRA_LeftFrontSafeFlag_bool;
    LCCRA_InRightFrontSafeFlag_bool = reqPorts->LCCRA_RightFrontSafeFlag_bool;
    LCCRA_InLeftRearSafeFlag_bool = reqPorts->LCCRA_LeftRearSafeFlag_bool;
    LCCRA_InRightRearSafeFlag_bool = reqPorts->LCCRA_RightRearSafeFlag_bool;
    LCFRCV_PilotOnLeverSwitch_bool = reqPorts->LCFRCV_PilotOnLeverSwitch_bool;
    LCFRCV_PilotOffLeverSwitch_bool = reqPorts->LCFRCV_PilotOffLeverSwitch_bool;
    VLCVEH_ACCStatus_nu = reqPorts->VLCVEH_ACCStatus_nu;
    LCFSEN_Nb_DCLCSwitchNVRAM_nu = reqPorts->LCFSEN_Nb_DCLCSwitchNVRAM_nu;
    LCCRA_InLeftHighLightID_nu = reqPorts->LCCRA_LeftHighLightID_nu;
    LCCRA_InRightHighLightID_nu = reqPorts->LCCRA_RightHighLightID_nu;
    LCFRCV_TurnLightReqSt_nu = reqPorts->LCFRCV_TurnLightReqSt_nu;

    LCFRCV_DoorOpen_bool = reqPorts->LCFRCV_DoorOpen_bool;
    LCFRCV_HoodOpen_bool = reqPorts->LCFRCV_HoodOpen_bool;
    LCFRCV_TrunkUnLock_bool = reqPorts->LCFRCV_TrunkUnLock_bool;
    LCFRCV_bPGear_bool = reqPorts->LCFRCV_bPGear_bool;
    LCCRA_inFrontSafeFlag_bool = reqPorts->LCCRA_inFrontSafeFlag_bool;
    LCFRCV_inFrontDangerObjID_nu = reqPorts->LCFRCV_inFrontDangerObjID_nu;
    EMFID_bRCA_bool = reqPorts->EMFID_bRCA_bool;
    EMFID_bRCC_bool = reqPorts->EMFID_bRCC_bool;
    EMFID_bFCA_bool = reqPorts->EMFID_bFCA_bool;
    EMFID_bFCB_bool = reqPorts->EMFID_bFCB_bool;
    LCFRCV_bPilotOnOff_bool = reqPorts->LCFRCV_bPilotOnOff_bool;
    ADCS4_AVM_Sts = reqPorts->ADCS4_AVM_Sts;
    ADCS11_Parking_WorkSts = reqPorts->ADCS11_Parking_WorkSts;

    /* TJASA function */
    TJASA_step();

    /* TJASA output wrapper */
    proPorts->TJASTM_SysStateTJA_nu = TJASTM_SysStateTJA_nu;
    proPorts->TJASTM_LatCtrlMode_nu = TJASTM_LatCtrlMode_nu;
    proPorts->TJATTG_LeCridrBndPosX0_met = TJATTG_LeCridrBndPosX0_met;
    proPorts->TJATTG_LeCridrBndPosY0_met = TJATTG_LeCridrBndPosY0_met;
    proPorts->TJATTG_LeCridrBndHeadAng_rad = TJATTG_LeCridrBndHeadAng_rad;
    proPorts->TJATTG_LeCridrBndCrv_1pm = TJATTG_LeCridrBndCrv_1pm;
    proPorts->TJATTG_LeCridrBndCrvChng_1pm2 = TJATTG_LeCridrBndCrvChng_1pm2;
    proPorts->TJATTG_LeCridrBndLength_met = TJATTG_LeCridrBndLength_met;
    proPorts->TJATTG_RiCridrBndPosX0_met = TJATTG_RiCridrBndPosX0_met;
    proPorts->TJATTG_RiCridrBndPosY0_met = TJATTG_RiCridrBndPosY0_met;
    proPorts->TJATTG_RiCridrBndHeadAng_rad = TJATTG_RiCridrBndHeadAng_rad;
    proPorts->TJATTG_RiCridrBndCrv_1pm = TJATTG_RiCridrBndCrv_1pm;
    proPorts->TJATTG_RiCridrBndCrvChng_1pm2 = TJATTG_RiCridrBndCrvChng_1pm2;
    proPorts->TJATTG_RiCridrBndLength_met = TJATTG_RiCridrBndLength_met;
    proPorts->TJATTG_TgtTrajPosX0_met = TJATTG_TgtTrajPosX0_met;
    proPorts->TJATTG_TgtTrajPosY0_met = TJATTG_TgtTrajPosY0_met;
    proPorts->TJATTG_TgtTrajHeadAng_rad = TJATTG_TgtTrajHeadAng_rad;
    proPorts->TJATTG_TgtTrajCrv_1pm = TJATTG_TgtTrajCrv_1pm;
    proPorts->TJATTG_TgtTrajCrvChng_1pm2 = TJATTG_TgtTrajCrvChng_1pm2;
    proPorts->TJATTG_TgtTrajLength_met = TJATTG_TgtTrajLength_met;
    proPorts->TJATVG_DistYTolLeTgtArea_met = TJATVG_DistYTolLeTgtArea_met;
    proPorts->TJATVG_DistYTolRiTgtArea_met = TJATVG_DistYTolRiTgtArea_met;
    proPorts->TJATVG_FTireAclMax_mps2 = TJATVG_FTireAclMax_mps2;
    proPorts->TJATVG_FTireAclMin_mps2 = TJATVG_FTireAclMin_mps2;
    proPorts->TJATVG_WeightTgtDistY_nu = TJATVG_WeightTgtDistY_nu;
    proPorts->TJATVG_WeightEndTime_nu = TJATVG_WeightEndTime_nu;
    proPorts->TJATVG_PredTimeCrv_sec = TJATVG_PredTimeCrv_sec;
    proPorts->TJATVG_PredTimeHeadAng_sec = TJATVG_PredTimeHeadAng_sec;
    proPorts->TJATVG_MaxCrvTrajGuiCtl_1pm = TJATVG_MaxCrvTrajGuiCtl_1pm;
    proPorts->TJATVG_MaxCrvGrdBuildup_1pms = TJATVG_MaxCrvGrdBuildup_1pms;
    proPorts->TJATVG_MaxCrvGrdRed_1pms = TJATVG_MaxCrvGrdRed_1pms;
    proPorts->TJATVG_MaxCrvGrdTGC_1pms = TJATVG_MaxCrvGrdTGC_1pms;
    proPorts->TJATVG_PlanningHorizon_sec = TJATVG_PlanningHorizon_sec;
    proPorts->TJATVG_TrqRampGrad_1ps = TJATVG_TrqRampGrad_1ps;
    proPorts->TJATVG_StrWhStifLimit_nu = TJATVG_StrWhStifLimit_nu;
    proPorts->TJATVG_MaxTrqScalLimit_nu = TJATVG_MaxTrqScalLimit_nu;
    proPorts->TJATVG_StrWhStifGrad_1ps = TJATVG_StrWhStifGrad_1ps;
    proPorts->TJATVG_MaxTrqScalGrad_1ps = TJATVG_MaxTrqScalGrad_1ps;
    proPorts->TJATVG_MaxJerkAllowed_mps3 = TJATVG_MaxJerkAllowed_mps3;
    proPorts->TJATVG_SensorTStamp_sec = TJATVG_SensorTStamp_sec;
    proPorts->TJATVG_ObstacleVelX_mps = TJATVG_ObstacleVelX_mps;
    proPorts->TJATVG_ObstacleAclX_mps2 = TJATVG_ObstacleAclX_mps2;
    proPorts->TJATVG_ObstacleWidth_met = TJATVG_ObstacleWidth_met;
    proPorts->TJATVG_ObstacleDistX_met = TJATVG_ObstacleDistX_met;
    proPorts->TJATVG_ObstacleDistY_met = TJATVG_ObstacleDistY_met;
    proPorts->TJATVG_LimiterTimeDuration_sec = TJATVG_LimiterTimeDuration_sec;
    proPorts->TJATTG_TgtCorridorInvalid_btf = TJATTG_TgtCorridorInvalid_btf;
    proPorts->TJATVG_TrajPlanServQu_nu = TJATVG_TrajPlanServQu_nu;
    proPorts->TJATVG_DeratingLevel_nu = TJATVG_DeratingLevel_nu;
    proPorts->TJATVG_CrvAmplActivated_nu = TJATVG_CrvAmplActivated_nu;
    proPorts->TJATVG_LimiterActivated_nu = TJATVG_LimiterActivated_nu;
    proPorts->TJATVG_LimiterType_nu = TJATVG_LimiterType_nu;
    proPorts->TJATTG_TransTriggerReplan_bool = TJATTG_TransTriggerReplan_bool;
    proPorts->TJATVG_TriggerReplan_nu = TJATVG_TriggerReplan_nu;
    proPorts->TJATVG_HighStatAccu_bool = TJATVG_HighStatAccu_bool;
    proPorts->TJATVG_LtcyCompActivated_nu = TJATVG_LtcyCompActivated_nu;
    proPorts->TJATVG_TrajGuiQu_nu = TJATVG_TrajGuiQu_nu;
    proPorts->TJASLC_LaneChangWarning_nu = TJASLC_LaneChangWarning_nu;
    proPorts->TJASTM_SysStateHWA_nu = TJASTM_SysStateHWA_nu;
    proPorts->TJASTM_NpilotSysInfo = TJASTM_NpilotSysInfo;
    proPorts->TJASTM_PilotAudioPlay = TJASTM_PilotAudioPlay;
    proPorts->TJASTM_LatCtrlHandsOffReleaseWarn_nu =
        TJASTM_LatCtrlHandsOffReleaseWarn_nu;
    proPorts->TJASLC_SLCAudioPlay_nu = TJASLC_SLCAudioPlay_nu;
    proPorts->TJASTM_PilotEnableACCSwitch_nu = TJASTM_PilotEnableACCSwitch_nu;
    proPorts->TJASTM_PilotDisableACCSwitch_nu = TJASTM_PilotDisableACCSwitch_nu;
    proPorts->TJASLC_Nb_DCLCSwitchNVRAM_nu = TJASLC_Nb_DCLCSwitchNVRAM_nu;
    proPorts->TJASLC_SLCHighLightID_nu = TJASLC_SLCHighLightID_nu;
    proPorts->TJASLC_LaneChangeInfo = TJASLC_LaneChangeInfo;
    proPorts->TJASLC_TurnLtDirctionReq_nu = TJASLC_TurnLtDirctionReq_nu;

    /* TJASA debug wrapper */
    debug->TJACMB_CombinedPosX0_met = TJACMB_CombinedPosX0_met;
    debug->TJACMB_CombinedPosY0_met = TJACMB_CombinedPosY0_met;
    debug->TJACMB_CombinedHeading_rad = TJACMB_CombinedHeading_rad;
    debug->TJACMB_CombinedCrv_1pm = TJACMB_CombinedCrv_1pm;
    debug->TJACMB_CombinedCrvChng_1pm2 = TJACMB_CombinedCrvChng_1pm2;
    debug->TJACMB_CombinedLength_met = TJACMB_CombinedLength_met;
    debug->TJACMB_LaneCrvStdDev_nu = TJACMB_LaneCrvStdDev_nu;
    debug->TJACMB_TraceCrvStdDev_nu = TJACMB_TraceCrvStdDev_nu;
    debug->TJASLC_ReadyToTrigger_nu = TJASLC_ReadyToTrigger_nu;
    debug->TJASLC_ManeuverState_nu = TJASLC_ManeuverState_nu;
    debug->TJASLC_LaneChangeTrig_nu = TJASLC_LaneChangeTrig_nu;
    debug->TJALKA_LnBndValid_nu = TJALKA_LnBndValid_nu;
    debug->TJACMB_Cancel_bool = TJACMB_Cancel_bool;
    debug->TJALKA_LanePredictValid_bool = TJALKA_LanePredictValid_bool;
    debug->TJALKA_Cancel_bool = TJALKA_Cancel_bool;
    debug->TJALKA_StrongReady_bool = TJALKA_StrongReady_bool;
    debug->TJALKA_WeakReady_bool = TJALKA_WeakReady_bool;
    debug->TJAOBF_ObjLaneValidDuration_bool = TJAOBF_ObjLaneValidDuration_bool;
    debug->TJAOBF_TgtObjDataValid_bool = TJAOBF_TgtObjDataValid_bool;
    debug->TJAGEN_LKAOnlySwitch_bool = TJAGEN_LKAOnlySwitch_bool;
    debug->TJAOBF_StrongReady_bool = TJAOBF_StrongReady_bool;
    debug->TJAOBF_Cancel_bool = TJAOBF_Cancel_bool;
    debug->TJAOBF_WeakReady_bool = TJAOBF_WeakReady_bool;
    debug->TJAGEN_Clearance_bool = TJAGEN_Clearance_bool;
    debug->TJAGEN_Degradation_bool = TJAGEN_Degradation_bool;
    debug->TJASLC_Cancel_bool = TJASLC_Cancel_bool;
    debug->TJASLC_StrongReady_bool = TJASLC_StrongReady_bool;
    debug->TJASLC_WeakReady_bool = TJASLC_WeakReady_bool;
    debug->TJASLC_TakeOverValid_bool = TJASLC_TakeOverValid_bool;
    debug->TJACMB_ObjectCorridor_bool = TJACMB_ObjectCorridor_bool;
    debug->TJACMB_StrongReady_bool = TJACMB_StrongReady_bool;
    debug->TJACMB_WeakReady_bool = TJACMB_WeakReady_bool;
    debug->TJAGEN_FunctionSwitch_bool = TJAGEN_FunctionSwitch_bool;
    debug->TJAGEN_CodeFunction_bool = TJAGEN_CodeFunction_bool;
    debug->TJAGEN_Error_bool = TJAGEN_Error_bool;
    debug->TJAGEN_Abort_bool = TJAGEN_Abort_bool;
    debug->TJAGEN_Cancel_nu = TJAGEN_Cancel_nu;
    debug->TJAGEN_StrongReady_bool = TJAGEN_StrongReady_bool;
    debug->TJAGEN_WeakReady_bool = TJAGEN_WeakReady_bool;
    debug->TJATTG_PredictionEnable_bool = TJATTG_PredictionEnable_bool;
    debug->TJAGEN_CancelStatus_btf = TJAGEN_CancelStatus_btf;
    debug->TJAGEN_WeakReadyInvalid_btf = TJAGEN_WeakReadyInvalid_btf;
    debug->TJALKA_LaneCenterInvalid_btf = TJALKA_LaneCenterInvalid_btf;
    debug->TJAOBF_ObjInLaneInvalid_btf = TJAOBF_ObjInLaneInvalid_btf;
    debug->TJAOBF_ObjFollowInvalid_btf = TJAOBF_ObjFollowInvalid_btf;
    debug->TJASLC_LeLaneChangeInvalid_btf = TJASLC_LeLaneChangeInvalid_btf;
    debug->TJASLC_RiLaneChangeInvalid_btf = TJASLC_RiLaneChangeInvalid_btf;
    debug->TJASLC_TriggerInvalid_btf = TJASLC_TriggerInvalid_btf;
    debug->TJACMB_CombinedInvalid_btf = TJACMB_CombinedInvalid_btf;
    debug->TJASTM_TJAInvalid_btf = TJASTM_TJAInvalid_btf;
    debug->TJALKA_LnQualityInv_btf = TJALKA_LnQualityInv_btf;
    debug->TJAOBF_TgtObjDataInvalid_btf = TJAOBF_TgtObjDataInvalid_btf;
    debug->TJASLC_CancelAbort_btf = TJASLC_CancelAbort_btf;
    debug->TJAGEN_StrongReadyInvalid_btf = TJAGEN_StrongReadyInvalid_btf;

    debug->LKA_LeLnCrvQualityValid_bool = LKA_LeLnCrvQualityValid_bool;
    debug->LKA_LeLnQualityValid_bool = LKA_LeLnQualityValid_bool;
    debug->SLC_StrongReadyBothSides_bool = SLC_StrongReadyBothSides_bool;
    debug->SLC_WeakReadyLeft_bool = SLC_WeakReadyLeft_bool;
    debug->SLC_WeakReadyBothSides_bool = SLC_WeakReadyBothSides_bool;
    debug->SLC_WeakReadyRight_bool = SLC_WeakReadyRight_bool;
    debug->SLC_TriggerRight_bool = SLC_TriggerRight_bool;
    debug->SLC_TriggerLeft_bool = SLC_TriggerLeft_bool;
    debug->SLC_LeverLeftEngaged_bool = SLC_LeverLeftEngaged_bool;
    debug->SLC_LeverRightEngaged_bool = SLC_LeverRightEngaged_bool;
    debug->SLC_MaxInitDurationExceeded_bool = SLC_MaxInitDurationExceeded_bool;
    debug->SLC_ManvStatePassive_bool = SLC_ManvStatePassive_bool;
    debug->SLC_Abort_bool = SLC_Abort_bool;
    debug->SLC_Cancel_bool = SLC_Cancel_bool;
    debug->SLC_LCM_End_bool = SLC_LCM_End_bool;
    debug->SLC_LCM_Start_bool = SLC_LCM_Start_bool;
    debug->SLC_LaneCheckValid_bool = SLC_LaneCheckValid_bool;
    debug->SLC_NewEgoLane_bool = SLC_NewEgoLane_bool;
    debug->SLC_LCM_Cancel_bool = SLC_LCM_Cancel_bool;
    debug->STM_Cancel_bool = STM_Cancel_bool;
    debug->SLC_PrevReset_bool = SLC_PrevReset_bool;
    debug->OBF_AccObjValidLaneCheck = OBF_AccObjValidLaneCheck;
    debug->OBF_AccObjValid = OBF_AccObjValid;
    debug->OBF_LeftLaneCheckValid_bool = OBF_LeftLaneCheckValid_bool;
    debug->OBF_AccObjValid_bool = OBF_AccObjValid_bool;
    debug->OBF_AccObjSwitch = OBF_AccObjSwitch;
    debug->OBF_MinDist2LeftBndInvalid = OBF_MinDist2LeftBndInvalid;
    debug->OBF_TargetOutsideEgoLane_bool = OBF_TargetOutsideEgoLane_bool;
    debug->OBF_RightLaneCheckValid_bool = OBF_RightLaneCheckValid_bool;
    debug->OBF_MinDist2RightBndInvalid = OBF_MinDist2RightBndInvalid;
    debug->OBF_LaneCheckValid_bool = OBF_LaneCheckValid_bool;
    debug->OBF_DistOrEgoLaneInvalid_bool = OBF_DistOrEgoLaneInvalid_bool;
    debug->OBF_TargetObjDataWR_bool = OBF_TargetObjDataWR_bool;
    debug->OBF_TargetObjDataSR_bool = OBF_TargetObjDataSR_bool;
    debug->CMB_ObjectFollowingOnly_bool = CMB_ObjectFollowingOnly_bool;
    debug->CMB_LaneQualityInvalid_bool = CMB_LaneQualityInvalid_bool;
    debug->TTG_ObjectUpdate_bool = TTG_ObjectUpdate_bool;
    debug->TTG_LaneUpdate_bool = TTG_LaneUpdate_bool;
    debug->TTG_CMBObjectCorridor_bool = TTG_CMBObjectCorridor_bool;
    debug->TTG_Enable_bool = TTG_Enable_bool;
    debug->TTG_LD_PredictFinish_bool = TTG_LD_PredictFinish_bool;
    debug->TTG_LD_Enable_bool = TTG_LD_Enable_bool;
    debug->TTG_CMB_Enable_bool = TTG_CMB_Enable_bool;
    debug->TTG_OD_Enable_bool = TTG_OD_Enable_bool;
    debug->TTG_Reset_bool = TTG_Reset_bool;
    debug->TTG_Predict_Enable_bool = TTG_Predict_Enable_bool;

    debug->TJAGEN_FunctionQuit_bool = TJAGEN_FunctionQuit_bool;
    debug->TJAGEN_SuspendedAndQuit_debug = TJAGEN_SuspendedAndQuit_debug;
    debug->TJAGEN_SuspendEnd_bool = TJAGEN_SuspendEnd_bool;
    debug->TJAGEN_SuspendStart_bool = TJAGEN_SuspendStart_bool;
    debug->TJASTM_SysStateTJAIn_nu = TJASTM_SysStateTJAIn_nu;
    debug->STM_SuspendTimeExpired_bool = STM_SuspendTimeExpired_bool;
    debug->STM_PrevRAMPOUT_bool = STM_PrevRAMPOUT_bool;
    debug->SLC_LCPLeft2Active_bool = SLC_LCPLeft2Active_bool;
    debug->SLC_LCPRight2Active_bool = SLC_LCPRight2Active_bool;
    debug->SLC_LCPRight2Passive_bool = SLC_LCPRight2Passive_bool;
    debug->SLC_LCPLeft2Passive_bool = SLC_LCPLeft2Passive_bool;
    debug->TJASTM_DrvTakeOver_bool = TJASTM_DrvTakeOver_bool;
    debug->TJATOW_DriverTakeOverWarning_nu = TJATOW_DriverTakeOverWarning_nu;
    debug->TJALKA_LaneIncoherence_btf = TJALKA_LaneIncoherence_btf;
    debug->TJALKA_LnIncoherenceStatus_nu = TJALKA_LnIncoherenceStatus_nu;

    debug->SLC_VehSpdTooLowInfo = SLC_VehSpdTooLowInfo;
    debug->SLC_LaneChangeOnGoingInfo = SLC_LaneChangeOnGoingInfo;
    debug->SLC_LaneChangeEndInfo = SLC_LaneChangeEndInfo;
    debug->SLC_LaneChangePendingInfo = SLC_LaneChangePendingInfo;
    debug->SLC_LaneChangeCancleInfo = SLC_LaneChangeCancleInfo;
    debug->SLC_AllowGoBack_bool = SLC_AllowGoBack_bool;
    debug->SLC_CenterDistToBoundary_met = SLC_CenterDistToBoundary_met;
    debug->SLC_IntoAbort_nu = SLC_IntoAbort_nu;
    debug->SLC_LaneChangeDirectionIn_nu = SLC_LaneChangeDirectionIn_nu;
    debug->SLC_SameLaneChangeDetc_bool = SLC_SameLaneChangeDetc_bool;
    debug->SLC_LaneChangeBackDetc_bool = SLC_LaneChangeBackDetc_bool;
    debug->SLC_ExitAbort_bool = SLC_ExitAbort_bool;
    debug->SLC_ExitAbortNewEgo_bool = SLC_ExitAbortNewEgo_bool;
    debug->SLC_AbortState_enum = SLC_AbortState_enum;
    debug->SLC_LaneChangeDirectionAbort_enum =
        SLC_LaneChangeDirectionAbort_enum;

    debug->TJASLC_Nb_DCLCSwitchNVRAM_nu = TJASLC_Nb_DCLCSwitchNVRAM_nu;
    debug->TJASLC_SLCHighLightID_nu = TJASLC_SLCHighLightID_nu;
    debug->TJASLC_LaneChangeInfo = TJASLC_LaneChangeInfo;
    debug->TJASLC_TurnLtDirctionReq_nu = TJASLC_TurnLtDirctionReq_nu;
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */