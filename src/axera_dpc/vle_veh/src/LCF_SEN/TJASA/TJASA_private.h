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
 * File                             : TJASA_private.h
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

#ifndef RTW_HEADER_TJASA_private_h_
#define RTW_HEADER_TJASA_private_h_
#include "rtwtypes.h"

/* Imported (extern) block signals */
extern boolean_T
    ABPR_LaneChangeDetected_bool;   /* '<Root>/Inport11'
                                     * Lane change detected flag--from ABPR module
                                     */
extern real32_T ABPR_LaneWidth_met; /* '<Root>/Inport5'
                                     * Lane width--from ABPR module
                                     */
extern boolean_T
    ABPR_ConstructionSite_bool; /* '<Root>/Inport13'
                                 * Construction site flag--from ABPR
                                 */
extern real32_T
    ABPR_UncoupledLaneWidth_met;       /* '<Root>/Inport22'
                                        * Uncoupled Lane Width--from ABPR module
                                        */
extern real32_T ABPR_ABDTimeStamp_sec; /* '<Root>/Inport91'
                                        * ABD Time Stamp--from ABPR module
                                        */
extern uint8_T ABPR_LeftLaneType_enum; /* '<Root>/Inport54'
                                        * Left lane type--from ABPR
                                        */
extern real32_T
    ABPR_LeLnClthPosX0_met; /* '<Root>/Inport23'
                             * Left Lane Clothoid PosX0--from ABPR module
                             */
extern real32_T
    ABPR_LeLnClthPosY0_met; /* '<Root>/Inport1'
                             * Left lane clothoid posY0--from ABPR module
                             */
extern uint16_T
    ABPR_LeLnInvalidQu_btf;           /* '<Root>/Inport7'
                                       * Left lane invalid qualifier--from ABPR
                                       */
extern uint8_T ABPR_LeLnQuality_perc; /* '<Root>/Inport15'
                                       * Left lane quality--from ABPR module
                                       */
extern uint8_T
    ABPR_LeCrvQuality_perc; /* '<Root>/Inport17'
                             * Left lane curve quality--from ABPR module
                             */
extern real32_T
    ABPR_LeLnClthHeading_rad; /* '<Root>/Inport24'
                               * Left Lane Clothoid Heading--from ABPR module
                               */
extern real32_T
    ABPR_LeLnClthCrv_1pm;                  /* '<Root>/Inport25'
                                            * Left lane clothoid curvature--from ABPR module
                                            */
extern real32_T ABPR_LeLnClthCrvChng_1pm2; /* '<Root>/Inport26'
                                            * Left Lane Clothoid Curve of
                                            * Change--from ABPR module
                                            */
extern real32_T
    ABPR_LeLnClthLength_met;                 /* '<Root>/Inport27'
                                              * Left lane clothoid length--from ABPR module
                                              */
extern real32_T ABPR_LeAdjLnClthPosX0_met;   /* '<Root>/Inport80'
                                              * Left Adjacent Lane Clothoid
                                              * PosX0--from ABPR module
                                              */
extern real32_T ABPR_LeAdjLnClthPosY0_met;   /* '<Root>/Inport40'
                                              * Left adjacent lane clothoid
                                              * posY0--from ABPR module
                                              */
extern uint16_T ABPR_LeAdjLnInvalidQu_btf;   /* '<Root>/Inport43'
                                              * Left adjacent lane invalid
                                              * quality--from ABPR module
                                              */
extern real32_T ABPR_LeAdjLnClthHeading_rad; /* '<Root>/Inport81'
                                              * Left Adjacent Lane Clothoid
                                              * Heading--from ABPR module
                                              */
extern real32_T ABPR_LeAdjLnClthCrv_1pm;     /* '<Root>/Inport82'
                                              * Left Adjacent Lane Clothoid
                                              * Curve--from ABPR module
                                              */
extern real32_T
    ABPR_LeAdjLnClthCrvChng_1pm2;           /* '<Root>/Inport83'
                                             * Left Adjacent Lane Clothoid Curve of
                                             * Change--from ABPR module
                                             */
extern real32_T ABPR_LeAdjLnClthLength_met; /* '<Root>/Inport84'
                                             * Left Adjacent Lane Clothoid
                                             * Length--from ABPR module
                                             */
extern uint8_T ABPR_RightLaneType_enum;     /* '<Root>/Inport55'
                                             * Right lane type--from ABPR
                                             */
extern real32_T
    ABPR_RiLnClthPosX0_met; /* '<Root>/Inport28'
                             * Right Lane Clothoid PosX0--from ABPR module
                             */
extern real32_T
    ABPR_RiLnClthPosY0_met; /* '<Root>/Inport2'
                             * Right lane clothoid posY0--from ABPR module
                             */
extern uint16_T
    ABPR_RiLnInvalidQu_btf;           /* '<Root>/Inport6'
                                       * Right lane invalid qualifier--from ABPR
                                       */
extern uint8_T ABPR_RiLnQuality_perc; /* '<Root>/Inport16'
                                       * Right lane quality--from ABPR module
                                       */
extern uint8_T
    ABPR_RiCrvQuality_perc; /* '<Root>/Inport18'
                             * Right lane curve quality--from ABPR module
                             */
extern real32_T
    ABPR_RiLnClthHeading_rad; /* '<Root>/Inport29'
                               * Right Lane Clothoid Heading--from ABPR module
                               */
extern real32_T
    ABPR_RiLnClthCrv_1pm;                  /* '<Root>/Inport30'
                                            * Right lane clothoid curvature--from ABPR module
                                            */
extern real32_T ABPR_RiLnClthCrvChng_1pm2; /* '<Root>/Inport31'
                                            * Right Lane Clothoid Curve of
                                            * Change--from ABPR module
                                            */
extern real32_T
    ABPR_RiLnClthLength_met;                 /* '<Root>/Inport20'
                                              * Right lane clothoid length--from ABPR module
                                              */
extern real32_T ABPR_RiAdjLnClthPosX0_met;   /* '<Root>/Inport89'
                                              * Right Adjacent Lane Clothoid
                                              * PosX0--from ABPR module
                                              */
extern real32_T ABPR_RiAdjLnClthPosY0_met;   /* '<Root>/Inport41'
                                              * Right adjacent lane clothoid
                                              * posY0--from ABPR module
                                              */
extern uint16_T ABPR_RiAdjLnInvalidQu_btf;   /* '<Root>/Inport42'
                                              * Right adjacent lane invalid
                                              * quality--from ABPR module
                                              */
extern real32_T ABPR_RiAdjLnClthHeading_rad; /* '<Root>/Inport85'
                                              * Right Adjacent Lane Clothoid
                                              * Heading--from ABPR module
                                              */
extern real32_T ABPR_RiAdjLnClthCrv_1pm;     /* '<Root>/Inport86'
                                              * Right Adjacent Lane Clothoid
                                              * Curve--from ABPR module
                                              */
extern real32_T
    ABPR_RiAdjLnClthCrvChng_1pm2;           /* '<Root>/Inport87'
                                             * Right Adjacent Lane Clothoid Curve of
                                             * Change--from ABPR module
                                             */
extern real32_T ABPR_RiAdjLnClthLength_met; /* '<Root>/Inport88'
                                             * Right Adjacent Lane Clothoid
                                             * Length--from ABPR module
                                             */
extern real32_T
    ABPR_CntrLnClthPosX0_met; /* '<Root>/Inport58'
                               * Control Lane Clothoid PosX0--from ABPR module
                               */
extern real32_T
    ABPR_CntrLnClthPosY0_met;               /* '<Root>/Inport3'
                                             * Control Lane Clothoid PosY0--from ABPR module
                                             */
extern real32_T ABPR_CntrLnClthHeading_rad; /* '<Root>/Inport4'
                                             * Control lane clothoid
                                             * heading--from ABPR module
                                             */
extern real32_T
    ABPR_CntrLnClthCrv_1pm;                  /* '<Root>/Inport9'
                                              * Control Lane Clothoid Curve--from ABPR module
                                              */
extern real32_T ABPR_CntrLnClthCrvChng_1pm2; /* '<Root>/Inport59'
                                              * Control Lane Clothoid Curve of
                                              * Change--from ABPR module
                                              */
extern real32_T
    ABPR_CntrLnClthLength_met;                /* '<Root>/Inport21'
                                               * Control Lane Clothoid Length--from ABPR module
                                               */
extern uint16_T ODPFOH_TgtObjClothoidInv_btf; /* '<Root>/Inport32'
                                               * Target object clothoid invalid
                                               * qualifier--from ODPR module
                                               */
extern real32_T
    ODPFOH_TgtObjPosX0_met; /* '<Root>/Inport60'
                             * Target object Clothoid PosX0--from ODPR module
                             */
extern real32_T
    ODPFOH_TgtObjPosY0_met;               /* '<Root>/Inport36'
                                           * Target object Clothoid PosY0--from ODPR module
                                           */
extern real32_T ODPFOH_TgtObjHeadAng_rad; /* '<Root>/Inport37'
                                           * Target object Clothoid
                                           * Heading--from ODPR module
                                           */
extern real32_T
    ODPFOH_TgtObjCrv_1pm;                  /* '<Root>/Inport38'
                                            * Target object Clothoid Curve--from ODPR module
                                            */
extern real32_T ODPFOH_TgtObjCrvChng_1pm2; /* '<Root>/Inport61'
                                            * Target object Clothoid Curve of
                                            * Change--from ODPR module
                                            */
extern real32_T
    ODPFOH_TgtObjLength_met; /* '<Root>/Inport39'
                              * Target object Clothoid Length--from ODPR module
                              */
extern real32_T
    ODPFOP_AccFRObjTStamp_sec;                /* '<Root>/Inport92'
                                               * Acc object time stamp--from ODPR module
                                               */
extern uint16_T ODPFOP_AccObjInvBitfield_btf; /* '<Root>/Inport33'
                                               * Acc object invalid
                                               * qualifier--from ODPR module
                                               */
extern real32_T ODPFOP_AccObjPosX_met;        /* '<Root>/Inport34'
                                               * Acc obejct PosX--from ODPR module
                                               */
extern real32_T ODPFOP_AccObjPosY_met;        /* '<Root>/Inport35'
                                               * Acc obejct PosY--from ODPR module
                                               */
extern boolean_T LCA_ActiveLeft_bool;         /* '<Root>/Inport46'
                                               * Active left flag--from LCA
                                               */
extern boolean_T LCA_ActiveRight_bool;        /* '<Root>/Inport47'
                                               * Active right flag--from LCA
                                               */
extern boolean_T LCA_WarningLeft_bool;        /* '<Root>/Inport48'
                                               * Warning left flag--from LCA
                                               */
extern boolean_T LCA_WarningRight_bool;       /* '<Root>/Inport49'
                                               * Warning right flag--from LCA
                                               */
extern boolean_T BSD_ActiveLeft_bool;         /* '<Root>/Inport50'
                                               * Active left flag--from BSD
                                               */
extern boolean_T BSD_ActiveRight_bool;        /* '<Root>/Inport51'
                                               * Active right flag--from BSD
                                               */
extern boolean_T BSD_WarningLeft_bool;        /* '<Root>/Inport52'
                                               * Warning left flag--from BSD
                                               */
extern boolean_T BSD_WarningRight_bool;       /* '<Root>/Inport53'
                                               * Warning right flag--from BSD
                                               */
extern uint16_T CUSTOM_PrjSpecQu_btf;         /* '<Root>/Inport12'
                                               * Specific qualifier--from CUSTOM
                                               */
extern boolean_T S_ODPSOP_MSFlag_RearLeft_nu; /* '<Root>/Inport56'
                                               * MS flag_rear left--from ODPSOP
                                               */
extern boolean_T
    S_ODPSOP_MSFlag_RearRight_nu; /* '<Root>/Inport57'
                                   * MS flag_rear right--from ODPSOP
                                   */
extern uint16_T
    TRJPLN_QuStatusTrajPlan_nu; /* '<Root>/Inport62'
                                 * Trajectory plan qualifier--from TRJPLN module
                                 */
extern uint8_T
    TRJCTR_QuServTrajCtr_nu; /* '<Root>/Inport63'
                              * Trajectory control qualifier--from TRJCTR module
                              */
extern E_MCTLCF_ControllingFunction_nu
    MDCTR_ControllingFunction_nu;            /* '<Root>/Inport64'
                                              * Controlling function--from MDCTR module
                                              */
extern real32_T LCFRCV_TSysCycleTimeSen_sec; /* '<Root>/Inport10'
                                              * System cycle time--from LCFRCV
                                              */
extern real32_T VDy_VehVelocity_kph;         /* '<Root>/Inport8'
                                              * Vehicle velocity--from VDy module
                                              */
extern boolean_T
    LCFRCV_TurnSignalLeverHold_bool; /* '<Root>/Inport19'
                                      * Turn signal level hold flag--from LCFRCV
                                      */
extern boolean_T
    LCFRCV_TurnSignalLeft_bool; /* '<Root>/Inport44'
                                 * Turn signal left flag--from LCFRCV
                                 */
extern boolean_T
    LCFRCV_TurnSignalRight_bool;            /* '<Root>/Inport45'
                                             * Turn signal right flag--from LCFRCV
                                             */
extern uint8_T LCFRCV_DrivingMode_nu;       /* '<Root>/Inport93'
                                             * Driving mode--from LCFRCV
                                             */
extern boolean_T LCFRCV_LKASwitch_nu;       /* '<Root>/Inport65'
                                             * LKA switch flag--from LCFRCV
                                             */
extern boolean_T LCFRCV_TJASwitch_nu;       /* '<Root>/Inport66'
                                             * TJA switch flag--from LCFRCV
                                             */
extern boolean_T LCFRCV_ErrorStateTJA_bool; /* '<Root>/Inport67'
                                             * Error state of TJA--from LCFRCV
                                             */
extern boolean_T LCFRCV_ErrorStateLKA_bool; /* '<Root>/Inport68'
                                             * Error state of LKA--from LCFRCV
                                             */
extern real32_T VDy_VehAclX_mps2;           /* '<Root>/Inport69'
                                             * Vehicle acceleration X--from VDy module
                                             */
extern real32_T VDy_VehAclY_mps2;           /* '<Root>/Inport70'
                                             * Vehicle acceleration Y--from VDy module
                                             */
extern boolean_T
    LCFRCV_SysStOnLatDMC_bool;          /* '<Root>/Inport14'
                                         * System state on latDMC--from LCFRCV
                                         */
extern real32_T LCFRCV_SteerWAngle_deg; /* '<Root>/Inport77'
                                         * steer wheel angle--from LCRCV
                                         */
extern real32_T VDy_VehCrv_1pm;         /* '<Root>/Inport76'
                                         * Vehicle curve--from VDy module
                                         */
extern real32_T VDy_VehYawRate_rps;     /* '<Root>/Inport78'
                                         * Vehicle yawrate--from VDy module
                                         */
extern real32_T VDy_VehVelX_mps;        /* '<Root>/Inport79'
                                         * Vehicle velocity X--from VDy module
                                         */
extern boolean_T LCFRCV_VehStopped_nu;  /* '<Root>/Inport90'
                                         * Vehicle stopped flag--from LCFRCV
                                         */
extern uint8_T
    VDPDRV_ActiveStCtrl_btf;                 /* '<Root>/Inport71'
                                              * Acitve state control--from VDPDRV module
                                              */
extern uint8_T VDPDRV_SysStError_btf;        /* '<Root>/Inport72'
                                              * System state error--from VDPDRV module
                                              */
extern uint16_T VDPDRV_VehStInvalid_btf;     /* '<Root>/Inport73'
                                              * Vehicle state invalid bitfield--from
                                              * VDPDRV module
                                              */
extern uint8_T VDPDRV_DrvStInvalid_btf;      /* '<Root>/Inport74'
                                              * Driver state invalid bitfield--from
                                              * VDPDRV module
                                              */
extern uint8_T VDPDRV_SysStNotAvailable_btf; /* '<Root>/Inport75'
                                              * System state not avaliable
                                              * bitfield--from VDPDRV module
                                              */
extern real32_T VDy_DashboardVelocity_kph;   /* '<Root>/Inport94'
                                              * Dashbord velocity--from VDy module
                                              */
extern real32_T LCFRCV_ManualTorque_nm;      /* '<Root>/Inport95'
                                              * Manual torque -- from LCFRCV
                                              */
extern real32_T
    LCFRCV_SteerWAngleGrad_degps;            /* '<Root>/Inport96'
                                              * Steering wheel angle grad -- from LCFRCV
                                              */
extern boolean_T LCFRCV_SLCHMISwitch_bool;   /* '<Root>/Inport97'
                                              * SLC HMI switch--from LCFRCV
                                              */
extern boolean_T LCFRCV_TJAAudioSwitch_bool; /* '<Root>/Inport98'
                                              * TJA audio switch--from LCFRCV
                                              */
extern boolean_T
    LCCRA_InLeftSafeFlag_bool; /* '<Root>/Inport99'
                                * Lane change detected flag--from ABPR module
                                */
extern boolean_T
    LCCRA_InRightSafeFlag_bool;                   /* '<Root>/Inport100'
                                                   * Lane change detected flag--from ABPR module
                                                   */
extern boolean_T LCCRA_InLeftFrontSafeFlag_bool;  /* '<Root>/Inport101'
                                                   * Lane change detected
                                                   * flag--from ABPR module
                                                   */
extern boolean_T LCCRA_InRightFrontSafeFlag_bool; /* '<Root>/Inport102'
                                                   * Lane change detected
                                                   * flag--from ABPR module
                                                   */
extern boolean_T LCCRA_InLeftRearSafeFlag_bool;   /* '<Root>/Inport103'
                                                   * Lane change detected
                                                   * flag--from ABPR module
                                                   */
extern boolean_T LCCRA_InRightRearSafeFlag_bool;  /* '<Root>/Inport104'
                                                   * Lane change detected
                                                   * flag--from ABPR module
                                                   */
extern boolean_T LCFRCV_PilotOnLeverSwitch_bool;  /* '<Root>/Inport105'
                                                   * Lane change detected
                                                   * flag--from ABPR module
                                                   */
extern boolean_T LCFRCV_PilotOffLeverSwitch_bool; /* '<Root>/Inport106'
                                                   * Lane change detected
                                                   * flag--from ABPR module
                                                   */
extern uint8_T
    VLCVEH_ACCStatus_nu;                       /* '<Root>/Inport107'
                                                * Lane change detected flag--from ABPR module
                                                */
extern boolean_T LCFSEN_Nb_DCLCSwitchNVRAM_nu; /* '<Root>/Inport108'
                                                * Right lane clothoid
                                                * curvature--from ABPR module
                                                */
extern int32_T LCCRA_InLeftHighLightID_nu;     /* '<Root>/Inport109'
                                                * Right lane clothoid
                                                * curvature--from ABPR module
                                                */
extern int32_T LCCRA_InRightHighLightID_nu;    /* '<Root>/Inport110'
                                                * Right lane clothoid
                                                * curvature--from ABPR module
                                                */
extern uint8_T LCFRCV_TurnLightReqSt_nu;       /* '<Root>/Inport111'
                                                * TJA switch flag--from LCFRCV
                                                */
extern boolean_T LCFRCV_DoorOpen_bool;         /* '<Root>/Inport112'
                                                * Active left flag--from LCA
                                                */
extern boolean_T LCFRCV_HoodOpen_bool;         /* '<Root>/Inport113'
                                                * Active left flag--from LCA
                                                */
extern boolean_T LCFRCV_TrunkUnLock_bool;      /* '<Root>/Inport114'
                                                * Active left flag--from LCA
                                                */
extern boolean_T LCFRCV_bPGear_bool;           /* '<Root>/Inport115'
                                                * Active right flag--from LCA
                                                */
extern boolean_T
    LCCRA_inFrontSafeFlag_bool;              /* '<Root>/Inport116'
                                              * Lane change detected flag--from ABPR module
                                              */
extern int32_T LCFRCV_inFrontDangerObjID_nu; /* '<Root>/Inport117'
                                              * Right lane clothoid
                                              * curvature--from ABPR module
                                              */
extern boolean_T EMFID_bRCA_bool;            /* '<Root>/Inport118'
                                              * Right lane invalid qualifier--from ABPR
                                              */
extern boolean_T EMFID_bRCC_bool;            /* '<Root>/Inport120'
                                              * Right lane invalid qualifier--from ABPR
                                              */
extern boolean_T EMFID_bFCA_bool;            /* '<Root>/Inport121'
                                              * Right lane invalid qualifier--from ABPR
                                              */
extern boolean_T EMFID_bFCB_bool;            /* '<Root>/Inport122'
                                              * Right lane invalid qualifier--from ABPR
                                              */
extern boolean_T LCFRCV_bPilotOnOff_bool;    /* '<Root>/Inport119'
                                              * Warning right flag--from BSD
                                              */
extern uint8_T ADCS4_AVM_Sts;                /* '<Root>/Inport123'
                                              * Left lane clothoid posY0--from ABPR module
                                              */
extern uint8_T ADCS11_Parking_WorkSts;       /* '<Root>/Inport124'
                                              * Warning right flag--from LCA
                                              */

#endif /* RTW_HEADER_TJASA_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
