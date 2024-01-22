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
 * File                             : TJASA_types.h
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

#ifndef RTW_HEADER_TJASA_types_h_
#define RTW_HEADER_TJASA_types_h_
#include "rtwtypes.h"

/* Model Code Variants */
#ifndef DEFINED_TYPEDEF_FOR_E_MCTLCF_ControllingFunction_nu_
#define DEFINED_TYPEDEF_FOR_E_MCTLCF_ControllingFunction_nu_

/* Indiccating if lane is valid */
typedef uint8_T E_MCTLCF_ControllingFunction_nu;

/* enum E_MCTLCF_ControllingFunction_nu */
#define E_MCTLCF_ControllingFunction_nu_LCF_OFF \
    ((E_MCTLCF_ControllingFunction_nu)0U) /* Default value */
#define E_MCTLCF_ControllingFunction_nu_LCF_TJA \
    ((E_MCTLCF_ControllingFunction_nu)1U)
#define E_MCTLCF_ControllingFunction_nu_LCF_LDP \
    ((E_MCTLCF_ControllingFunction_nu)2U)
#define E_MCTLCF_ControllingFunction_nu_LCF_LDPOC \
    ((E_MCTLCF_ControllingFunction_nu)3U)
#define E_MCTLCF_ControllingFunction_nu_LCF_RDP \
    ((E_MCTLCF_ControllingFunction_nu)4U)
#define E_MCTLCF_ControllingFunction_nu_LCF_ALCA \
    ((E_MCTLCF_ControllingFunction_nu)5U)
#endif

#ifndef DEFINED_TYPEDEF_FOR_E_TJASTM_SysStateTJA_nu_
#define DEFINED_TYPEDEF_FOR_E_TJASTM_SysStateTJA_nu_

/* System state */
typedef uint8_T E_TJASTM_SysStateTJA_nu;

/* enum E_TJASTM_SysStateTJA_nu */
#define E_TJASTM_SysStateTJA_nu_SYSST_NOTPRESENT \
    ((E_TJASTM_SysStateTJA_nu)0U) /* Default value */
#define E_TJASTM_SysStateTJA_nu_SYSST_PASSIVE ((E_TJASTM_SysStateTJA_nu)1U)
#define E_TJASTM_SysStateTJA_nu_SYSST_STANDBY ((E_TJASTM_SysStateTJA_nu)2U)
#define E_TJASTM_SysStateTJA_nu_SYSST_CONTROLLING ((E_TJASTM_SysStateTJA_nu)3U)
#define E_TJASTM_SysStateTJA_nu_SYSST_SUSPENDED ((E_TJASTM_SysStateTJA_nu)4U)
#define E_TJASTM_SysStateTJA_nu_SYSST_RAMPOUT ((E_TJASTM_SysStateTJA_nu)5U)
#define E_TJASTM_SysStateTJA_nu_SYSST_ERROR ((E_TJASTM_SysStateTJA_nu)6U)
#define E_TJASTM_SysStateTJA_nu_SYSST_OFF ((E_TJASTM_SysStateTJA_nu)7U)
#define E_TJASTM_SysStateTJA_nu_SYSST_WAITACC ((E_TJASTM_SysStateTJA_nu)8U)
#endif

#ifndef DEFINED_TYPEDEF_FOR_E_TJASTM_LatCtrlMode_nu_
#define DEFINED_TYPEDEF_FOR_E_TJASTM_LatCtrlMode_nu_

/*  lateral control mode */
typedef uint8_T E_TJASTM_LatCtrlMode_nu;

/* enum E_TJASTM_LatCtrlMode_nu */
#define E_TJASTM_LatCtrlMode_nu_LATCTRLMD_PASSIVE \
    ((E_TJASTM_LatCtrlMode_nu)0U) /* Default value */
#define E_TJASTM_LatCtrlMode_nu_LATCTRLMD_LC ((E_TJASTM_LatCtrlMode_nu)1U)
#define E_TJASTM_LatCtrlMode_nu_LATCTRLMD_OF ((E_TJASTM_LatCtrlMode_nu)2U)
#define E_TJASTM_LatCtrlMode_nu_LATCTRLMD_CMB ((E_TJASTM_LatCtrlMode_nu)3U)
#define E_TJASTM_LatCtrlMode_nu_LATCTRLMD_SALC ((E_TJASTM_LatCtrlMode_nu)4U)
#define E_TJASTM_LatCtrlMode_nu_LATCTRLMD_LC_RQ ((E_TJASTM_LatCtrlMode_nu)5U)
#define E_TJASTM_LatCtrlMode_nu_LATCTRLMD_OF_RQ ((E_TJASTM_LatCtrlMode_nu)6U)
#define E_TJASTM_LatCtrlMode_nu_LATCTRLMD_CMB_RQ ((E_TJASTM_LatCtrlMode_nu)7U)
#define E_TJASTM_LatCtrlMode_nu_LATCTRLMD_SALC_RQ ((E_TJASTM_LatCtrlMode_nu)8U)
#endif

#ifndef DEFINED_TYPEDEF_FOR_E_TJASLC_ManeuverState_nu_
#define DEFINED_TYPEDEF_FOR_E_TJASLC_ManeuverState_nu_

/* Maneuver state */
typedef uint8_T E_TJASLC_ManeuverState_nu;

/* enum E_TJASLC_ManeuverState_nu */
#define E_TJASLC_ManeuverState_nu_PASSIVE \
    ((E_TJASLC_ManeuverState_nu)0U) /* Default value */
#define E_TJASLC_ManeuverState_nu_TRIGREADY ((E_TJASLC_ManeuverState_nu)1U)
#define E_TJASLC_ManeuverState_nu_LCPSTART ((E_TJASLC_ManeuverState_nu)2U)
#define E_TJASLC_ManeuverState_nu_LATMVSTART ((E_TJASLC_ManeuverState_nu)3U)
#define E_TJASLC_ManeuverState_nu_LCMSTART ((E_TJASLC_ManeuverState_nu)4U)
#define E_TJASLC_ManeuverState_nu_NEWEGO ((E_TJASLC_ManeuverState_nu)5U)
#define E_TJASLC_ManeuverState_nu_LCMEND ((E_TJASLC_ManeuverState_nu)6U)
#define E_TJASLC_ManeuverState_nu_ABORT ((E_TJASLC_ManeuverState_nu)7U)
#endif

#ifndef DEFINED_TYPEDEF_FOR_E_TJALKA_LnBndValid_nu_
#define DEFINED_TYPEDEF_FOR_E_TJALKA_LnBndValid_nu_

/* Indiccating if lane is valid */
typedef uint8_T E_TJALKA_LnBndValid_nu;

/* enum E_TJALKA_LnBndValid_nu */
#define E_TJALKA_LnBndValid_nu_BND_NOT_VALID \
    ((E_TJALKA_LnBndValid_nu)0U) /* Default value */
#define E_TJALKA_LnBndValid_nu_BND_VAL_LEFT_ONLY ((E_TJALKA_LnBndValid_nu)1U)
#define E_TJALKA_LnBndValid_nu_BND_VAL_RIGHT_ONLY ((E_TJALKA_LnBndValid_nu)2U)
#define E_TJALKA_LnBndValid_nu_BND_VAL_BOTH_SIDE ((E_TJALKA_LnBndValid_nu)3U)
#endif

#ifndef DEFINED_TYPEDEF_FOR_E_TJASLC_LaneChangeTrig_nu_
#define DEFINED_TYPEDEF_FOR_E_TJASLC_LaneChangeTrig_nu_

/* Lane change trigger */
typedef uint8_T E_TJASLC_LaneChangeTrig_nu;

/* enum E_TJASLC_LaneChangeTrig_nu */
#define E_TJASLC_LaneChangeTrig_nu_LNCHNG_NO_TRIG \
    ((E_TJASLC_LaneChangeTrig_nu)0U) /* Default value */
#define E_TJASLC_LaneChangeTrig_nu_LNCHNG_LEFT_TRIG \
    ((E_TJASLC_LaneChangeTrig_nu)1U)
#define E_TJASLC_LaneChangeTrig_nu_LNCHNG_RIGHT_TRIG \
    ((E_TJASLC_LaneChangeTrig_nu)2U)
#endif

#ifndef DEFINED_TYPEDEF_FOR_E_TJASLC_AbortState_nu_
#define DEFINED_TYPEDEF_FOR_E_TJASLC_AbortState_nu_

/* Ready to trigger type */
typedef uint8_T E_TJASLC_AbortState_nu;

/* enum E_TJASLC_AbortState_nu */
#define E_TJASLC_AbortState_nu_ABORT_NOACTIVE \
    ((E_TJASLC_AbortState_nu)0U) /* Default value */
#define E_TJASLC_AbortState_nu_ABORT_ABORT ((E_TJASLC_AbortState_nu)1U)
#define E_TJASLC_AbortState_nu_ABORT_NEWEGO ((E_TJASLC_AbortState_nu)2U)
#endif

#ifndef DEFINED_TYPEDEF_FOR_E_TJASLC_LaneChangeWarning_nu_
#define DEFINED_TYPEDEF_FOR_E_TJASLC_LaneChangeWarning_nu_

/* lane change warning side */
typedef uint8_T E_TJASLC_LaneChangeWarning_nu;

/* enum E_TJASLC_LaneChangeWarning_nu */
#define E_TJASLC_LaneChangeWarning_nu_NO_WARNING \
    ((E_TJASLC_LaneChangeWarning_nu)0U) /* Default value */
#define E_TJASLC_LaneChangeWarning_nu_LEFT_WARNING \
    ((E_TJASLC_LaneChangeWarning_nu)1U)
#define E_TJASLC_LaneChangeWarning_nu_RIGHT_WARNING \
    ((E_TJASLC_LaneChangeWarning_nu)2U)
#endif

#ifndef DEFINED_TYPEDEF_FOR_E_TJASLC_ReadyToTrigger_nu_
#define DEFINED_TYPEDEF_FOR_E_TJASLC_ReadyToTrigger_nu_

/* Ready to trigger type */
typedef uint8_T E_TJASLC_ReadyToTrigger_nu;

/* enum E_TJASLC_ReadyToTrigger_nu */
#define E_TJASLC_ReadyToTrigger_nu_TRIGREADY_NONE \
    ((E_TJASLC_ReadyToTrigger_nu)0U) /* Default value */
#define E_TJASLC_ReadyToTrigger_nu_TRIGREADY_LEFT \
    ((E_TJASLC_ReadyToTrigger_nu)1U)
#define E_TJASLC_ReadyToTrigger_nu_TRIGREADY_RIGHT \
    ((E_TJASLC_ReadyToTrigger_nu)2U)
#define E_TJASLC_ReadyToTrigger_nu_TRIGREADY_BOTH \
    ((E_TJASLC_ReadyToTrigger_nu)3U)
#endif

#ifndef DEFINED_TYPEDEF_FOR_E_TJASTM_SysStateHWA_nu_
#define DEFINED_TYPEDEF_FOR_E_TJASTM_SysStateHWA_nu_

/* Lane change trigger */
typedef uint8_T E_TJASTM_SysStateHWA_nu;

/* enum E_TJASTM_SysStateHWA_nu */
#define E_TJASTM_SysStateHWA_nu_HWAState_OFF \
    ((E_TJASTM_SysStateHWA_nu)0U) /* Default value */
#define E_TJASTM_SysStateHWA_nu_HWAState_Standby ((E_TJASTM_SysStateHWA_nu)1U)
#define E_TJASTM_SysStateHWA_nu_HWAState_Active ((E_TJASTM_SysStateHWA_nu)2U)
#define E_TJASTM_SysStateHWA_nu_HWAState_ACCOverride \
    ((E_TJASTM_SysStateHWA_nu)3U)
#define E_TJASTM_SysStateHWA_nu_HWAState_LCCSuspend \
    ((E_TJASTM_SysStateHWA_nu)4U)
#define E_TJASTM_SysStateHWA_nu_HWAState_BothSuspend \
    ((E_TJASTM_SysStateHWA_nu)5U)
#define E_TJASTM_SysStateHWA_nu_HWAState_Error ((E_TJASTM_SysStateHWA_nu)6U)
#define E_TJASTM_SysStateHWA_nu_HWAState_Passive ((E_TJASTM_SysStateHWA_nu)7U)
#endif

#ifndef DEFINED_TYPEDEF_FOR_Bus_TgtOrCridrParam_nu_
#define DEFINED_TYPEDEF_FOR_Bus_TgtOrCridrParam_nu_

typedef struct {
    real32_T PosX0_met;
    real32_T PosY0_met;
    real32_T Heading_rad;
    real32_T Crv_1pm;
    real32_T CrvChng_1pm2;
    real32_T Length_met;
} Bus_TgtOrCridrParam_nu;

#endif

#ifndef DEFINED_TYPEDEF_FOR_Bus_TgtTrajAndCridrBnd_nu_
#define DEFINED_TYPEDEF_FOR_Bus_TgtTrajAndCridrBnd_nu_

typedef struct {
    Bus_TgtOrCridrParam_nu TTG_LeftCorridorClothoid_bus;
    Bus_TgtOrCridrParam_nu TTG_RightCorridorClothoid_bus;
    Bus_TgtOrCridrParam_nu TTG_TargetTrajectory_bus;
} Bus_TgtTrajAndCridrBnd_nu;

#endif

#ifndef DEFINED_TYPEDEF_FOR_E_TJATVG_TrajGuiQu_nu_
#define DEFINED_TYPEDEF_FOR_E_TJATVG_TrajGuiQu_nu_

/* Indiccating if lane is valid */
typedef uint8_T E_TJATVG_TrajGuiQu_nu;

/* enum E_TJATVG_TrajGuiQu_nu */
#define E_TJATVG_TrajGuiQu_nu_TGQ_REQ_OFF \
    ((E_TJATVG_TrajGuiQu_nu)0U) /* Default value */
#define E_TJATVG_TrajGuiQu_nu_TGQ_REQ ((E_TJATVG_TrajGuiQu_nu)1U)
#define E_TJATVG_TrajGuiQu_nu_TGQ_REQ_FREEZE ((E_TJATVG_TrajGuiQu_nu)3U)
#define E_TJATVG_TrajGuiQu_nu_TGQ_REQ_FFC ((E_TJATVG_TrajGuiQu_nu)4U)
#define E_TJATVG_TrajGuiQu_nu_TGQ_REQ_REFCHNG ((E_TJATVG_TrajGuiQu_nu)5U)
#define E_TJATVG_TrajGuiQu_nu_TGQ_REQ_SLCQ2LCC ((E_TJATVG_TrajGuiQu_nu)6U)
#define E_TJATVG_TrajGuiQu_nu_TGQ_REQ_LANECHANG ((E_TJATVG_TrajGuiQu_nu)7U)
#endif
#endif /* RTW_HEADER_TJASA_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
