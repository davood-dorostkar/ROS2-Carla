/*
 * File: MCTLFC_private.h
 *
 * Code generated for Simulink model 'MCTLFC'.
 *
 * Model version                  : 1.733
 * Simulink Coder version         : 9.4 (R2020b) 29-Jul-2020
 * C/C++ source code generated on : Mon Mar 22 17:58:27 2021
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-32 (Windows32)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_MCTLFC_private_h_
#define RTW_HEADER_MCTLFC_private_h_
#include "rtwtypes.h"

/* Imported (extern) block signals */
extern real32_T MCTLFC_LCFRCV_SysCycleTimeSen_sec;     /* '<Root>/Inport' */
extern uint8_T MCTLFC_LCFRCV_TJASTM_SysStateTJA_nu;    /* '<Root>/Inport1' */
extern uint8_T MCTLFC_DPLSMI_SysStateLDP_nu;           /* '<Root>/Inport2' */
extern uint8_T MCTLFC_DPOSTM_SysStateLDPOC_nu;         /* '<Root>/Inport3' */
extern uint8_T MCTLFC_DPRSMI_SysStateRDP_nu;           /* '<Root>/Inport4' */
extern uint8_T MCTLFC_LCRSMI_SysStateALCA_nu;          /* '<Root>/Inport5' */
extern uint8_T MCTLFC_LCSSTM_SysStateAOLC_nu;          /* '<Root>/Inport6' */
extern uint8_T MCTLFC_ESASTM_SysStateESA_nu;           /* '<Root>/Inport7' */
extern uint8_T MCTLFC_SysStateLDP_nu;                  /* '<Root>/Inport8' */
extern real32_T MCTLFC_DPLTTG_LeCridrBndPosX0_met;     /* '<Root>/Inport9' */
extern real32_T MCTLFC_DPLTTG_LeCridrBndPosY0_met;     /* '<Root>/Inport10' */
extern real32_T MCTLFC_DPLTTG_LeCridrBndHeadAng_rad;   /* '<Root>/Inport11' */
extern real32_T MCTLFC_DPLTTG_LeCridrBndCrv_1pm;       /* '<Root>/Inport12' */
extern real32_T MCTLFC_DPLTTG_LeCridrBndCrvChng_1pm2;  /* '<Root>/Inport13' */
extern real32_T MCTLFC_DPLTTG_LeCridrBndLength_met;    /* '<Root>/Inport14' */
extern real32_T MCTLFC_DPLTTG_RiCridrBndPosX0_met;     /* '<Root>/Inport15' */
extern real32_T MCTLFC_DPLTTG_RiCridrBndPosY0_met;     /* '<Root>/Inport16' */
extern real32_T MCTLFC_DPLTTG_RiCridrBndHeadAng_rad;   /* '<Root>/Inport17' */
extern real32_T MCTLFC_DPLTTG_RiCridrBndCrv_1pm;       /* '<Root>/Inport18' */
extern real32_T MCTLFC_DPLTTG_RiCridrBndCrvChng_1pm2;  /* '<Root>/Inport19' */
extern real32_T MCTLFC_DPLTTG_RiCridrBndLength_met;    /* '<Root>/Inport20' */
extern real32_T MCTLFC_DPLTTG_TgtTrajPosX0_met;        /* '<Root>/Inport21' */
extern real32_T MCTLFC_DPLTTG_TgtTrajPosY0_met;        /* '<Root>/Inport22' */
extern real32_T MCTLFC_DPLTTG_TgtTrajHeadAng_rad;      /* '<Root>/Inport23' */
extern real32_T MCTLFC_DPLTTG_TgtTrajCrv_1pm;          /* '<Root>/Inport24' */
extern real32_T MCTLFC_DPLTTG_TgtTrajCrvChng_1pm2;     /* '<Root>/Inport25' */
extern real32_T MCTLFC_DPLTTG_TgtTrajLength_met;       /* '<Root>/Inport26' */
extern uint8_T MCTLFC_DPLTVG_TrajPlanServQu_nu;        /* '<Root>/Inport27' */
extern real32_T MCTLFC_DPLTVG_WeightTgtDistY_nu;       /* '<Root>/Inport28' */
extern real32_T MCTLFC_DPLTVG_WeightEndTime_nu;        /* '<Root>/Inport29' */
extern real32_T MCTLFC_DPLTVG_DistYToLeTgtArea_met;    /* '<Root>/Inport30' */
extern real32_T MCTLFC_DPLTVG_DistYToRiTgtArea_met;    /* '<Root>/Inport31' */
extern real32_T MCTLFC_DPLTVG_FTireAclMax_mps2;        /* '<Root>/Inport32' */
extern real32_T MCTLFC_DPLTVG_FTireAclMin_mps2;        /* '<Root>/Inport33' */
extern uint8_T MCTLFC_DPLTVG_TrajGuiQu_nu;             /* '<Root>/Inport34' */
extern boolean_T MCTLFC_DPLTVG_TriggerReplan_bool;     /* '<Root>/Inport35' */
extern real32_T MCTLFC_DPLTVG_PredTimeHeadAng_sec;     /* '<Root>/Inport36' */
extern real32_T MCTLFC_DPLTVG_PredTimeCrv_sec;         /* '<Root>/Inport37' */
extern real32_T MCTLFC_DPLTVG_PlanningHorzion_sec;     /* '<Root>/Inport38' */
extern real32_T MCTLFC_DPLTVG_ObstacleVelX_mps;        /* '<Root>/Inport39' */
extern real32_T MCTLFC_DPLTVG_ObstacleAclX_mps2;       /* '<Root>/Inport40' */
extern real32_T MCTLFC_DPLTVG_ObstacleWidth_met;       /* '<Root>/Inport41' */
extern real32_T MCTLFC_DPLTVG_ObstacleDistX_met;       /* '<Root>/Inport42' */
extern real32_T MCTLFC_DPLTVG_ObstacleDistY_met;       /* '<Root>/Inport43' */
extern boolean_T MCTLFC_DPLTVG_LtcyCompActivated_bool; /* '<Root>/Inport44' */
extern real32_T MCTLFC_DPLTVG_SensorTStamp_sec;        /* '<Root>/Inport45' */
extern real32_T MCTLFC_DPLTVG_MaxCrvTrajGuiCtrl_1pm;   /* '<Root>/Inport46' */
extern real32_T MCTLFC_DPLTVG_MaxCrvGrdBuildup_1pms;   /* '<Root>/Inport47' */
extern real32_T MCTLFC_DPLTVG_MaxCrvGrdRed_1pms;       /* '<Root>/Inport48' */
extern real32_T MCTLFC_DPLTVG_GrdLimitTgtCrvTGC_1pms;  /* '<Root>/Inport49' */
extern real32_T MCTLFC_DPLTVG_StrWhStifLimit_nu;       /* '<Root>/Inport50' */
extern real32_T MCTLFC_DPLTVG_StrWhStifGrad_1ps;       /* '<Root>/Inport252' */
extern real32_T MCTLFC_DPLTVG_TrqRampGrad_1ps;         /* '<Root>/Inport253' */
extern real32_T MCTLFC_DPLTVG_MaxTrqScaILimit_nu;      /* '<Root>/Inport51' */
extern real32_T MCTLFC_DPLTVG_MaxTrqScalGrad_1ps;      /* '<Root>/Inport250' */
extern boolean_T MCTLFC_DPLTVG_HighStatAccu_bool;      /* '<Root>/Inport251' */
extern boolean_T MCTLFC_DPLTVG_LimiterActivated_bool;  /* '<Root>/Inport52' */
extern real32_T MCTLFC_DPLTVG_LimiterTimeDuration_sec; /* '<Root>/Inport53' */
extern real32_T MCTLFC_DPLTVG_MaxJerkAllowed_mps3;     /* '<Root>/Inport258' */
extern uint8_T MCTLFC_DPLTVG_DeratingLevel_nu;         /* '<Root>/Inport262' */
extern uint8_T MCTLFC_SysStateRDP_nu;                  /* '<Root>/Inport54' */
extern real32_T MCTLFC_DPRTTG_LeCridrBndPosX0_met;     /* '<Root>/Inport55' */
extern real32_T MCTLFC_DPRTTG_LeCridrBndPosY0_met;     /* '<Root>/Inport56' */
extern real32_T MCTLFC_DPRTTG_LeCridrBndHeadAng_rad;   /* '<Root>/Inport57' */
extern real32_T MCTLFC_DPRTTG_LeCridrBndCrv_1pm;       /* '<Root>/Inport58' */
extern real32_T MCTLFC_DPRTTG_LeCridrBndCrvChng_1pm2;  /* '<Root>/Inport59' */
extern real32_T MCTLFC_DPRTTG_LeCridrBndLength_met;    /* '<Root>/Inport60' */
extern real32_T MCTLFC_DPRTTG_RiCridrBndPosX0_met;     /* '<Root>/Inport61' */
extern real32_T MCTLFC_DPRTTG_RiCridrBndPosY0_met;     /* '<Root>/Inport62' */
extern real32_T MCTLFC_DPRTTG_RiCridrBndHeadAng_rad;   /* '<Root>/Inport63' */
extern real32_T MCTLFC_DPRTTG_RiCridrBndCrv_1pm;       /* '<Root>/Inport64' */
extern real32_T MCTLFC_DPRTTG_RiCridrBndCrvChng_1pm2;  /* '<Root>/Inport65' */
extern real32_T MCTLFC_DPRTTG_RiCridrBndLength_met;    /* '<Root>/Inport66' */
extern real32_T MCTLFC_DPRTTG_TgtTrajPosX0_met;        /* '<Root>/Inport67' */
extern real32_T MCTLFC_DPRTTG_TgtTrajPosY0_met;        /* '<Root>/Inport68' */
extern real32_T MCTLFC_DPRTTG_TgtTrajHeadAng_rad;      /* '<Root>/Inport69' */
extern real32_T MCTLFC_DPRTTG_TgtTrajCrv_1pm;          /* '<Root>/Inport70' */
extern real32_T MCTLFC_DPRTTG_TgtTrajCrvChng_1pm2;     /* '<Root>/Inport71' */
extern real32_T MCTLFC_DPRTTG_TgtTrajLength_met;       /* '<Root>/Inport72' */
extern uint8_T MCTLFC_DPRTVG_TrajPlanServQu_nu;        /* '<Root>/Inport73' */
extern real32_T MCTLFC_DPRTVG_WeightTgtDistY_nu;       /* '<Root>/Inport74' */
extern real32_T MCTLFC_DPRTVG_WeightEndTime_nu;        /* '<Root>/Inport75' */
extern real32_T MCTLFC_DPRTVG_DistYToLeTgtArea_met;    /* '<Root>/Inport76' */
extern real32_T MCTLFC_DPRTVG_DistYToRiTgtArea_met;    /* '<Root>/Inport77' */
extern real32_T MCTLFC_DPRTVG_FTireAclMax_mps2;        /* '<Root>/Inport78' */
extern real32_T MCTLFC_DPRTVG_FTireAclMin_mps2;        /* '<Root>/Inport79' */
extern uint8_T MCTLFC_DPRTVG_TrajGuiQu_nu;             /* '<Root>/Inport80' */
extern boolean_T MCTLFC_DPRTVG_TriggerReplan_bool;     /* '<Root>/Inport81' */
extern real32_T MCTLFC_DPRTVG_PredTimeHeadAng_sec;     /* '<Root>/Inport82' */
extern real32_T MCTLFC_DPRTVG_PredTimeCrv_sec;         /* '<Root>/Inport83' */
extern real32_T MCTLFC_DPRTVG_PlanningHorzion_sec;     /* '<Root>/Inport84' */
extern real32_T MCTLFC_DPRTVG_ObstacleVelX_mps;        /* '<Root>/Inport85' */
extern real32_T MCTLFC_DPRTVG_ObstacleAclX_mps2;       /* '<Root>/Inport86' */
extern real32_T MCTLFC_DPRTVG_ObstacleWidth_met;       /* '<Root>/Inport87' */
extern real32_T MCTLFC_DPRTVG_ObstacleDistX_met;       /* '<Root>/Inport88' */
extern real32_T MCTLFC_DPRTVG_ObstacleDistY_met;       /* '<Root>/Inport89' */
extern boolean_T MCTLFC_DPRTVG_LtcyCompActivated_bool; /* '<Root>/Inport90' */
extern real32_T MCTLFC_DPRTVG_SensorTStamp_sec;        /* '<Root>/Inport91' */
extern real32_T MCTLFC_DPRTVG_MaxCrvTrajGuiCtrl_1pm;   /* '<Root>/Inport92' */
extern real32_T MCTLFC_DPRTVG_MaxCrvGrdBuildup_1pms;   /* '<Root>/Inport93' */
extern real32_T MCTLFC_DPRTVG_MaxCrvGrdRed_1pms;       /* '<Root>/Inport94' */
extern real32_T MCTLFC_DPRTVG_GrdLimitTgtCrvTGC_1pms;  /* '<Root>/Inport95' */
extern real32_T MCTLFC_DPRTVG_StrWhStifLimit_nu;       /* '<Root>/Inport96' */
extern real32_T MCTLFC_DPRTVG_StrWhStifGrad_1ps;       /* '<Root>/Inport248' */
extern real32_T MCTLFC_DPRTVG_TrqRampGrad_1ps;         /* '<Root>/Inport249' */
extern real32_T MCTLFC_DPRTVG_MaxTrqScaILimit_nu;      /* '<Root>/Inport97' */
extern real32_T MCTLFC_DPRTVG_MaxTrqScalGrad_1ps;      /* '<Root>/Inport246' */
extern boolean_T MCTLFC_DPRTVG_HighStatAccu_bool;      /* '<Root>/Inport247' */
extern boolean_T MCTLFC_DPRTVG_LimiterActivated_bool;  /* '<Root>/Inport98' */
extern real32_T MCTLFC_DPRTVG_LimiterTimeDuration_sec; /* '<Root>/Inport99' */
extern real32_T MCTLFC_DPRTVG_MaxJerkAllowed_mps3;     /* '<Root>/Inport266' */
extern uint8_T MCTLFC_DPRTVG_DeratingLevel_nu;         /* '<Root>/Inport267' */
extern uint8_T MCTLFC_SysStateLDPOC_nu;                /* '<Root>/Inport100' */
extern real32_T MCTLFC_DPOTTG_LeCridrBndPosX0_met;     /* '<Root>/Inport101' */
extern real32_T MCTLFC_DPOTTG_LeCridrBndPosY0_met;     /* '<Root>/Inport102' */
extern real32_T MCTLFC_DPOTTG_LeCridrBndHeadAng_rad;   /* '<Root>/Inport103' */
extern real32_T MCTLFC_DPOTTG_LeCridrBndCrv_1pm;       /* '<Root>/Inport104' */
extern real32_T MCTLFC_DPOTTG_LeCridrBndCrvChng_1pm2;  /* '<Root>/Inport105' */
extern real32_T MCTLFC_DPOTTG_LeCridrBndLength_met;    /* '<Root>/Inport106' */
extern real32_T MCTLFC_DPOTTG_RiCridrBndPosX0_met;     /* '<Root>/Inport107' */
extern real32_T MCTLFC_DPOTTG_RiCridrBndPosY0_met;     /* '<Root>/Inport108' */
extern real32_T MCTLFC_DPOTTG_RiCridrBndHeadAng_rad;   /* '<Root>/Inport109' */
extern real32_T MCTLFC_DPOTTG_RiCridrBndCrv_1pm;       /* '<Root>/Inport110' */
extern real32_T MCTLFC_DPOTTG_RiCridrBndCrvChng_1pm2;  /* '<Root>/Inport111' */
extern real32_T MCTLFC_DPOTTG_RiCridrBndLength_met;    /* '<Root>/Inport112' */
extern real32_T MCTLFC_DPOTTG_TgtTrajPosX0_met;        /* '<Root>/Inport113' */
extern real32_T MCTLFC_DPOTTG_TgtTrajPosY0_met;        /* '<Root>/Inport114' */
extern real32_T MCTLFC_DPOTTG_TgtTrajHeadAng_rad;      /* '<Root>/Inport115' */
extern real32_T MCTLFC_DPOTTG_TgtTrajCrv_1pm;          /* '<Root>/Inport116' */
extern real32_T MCTLFC_DPOTTG_TgtTrajCrvChng_1pm2;     /* '<Root>/Inport117' */
extern real32_T MCTLFC_DPOTTG_TgtTrajLength_met;       /* '<Root>/Inport118' */
extern uint8_T MCTLFC_DPOTVG_TrajPlanServQu_nu;        /* '<Root>/Inport119' */
extern real32_T MCTLFC_DPOTVG_WeightTgtDistY_nu;       /* '<Root>/Inport120' */
extern real32_T MCTLFC_DPOTVG_WeightEndTime_nu;        /* '<Root>/Inport121' */
extern real32_T MCTLFC_DPOTVG_DistYToLeTgtArea_met;    /* '<Root>/Inport122' */
extern real32_T MCTLFC_DPOTVG_DistYToRiTgtArea_met;    /* '<Root>/Inport123' */
extern real32_T MCTLFC_DPOTVG_FTireAclMax_mps2;        /* '<Root>/Inport124' */
extern real32_T MCTLFC_DPOTVG_FTireAclMin_mps2;        /* '<Root>/Inport125' */
extern uint8_T MCTLFC_DPOTVG_TrajGuiQu_nu;             /* '<Root>/Inport126' */
extern boolean_T MCTLFC_DPOTVG_TriggerReplan_bool;     /* '<Root>/Inport127' */
extern real32_T MCTLFC_DPOTVG_PredTimeHeadAng_sec;     /* '<Root>/Inport128' */
extern real32_T MCTLFC_DPOTVG_PredTimeCrv_sec;         /* '<Root>/Inport129' */
extern real32_T MCTLFC_DPOTVG_PlanningHorzion_sec;     /* '<Root>/Inport130' */
extern real32_T MCTLFC_DPOTVG_ObstacleVelX_mps;        /* '<Root>/Inport131' */
extern real32_T MCTLFC_DPOTVG_ObstacleAclX_mps2;       /* '<Root>/Inport132' */
extern real32_T MCTLFC_DPOTVG_ObstacleWidth_met;       /* '<Root>/Inport133' */
extern real32_T MCTLFC_DPOTVG_ObstacleDistX_met;       /* '<Root>/Inport134' */
extern real32_T MCTLFC_DPOTVG_ObstacleDistY_met;       /* '<Root>/Inport135' */
extern boolean_T MCTLFC_DPOTVG_LtcyCompActivated_bool; /* '<Root>/Inport136' */
extern real32_T MCTLFC_DPOTVG_SensorTStamp_sec;        /* '<Root>/Inport137' */
extern real32_T MCTLFC_DPOTVG_MaxCrvTrajGuiCtrl_1pm;   /* '<Root>/Inport138' */
extern real32_T MCTLFC_DPOTVG_MaxCrvGrdBuildup_1pms;   /* '<Root>/Inport139' */
extern real32_T MCTLFC_DPOTVG_MaxCrvGrdRed_1pms;       /* '<Root>/Inport140' */
extern real32_T MCTLFC_DPOTVG_GrdLimitTgtCrvTGC_1pms;  /* '<Root>/Inport141' */
extern real32_T MCTLFC_DPOTVG_StrWhStifLimit_nu;       /* '<Root>/Inport142' */
extern real32_T MCTLFC_DPOTVG_StrWhStifGrad_1ps;       /* '<Root>/Inport244' */
extern real32_T MCTLFC_DPOTVG_TrqRampGrad_1ps;         /* '<Root>/Inport245' */
extern real32_T MCTLFC_DPOTVG_MaxTrqScaILimit_nu;      /* '<Root>/Inport143' */
extern real32_T MCTLFC_DPOTVG_MaxTrqScalGrad_1ps;      /* '<Root>/Inport242' */
extern boolean_T MCTLFC_DPOTVG_HighStatAccu_bool;      /* '<Root>/Inport243' */
extern boolean_T MCTLFC_DPOTVG_LimiterActivated_bool;  /* '<Root>/Inport144' */
extern real32_T MCTLFC_DPOTVG_LimiterTimeDuration_sec; /* '<Root>/Inport145' */
extern real32_T MCTLFC_DPOTVG_MaxJerkAllowed_mps3;     /* '<Root>/Inport259' */
extern uint8_T MCTLFC_DPOTVG_DeratingLevel_nu;         /* '<Root>/Inport263' */
extern uint8_T MCTLFC_SysStateTJA_nu;                  /* '<Root>/Inport146' */
extern real32_T MCTLFC_TJATTG_LeCridrBndPosX0_met;     /* '<Root>/Inport147' */
extern real32_T MCTLFC_TJATTG_LeCridrBndPosY0_met;     /* '<Root>/Inport148' */
extern real32_T MCTLFC_TJATTG_LeCridrBndHeadAng_rad;   /* '<Root>/Inport149' */
extern real32_T MCTLFC_TJATTG_LeCridrBndCrv_1pm;       /* '<Root>/Inport150' */
extern real32_T MCTLFC_TJATTG_LeCridrBndCrvChng_1pm2;  /* '<Root>/Inport151' */
extern real32_T MCTLFC_TJATTG_LeCridrBndLength_met;    /* '<Root>/Inport152' */
extern real32_T MCTLFC_TJATTG_RiCridrBndPosX0_met;     /* '<Root>/Inport153' */
extern real32_T MCTLFC_TJATTG_RiCridrBndPosY0_met;     /* '<Root>/Inport154' */
extern real32_T MCTLFC_TJATTG_RiCridrBndHeadAng_rad;   /* '<Root>/Inport155' */
extern real32_T MCTLFC_TJATTG_RiCridrBndCrv_1pm;       /* '<Root>/Inport156' */
extern real32_T MCTLFC_TJATTG_RiCridrBndCrvChng_1pm2;  /* '<Root>/Inport157' */
extern real32_T MCTLFC_TJATTG_RiCridrBndLength_met;    /* '<Root>/Inport158' */
extern real32_T MCTLFC_TJATTG_TgtTrajPosX0_met;        /* '<Root>/Inport159' */
extern real32_T MCTLFC_TJATTG_TgtTrajPosY0_met;        /* '<Root>/Inport160' */
extern real32_T MCTLFC_TJATTG_TgtTrajHeadAng_rad;      /* '<Root>/Inport161' */
extern real32_T MCTLFC_TJATTG_TgtTrajCrv_1pm;          /* '<Root>/Inport162' */
extern real32_T MCTLFC_TJATTG_TgtTrajCrvChng_1pm2;     /* '<Root>/Inport163' */
extern real32_T MCTLFC_TJATTG_TgtTrajLength_met;       /* '<Root>/Inport164' */
extern uint8_T MCTLFC_TJATVG_TrajPlanServQu_nu;        /* '<Root>/Inport165' */
extern real32_T MCTLFC_TJATVG_WeightTgtDistY_nu;       /* '<Root>/Inport166' */
extern real32_T MCTLFC_TJATVG_WeightEndTime_nu;        /* '<Root>/Inport167' */
extern real32_T MCTLFC_TJATVG_DistYToLeTgtArea_met;    /* '<Root>/Inport168' */
extern real32_T MCTLFC_TJATVG_DistYToRiTgtArea_met;    /* '<Root>/Inport169' */
extern real32_T MCTLFC_TJATVG_FTireAclMax_mps2;        /* '<Root>/Inport170' */
extern real32_T MCTLFC_TJATVG_FTireAclMin_mps2;        /* '<Root>/Inport171' */
extern uint8_T MCTLFC_TJATVG_TrajGuiQu_nu;             /* '<Root>/Inport172' */
extern boolean_T MCTLFC_TJATVG_TriggerReplan_bool;     /* '<Root>/Inport173' */
extern real32_T MCTLFC_TJATVG_PredTimeHeadAng_sec;     /* '<Root>/Inport174' */
extern real32_T MCTLFC_TJATVG_PredTimeCrv_sec;         /* '<Root>/Inport175' */
extern real32_T MCTLFC_TJATVG_PlanningHorzion_sec;     /* '<Root>/Inport176' */
extern real32_T MCTLFC_TJATVG_ObstacleVelX_mps;        /* '<Root>/Inport177' */
extern real32_T MCTLFC_TJATVG_ObstacleAclX_mps2;       /* '<Root>/Inport178' */
extern real32_T MCTLFC_TJATVG_ObstacleWidth_met;       /* '<Root>/Inport179' */
extern real32_T MCTLFC_TJATVG_ObstacleDistX_met;       /* '<Root>/Inport180' */
extern real32_T MCTLFC_TJATVG_ObstacleDistY_met;       /* '<Root>/Inport181' */
extern boolean_T MCTLFC_TJATVG_LtcyCompActivated_bool; /* '<Root>/Inport182' */
extern real32_T MCTLFC_TJATVG_SensorTStamp_sec;        /* '<Root>/Inport183' */
extern real32_T MCTLFC_TJATVG_MaxCrvTrajGuiCtrl_1pm;   /* '<Root>/Inport184' */
extern real32_T MCTLFC_TJATVG_MaxCrvGrdBuildup_1pms;   /* '<Root>/Inport185' */
extern real32_T MCTLFC_TJATVG_MaxCrvGrdRed_1pms;       /* '<Root>/Inport186' */
extern real32_T MCTLFC_TJATVG_GrdLimitTgtCrvTGC_1pms;  /* '<Root>/Inport187' */
extern real32_T MCTLFC_TJATVG_StrWhStifLimit_nu;       /* '<Root>/Inport188' */
extern real32_T MCTLFC_TJATVG_StrWhStifGrad_1ps;       /* '<Root>/Inport240' */
extern real32_T MCTLFC_TJATVG_TrqRampGrad_1ps;         /* '<Root>/Inport241' */
extern real32_T MCTLFC_TJATVG_MaxTrqScaILimit_nu;      /* '<Root>/Inport189' */
extern real32_T MCTLFC_TJATVG_MaxTrqScalGrad_1ps;      /* '<Root>/Inport238' */
extern boolean_T MCTLFC_TJATVG_HighStatAccu_bool;      /* '<Root>/Inport239' */
extern boolean_T MCTLFC_TJATVG_LimiterActivated_bool;  /* '<Root>/Inport190' */
extern real32_T MCTLFC_TJATVG_LimiterTimeDuration_sec; /* '<Root>/Inport191' */
extern real32_T MCTLFC_TJATVG_MaxJerkAllowed_mps3;     /* '<Root>/Inport260' */
extern uint8_T MCTLFC_TJATVG_DeratingLevel_nu;         /* '<Root>/Inport264' */
extern uint8_T MCTLFC_SysStateALCA_nu;                 /* '<Root>/Inport192' */
extern real32_T MCTLFC_LCRTTG_LeCridrBndPosX0_met;     /* '<Root>/Inport193' */
extern real32_T MCTLFC_LCRTTG_LeCridrBndPosY0_met;     /* '<Root>/Inport194' */
extern real32_T MCTLFC_LCRTTG_LeCridrBndHeadAng_rad;   /* '<Root>/Inport195' */
extern real32_T MCTLFC_LCRTTG_LeCridrBndCrv_1pm;       /* '<Root>/Inport196' */
extern real32_T MCTLFC_LCRTTG_LeCridrBndCrvChng_1pm2;  /* '<Root>/Inport197' */
extern real32_T MCTLFC_LCRTTG_LeCridrBndLength_met;    /* '<Root>/Inport198' */
extern real32_T MCTLFC_LCRTTG_RiCridrBndPosX0_met;     /* '<Root>/Inport199' */
extern real32_T MCTLFC_LCRTTG_RiCridrBndPosY0_met;     /* '<Root>/Inport200' */
extern real32_T MCTLFC_LCRTTG_RiCridrBndHeadAng_rad;   /* '<Root>/Inport201' */
extern real32_T MCTLFC_LCRTTG_RiCridrBndCrv_1pm;       /* '<Root>/Inport202' */
extern real32_T MCTLFC_LCRTTG_RiCridrBndCrvChng_1pm2;  /* '<Root>/Inport203' */
extern real32_T MCTLFC_LCRTTG_RiCridrBndLength_met;    /* '<Root>/Inport204' */
extern real32_T MCTLFC_LCRTTG_TgtTrajPosX0_met;        /* '<Root>/Inport205' */
extern real32_T MCTLFC_LCRTTG_TgtTrajPosY0_met;        /* '<Root>/Inport206' */
extern real32_T MCTLFC_LCRTTG_TgtTrajHeadAng_rad;      /* '<Root>/Inport207' */
extern real32_T MCTLFC_LCRTTG_TgtTrajCrv_1pm;          /* '<Root>/Inport208' */
extern real32_T MCTLFC_LCRTTG_TgtTrajCrvChng_1pm2;     /* '<Root>/Inport209' */
extern real32_T MCTLFC_LCRTTG_TgtTrajLength_met;       /* '<Root>/Inport210' */
extern uint8_T MCTLFC_LCRTVG_TrajPlanServQu_nu;        /* '<Root>/Inport211' */
extern real32_T MCTLFC_LCRTVG_WeightTgtDistY_nu;       /* '<Root>/Inport212' */
extern real32_T MCTLFC_LCRTVG_WeightEndTime_nu;        /* '<Root>/Inport213' */
extern real32_T MCTLFC_LCRTVG_DistYToLeTgtArea_met;    /* '<Root>/Inport214' */
extern real32_T MCTLFC_LCRTVG_DistYToRiTgtArea_met;    /* '<Root>/Inport215' */
extern real32_T MCTLFC_LCRTVG_FTireAclMax_mps2;        /* '<Root>/Inport216' */
extern real32_T MCTLFC_LCRTVG_FTireAclMin_mps2;        /* '<Root>/Inport217' */
extern uint8_T MCTLFC_LCRTVG_TrajGuiQu_nu;             /* '<Root>/Inport218' */
extern boolean_T MCTLFC_LCRTVG_TriggerReplan_bool;     /* '<Root>/Inport219' */
extern real32_T MCTLFC_LCRTVG_PredTimeHeadAng_sec;     /* '<Root>/Inport220' */
extern real32_T MCTLFC_LCRTVG_PredTimeCrv_sec;         /* '<Root>/Inport221' */
extern real32_T MCTLFC_LCRTVG_PlanningHorzion_sec;     /* '<Root>/Inport222' */
extern real32_T MCTLFC_LCRTVG_ObstacleVelX_mps;        /* '<Root>/Inport223' */
extern real32_T MCTLFC_LCRTVG_ObstacleAclX_mps2;       /* '<Root>/Inport224' */
extern real32_T MCTLFC_LCRTVG_ObstacleWidth_met;       /* '<Root>/Inport225' */
extern real32_T MCTLFC_LCRTVG_ObstacleDistX_met;       /* '<Root>/Inport226' */
extern real32_T MCTLFC_LCRTVG_ObstacleDistY_met;       /* '<Root>/Inport227' */
extern boolean_T MCTLFC_LCRTVG_LtcyCompActivated_bool; /* '<Root>/Inport228' */
extern real32_T MCTLFC_LCRTVG_SensorTStamp_sec;        /* '<Root>/Inport229' */
extern real32_T MCTLFC_LCRTVG_MaxCrvTrajGuiCtrl_1pm;   /* '<Root>/Inport230' */
extern real32_T MCTLFC_LCRTVG_MaxCrvGrdBuildup_1pms;   /* '<Root>/Inport231' */
extern real32_T MCTLFC_LCRTVG_MaxCrvGrdRed_1pms;       /* '<Root>/Inport232' */
extern real32_T MCTLFC_LCRTVG_GrdLimitTgtCrvTGC_1pms;  /* '<Root>/Inport233' */
extern real32_T MCTLFC_LCRTVG_StrWhStifLimit_nu;       /* '<Root>/Inport234' */
extern real32_T MCTLFC_LCRTVG_StrWhStifGrad_1ps;       /* '<Root>/Inport256' */
extern real32_T MCTLFC_LCRTVG_TrqRampGrad_1ps;         /* '<Root>/Inport257' */
extern real32_T MCTLFC_LCRTVG_MaxTrqScaILimit_nu;      /* '<Root>/Inport235' */
extern real32_T MCTLFC_LCRTVG_MaxTrqScalGrad_1ps;      /* '<Root>/Inport254' */
extern boolean_T MCTLFC_LCRTVG_HighStatAccu_bool;      /* '<Root>/Inport255' */
extern boolean_T MCTLFC_LCRTVG_LimiterActivated_bool;  /* '<Root>/Inport236' */
extern real32_T MCTLFC_LCRTVG_LimiterTimeDuration_sec; /* '<Root>/Inport237' */
extern real32_T MCTLFC_LCRTVG_MaxJerkAllowed_mps3;     /* '<Root>/Inport261' */
extern uint8_T MCTLFC_LCRTVG_DeratingLevel_nu;         /* '<Root>/Inport265' */

#endif /* RTW_HEADER_MCTLFC_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
