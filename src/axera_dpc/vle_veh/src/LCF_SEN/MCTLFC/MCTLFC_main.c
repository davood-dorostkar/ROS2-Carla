/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
#include "MCTLFC.h"
#include "MCTLFC_main.h"

#define ASW_QM_CORE2_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
// MCTLFC variables define
real32_T MCTLFC_LCFRCV_SysCycleTimeSen_sec;     /* '<Root>/Inport' */
uint8_T MCTLFC_LCFRCV_TJASTM_SysStateTJA_nu;    /* '<Root>/Inport1' */
uint8_T MCTLFC_DPLSMI_SysStateLDP_nu;           /* '<Root>/Inport2' */
uint8_T MCTLFC_DPOSTM_SysStateLDPOC_nu;         /* '<Root>/Inport3' */
uint8_T MCTLFC_DPRSMI_SysStateRDP_nu;           /* '<Root>/Inport4' */
uint8_T MCTLFC_LCRSMI_SysStateALCA_nu;          /* '<Root>/Inport5' */
uint8_T MCTLFC_LCSSTM_SysStateAOLC_nu;          /* '<Root>/Inport6' */
uint8_T MCTLFC_ESASTM_SysStateESA_nu;           /* '<Root>/Inport7' */
uint8_T MCTLFC_SysStateLDP_nu;                  /* '<Root>/Inport8' */
real32_T MCTLFC_DPLTTG_LeCridrBndPosX0_met;     /* '<Root>/Inport9' */
real32_T MCTLFC_DPLTTG_LeCridrBndPosY0_met;     /* '<Root>/Inport10' */
real32_T MCTLFC_DPLTTG_LeCridrBndHeadAng_rad;   /* '<Root>/Inport11' */
real32_T MCTLFC_DPLTTG_LeCridrBndCrv_1pm;       /* '<Root>/Inport12' */
real32_T MCTLFC_DPLTTG_LeCridrBndCrvChng_1pm2;  /* '<Root>/Inport13' */
real32_T MCTLFC_DPLTTG_LeCridrBndLength_met;    /* '<Root>/Inport14' */
real32_T MCTLFC_DPLTTG_RiCridrBndPosX0_met;     /* '<Root>/Inport15' */
real32_T MCTLFC_DPLTTG_RiCridrBndPosY0_met;     /* '<Root>/Inport16' */
real32_T MCTLFC_DPLTTG_RiCridrBndHeadAng_rad;   /* '<Root>/Inport17' */
real32_T MCTLFC_DPLTTG_RiCridrBndCrv_1pm;       /* '<Root>/Inport18' */
real32_T MCTLFC_DPLTTG_RiCridrBndCrvChng_1pm2;  /* '<Root>/Inport19' */
real32_T MCTLFC_DPLTTG_RiCridrBndLength_met;    /* '<Root>/Inport20' */
real32_T MCTLFC_DPLTTG_TgtTrajPosX0_met;        /* '<Root>/Inport21' */
real32_T MCTLFC_DPLTTG_TgtTrajPosY0_met;        /* '<Root>/Inport22' */
real32_T MCTLFC_DPLTTG_TgtTrajHeadAng_rad;      /* '<Root>/Inport23' */
real32_T MCTLFC_DPLTTG_TgtTrajCrv_1pm;          /* '<Root>/Inport24' */
real32_T MCTLFC_DPLTTG_TgtTrajCrvChng_1pm2;     /* '<Root>/Inport25' */
real32_T MCTLFC_DPLTTG_TgtTrajLength_met;       /* '<Root>/Inport26' */
uint8_T MCTLFC_DPLTVG_TrajPlanServQu_nu;        /* '<Root>/Inport27' */
real32_T MCTLFC_DPLTVG_WeightTgtDistY_nu;       /* '<Root>/Inport28' */
real32_T MCTLFC_DPLTVG_WeightEndTime_nu;        /* '<Root>/Inport29' */
real32_T MCTLFC_DPLTVG_DistYToLeTgtArea_met;    /* '<Root>/Inport30' */
real32_T MCTLFC_DPLTVG_DistYToRiTgtArea_met;    /* '<Root>/Inport31' */
real32_T MCTLFC_DPLTVG_FTireAclMax_mps2;        /* '<Root>/Inport32' */
real32_T MCTLFC_DPLTVG_FTireAclMin_mps2;        /* '<Root>/Inport33' */
uint8_T MCTLFC_DPLTVG_TrajGuiQu_nu;             /* '<Root>/Inport34' */
boolean_T MCTLFC_DPLTVG_TriggerReplan_bool;     /* '<Root>/Inport35' */
real32_T MCTLFC_DPLTVG_PredTimeHeadAng_sec;     /* '<Root>/Inport36' */
real32_T MCTLFC_DPLTVG_PredTimeCrv_sec;         /* '<Root>/Inport37' */
real32_T MCTLFC_DPLTVG_PlanningHorzion_sec;     /* '<Root>/Inport38' */
real32_T MCTLFC_DPLTVG_ObstacleVelX_mps;        /* '<Root>/Inport39' */
real32_T MCTLFC_DPLTVG_ObstacleAclX_mps2;       /* '<Root>/Inport40' */
real32_T MCTLFC_DPLTVG_ObstacleWidth_met;       /* '<Root>/Inport41' */
real32_T MCTLFC_DPLTVG_ObstacleDistX_met;       /* '<Root>/Inport42' */
real32_T MCTLFC_DPLTVG_ObstacleDistY_met;       /* '<Root>/Inport43' */
boolean_T MCTLFC_DPLTVG_LtcyCompActivated_bool; /* '<Root>/Inport44' */
real32_T MCTLFC_DPLTVG_SensorTStamp_sec;        /* '<Root>/Inport45' */
real32_T MCTLFC_DPLTVG_MaxCrvTrajGuiCtrl_1pm;   /* '<Root>/Inport46' */
real32_T MCTLFC_DPLTVG_MaxCrvGrdBuildup_1pms;   /* '<Root>/Inport47' */
real32_T MCTLFC_DPLTVG_MaxCrvGrdRed_1pms;       /* '<Root>/Inport48' */
real32_T MCTLFC_DPLTVG_GrdLimitTgtCrvTGC_1pms;  /* '<Root>/Inport49' */
real32_T MCTLFC_DPLTVG_StrWhStifLimit_nu;       /* '<Root>/Inport50' */
real32_T MCTLFC_DPLTVG_StrWhStifGrad_1ps;       /* '<Root>/Inport252' */
real32_T MCTLFC_DPLTVG_TrqRampGrad_1ps;         /* '<Root>/Inport253' */
real32_T MCTLFC_DPLTVG_MaxTrqScaILimit_nu;      /* '<Root>/Inport51' */
real32_T MCTLFC_DPLTVG_MaxTrqScalGrad_1ps;      /* '<Root>/Inport250' */
boolean_T MCTLFC_DPLTVG_HighStatAccu_bool;      /* '<Root>/Inport251' */
boolean_T MCTLFC_DPLTVG_LimiterActivated_bool;  /* '<Root>/Inport52' */
real32_T MCTLFC_DPLTVG_LimiterTimeDuration_sec; /* '<Root>/Inport53' */
real32_T MCTLFC_DPLTVG_MaxJerkAllowed_mps3;     /* '<Root>/Inport258' */
real32_T MCTLFC_DPLTVG_DeratingLevel_nu;        /* '<Root>/Inport262' */
uint8_T MCTLFC_SysStateRDP_nu;                  /* '<Root>/Inport54' */
real32_T MCTLFC_DPRTTG_LeCridrBndPosX0_met;     /* '<Root>/Inport55' */
real32_T MCTLFC_DPRTTG_LeCridrBndPosY0_met;     /* '<Root>/Inport56' */
real32_T MCTLFC_DPRTTG_LeCridrBndHeadAng_rad;   /* '<Root>/Inport57' */
real32_T MCTLFC_DPRTTG_LeCridrBndCrv_1pm;       /* '<Root>/Inport58' */
real32_T MCTLFC_DPRTTG_LeCridrBndCrvChng_1pm2;  /* '<Root>/Inport59' */
real32_T MCTLFC_DPRTTG_LeCridrBndLength_met;    /* '<Root>/Inport60' */
real32_T MCTLFC_DPRTTG_RiCridrBndPosX0_met;     /* '<Root>/Inport61' */
real32_T MCTLFC_DPRTTG_RiCridrBndPosY0_met;     /* '<Root>/Inport62' */
real32_T MCTLFC_DPRTTG_RiCridrBndHeadAng_rad;   /* '<Root>/Inport63' */
real32_T MCTLFC_DPRTTG_RiCridrBndCrv_1pm;       /* '<Root>/Inport64' */
real32_T MCTLFC_DPRTTG_RiCridrBndCrvChng_1pm2;  /* '<Root>/Inport65' */
real32_T MCTLFC_DPRTTG_RiCridrBndLength_met;    /* '<Root>/Inport66' */
real32_T MCTLFC_DPRTTG_TgtTrajPosX0_met;        /* '<Root>/Inport67' */
real32_T MCTLFC_DPRTTG_TgtTrajPosY0_met;        /* '<Root>/Inport68' */
real32_T MCTLFC_DPRTTG_TgtTrajHeadAng_rad;      /* '<Root>/Inport69' */
real32_T MCTLFC_DPRTTG_TgtTrajCrv_1pm;          /* '<Root>/Inport70' */
real32_T MCTLFC_DPRTTG_TgtTrajCrvChng_1pm2;     /* '<Root>/Inport71' */
real32_T MCTLFC_DPRTTG_TgtTrajLength_met;       /* '<Root>/Inport72' */
uint8_T MCTLFC_DPRTVG_TrajPlanServQu_nu;        /* '<Root>/Inport73' */
real32_T MCTLFC_DPRTVG_WeightTgtDistY_nu;       /* '<Root>/Inport74' */
real32_T MCTLFC_DPRTVG_WeightEndTime_nu;        /* '<Root>/Inport75' */
real32_T MCTLFC_DPRTVG_DistYToLeTgtArea_met;    /* '<Root>/Inport76' */
real32_T MCTLFC_DPRTVG_DistYToRiTgtArea_met;    /* '<Root>/Inport77' */
real32_T MCTLFC_DPRTVG_FTireAclMax_mps2;        /* '<Root>/Inport78' */
real32_T MCTLFC_DPRTVG_FTireAclMin_mps2;        /* '<Root>/Inport79' */
uint8_T MCTLFC_DPRTVG_TrajGuiQu_nu;             /* '<Root>/Inport80' */
boolean_T MCTLFC_DPRTVG_TriggerReplan_bool;     /* '<Root>/Inport81' */
real32_T MCTLFC_DPRTVG_PredTimeHeadAng_sec;     /* '<Root>/Inport82' */
real32_T MCTLFC_DPRTVG_PredTimeCrv_sec;         /* '<Root>/Inport83' */
real32_T MCTLFC_DPRTVG_PlanningHorzion_sec;     /* '<Root>/Inport84' */
real32_T MCTLFC_DPRTVG_ObstacleVelX_mps;        /* '<Root>/Inport85' */
real32_T MCTLFC_DPRTVG_ObstacleAclX_mps2;       /* '<Root>/Inport86' */
real32_T MCTLFC_DPRTVG_ObstacleWidth_met;       /* '<Root>/Inport87' */
real32_T MCTLFC_DPRTVG_ObstacleDistX_met;       /* '<Root>/Inport88' */
real32_T MCTLFC_DPRTVG_ObstacleDistY_met;       /* '<Root>/Inport89' */
boolean_T MCTLFC_DPRTVG_LtcyCompActivated_bool; /* '<Root>/Inport90' */
real32_T MCTLFC_DPRTVG_SensorTStamp_sec;        /* '<Root>/Inport91' */
real32_T MCTLFC_DPRTVG_MaxCrvTrajGuiCtrl_1pm;   /* '<Root>/Inport92' */
real32_T MCTLFC_DPRTVG_MaxCrvGrdBuildup_1pms;   /* '<Root>/Inport93' */
real32_T MCTLFC_DPRTVG_MaxCrvGrdRed_1pms;       /* '<Root>/Inport94' */
real32_T MCTLFC_DPRTVG_GrdLimitTgtCrvTGC_1pms;  /* '<Root>/Inport95' */
real32_T MCTLFC_DPRTVG_StrWhStifLimit_nu;       /* '<Root>/Inport96' */
real32_T MCTLFC_DPRTVG_StrWhStifGrad_1ps;       /* '<Root>/Inport248' */
real32_T MCTLFC_DPRTVG_TrqRampGrad_1ps;         /* '<Root>/Inport249' */
real32_T MCTLFC_DPRTVG_MaxTrqScaILimit_nu;      /* '<Root>/Inport97' */
real32_T MCTLFC_DPRTVG_MaxTrqScalGrad_1ps;      /* '<Root>/Inport246' */
boolean_T MCTLFC_DPRTVG_HighStatAccu_bool;      /* '<Root>/Inport247' */
boolean_T MCTLFC_DPRTVG_LimiterActivated_bool;  /* '<Root>/Inport98' */
real32_T MCTLFC_DPRTVG_LimiterTimeDuration_sec; /* '<Root>/Inport99' */
real32_T MCTLFC_DPRTVG_MaxJerkAllowed_mps3;     /* '<Root>/Inport266' */
real32_T MCTLFC_DPRTVG_DeratingLevel_nu;        /* '<Root>/Inport267' */
uint8_T MCTLFC_SysStateLDPOC_nu;                /* '<Root>/Inport100' */
real32_T MCTLFC_DPOTTG_LeCridrBndPosX0_met;     /* '<Root>/Inport101' */
real32_T MCTLFC_DPOTTG_LeCridrBndPosY0_met;     /* '<Root>/Inport102' */
real32_T MCTLFC_DPOTTG_LeCridrBndHeadAng_rad;   /* '<Root>/Inport103' */
real32_T MCTLFC_DPOTTG_LeCridrBndCrv_1pm;       /* '<Root>/Inport104' */
real32_T MCTLFC_DPOTTG_LeCridrBndCrvChng_1pm2;  /* '<Root>/Inport105' */
real32_T MCTLFC_DPOTTG_LeCridrBndLength_met;    /* '<Root>/Inport106' */
real32_T MCTLFC_DPOTTG_RiCridrBndPosX0_met;     /* '<Root>/Inport107' */
real32_T MCTLFC_DPOTTG_RiCridrBndPosY0_met;     /* '<Root>/Inport108' */
real32_T MCTLFC_DPOTTG_RiCridrBndHeadAng_rad;   /* '<Root>/Inport109' */
real32_T MCTLFC_DPOTTG_RiCridrBndCrv_1pm;       /* '<Root>/Inport110' */
real32_T MCTLFC_DPOTTG_RiCridrBndCrvChng_1pm2;  /* '<Root>/Inport111' */
real32_T MCTLFC_DPOTTG_RiCridrBndLength_met;    /* '<Root>/Inport112' */
real32_T MCTLFC_DPOTTG_TgtTrajPosX0_met;        /* '<Root>/Inport113' */
real32_T MCTLFC_DPOTTG_TgtTrajPosY0_met;        /* '<Root>/Inport114' */
real32_T MCTLFC_DPOTTG_TgtTrajHeadAng_rad;      /* '<Root>/Inport115' */
real32_T MCTLFC_DPOTTG_TgtTrajCrv_1pm;          /* '<Root>/Inport116' */
real32_T MCTLFC_DPOTTG_TgtTrajCrvChng_1pm2;     /* '<Root>/Inport117' */
real32_T MCTLFC_DPOTTG_TgtTrajLength_met;       /* '<Root>/Inport118' */
uint8_T MCTLFC_DPOTVG_TrajPlanServQu_nu;        /* '<Root>/Inport119' */
real32_T MCTLFC_DPOTVG_WeightTgtDistY_nu;       /* '<Root>/Inport120' */
real32_T MCTLFC_DPOTVG_WeightEndTime_nu;        /* '<Root>/Inport121' */
real32_T MCTLFC_DPOTVG_DistYToLeTgtArea_met;    /* '<Root>/Inport122' */
real32_T MCTLFC_DPOTVG_DistYToRiTgtArea_met;    /* '<Root>/Inport123' */
real32_T MCTLFC_DPOTVG_FTireAclMax_mps2;        /* '<Root>/Inport124' */
real32_T MCTLFC_DPOTVG_FTireAclMin_mps2;        /* '<Root>/Inport125' */
uint8_T MCTLFC_DPOTVG_TrajGuiQu_nu;             /* '<Root>/Inport126' */
boolean_T MCTLFC_DPOTVG_TriggerReplan_bool;     /* '<Root>/Inport127' */
real32_T MCTLFC_DPOTVG_PredTimeHeadAng_sec;     /* '<Root>/Inport128' */
real32_T MCTLFC_DPOTVG_PredTimeCrv_sec;         /* '<Root>/Inport129' */
real32_T MCTLFC_DPOTVG_PlanningHorzion_sec;     /* '<Root>/Inport130' */
real32_T MCTLFC_DPOTVG_ObstacleVelX_mps;        /* '<Root>/Inport131' */
real32_T MCTLFC_DPOTVG_ObstacleAclX_mps2;       /* '<Root>/Inport132' */
real32_T MCTLFC_DPOTVG_ObstacleWidth_met;       /* '<Root>/Inport133' */
real32_T MCTLFC_DPOTVG_ObstacleDistX_met;       /* '<Root>/Inport134' */
real32_T MCTLFC_DPOTVG_ObstacleDistY_met;       /* '<Root>/Inport135' */
boolean_T MCTLFC_DPOTVG_LtcyCompActivated_bool; /* '<Root>/Inport136' */
real32_T MCTLFC_DPOTVG_SensorTStamp_sec;        /* '<Root>/Inport137' */
real32_T MCTLFC_DPOTVG_MaxCrvTrajGuiCtrl_1pm;   /* '<Root>/Inport138' */
real32_T MCTLFC_DPOTVG_MaxCrvGrdBuildup_1pms;   /* '<Root>/Inport139' */
real32_T MCTLFC_DPOTVG_MaxCrvGrdRed_1pms;       /* '<Root>/Inport140' */
real32_T MCTLFC_DPOTVG_GrdLimitTgtCrvTGC_1pms;  /* '<Root>/Inport141' */
real32_T MCTLFC_DPOTVG_StrWhStifLimit_nu;       /* '<Root>/Inport142' */
real32_T MCTLFC_DPOTVG_StrWhStifGrad_1ps;       /* '<Root>/Inport244' */
real32_T MCTLFC_DPOTVG_TrqRampGrad_1ps;         /* '<Root>/Inport245' */
real32_T MCTLFC_DPOTVG_MaxTrqScaILimit_nu;      /* '<Root>/Inport143' */
real32_T MCTLFC_DPOTVG_MaxTrqScalGrad_1ps;      /* '<Root>/Inport242' */
boolean_T MCTLFC_DPOTVG_HighStatAccu_bool;      /* '<Root>/Inport243' */
boolean_T MCTLFC_DPOTVG_LimiterActivated_bool;  /* '<Root>/Inport144' */
real32_T MCTLFC_DPOTVG_LimiterTimeDuration_sec; /* '<Root>/Inport145' */
real32_T MCTLFC_DPOTVG_MaxJerkAllowed_mps3;     /* '<Root>/Inport259' */
real32_T MCTLFC_DPOTVG_DeratingLevel_nu;        /* '<Root>/Inport263' */
uint8_T MCTLFC_SysStateTJA_nu;                  /* '<Root>/Inport146' */
real32_T MCTLFC_TJATTG_LeCridrBndPosX0_met;     /* '<Root>/Inport147' */
real32_T MCTLFC_TJATTG_LeCridrBndPosY0_met;     /* '<Root>/Inport148' */
real32_T MCTLFC_TJATTG_LeCridrBndHeadAng_rad;   /* '<Root>/Inport149' */
real32_T MCTLFC_TJATTG_LeCridrBndCrv_1pm;       /* '<Root>/Inport150' */
real32_T MCTLFC_TJATTG_LeCridrBndCrvChng_1pm2;  /* '<Root>/Inport151' */
real32_T MCTLFC_TJATTG_LeCridrBndLength_met;    /* '<Root>/Inport152' */
real32_T MCTLFC_TJATTG_RiCridrBndPosX0_met;     /* '<Root>/Inport153' */
real32_T MCTLFC_TJATTG_RiCridrBndPosY0_met;     /* '<Root>/Inport154' */
real32_T MCTLFC_TJATTG_RiCridrBndHeadAng_rad;   /* '<Root>/Inport155' */
real32_T MCTLFC_TJATTG_RiCridrBndCrv_1pm;       /* '<Root>/Inport156' */
real32_T MCTLFC_TJATTG_RiCridrBndCrvChng_1pm2;  /* '<Root>/Inport157' */
real32_T MCTLFC_TJATTG_RiCridrBndLength_met;    /* '<Root>/Inport158' */
real32_T MCTLFC_TJATTG_TgtTrajPosX0_met;        /* '<Root>/Inport159' */
real32_T MCTLFC_TJATTG_TgtTrajPosY0_met;        /* '<Root>/Inport160' */
real32_T MCTLFC_TJATTG_TgtTrajHeadAng_rad;      /* '<Root>/Inport161' */
real32_T MCTLFC_TJATTG_TgtTrajCrv_1pm;          /* '<Root>/Inport162' */
real32_T MCTLFC_TJATTG_TgtTrajCrvChng_1pm2;     /* '<Root>/Inport163' */
real32_T MCTLFC_TJATTG_TgtTrajLength_met;       /* '<Root>/Inport164' */
uint8_T MCTLFC_TJATVG_TrajPlanServQu_nu;        /* '<Root>/Inport165' */
real32_T MCTLFC_TJATVG_WeightTgtDistY_nu;       /* '<Root>/Inport166' */
real32_T MCTLFC_TJATVG_WeightEndTime_nu;        /* '<Root>/Inport167' */
real32_T MCTLFC_TJATVG_DistYToLeTgtArea_met;    /* '<Root>/Inport168' */
real32_T MCTLFC_TJATVG_DistYToRiTgtArea_met;    /* '<Root>/Inport169' */
real32_T MCTLFC_TJATVG_FTireAclMax_mps2;        /* '<Root>/Inport170' */
real32_T MCTLFC_TJATVG_FTireAclMin_mps2;        /* '<Root>/Inport171' */
uint8_T MCTLFC_TJATVG_TrajGuiQu_nu;             /* '<Root>/Inport172' */
boolean_T MCTLFC_TJATVG_TriggerReplan_bool;     /* '<Root>/Inport173' */
real32_T MCTLFC_TJATVG_PredTimeHeadAng_sec;     /* '<Root>/Inport174' */
real32_T MCTLFC_TJATVG_PredTimeCrv_sec;         /* '<Root>/Inport175' */
real32_T MCTLFC_TJATVG_PlanningHorzion_sec;     /* '<Root>/Inport176' */
real32_T MCTLFC_TJATVG_ObstacleVelX_mps;        /* '<Root>/Inport177' */
real32_T MCTLFC_TJATVG_ObstacleAclX_mps2;       /* '<Root>/Inport178' */
real32_T MCTLFC_TJATVG_ObstacleWidth_met;       /* '<Root>/Inport179' */
real32_T MCTLFC_TJATVG_ObstacleDistX_met;       /* '<Root>/Inport180' */
real32_T MCTLFC_TJATVG_ObstacleDistY_met;       /* '<Root>/Inport181' */
boolean_T MCTLFC_TJATVG_LtcyCompActivated_bool; /* '<Root>/Inport182' */
real32_T MCTLFC_TJATVG_SensorTStamp_sec;        /* '<Root>/Inport183' */
real32_T MCTLFC_TJATVG_MaxCrvTrajGuiCtrl_1pm;   /* '<Root>/Inport184' */
real32_T MCTLFC_TJATVG_MaxCrvGrdBuildup_1pms;   /* '<Root>/Inport185' */
real32_T MCTLFC_TJATVG_MaxCrvGrdRed_1pms;       /* '<Root>/Inport186' */
real32_T MCTLFC_TJATVG_GrdLimitTgtCrvTGC_1pms;  /* '<Root>/Inport187' */
real32_T MCTLFC_TJATVG_StrWhStifLimit_nu;       /* '<Root>/Inport188' */
real32_T MCTLFC_TJATVG_StrWhStifGrad_1ps;       /* '<Root>/Inport240' */
real32_T MCTLFC_TJATVG_TrqRampGrad_1ps;         /* '<Root>/Inport241' */
real32_T MCTLFC_TJATVG_MaxTrqScaILimit_nu;      /* '<Root>/Inport189' */
real32_T MCTLFC_TJATVG_MaxTrqScalGrad_1ps;      /* '<Root>/Inport238' */
boolean_T MCTLFC_TJATVG_HighStatAccu_bool;      /* '<Root>/Inport239' */
boolean_T MCTLFC_TJATVG_LimiterActivated_bool;  /* '<Root>/Inport190' */
real32_T MCTLFC_TJATVG_LimiterTimeDuration_sec; /* '<Root>/Inport191' */
real32_T MCTLFC_TJATVG_MaxJerkAllowed_mps3;     /* '<Root>/Inport260' */
real32_T MCTLFC_TJATVG_DeratingLevel_nu;        /* '<Root>/Inport264' */
uint8_T MCTLFC_SysStateALCA_nu;                 /* '<Root>/Inport192' */
real32_T MCTLFC_LCRTTG_LeCridrBndPosX0_met;     /* '<Root>/Inport193' */
real32_T MCTLFC_LCRTTG_LeCridrBndPosY0_met;     /* '<Root>/Inport194' */
real32_T MCTLFC_LCRTTG_LeCridrBndHeadAng_rad;   /* '<Root>/Inport195' */
real32_T MCTLFC_LCRTTG_LeCridrBndCrv_1pm;       /* '<Root>/Inport196' */
real32_T MCTLFC_LCRTTG_LeCridrBndCrvChng_1pm2;  /* '<Root>/Inport197' */
real32_T MCTLFC_LCRTTG_LeCridrBndLength_met;    /* '<Root>/Inport198' */
real32_T MCTLFC_LCRTTG_RiCridrBndPosX0_met;     /* '<Root>/Inport199' */
real32_T MCTLFC_LCRTTG_RiCridrBndPosY0_met;     /* '<Root>/Inport200' */
real32_T MCTLFC_LCRTTG_RiCridrBndHeadAng_rad;   /* '<Root>/Inport201' */
real32_T MCTLFC_LCRTTG_RiCridrBndCrv_1pm;       /* '<Root>/Inport202' */
real32_T MCTLFC_LCRTTG_RiCridrBndCrvChng_1pm2;  /* '<Root>/Inport203' */
real32_T MCTLFC_LCRTTG_RiCridrBndLength_met;    /* '<Root>/Inport204' */
real32_T MCTLFC_LCRTTG_TgtTrajPosX0_met;        /* '<Root>/Inport205' */
real32_T MCTLFC_LCRTTG_TgtTrajPosY0_met;        /* '<Root>/Inport206' */
real32_T MCTLFC_LCRTTG_TgtTrajHeadAng_rad;      /* '<Root>/Inport207' */
real32_T MCTLFC_LCRTTG_TgtTrajCrv_1pm;          /* '<Root>/Inport208' */
real32_T MCTLFC_LCRTTG_TgtTrajCrvChng_1pm2;     /* '<Root>/Inport209' */
real32_T MCTLFC_LCRTTG_TgtTrajLength_met;       /* '<Root>/Inport210' */
uint8_T MCTLFC_LCRTVG_TrajPlanServQu_nu;        /* '<Root>/Inport211' */
real32_T MCTLFC_LCRTVG_WeightTgtDistY_nu;       /* '<Root>/Inport212' */
real32_T MCTLFC_LCRTVG_WeightEndTime_nu;        /* '<Root>/Inport213' */
real32_T MCTLFC_LCRTVG_DistYToLeTgtArea_met;    /* '<Root>/Inport214' */
real32_T MCTLFC_LCRTVG_DistYToRiTgtArea_met;    /* '<Root>/Inport215' */
real32_T MCTLFC_LCRTVG_FTireAclMax_mps2;        /* '<Root>/Inport216' */
real32_T MCTLFC_LCRTVG_FTireAclMin_mps2;        /* '<Root>/Inport217' */
uint8_T MCTLFC_LCRTVG_TrajGuiQu_nu;             /* '<Root>/Inport218' */
boolean_T MCTLFC_LCRTVG_TriggerReplan_bool;     /* '<Root>/Inport219' */
real32_T MCTLFC_LCRTVG_PredTimeHeadAng_sec;     /* '<Root>/Inport220' */
real32_T MCTLFC_LCRTVG_PredTimeCrv_sec;         /* '<Root>/Inport221' */
real32_T MCTLFC_LCRTVG_PlanningHorzion_sec;     /* '<Root>/Inport222' */
real32_T MCTLFC_LCRTVG_ObstacleVelX_mps;        /* '<Root>/Inport223' */
real32_T MCTLFC_LCRTVG_ObstacleAclX_mps2;       /* '<Root>/Inport224' */
real32_T MCTLFC_LCRTVG_ObstacleWidth_met;       /* '<Root>/Inport225' */
real32_T MCTLFC_LCRTVG_ObstacleDistX_met;       /* '<Root>/Inport226' */
real32_T MCTLFC_LCRTVG_ObstacleDistY_met;       /* '<Root>/Inport227' */
boolean_T MCTLFC_LCRTVG_LtcyCompActivated_bool; /* '<Root>/Inport228' */
real32_T MCTLFC_LCRTVG_SensorTStamp_sec;        /* '<Root>/Inport229' */
real32_T MCTLFC_LCRTVG_MaxCrvTrajGuiCtrl_1pm;   /* '<Root>/Inport230' */
real32_T MCTLFC_LCRTVG_MaxCrvGrdBuildup_1pms;   /* '<Root>/Inport231' */
real32_T MCTLFC_LCRTVG_MaxCrvGrdRed_1pms;       /* '<Root>/Inport232' */
real32_T MCTLFC_LCRTVG_GrdLimitTgtCrvTGC_1pms;  /* '<Root>/Inport233' */
real32_T MCTLFC_LCRTVG_StrWhStifLimit_nu;       /* '<Root>/Inport234' */
real32_T MCTLFC_LCRTVG_StrWhStifGrad_1ps;       /* '<Root>/Inport256' */
real32_T MCTLFC_LCRTVG_TrqRampGrad_1ps;         /* '<Root>/Inport257' */
real32_T MCTLFC_LCRTVG_MaxTrqScaILimit_nu;      /* '<Root>/Inport235' */
real32_T MCTLFC_LCRTVG_MaxTrqScalGrad_1ps;      /* '<Root>/Inport254' */
boolean_T MCTLFC_LCRTVG_HighStatAccu_bool;      /* '<Root>/Inport255' */
boolean_T MCTLFC_LCRTVG_LimiterActivated_bool;  /* '<Root>/Inport236' */
real32_T MCTLFC_LCRTVG_LimiterTimeDuration_sec; /* '<Root>/Inport237' */
real32_T MCTLFC_LCRTVG_MaxJerkAllowed_mps3;     /* '<Root>/Inport261' */
real32_T MCTLFC_LCRTVG_DeratingLevel_nu;        /* '<Root>/Inport265' */
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  Functionname: MCTLFC_Init                                  */ /*!

                                @brief: MCTLFC function init reset

                                @description:the function first exec reset
                              process

                                @param[in]:void

                                @return: void
                              *****************************************************************************/
void MCTLFC_Init(void) { MCTLFC_initialize(); }

/*****************************************************************************
  Functionname: MCTLFC_Exec                                  */ /*!

                                @brief:  MCTLFC Exec main function

                                @description:  MCTLFC main function

                                @param[in]:reqPorts   MCTLFC input
                                                       params     MCTLFC
                              parameter input
                                                       proPorts   MCTLFC output
                                                       debugInfo  MCTLFC debug
                              information

                                @return:void
                              *****************************************************************************/
void MCTLFC_Exec(const sMCTLFCInReq_st* reqPorts,
                 const sMCTLFCParam_st* params,
                 sMCTLFCOut_st* proPorts,
                 sMCTLFCDebug_st* debugInfo) {
    // printf("--------------------MCTLFC--------------------\n");
    debugInfo->uiVersionNum_nu = MCTLFC_VIRSION;
    MCTLFC_Input_initialize(reqPorts);
    MCTLFC_step();
    MCTLFC_Output_initialize(proPorts, debugInfo);
}

/*****************************************************************************
  Functionname: MCTLFC_Input_initialize                                  */ /*!

        @brief: write MCTLFC reqPorts input to inner

        @description: write MCTLFC reqPorts input to inner

        @param[in]:reqPorts   MCTLFC input

        @return:void
      *****************************************************************************/
void MCTLFC_Input_initialize(const sMCTLFCInReq_st* reqPorts) {
    MCTLFC_LCFRCV_SysCycleTimeSen_sec =
        reqPorts->MCTLFC_LCFRCV_SysCycleTimeSen_sec; /* '<Root>/Inport' */
    MCTLFC_LCFRCV_TJASTM_SysStateTJA_nu =
        reqPorts->MCTLFC_LCFRCV_TJASTM_SysStateTJA_nu; /* '<Root>/Inport1' */
    MCTLFC_DPLSMI_SysStateLDP_nu =
        reqPorts->MCTLFC_DPLSMI_SysStateLDP_nu; /* '<Root>/Inport2' */
    MCTLFC_DPOSTM_SysStateLDPOC_nu =
        reqPorts->MCTLFC_DPOSTM_SysStateLDPOC_nu; /* '<Root>/Inport3' */
    MCTLFC_DPRSMI_SysStateRDP_nu =
        reqPorts->MCTLFC_DPRSMI_SysStateRDP_nu; /* '<Root>/Inport4' */
    MCTLFC_LCRSMI_SysStateALCA_nu =
        reqPorts->MCTLFC_LCRSMI_SysStateALCA_nu; /* '<Root>/Inport5' */
    MCTLFC_LCSSTM_SysStateAOLC_nu =
        reqPorts->MCTLFC_LCSSTM_SysStateAOLC_nu; /* '<Root>/Inport6' */
    MCTLFC_ESASTM_SysStateESA_nu =
        reqPorts->MCTLFC_ESASTM_SysStateESA_nu; /* '<Root>/Inport7' */
    MCTLFC_SysStateLDP_nu =
        reqPorts->MCTLFC_SysStateLDP_nu; /* '<Root>/Inport8' */
    MCTLFC_DPLTTG_LeCridrBndPosX0_met =
        reqPorts->MCTLFC_DPLTTG_LeCridrBndPosX0_met; /* '<Root>/Inport9' */
    MCTLFC_DPLTTG_LeCridrBndPosY0_met =
        reqPorts->MCTLFC_DPLTTG_LeCridrBndPosY0_met; /* '<Root>/Inport10' */
    MCTLFC_DPLTTG_LeCridrBndHeadAng_rad =
        reqPorts->MCTLFC_DPLTTG_LeCridrBndHeadAng_rad; /* '<Root>/Inport11' */
    MCTLFC_DPLTTG_LeCridrBndCrv_1pm =
        reqPorts->MCTLFC_DPLTTG_LeCridrBndCrv_1pm; /* '<Root>/Inport12' */
    MCTLFC_DPLTTG_LeCridrBndCrvChng_1pm2 =
        reqPorts->MCTLFC_DPLTTG_LeCridrBndCrvChng_1pm2; /* '<Root>/Inport13' */
    MCTLFC_DPLTTG_LeCridrBndLength_met =
        reqPorts->MCTLFC_DPLTTG_LeCridrBndLength_met; /* '<Root>/Inport14' */
    MCTLFC_DPLTTG_RiCridrBndPosX0_met =
        reqPorts->MCTLFC_DPLTTG_RiCridrBndPosX0_met; /* '<Root>/Inport15' */
    MCTLFC_DPLTTG_RiCridrBndPosY0_met =
        reqPorts->MCTLFC_DPLTTG_RiCridrBndPosY0_met; /* '<Root>/Inport16' */
    MCTLFC_DPLTTG_RiCridrBndHeadAng_rad =
        reqPorts->MCTLFC_DPLTTG_RiCridrBndHeadAng_rad; /* '<Root>/Inport17' */
    MCTLFC_DPLTTG_RiCridrBndCrv_1pm =
        reqPorts->MCTLFC_DPLTTG_RiCridrBndCrv_1pm; /* '<Root>/Inport18' */
    MCTLFC_DPLTTG_RiCridrBndCrvChng_1pm2 =
        reqPorts->MCTLFC_DPLTTG_RiCridrBndCrvChng_1pm2; /* '<Root>/Inport19' */
    MCTLFC_DPLTTG_RiCridrBndLength_met =
        reqPorts->MCTLFC_DPLTTG_RiCridrBndLength_met; /* '<Root>/Inport20' */
    MCTLFC_DPLTTG_TgtTrajPosX0_met =
        reqPorts->MCTLFC_DPLTTG_TgtTrajPosX0_met; /* '<Root>/Inport21' */
    MCTLFC_DPLTTG_TgtTrajPosY0_met =
        reqPorts->MCTLFC_DPLTTG_TgtTrajPosY0_met; /* '<Root>/Inport22' */
    MCTLFC_DPLTTG_TgtTrajHeadAng_rad =
        reqPorts->MCTLFC_DPLTTG_TgtTrajHeadAng_rad; /* '<Root>/Inport23' */
    MCTLFC_DPLTTG_TgtTrajCrv_1pm =
        reqPorts->MCTLFC_DPLTTG_TgtTrajCrv_1pm; /* '<Root>/Inport24' */
    MCTLFC_DPLTTG_TgtTrajCrvChng_1pm2 =
        reqPorts->MCTLFC_DPLTTG_TgtTrajCrvChng_1pm2; /* '<Root>/Inport25' */
    MCTLFC_DPLTTG_TgtTrajLength_met =
        reqPorts->MCTLFC_DPLTTG_TgtTrajLength_met; /* '<Root>/Inport26' */
    MCTLFC_DPLTVG_TrajPlanServQu_nu =
        reqPorts->MCTLFC_DPLTVG_TrajPlanServQu_nu; /* '<Root>/Inport27' */
    MCTLFC_DPLTVG_WeightTgtDistY_nu =
        reqPorts->MCTLFC_DPLTVG_WeightTgtDistY_nu; /* '<Root>/Inport28' */
    MCTLFC_DPLTVG_WeightEndTime_nu =
        reqPorts->MCTLFC_DPLTVG_WeightEndTime_nu; /* '<Root>/Inport29' */
    MCTLFC_DPLTVG_DistYToLeTgtArea_met =
        reqPorts->MCTLFC_DPLTVG_DistYToLeTgtArea_met; /* '<Root>/Inport30' */
    MCTLFC_DPLTVG_DistYToRiTgtArea_met =
        reqPorts->MCTLFC_DPLTVG_DistYToRiTgtArea_met; /* '<Root>/Inport31' */
    MCTLFC_DPLTVG_FTireAclMax_mps2 =
        reqPorts->MCTLFC_DPLTVG_FTireAclMax_mps2; /* '<Root>/Inport32' */
    MCTLFC_DPLTVG_FTireAclMin_mps2 =
        reqPorts->MCTLFC_DPLTVG_FTireAclMin_mps2; /* '<Root>/Inport33' */
    MCTLFC_DPLTVG_TrajGuiQu_nu =
        reqPorts->MCTLFC_DPLTVG_TrajGuiQu_nu; /* '<Root>/Inport34' */
    MCTLFC_DPLTVG_TriggerReplan_bool =
        reqPorts->MCTLFC_DPLTVG_TriggerReplan_bool; /* '<Root>/Inport35' */
    MCTLFC_DPLTVG_PredTimeHeadAng_sec =
        reqPorts->MCTLFC_DPLTVG_PredTimeHeadAng_sec; /* '<Root>/Inport36' */
    MCTLFC_DPLTVG_PredTimeCrv_sec =
        reqPorts->MCTLFC_DPLTVG_PredTimeCrv_sec; /* '<Root>/Inport37' */
    MCTLFC_DPLTVG_PlanningHorzion_sec =
        reqPorts->MCTLFC_DPLTVG_PlanningHorzion_sec; /* '<Root>/Inport38' */
    MCTLFC_DPLTVG_ObstacleVelX_mps =
        reqPorts->MCTLFC_DPLTVG_ObstacleVelX_mps; /* '<Root>/Inport39' */
    MCTLFC_DPLTVG_ObstacleAclX_mps2 =
        reqPorts->MCTLFC_DPLTVG_ObstacleAclX_mps2; /* '<Root>/Inport40' */
    MCTLFC_DPLTVG_ObstacleWidth_met =
        reqPorts->MCTLFC_DPLTVG_ObstacleWidth_met; /* '<Root>/Inport41' */
    MCTLFC_DPLTVG_ObstacleDistX_met =
        reqPorts->MCTLFC_DPLTVG_ObstacleDistX_met; /* '<Root>/Inport42' */
    MCTLFC_DPLTVG_ObstacleDistY_met =
        reqPorts->MCTLFC_DPLTVG_ObstacleDistY_met; /* '<Root>/Inport43' */
    MCTLFC_DPLTVG_LtcyCompActivated_bool =
        reqPorts->MCTLFC_DPLTVG_LtcyCompActivated_bool; /* '<Root>/Inport44' */
    MCTLFC_DPLTVG_SensorTStamp_sec =
        reqPorts->MCTLFC_DPLTVG_SensorTStamp_sec; /* '<Root>/Inport45' */
    MCTLFC_DPLTVG_MaxCrvTrajGuiCtrl_1pm =
        reqPorts->MCTLFC_DPLTVG_MaxCrvTrajGuiCtrl_1pm; /* '<Root>/Inport46' */
    MCTLFC_DPLTVG_MaxCrvGrdBuildup_1pms =
        reqPorts->MCTLFC_DPLTVG_MaxCrvGrdBuildup_1pms; /* '<Root>/Inport47' */
    MCTLFC_DPLTVG_MaxCrvGrdRed_1pms =
        reqPorts->MCTLFC_DPLTVG_MaxCrvGrdRed_1pms; /* '<Root>/Inport48' */
    MCTLFC_DPLTVG_GrdLimitTgtCrvTGC_1pms =
        reqPorts->MCTLFC_DPLTVG_GrdLimitTgtCrvTGC_1pms; /* '<Root>/Inport49' */
    MCTLFC_DPLTVG_StrWhStifLimit_nu =
        reqPorts->MCTLFC_DPLTVG_StrWhStifLimit_nu; /* '<Root>/Inport50' */
    MCTLFC_DPLTVG_StrWhStifGrad_1ps =
        reqPorts->MCTLFC_DPLTVG_StrWhStifGrad_1ps; /* '<Root>/Inport252' */
    MCTLFC_DPLTVG_TrqRampGrad_1ps =
        reqPorts->MCTLFC_DPLTVG_TrqRampGrad_1ps; /* '<Root>/Inport253' */
    MCTLFC_DPLTVG_MaxTrqScaILimit_nu =
        reqPorts->MCTLFC_DPLTVG_MaxTrqScaILimit_nu; /* '<Root>/Inport51' */
    MCTLFC_DPLTVG_MaxTrqScalGrad_1ps =
        reqPorts->MCTLFC_DPLTVG_MaxTrqScalGrad_1ps; /* '<Root>/Inport250' */
    MCTLFC_DPLTVG_HighStatAccu_bool =
        reqPorts->MCTLFC_DPLTVG_HighStatAccu_bool; /* '<Root>/Inport251' */
    MCTLFC_DPLTVG_LimiterActivated_bool =
        reqPorts->MCTLFC_DPLTVG_LimiterActivated_bool; /* '<Root>/Inport52' */
    MCTLFC_DPLTVG_LimiterTimeDuration_sec =
        reqPorts->MCTLFC_DPLTVG_LimiterTimeDuration_sec; /* '<Root>/Inport53' */
    MCTLFC_DPLTVG_MaxJerkAllowed_mps3 =
        reqPorts->MCTLFC_DPLTVG_MaxJerkAllowed_mps3; /* '<Root>/Inport258' */
    MCTLFC_DPLTVG_DeratingLevel_nu =
        reqPorts->MCTLFC_DPLTVG_DeratingLevel_nu; /* '<Root>/Inport262' */
    MCTLFC_SysStateRDP_nu =
        reqPorts->MCTLFC_SysStateRDP_nu; /* '<Root>/Inport54' */
    MCTLFC_DPRTTG_LeCridrBndPosX0_met =
        reqPorts->MCTLFC_DPRTTG_LeCridrBndPosX0_met; /* '<Root>/Inport55' */
    MCTLFC_DPRTTG_LeCridrBndPosY0_met =
        reqPorts->MCTLFC_DPRTTG_LeCridrBndPosY0_met; /* '<Root>/Inport56' */
    MCTLFC_DPRTTG_LeCridrBndHeadAng_rad =
        reqPorts->MCTLFC_DPRTTG_LeCridrBndHeadAng_rad; /* '<Root>/Inport57' */
    MCTLFC_DPRTTG_LeCridrBndCrv_1pm =
        reqPorts->MCTLFC_DPRTTG_LeCridrBndCrv_1pm; /* '<Root>/Inport58' */
    MCTLFC_DPRTTG_LeCridrBndCrvChng_1pm2 =
        reqPorts->MCTLFC_DPRTTG_LeCridrBndCrvChng_1pm2; /* '<Root>/Inport59' */
    MCTLFC_DPRTTG_LeCridrBndLength_met =
        reqPorts->MCTLFC_DPRTTG_LeCridrBndLength_met; /* '<Root>/Inport60' */
    MCTLFC_DPRTTG_RiCridrBndPosX0_met =
        reqPorts->MCTLFC_DPRTTG_RiCridrBndPosX0_met; /* '<Root>/Inport61' */
    MCTLFC_DPRTTG_RiCridrBndPosY0_met =
        reqPorts->MCTLFC_DPRTTG_RiCridrBndPosY0_met; /* '<Root>/Inport62' */
    MCTLFC_DPRTTG_RiCridrBndHeadAng_rad =
        reqPorts->MCTLFC_DPRTTG_RiCridrBndHeadAng_rad; /* '<Root>/Inport63' */
    MCTLFC_DPRTTG_RiCridrBndCrv_1pm =
        reqPorts->MCTLFC_DPRTTG_RiCridrBndCrv_1pm; /* '<Root>/Inport64' */
    MCTLFC_DPRTTG_RiCridrBndCrvChng_1pm2 =
        reqPorts->MCTLFC_DPRTTG_RiCridrBndCrvChng_1pm2; /* '<Root>/Inport65' */
    MCTLFC_DPRTTG_RiCridrBndLength_met =
        reqPorts->MCTLFC_DPRTTG_RiCridrBndLength_met; /* '<Root>/Inport66' */
    MCTLFC_DPRTTG_TgtTrajPosX0_met =
        reqPorts->MCTLFC_DPRTTG_TgtTrajPosX0_met; /* '<Root>/Inport67' */
    MCTLFC_DPRTTG_TgtTrajPosY0_met =
        reqPorts->MCTLFC_DPRTTG_TgtTrajPosY0_met; /* '<Root>/Inport68' */
    MCTLFC_DPRTTG_TgtTrajHeadAng_rad =
        reqPorts->MCTLFC_DPRTTG_TgtTrajHeadAng_rad; /* '<Root>/Inport69' */
    MCTLFC_DPRTTG_TgtTrajCrv_1pm =
        reqPorts->MCTLFC_DPRTTG_TgtTrajCrv_1pm; /* '<Root>/Inport70' */
    MCTLFC_DPRTTG_TgtTrajCrvChng_1pm2 =
        reqPorts->MCTLFC_DPRTTG_TgtTrajCrvChng_1pm2; /* '<Root>/Inport71' */
    MCTLFC_DPRTTG_TgtTrajLength_met =
        reqPorts->MCTLFC_DPRTTG_TgtTrajLength_met; /* '<Root>/Inport72' */
    MCTLFC_DPRTVG_TrajPlanServQu_nu =
        reqPorts->MCTLFC_DPRTVG_TrajPlanServQu_nu; /* '<Root>/Inport73' */
    MCTLFC_DPRTVG_WeightTgtDistY_nu =
        reqPorts->MCTLFC_DPRTVG_WeightTgtDistY_nu; /* '<Root>/Inport74' */
    MCTLFC_DPRTVG_WeightEndTime_nu =
        reqPorts->MCTLFC_DPRTVG_WeightEndTime_nu; /* '<Root>/Inport75' */
    MCTLFC_DPRTVG_DistYToLeTgtArea_met =
        reqPorts->MCTLFC_DPRTVG_DistYToLeTgtArea_met; /* '<Root>/Inport76' */
    MCTLFC_DPRTVG_DistYToRiTgtArea_met =
        reqPorts->MCTLFC_DPRTVG_DistYToRiTgtArea_met; /* '<Root>/Inport77' */
    MCTLFC_DPRTVG_FTireAclMax_mps2 =
        reqPorts->MCTLFC_DPRTVG_FTireAclMax_mps2; /* '<Root>/Inport78' */
    MCTLFC_DPRTVG_FTireAclMin_mps2 =
        reqPorts->MCTLFC_DPRTVG_FTireAclMin_mps2; /* '<Root>/Inport79' */
    MCTLFC_DPRTVG_TrajGuiQu_nu =
        reqPorts->MCTLFC_DPRTVG_TrajGuiQu_nu; /* '<Root>/Inport80' */
    MCTLFC_DPRTVG_TriggerReplan_bool =
        reqPorts->MCTLFC_DPRTVG_TriggerReplan_bool; /* '<Root>/Inport81' */
    MCTLFC_DPRTVG_PredTimeHeadAng_sec =
        reqPorts->MCTLFC_DPRTVG_PredTimeHeadAng_sec; /* '<Root>/Inport82' */
    MCTLFC_DPRTVG_PredTimeCrv_sec =
        reqPorts->MCTLFC_DPRTVG_PredTimeCrv_sec; /* '<Root>/Inport83' */
    MCTLFC_DPRTVG_PlanningHorzion_sec =
        reqPorts->MCTLFC_DPRTVG_PlanningHorzion_sec; /* '<Root>/Inport84' */
    MCTLFC_DPRTVG_ObstacleVelX_mps =
        reqPorts->MCTLFC_DPRTVG_ObstacleVelX_mps; /* '<Root>/Inport85' */
    MCTLFC_DPRTVG_ObstacleAclX_mps2 =
        reqPorts->MCTLFC_DPRTVG_ObstacleAclX_mps2; /* '<Root>/Inport86' */
    MCTLFC_DPRTVG_ObstacleWidth_met =
        reqPorts->MCTLFC_DPRTVG_ObstacleWidth_met; /* '<Root>/Inport87' */
    MCTLFC_DPRTVG_ObstacleDistX_met =
        reqPorts->MCTLFC_DPRTVG_ObstacleDistX_met; /* '<Root>/Inport88' */
    MCTLFC_DPRTVG_ObstacleDistY_met =
        reqPorts->MCTLFC_DPRTVG_ObstacleDistY_met; /* '<Root>/Inport89' */
    MCTLFC_DPRTVG_LtcyCompActivated_bool =
        reqPorts->MCTLFC_DPRTVG_LtcyCompActivated_bool; /* '<Root>/Inport90' */
    MCTLFC_DPRTVG_SensorTStamp_sec =
        reqPorts->MCTLFC_DPRTVG_SensorTStamp_sec; /* '<Root>/Inport91' */
    MCTLFC_DPRTVG_MaxCrvTrajGuiCtrl_1pm =
        reqPorts->MCTLFC_DPRTVG_MaxCrvTrajGuiCtrl_1pm; /* '<Root>/Inport92' */
    MCTLFC_DPRTVG_MaxCrvGrdBuildup_1pms =
        reqPorts->MCTLFC_DPRTVG_MaxCrvGrdBuildup_1pms; /* '<Root>/Inport93' */
    MCTLFC_DPRTVG_MaxCrvGrdRed_1pms =
        reqPorts->MCTLFC_DPRTVG_MaxCrvGrdRed_1pms; /* '<Root>/Inport94' */
    MCTLFC_DPRTVG_GrdLimitTgtCrvTGC_1pms =
        reqPorts->MCTLFC_DPRTVG_GrdLimitTgtCrvTGC_1pms; /* '<Root>/Inport95' */
    MCTLFC_DPRTVG_StrWhStifLimit_nu =
        reqPorts->MCTLFC_DPRTVG_StrWhStifLimit_nu; /* '<Root>/Inport96' */
    MCTLFC_DPRTVG_StrWhStifGrad_1ps =
        reqPorts->MCTLFC_DPRTVG_StrWhStifGrad_1ps; /* '<Root>/Inport248' */
    MCTLFC_DPRTVG_TrqRampGrad_1ps =
        reqPorts->MCTLFC_DPRTVG_TrqRampGrad_1ps; /* '<Root>/Inport249' */
    MCTLFC_DPRTVG_MaxTrqScaILimit_nu =
        reqPorts->MCTLFC_DPRTVG_MaxTrqScaILimit_nu; /* '<Root>/Inport97' */
    MCTLFC_DPRTVG_MaxTrqScalGrad_1ps =
        reqPorts->MCTLFC_DPRTVG_MaxTrqScalGrad_1ps; /* '<Root>/Inport246' */
    MCTLFC_DPRTVG_HighStatAccu_bool =
        reqPorts->MCTLFC_DPRTVG_HighStatAccu_bool; /* '<Root>/Inport247' */
    MCTLFC_DPRTVG_LimiterActivated_bool =
        reqPorts->MCTLFC_DPRTVG_LimiterActivated_bool; /* '<Root>/Inport98' */
    MCTLFC_DPRTVG_LimiterTimeDuration_sec =
        reqPorts->MCTLFC_DPRTVG_LimiterTimeDuration_sec; /* '<Root>/Inport99' */
    MCTLFC_DPRTVG_MaxJerkAllowed_mps3 =
        reqPorts->MCTLFC_DPRTVG_MaxJerkAllowed_mps3; /* '<Root>/Inport266' */
    MCTLFC_DPRTVG_DeratingLevel_nu =
        reqPorts->MCTLFC_DPRTVG_DeratingLevel_nu; /* '<Root>/Inport267' */
    MCTLFC_SysStateLDPOC_nu =
        reqPorts->MCTLFC_SysStateLDPOC_nu; /* '<Root>/Inport100' */
    MCTLFC_DPOTTG_LeCridrBndPosX0_met =
        reqPorts->MCTLFC_DPOTTG_LeCridrBndPosX0_met; /* '<Root>/Inport101' */
    MCTLFC_DPOTTG_LeCridrBndPosY0_met =
        reqPorts->MCTLFC_DPOTTG_LeCridrBndPosY0_met; /* '<Root>/Inport102' */
    MCTLFC_DPOTTG_LeCridrBndHeadAng_rad =
        reqPorts->MCTLFC_DPOTTG_LeCridrBndHeadAng_rad; /* '<Root>/Inport103' */
    MCTLFC_DPOTTG_LeCridrBndCrv_1pm =
        reqPorts->MCTLFC_DPOTTG_LeCridrBndCrv_1pm; /* '<Root>/Inport104' */
    MCTLFC_DPOTTG_LeCridrBndCrvChng_1pm2 =
        reqPorts->MCTLFC_DPOTTG_LeCridrBndCrvChng_1pm2; /* '<Root>/Inport105' */
    MCTLFC_DPOTTG_LeCridrBndLength_met =
        reqPorts->MCTLFC_DPOTTG_LeCridrBndLength_met; /* '<Root>/Inport106' */
    MCTLFC_DPOTTG_RiCridrBndPosX0_met =
        reqPorts->MCTLFC_DPOTTG_RiCridrBndPosX0_met; /* '<Root>/Inport107' */
    MCTLFC_DPOTTG_RiCridrBndPosY0_met =
        reqPorts->MCTLFC_DPOTTG_RiCridrBndPosY0_met; /* '<Root>/Inport108' */
    MCTLFC_DPOTTG_RiCridrBndHeadAng_rad =
        reqPorts->MCTLFC_DPOTTG_RiCridrBndHeadAng_rad; /* '<Root>/Inport109' */
    MCTLFC_DPOTTG_RiCridrBndCrv_1pm =
        reqPorts->MCTLFC_DPOTTG_RiCridrBndCrv_1pm; /* '<Root>/Inport110' */
    MCTLFC_DPOTTG_RiCridrBndCrvChng_1pm2 =
        reqPorts->MCTLFC_DPOTTG_RiCridrBndCrvChng_1pm2; /* '<Root>/Inport111' */
    MCTLFC_DPOTTG_RiCridrBndLength_met =
        reqPorts->MCTLFC_DPOTTG_RiCridrBndLength_met; /* '<Root>/Inport112' */
    MCTLFC_DPOTTG_TgtTrajPosX0_met =
        reqPorts->MCTLFC_DPOTTG_TgtTrajPosX0_met; /* '<Root>/Inport113' */
    MCTLFC_DPOTTG_TgtTrajPosY0_met =
        reqPorts->MCTLFC_DPOTTG_TgtTrajPosY0_met; /* '<Root>/Inport114' */
    MCTLFC_DPOTTG_TgtTrajHeadAng_rad =
        reqPorts->MCTLFC_DPOTTG_TgtTrajHeadAng_rad; /* '<Root>/Inport115' */
    MCTLFC_DPOTTG_TgtTrajCrv_1pm =
        reqPorts->MCTLFC_DPOTTG_TgtTrajCrv_1pm; /* '<Root>/Inport116' */
    MCTLFC_DPOTTG_TgtTrajCrvChng_1pm2 =
        reqPorts->MCTLFC_DPOTTG_TgtTrajCrvChng_1pm2; /* '<Root>/Inport117' */
    MCTLFC_DPOTTG_TgtTrajLength_met =
        reqPorts->MCTLFC_DPOTTG_TgtTrajLength_met; /* '<Root>/Inport118' */
    MCTLFC_DPOTVG_TrajPlanServQu_nu =
        reqPorts->MCTLFC_DPOTVG_TrajPlanServQu_nu; /* '<Root>/Inport119' */
    MCTLFC_DPOTVG_WeightTgtDistY_nu =
        reqPorts->MCTLFC_DPOTVG_WeightTgtDistY_nu; /* '<Root>/Inport120' */
    MCTLFC_DPOTVG_WeightEndTime_nu =
        reqPorts->MCTLFC_DPOTVG_WeightEndTime_nu; /* '<Root>/Inport121' */
    MCTLFC_DPOTVG_DistYToLeTgtArea_met =
        reqPorts->MCTLFC_DPOTVG_DistYToLeTgtArea_met; /* '<Root>/Inport122' */
    MCTLFC_DPOTVG_DistYToRiTgtArea_met =
        reqPorts->MCTLFC_DPOTVG_DistYToRiTgtArea_met; /* '<Root>/Inport123' */
    MCTLFC_DPOTVG_FTireAclMax_mps2 =
        reqPorts->MCTLFC_DPOTVG_FTireAclMax_mps2; /* '<Root>/Inport124' */
    MCTLFC_DPOTVG_FTireAclMin_mps2 =
        reqPorts->MCTLFC_DPOTVG_FTireAclMin_mps2; /* '<Root>/Inport125' */
    MCTLFC_DPOTVG_TrajGuiQu_nu =
        reqPorts->MCTLFC_DPOTVG_TrajGuiQu_nu; /* '<Root>/Inport126' */
    MCTLFC_DPOTVG_TriggerReplan_bool =
        reqPorts->MCTLFC_DPOTVG_TriggerReplan_bool; /* '<Root>/Inport127' */
    MCTLFC_DPOTVG_PredTimeHeadAng_sec =
        reqPorts->MCTLFC_DPOTVG_PredTimeHeadAng_sec; /* '<Root>/Inport128' */
    MCTLFC_DPOTVG_PredTimeCrv_sec =
        reqPorts->MCTLFC_DPOTVG_PredTimeCrv_sec; /* '<Root>/Inport129' */
    MCTLFC_DPOTVG_PlanningHorzion_sec =
        reqPorts->MCTLFC_DPOTVG_PlanningHorzion_sec; /* '<Root>/Inport130' */
    MCTLFC_DPOTVG_ObstacleVelX_mps =
        reqPorts->MCTLFC_DPOTVG_ObstacleVelX_mps; /* '<Root>/Inport131' */
    MCTLFC_DPOTVG_ObstacleAclX_mps2 =
        reqPorts->MCTLFC_DPOTVG_ObstacleAclX_mps2; /* '<Root>/Inport132' */
    MCTLFC_DPOTVG_ObstacleWidth_met =
        reqPorts->MCTLFC_DPOTVG_ObstacleWidth_met; /* '<Root>/Inport133' */
    MCTLFC_DPOTVG_ObstacleDistX_met =
        reqPorts->MCTLFC_DPOTVG_ObstacleDistX_met; /* '<Root>/Inport134' */
    MCTLFC_DPOTVG_ObstacleDistY_met =
        reqPorts->MCTLFC_DPOTVG_ObstacleDistY_met; /* '<Root>/Inport135' */
    MCTLFC_DPOTVG_LtcyCompActivated_bool =
        reqPorts->MCTLFC_DPOTVG_LtcyCompActivated_bool; /* '<Root>/Inport136' */
    MCTLFC_DPOTVG_SensorTStamp_sec =
        reqPorts->MCTLFC_DPOTVG_SensorTStamp_sec; /* '<Root>/Inport137' */
    MCTLFC_DPOTVG_MaxCrvTrajGuiCtrl_1pm =
        reqPorts->MCTLFC_DPOTVG_MaxCrvTrajGuiCtrl_1pm; /* '<Root>/Inport138' */
    MCTLFC_DPOTVG_MaxCrvGrdBuildup_1pms =
        reqPorts->MCTLFC_DPOTVG_MaxCrvGrdBuildup_1pms; /* '<Root>/Inport139' */
    MCTLFC_DPOTVG_MaxCrvGrdRed_1pms =
        reqPorts->MCTLFC_DPOTVG_MaxCrvGrdRed_1pms; /* '<Root>/Inport140' */
    MCTLFC_DPOTVG_GrdLimitTgtCrvTGC_1pms =
        reqPorts->MCTLFC_DPOTVG_GrdLimitTgtCrvTGC_1pms; /* '<Root>/Inport141' */
    MCTLFC_DPOTVG_StrWhStifLimit_nu =
        reqPorts->MCTLFC_DPOTVG_StrWhStifLimit_nu; /* '<Root>/Inport142' */
    MCTLFC_DPOTVG_StrWhStifGrad_1ps =
        reqPorts->MCTLFC_DPOTVG_StrWhStifGrad_1ps; /* '<Root>/Inport244' */
    MCTLFC_DPOTVG_TrqRampGrad_1ps =
        reqPorts->MCTLFC_DPOTVG_TrqRampGrad_1ps; /* '<Root>/Inport245' */
    MCTLFC_DPOTVG_MaxTrqScaILimit_nu =
        reqPorts->MCTLFC_DPOTVG_MaxTrqScaILimit_nu; /* '<Root>/Inport143' */
    MCTLFC_DPOTVG_MaxTrqScalGrad_1ps =
        reqPorts->MCTLFC_DPOTVG_MaxTrqScalGrad_1ps; /* '<Root>/Inport242' */
    MCTLFC_DPOTVG_HighStatAccu_bool =
        reqPorts->MCTLFC_DPOTVG_HighStatAccu_bool; /* '<Root>/Inport243' */
    MCTLFC_DPOTVG_LimiterActivated_bool =
        reqPorts->MCTLFC_DPOTVG_LimiterActivated_bool; /* '<Root>/Inport144' */
    MCTLFC_DPOTVG_LimiterTimeDuration_sec =
        reqPorts
            ->MCTLFC_DPOTVG_LimiterTimeDuration_sec; /* '<Root>/Inport145' */
    MCTLFC_DPOTVG_MaxJerkAllowed_mps3 =
        reqPorts->MCTLFC_DPOTVG_MaxJerkAllowed_mps3; /* '<Root>/Inport259' */
    MCTLFC_DPOTVG_DeratingLevel_nu =
        reqPorts->MCTLFC_DPOTVG_DeratingLevel_nu; /* '<Root>/Inport263' */
    MCTLFC_SysStateTJA_nu =
        reqPorts->MCTLFC_SysStateTJA_nu; /* '<Root>/Inport146' */
    MCTLFC_TJATTG_LeCridrBndPosX0_met =
        reqPorts->MCTLFC_TJATTG_LeCridrBndPosX0_met; /* '<Root>/Inport147' */
    MCTLFC_TJATTG_LeCridrBndPosY0_met =
        reqPorts->MCTLFC_TJATTG_LeCridrBndPosY0_met; /* '<Root>/Inport148' */
    MCTLFC_TJATTG_LeCridrBndHeadAng_rad =
        reqPorts->MCTLFC_TJATTG_LeCridrBndHeadAng_rad; /* '<Root>/Inport149' */
    MCTLFC_TJATTG_LeCridrBndCrv_1pm =
        reqPorts->MCTLFC_TJATTG_LeCridrBndCrv_1pm; /* '<Root>/Inport150' */
    MCTLFC_TJATTG_LeCridrBndCrvChng_1pm2 =
        reqPorts->MCTLFC_TJATTG_LeCridrBndCrvChng_1pm2; /* '<Root>/Inport151' */
    MCTLFC_TJATTG_LeCridrBndLength_met =
        reqPorts->MCTLFC_TJATTG_LeCridrBndLength_met; /* '<Root>/Inport152' */
    MCTLFC_TJATTG_RiCridrBndPosX0_met =
        reqPorts->MCTLFC_TJATTG_RiCridrBndPosX0_met; /* '<Root>/Inport153' */
    MCTLFC_TJATTG_RiCridrBndPosY0_met =
        reqPorts->MCTLFC_TJATTG_RiCridrBndPosY0_met; /* '<Root>/Inport154' */
    MCTLFC_TJATTG_RiCridrBndHeadAng_rad =
        reqPorts->MCTLFC_TJATTG_RiCridrBndHeadAng_rad; /* '<Root>/Inport155' */
    MCTLFC_TJATTG_RiCridrBndCrv_1pm =
        reqPorts->MCTLFC_TJATTG_RiCridrBndCrv_1pm; /* '<Root>/Inport156' */
    MCTLFC_TJATTG_RiCridrBndCrvChng_1pm2 =
        reqPorts->MCTLFC_TJATTG_RiCridrBndCrvChng_1pm2; /* '<Root>/Inport157' */
    MCTLFC_TJATTG_RiCridrBndLength_met =
        reqPorts->MCTLFC_TJATTG_RiCridrBndLength_met; /* '<Root>/Inport158' */
    MCTLFC_TJATTG_TgtTrajPosX0_met =
        reqPorts->MCTLFC_TJATTG_TgtTrajPosX0_met; /* '<Root>/Inport159' */
    MCTLFC_TJATTG_TgtTrajPosY0_met =
        reqPorts->MCTLFC_TJATTG_TgtTrajPosY0_met; /* '<Root>/Inport160' */
    MCTLFC_TJATTG_TgtTrajHeadAng_rad =
        reqPorts->MCTLFC_TJATTG_TgtTrajHeadAng_rad; /* '<Root>/Inport161' */
    MCTLFC_TJATTG_TgtTrajCrv_1pm =
        reqPorts->MCTLFC_TJATTG_TgtTrajCrv_1pm; /* '<Root>/Inport162' */
    MCTLFC_TJATTG_TgtTrajCrvChng_1pm2 =
        reqPorts->MCTLFC_TJATTG_TgtTrajCrvChng_1pm2; /* '<Root>/Inport163' */
    MCTLFC_TJATTG_TgtTrajLength_met =
        reqPorts->MCTLFC_TJATTG_TgtTrajLength_met; /* '<Root>/Inport164' */
    MCTLFC_TJATVG_TrajPlanServQu_nu =
        reqPorts->MCTLFC_TJATVG_TrajPlanServQu_nu; /* '<Root>/Inport165' */
    MCTLFC_TJATVG_WeightTgtDistY_nu =
        reqPorts->MCTLFC_TJATVG_WeightTgtDistY_nu; /* '<Root>/Inport166' */
    MCTLFC_TJATVG_WeightEndTime_nu =
        reqPorts->MCTLFC_TJATVG_WeightEndTime_nu; /* '<Root>/Inport167' */
    MCTLFC_TJATVG_DistYToLeTgtArea_met =
        reqPorts->MCTLFC_TJATVG_DistYToLeTgtArea_met; /* '<Root>/Inport168' */
    MCTLFC_TJATVG_DistYToRiTgtArea_met =
        reqPorts->MCTLFC_TJATVG_DistYToRiTgtArea_met; /* '<Root>/Inport169' */
    MCTLFC_TJATVG_FTireAclMax_mps2 =
        reqPorts->MCTLFC_TJATVG_FTireAclMax_mps2; /* '<Root>/Inport170' */
    MCTLFC_TJATVG_FTireAclMin_mps2 =
        reqPorts->MCTLFC_TJATVG_FTireAclMin_mps2; /* '<Root>/Inport171' */
    MCTLFC_TJATVG_TrajGuiQu_nu =
        reqPorts->MCTLFC_TJATVG_TrajGuiQu_nu; /* '<Root>/Inport172' */
    MCTLFC_TJATVG_TriggerReplan_bool =
        reqPorts->MCTLFC_TJATVG_TriggerReplan_bool; /* '<Root>/Inport173' */
    MCTLFC_TJATVG_PredTimeHeadAng_sec =
        reqPorts->MCTLFC_TJATVG_PredTimeHeadAng_sec; /* '<Root>/Inport174' */
    MCTLFC_TJATVG_PredTimeCrv_sec =
        reqPorts->MCTLFC_TJATVG_PredTimeCrv_sec; /* '<Root>/Inport175' */
    MCTLFC_TJATVG_PlanningHorzion_sec =
        reqPorts->MCTLFC_TJATVG_PlanningHorzion_sec; /* '<Root>/Inport176' */
    MCTLFC_TJATVG_ObstacleVelX_mps =
        reqPorts->MCTLFC_TJATVG_ObstacleVelX_mps; /* '<Root>/Inport177' */
    MCTLFC_TJATVG_ObstacleAclX_mps2 =
        reqPorts->MCTLFC_TJATVG_ObstacleAclX_mps2; /* '<Root>/Inport178' */
    MCTLFC_TJATVG_ObstacleWidth_met =
        reqPorts->MCTLFC_TJATVG_ObstacleWidth_met; /* '<Root>/Inport179' */
    MCTLFC_TJATVG_ObstacleDistX_met =
        reqPorts->MCTLFC_TJATVG_ObstacleDistX_met; /* '<Root>/Inport180' */
    MCTLFC_TJATVG_ObstacleDistY_met =
        reqPorts->MCTLFC_TJATVG_ObstacleDistY_met; /* '<Root>/Inport181' */
    MCTLFC_TJATVG_LtcyCompActivated_bool =
        reqPorts->MCTLFC_TJATVG_LtcyCompActivated_bool; /* '<Root>/Inport182' */
    MCTLFC_TJATVG_SensorTStamp_sec =
        reqPorts->MCTLFC_TJATVG_SensorTStamp_sec; /* '<Root>/Inport183' */
    MCTLFC_TJATVG_MaxCrvTrajGuiCtrl_1pm =
        reqPorts->MCTLFC_TJATVG_MaxCrvTrajGuiCtrl_1pm; /* '<Root>/Inport184' */
    MCTLFC_TJATVG_MaxCrvGrdBuildup_1pms =
        reqPorts->MCTLFC_TJATVG_MaxCrvGrdBuildup_1pms; /* '<Root>/Inport185' */
    MCTLFC_TJATVG_MaxCrvGrdRed_1pms =
        reqPorts->MCTLFC_TJATVG_MaxCrvGrdRed_1pms; /* '<Root>/Inport186' */
    MCTLFC_TJATVG_GrdLimitTgtCrvTGC_1pms =
        reqPorts->MCTLFC_TJATVG_GrdLimitTgtCrvTGC_1pms; /* '<Root>/Inport187' */
    MCTLFC_TJATVG_StrWhStifLimit_nu =
        reqPorts->MCTLFC_TJATVG_StrWhStifLimit_nu; /* '<Root>/Inport188' */
    MCTLFC_TJATVG_StrWhStifGrad_1ps =
        reqPorts->MCTLFC_TJATVG_StrWhStifGrad_1ps; /* '<Root>/Inport240' */
    MCTLFC_TJATVG_TrqRampGrad_1ps =
        reqPorts->MCTLFC_TJATVG_TrqRampGrad_1ps; /* '<Root>/Inport241' */
    MCTLFC_TJATVG_MaxTrqScaILimit_nu =
        reqPorts->MCTLFC_TJATVG_MaxTrqScaILimit_nu; /* '<Root>/Inport189' */
    MCTLFC_TJATVG_MaxTrqScalGrad_1ps =
        reqPorts->MCTLFC_TJATVG_MaxTrqScalGrad_1ps; /* '<Root>/Inport238' */
    MCTLFC_TJATVG_HighStatAccu_bool =
        reqPorts->MCTLFC_TJATVG_HighStatAccu_bool; /* '<Root>/Inport239' */
    MCTLFC_TJATVG_LimiterActivated_bool =
        reqPorts->MCTLFC_TJATVG_LimiterActivated_bool; /* '<Root>/Inport190' */
    MCTLFC_TJATVG_LimiterTimeDuration_sec =
        reqPorts
            ->MCTLFC_TJATVG_LimiterTimeDuration_sec; /* '<Root>/Inport191' */
    MCTLFC_TJATVG_MaxJerkAllowed_mps3 =
        reqPorts->MCTLFC_TJATVG_MaxJerkAllowed_mps3; /* '<Root>/Inport260' */
    MCTLFC_TJATVG_DeratingLevel_nu =
        reqPorts->MCTLFC_TJATVG_DeratingLevel_nu; /* '<Root>/Inport264' */
    MCTLFC_SysStateALCA_nu =
        reqPorts->MCTLFC_SysStateALCA_nu; /* '<Root>/Inport192' */
    MCTLFC_LCRTTG_LeCridrBndPosX0_met =
        reqPorts->MCTLFC_LCRTTG_LeCridrBndPosX0_met; /* '<Root>/Inport193' */
    MCTLFC_LCRTTG_LeCridrBndPosY0_met =
        reqPorts->MCTLFC_LCRTTG_LeCridrBndPosY0_met; /* '<Root>/Inport194' */
    MCTLFC_LCRTTG_LeCridrBndHeadAng_rad =
        reqPorts->MCTLFC_LCRTTG_LeCridrBndHeadAng_rad; /* '<Root>/Inport195' */
    MCTLFC_LCRTTG_LeCridrBndCrv_1pm =
        reqPorts->MCTLFC_LCRTTG_LeCridrBndCrv_1pm; /* '<Root>/Inport196' */
    MCTLFC_LCRTTG_LeCridrBndCrvChng_1pm2 =
        reqPorts->MCTLFC_LCRTTG_LeCridrBndCrvChng_1pm2; /* '<Root>/Inport197' */
    MCTLFC_LCRTTG_LeCridrBndLength_met =
        reqPorts->MCTLFC_LCRTTG_LeCridrBndLength_met; /* '<Root>/Inport198' */
    MCTLFC_LCRTTG_RiCridrBndPosX0_met =
        reqPorts->MCTLFC_LCRTTG_RiCridrBndPosX0_met; /* '<Root>/Inport199' */
    MCTLFC_LCRTTG_RiCridrBndPosY0_met =
        reqPorts->MCTLFC_LCRTTG_RiCridrBndPosY0_met; /* '<Root>/Inport200' */
    MCTLFC_LCRTTG_RiCridrBndHeadAng_rad =
        reqPorts->MCTLFC_LCRTTG_RiCridrBndHeadAng_rad; /* '<Root>/Inport201' */
    MCTLFC_LCRTTG_RiCridrBndCrv_1pm =
        reqPorts->MCTLFC_LCRTTG_RiCridrBndCrv_1pm; /* '<Root>/Inport202' */
    MCTLFC_LCRTTG_RiCridrBndCrvChng_1pm2 =
        reqPorts->MCTLFC_LCRTTG_RiCridrBndCrvChng_1pm2; /* '<Root>/Inport203' */
    MCTLFC_LCRTTG_RiCridrBndLength_met =
        reqPorts->MCTLFC_LCRTTG_RiCridrBndLength_met; /* '<Root>/Inport204' */
    MCTLFC_LCRTTG_TgtTrajPosX0_met =
        reqPorts->MCTLFC_LCRTTG_TgtTrajPosX0_met; /* '<Root>/Inport205' */
    MCTLFC_LCRTTG_TgtTrajPosY0_met =
        reqPorts->MCTLFC_LCRTTG_TgtTrajPosY0_met; /* '<Root>/Inport206' */
    MCTLFC_LCRTTG_TgtTrajHeadAng_rad =
        reqPorts->MCTLFC_LCRTTG_TgtTrajHeadAng_rad; /* '<Root>/Inport207' */
    MCTLFC_LCRTTG_TgtTrajCrv_1pm =
        reqPorts->MCTLFC_LCRTTG_TgtTrajCrv_1pm; /* '<Root>/Inport208' */
    MCTLFC_LCRTTG_TgtTrajCrvChng_1pm2 =
        reqPorts->MCTLFC_LCRTTG_TgtTrajCrvChng_1pm2; /* '<Root>/Inport209' */
    MCTLFC_LCRTTG_TgtTrajLength_met =
        reqPorts->MCTLFC_LCRTTG_TgtTrajLength_met; /* '<Root>/Inport210' */
    MCTLFC_LCRTVG_TrajPlanServQu_nu =
        reqPorts->MCTLFC_LCRTVG_TrajPlanServQu_nu; /* '<Root>/Inport211' */
    MCTLFC_LCRTVG_WeightTgtDistY_nu =
        reqPorts->MCTLFC_LCRTVG_WeightTgtDistY_nu; /* '<Root>/Inport212' */
    MCTLFC_LCRTVG_WeightEndTime_nu =
        reqPorts->MCTLFC_LCRTVG_WeightEndTime_nu; /* '<Root>/Inport213' */
    MCTLFC_LCRTVG_DistYToLeTgtArea_met =
        reqPorts->MCTLFC_LCRTVG_DistYToLeTgtArea_met; /* '<Root>/Inport214' */
    MCTLFC_LCRTVG_DistYToRiTgtArea_met =
        reqPorts->MCTLFC_LCRTVG_DistYToRiTgtArea_met; /* '<Root>/Inport215' */
    MCTLFC_LCRTVG_FTireAclMax_mps2 =
        reqPorts->MCTLFC_LCRTVG_FTireAclMax_mps2; /* '<Root>/Inport216' */
    MCTLFC_LCRTVG_FTireAclMin_mps2 =
        reqPorts->MCTLFC_LCRTVG_FTireAclMin_mps2; /* '<Root>/Inport217' */
    MCTLFC_LCRTVG_TrajGuiQu_nu =
        reqPorts->MCTLFC_LCRTVG_TrajGuiQu_nu; /* '<Root>/Inport218' */
    MCTLFC_LCRTVG_TriggerReplan_bool =
        reqPorts->MCTLFC_LCRTVG_TriggerReplan_bool; /* '<Root>/Inport219' */
    MCTLFC_LCRTVG_PredTimeHeadAng_sec =
        reqPorts->MCTLFC_LCRTVG_PredTimeHeadAng_sec; /* '<Root>/Inport220' */
    MCTLFC_LCRTVG_PredTimeCrv_sec =
        reqPorts->MCTLFC_LCRTVG_PredTimeCrv_sec; /* '<Root>/Inport221' */
    MCTLFC_LCRTVG_PlanningHorzion_sec =
        reqPorts->MCTLFC_LCRTVG_PlanningHorzion_sec; /* '<Root>/Inport222' */
    MCTLFC_LCRTVG_ObstacleVelX_mps =
        reqPorts->MCTLFC_LCRTVG_ObstacleVelX_mps; /* '<Root>/Inport223' */
    MCTLFC_LCRTVG_ObstacleAclX_mps2 =
        reqPorts->MCTLFC_LCRTVG_ObstacleAclX_mps2; /* '<Root>/Inport224' */
    MCTLFC_LCRTVG_ObstacleWidth_met =
        reqPorts->MCTLFC_LCRTVG_ObstacleWidth_met; /* '<Root>/Inport225' */
    MCTLFC_LCRTVG_ObstacleDistX_met =
        reqPorts->MCTLFC_LCRTVG_ObstacleDistX_met; /* '<Root>/Inport226' */
    MCTLFC_LCRTVG_ObstacleDistY_met =
        reqPorts->MCTLFC_LCRTVG_ObstacleDistY_met; /* '<Root>/Inport227' */
    MCTLFC_LCRTVG_LtcyCompActivated_bool =
        reqPorts->MCTLFC_LCRTVG_LtcyCompActivated_bool; /* '<Root>/Inport228' */
    MCTLFC_LCRTVG_SensorTStamp_sec =
        reqPorts->MCTLFC_LCRTVG_SensorTStamp_sec; /* '<Root>/Inport229' */
    MCTLFC_LCRTVG_MaxCrvTrajGuiCtrl_1pm =
        reqPorts->MCTLFC_LCRTVG_MaxCrvTrajGuiCtrl_1pm; /* '<Root>/Inport230' */
    MCTLFC_LCRTVG_MaxCrvGrdBuildup_1pms =
        reqPorts->MCTLFC_LCRTVG_MaxCrvGrdBuildup_1pms; /* '<Root>/Inport231' */
    MCTLFC_LCRTVG_MaxCrvGrdRed_1pms =
        reqPorts->MCTLFC_LCRTVG_MaxCrvGrdRed_1pms; /* '<Root>/Inport232' */
    MCTLFC_LCRTVG_GrdLimitTgtCrvTGC_1pms =
        reqPorts->MCTLFC_LCRTVG_GrdLimitTgtCrvTGC_1pms; /* '<Root>/Inport233' */
    MCTLFC_LCRTVG_StrWhStifLimit_nu =
        reqPorts->MCTLFC_LCRTVG_StrWhStifLimit_nu; /* '<Root>/Inport234' */
    MCTLFC_LCRTVG_StrWhStifGrad_1ps =
        reqPorts->MCTLFC_LCRTVG_StrWhStifGrad_1ps; /* '<Root>/Inport256' */
    MCTLFC_LCRTVG_TrqRampGrad_1ps =
        reqPorts->MCTLFC_LCRTVG_TrqRampGrad_1ps; /* '<Root>/Inport257' */
    MCTLFC_LCRTVG_MaxTrqScaILimit_nu =
        reqPorts->MCTLFC_LCRTVG_MaxTrqScaILimit_nu; /* '<Root>/Inport235' */
    MCTLFC_LCRTVG_MaxTrqScalGrad_1ps =
        reqPorts->MCTLFC_LCRTVG_MaxTrqScalGrad_1ps; /* '<Root>/Inport254' */
    MCTLFC_LCRTVG_HighStatAccu_bool =
        reqPorts->MCTLFC_LCRTVG_HighStatAccu_bool; /* '<Root>/Inport255' */
    MCTLFC_LCRTVG_LimiterActivated_bool =
        reqPorts->MCTLFC_LCRTVG_LimiterActivated_bool; /* '<Root>/Inport236' */
    MCTLFC_LCRTVG_LimiterTimeDuration_sec =
        reqPorts
            ->MCTLFC_LCRTVG_LimiterTimeDuration_sec; /* '<Root>/Inport237' */
    MCTLFC_LCRTVG_MaxJerkAllowed_mps3 =
        reqPorts->MCTLFC_LCRTVG_MaxJerkAllowed_mps3; /* '<Root>/Inport261' */
    MCTLFC_LCRTVG_DeratingLevel_nu =
        reqPorts->MCTLFC_LCRTVG_DeratingLevel_nu; /* '<Root>/Inport265' */
}

/*****************************************************************************
  Functionname: MCTLFC_Output_initialize                                  */ /*!

      @brief: write inner result to MCTLFC proPorts

      @description: write inner result to MCTLFC proPorts

      @param[in]:proPorts   MCTLFC output

      @return:void
    *****************************************************************************/
void MCTLFC_Output_initialize(sMCTLFCOut_st* proPorts,
                              sMCTLFCDebug_st* debugInfo) {
    debugInfo->MCTLFC_t_prio_ALCA = MCTLFC_t_prio_ALCA;   /* '<S5>/Selector' */
    debugInfo->MCTLFC_t_prio_AOLC = MCTLFC_t_prio_AOLC;   /* '<S5>/Selector1' */
    debugInfo->MCTLFC_t_prio_ESA = MCTLFC_t_prio_ESA;     /* '<S5>/Selector2' */
    debugInfo->MCTLFC_t_prio_LDP = MCTLFC_t_prio_LDP;     /* '<S5>/Selector3' */
    debugInfo->MCTLFC_t_prio_LDPOC = MCTLFC_t_prio_LDPOC; /* '<S5>/Selector4' */
    debugInfo->MCTLFC_t_prio_RDP = MCTLFC_t_prio_RDP;     /* '<S5>/Selector5' */
    debugInfo->MCTLFC_t_prio_TJA = MCTLFC_t_prio_TJA;     /* '<S5>/Selector6' */
    debugInfo->MCTLFC_t_prio_max = MCTLFC_t_prio_max;     /* '<S5>/Max' */

    proPorts->CSCLTA_LeCridrBndPosX0_met =
        CSCLTA_LeCridrBndPosX0_met; /* '<S7>/Multiport Switch1'
                                     * S_CSCLTA_LeCridrBndPosX0_met, float32,
                                     * the PosX0 value of left corridor bound,
                                     * [-300, 300]
                                     */
    proPorts->CSCLTA_LeCridrBndPosY0_met =
        CSCLTA_LeCridrBndPosY0_met; /* '<S7>/Multiport Switch1'
                                     * S_CSCLTA_LeCridrBndPosY0_met, float32,
                                     * the PosY0 value of left corridor bound,
                                     * [-15, 15]
                                     */
    proPorts->CSCLTA_LeCridrHeadAng_rad =
        CSCLTA_LeCridrHeadAng_rad; /* '<S7>/Multiport Switch1'
                                    * S_CSCLTA_LeCridrBndHeadAng_rad, float32,
                                    * the heading angle value of left corridor
                                    * bound,
                                    * [-0.78539816, 0.78539816]
                                    */
    proPorts->CSCLTA_LeCridrBndCrv_1pm =
        CSCLTA_LeCridrBndCrv_1pm; /* '<S7>/Multiport Switch1'
                                   * S_CSCLTA_LeCridrBndCrv_1pm, float32, the
                                   * curve value of left corridor bound, [-0.1,
                                   * 0.1]
                                   */
    proPorts->CSCLTA_LeCridrBndCrvChng_1pm2 =
        CSCLTA_LeCridrBndCrvChng_1pm2; /* '<S7>/Multiport Switch1'
                                        * S_CSCLTA_LeCridrBndCrvChng_1pm2, the
                                        * curve deviation value of left corridor
                                        * bound, [-0.001, 0.001]
                                        */
    proPorts->CSCLTA_LeCridrBndLength_met =
        CSCLTA_LeCridrBndLength_met; /* '<S7>/Multiport Switch1'
                                      * S_CSCLTA_LeCridrBndLength_met, the
                                      * length value of left corridor bound, [0,
                                      * 150]
                                      */
    proPorts->CSCLTA_RiCridrBndPosX0_met =
        CSCLTA_RiCridrBndPosX0_met; /* '<S7>/Multiport Switch1'
                                     * S_CSCLTA_RiCridrBndPosX0_met, float32,
                                     * the PosX0 value of right corridor bound,
                                     * [-300, 300]
                                     */
    proPorts->CSCLTA_RiCridrBndPosY0_met =
        CSCLTA_RiCridrBndPosY0_met; /* '<S7>/Multiport Switch1'
                                     * S_CSCLTA_RiCridrBndPosY0_met, float32,
                                     * the PosY0 value of right corridor bound,
                                     * [-15, 15]
                                     */
    proPorts->CSCLTA_RiCridrHeadAng_rad =
        CSCLTA_RiCridrHeadAng_rad; /* '<S7>/Multiport Switch1'
                                    * S_CSCLTA_RiCridrBndHeadAng_rad, float32,
                                    * the heading angle value of right corridor
                                    * bound, [-0.78539816, 0.78539816]
                                    */
    proPorts->CSCLTA_RiCridrBndCrv_1pm =
        CSCLTA_RiCridrBndCrv_1pm; /* '<S7>/Multiport Switch1'
                                   * S_CSCLTA_RiCridrBndCrv_1pm, float32, the
                                   * curve value of right corridor bound, [-0.1,
                                   * 0.1]
                                   */
    proPorts->CSCLTA_RiCridrBndCrvChng_1pm2 =
        CSCLTA_RiCridrBndCrvChng_1pm2; /* '<S7>/Multiport Switch1'
                                        * S_CSCLTA_RiCridrBndCrvChng_1pm2, the
                                        * curve deviation value of right
                                        * corridor bound, [-0.001, 0.001]
                                        */
    proPorts->CSCLTA_RiCridrBndLength_met =
        CSCLTA_RiCridrBndLength_met; /* '<S7>/Multiport Switch1'
                                      * S_CSCLTA_RiCridrBndLength_met, the
                                      * length value of right corridor bound,
                                      * [0, 150]
                                      */
    proPorts->CSCLTA_TgtTrajPosX0_met =
        CSCLTA_TgtTrajPosX0_met; /* '<S7>/Multiport Switch1'
                                  * S_CSCLTA_TgtTrajPosX0_met, float32, the
                                  * PosX0 value of target corridor bound, [-300,
                                  * 300]
                                  */
    proPorts->CSCLTA_TgtTrajPosY0_met =
        CSCLTA_TgtTrajPosY0_met; /* '<S7>/Multiport Switch1'
                                  * S_CSCLTA_TgtTrajPosY0_met, float32, the
                                  * PosY0 value of target corridor bound, [-15,
                                  * 15]
                                  */
    proPorts->CSCLTA_TgtTrajHeadAng_rad =
        CSCLTA_TgtTrajHeadAng_rad; /* '<S7>/Multiport Switch1'
                                    * S_CSCLTA_TgtTrajHeadAng_rad, float32, the
                                    * heading angle value of target corridor
                                    * bound, [-0.78539816, 0.78539816]
                                    */
    proPorts->CSCLTA_TgtTrajCrv_1pm =
        CSCLTA_TgtTrajCrv_1pm; /* '<S7>/Multiport Switch1'
                                * S_CSCLTA_TgtTrajCrv_1pm, float32, the curve
                                * value of target corridor bound, [-0.1, 0.1]
                                */
    proPorts->CSCLTA_TgtTrajCrvChng_1pm2 =
        CSCLTA_TgtTrajCrvChng_1pm2; /* '<S7>/Multiport Switch1'
                                     * S_CSCLTA_TgtTrajCrvChng_1pm2, the curve
                                     * deviation value of target corridor bound,
                                     * [-0.001, 0.001]
                                     */
    proPorts->CSCLTA_TgtTrajLength_met =
        CSCLTA_TgtTrajLength_met; /* '<S7>/Multiport Switch1'
                                   * S_CSCLTA_TgtTrajLength_met, the length
                                   * value of target corridor bound, [0, 150]
                                   */
    proPorts->CSCLTA_WeightTgtDistY_nu =
        CSCLTA_WeightTgtDistY_nu; /* '<S7>/Multiport Switch1'
                                   * S_CSCLTA_WeightTgtDistY_nu, float32, The
                                   * importance factor of the lateral deviation,
                                   * [0,1]
                                   */
    proPorts->CSCLTA_WeightEndTime_nu =
        CSCLTA_WeightEndTime_nu; /* '<S7>/Multiport Switch1'
                                  * S_CSCLTA_WeightEndTime_nu, float32, The
                                  * importance factor of the time required for
                                  * the planned trajectory, [0,1]
                                  */
    proPorts->CSCLTA_DistYToILeTgtArea_met =
        CSCLTA_DistYToILeTgtArea_met; /* '<S7>/Multiport Switch1'
                                       * S_CSCLTA_DistYTolLeTgtArea_met,
                                       * float32, lateral tolerance left
                                       * boundary value , [0,10]
                                       */
    proPorts->CSCLTA_DistYToIRiTgtArea_met =
        CSCLTA_DistYToIRiTgtArea_met; /* '<S7>/Multiport Switch1'
                                       * S_CSCLTA_DistYTolRiTgtArea_met,
                                       * float32, lateral tolerance right
                                       * boundary value , [0,10]
                                       */
    proPorts->CSCLTA_FTireAclMax_mps2 =
        CSCLTA_FTireAclMax_mps2; /* '<S7>/Multiport Switch1'
                                  * S_CSCLTA_FTireAclMax_mps2, float32, lateral
                                  * acceleration upper limiting value, [-20,20]
                                  */
    proPorts->CSCLTA_FTireAclMin_mps2 =
        CSCLTA_FTireAclMin_mps2; /* '<S7>/Multiport Switch1'
                                  * S_CSCLTA_FTireAclMin_mps2, float32, lateral
                                  * acceleration lower limiting value, [-20,20]
                                  */
    proPorts->CSCLTA_PredTimeHeadAng_sec =
        CSCLTA_PredTimeHeadAng_sec; /* '<S7>/Multiport Switch1'
                                     * S_CSCLTA_PredTimeHeadAng_sec, float32，
                                     * todo, [0，60]
                                     */
    proPorts->CSCLTA_PredTimeCrv_sec =
        CSCLTA_PredTimeCrv_sec; /* '<S7>/Multiport Switch1'
                                 * S_CSCLTA_PredictionTimeCrv_sec,float32,
                                 * todo,[0，60]
                                 */
    proPorts->CSCLTA_PlanningHorzion_sec =
        CSCLTA_PlanningHorzion_sec; /* '<S7>/Multiport Switch1'
                                     * S_CSCLTA_PlanningHorizon_sec, float32,
                                     * max Planning horizon(time) of the
                                     * trajectory, [0，60]
                                     */
    proPorts->CSCLTA_ObstacleVelX_mps =
        CSCLTA_ObstacleVelX_mps; /* '<S7>/Multiport Switch1'
                                  * S_CSCLTA_ObstacleVelX_mps, float32, the
                                  * obstacle velocity X in target trajectory if
                                  * it is existed ,
                                  * [-20,150]
                                  */
    proPorts->CSCLTA_ObstacleAclX_mps2 =
        CSCLTA_ObstacleAclX_mps2; /* '<S7>/Multiport Switch1'
                                   * S_CSCLTA_ObstacleAclX_mps2, float32, the
                                   * obstacle accel X in target trajectory if it
                                   * is existed ,
                                   * [-20,20]
                                   */
    proPorts->CSCLTA_ObstacleWidth_met =
        CSCLTA_ObstacleWidth_met; /* '<S7>/Multiport Switch1'
                                   * S_CSCLTA_ObstacleWidth_met, float32, the
                                   * obstacle width in target trajectory if it
                                   * is existed , [0,150]
                                   */
    proPorts->CSCLTA_ObstacleDistX_met =
        CSCLTA_ObstacleDistX_met; /* '<S7>/Multiport Switch1'
                                   * S_CSCLTA_ObstacleDistX_met, float32, the
                                   * obstacle distance X in target trajectory if
                                   * it is existed ,
                                   * [-1000,1000]
                                   */
    proPorts->CSCLTA_ObstacleDistY_met =
        CSCLTA_ObstacleDistY_met; /* '<S7>/Multiport Switch1'
                                   * S_CSCLTA_ObstacleDistY_met, float32, the
                                   * obstacle distance X in target trajectory if
                                   * it is existed ,
                                   * [-1000,1000]
                                   */
    proPorts->CSCLTA_SensorTStamp_sec =
        CSCLTA_SensorTStamp_sec; /* '<S7>/Multiport Switch1'
                                  * S_CSCLTA_SensorTStamp_sec, float32, time
                                  * stamp of the camera signal from camera
                                  * sensor,[0,4295]
                                  */
    proPorts->CSCLTA_MaxCrvTrajGuiCtrl_1pm =
        CSCLTA_MaxCrvTrajGuiCtrl_1pm; /* '<S7>/Multiport Switch1' */
    proPorts->CSCLTA_MaxCrvGrdBuildup_1pms =
        CSCLTA_MaxCrvGrdBuildup_1pms; /* '<S7>/Multiport Switch1' */
    proPorts->CSCLTA_MaxCrvGrdRed_1pms =
        CSCLTA_MaxCrvGrdRed_1pms; /* '<S7>/Multiport Switch1' */
    proPorts->CSCLTA_GrdLimitTgtCrvGC_1pms =
        CSCLTA_GrdLimitTgtCrvGC_1pms; /* '<S7>/Multiport Switch1' */
    proPorts->CSCLTA_StrWhStifLimit_nu =
        CSCLTA_StrWhStifLimit_nu; /* '<S7>/Multiport Switch1' */
    proPorts->CSCLTA_StrWhStifGrad_1ps =
        CSCLTA_StrWhStifGrad_1ps; /* '<S7>/Multiport Switch1' */
    proPorts->CSCLTA_TrqRampGrad_1ps =
        CSCLTA_TrqRampGrad_1ps; /* '<S7>/Multiport Switch1' */
    proPorts->CSCLTA_MaxTrqScalLimit_nu =
        CSCLTA_MaxTrqScalLimit_nu; /* '<S7>/Multiport Switch1' */
    proPorts->CSCLTA_MaxTrqScalGrad_nu =
        CSCLTA_MaxTrqScalGrad_nu; /* '<S7>/Multiport Switch1' */
    proPorts->CSCLTA_LimiterDuration_sec =
        CSCLTA_LimiterDuration_sec; /* '<S7>/Multiport Switch1' */
    proPorts->CSCLTA_MaxJerkAllowed_mps3 =
        CSCLTA_MaxJerkAllowed_mps3; /* '<S7>/Multiport Switch1'
                                     * S_CSCLTA_MaxJerkAllowed_mps3, float32,
                                     * Maximum Jerk Allowed in the trajectory
                                     * planning, [0,50]
                                     */
    proPorts->CSCLTA_DeratingLevel_nu =
        CSCLTA_DeratingLevel_nu; /* '<S7>/Multiport Switch1' */
    proPorts->CSCLTA_ControllingFunction_nu =
        CSCLTA_ControllingFunction_nu; /* '<S5>/Switch' */
    proPorts->CSCLTA_SysStateLCF =
        CSCLTA_SysStateLCF; /* '<S7>/Multiport Switch1'
                             * S_CSCLTA_SysStateLCF_enum,
                             * uint8, lateral control function system state enum
                             * value, [0,6],E_LCF_SYSSTATE_NOTPRESENT = 0;
                             * E_LCF_SYSSTATE_DISABLED = 1;
                             * E_LCF_SYSSTATE_PASSIVE = 2;
                             * E_LCF_SYSSTATE_REQUEST = 3;
                             * E_LCF_SYSSTATE_CONTROLLING = 4;
                             * E_LCF_SYSSTATE_RAMPOUT = 5; E_LCF_SYSSTATE_ERROR
                             * = 6;
                             */
    proPorts->CSCLTA_TgtPlanServQu_nu =
        CSCLTA_TgtPlanServQu_nu; /* '<S7>/Multiport Switch1'
                                  * S_CSCLTA_TrajPlanServQu_nu,
                                  * uint8, todo,
                                  * [0,255],P_TJATVG_TrajPlanValServQu_nu
                                  * =
                                  * +/ 0x00,P_TJATVG_TrajPlanValSrvQuSALC_nu
                                  * = +/ 0x10,
                                  */
    proPorts->CSCLTA_TrajGuiQualifier_nu =
        CSCLTA_TrajGuiQualifier_nu; /* '<S7>/Multiport Switch1'
                                              * S_CSCLTA_TrajGuiQualifier_nu,
                                       uint8, qualifier value of trajectory
                                       guidence, The qualifier indicates if/how
                                       the target curvature should be
                                       considered, [0,5] .E_LCF_TGQ_REQ_OFF= 0,
                                       E_LCF_TGQ_REQ	= 1,
                                       E_LCF_TGQ_REQ_FREEZE= 3,
                                       E_LCF_TGQ_REQ_FFC	= 4,
                                       E_LCF_TGQ_REQ_REFCHNG= 5

                                              */
    proPorts->CSCLTA_TriggerReplan_nu =
        CSCLTA_TriggerReplan_nu; /* '<S7>/Multiport Switch1'
                                  * S_CSCLTA_TriggerReplan_nu,uint8,trigger
                                  * replan signal from CSCLTA module,[0，1]
                                  */
    proPorts->CSCLTA_LatencyCompActivated_bool =
        CSCLTA_LatencyCompActivated_bool; /* '<S7>/Multiport Switch1'
                                           * S_CSCLTA_LatencyCompActivated_nu,
                                           * uint8, the trigger flag for latency
                                           * compensation function, [0,1] 1:
                                           * latency compensation enable, 0:
                                           * latency compensation disable
                                           */
    proPorts->CSCLTA_HighStatAccu_bool =
        CSCLTA_HighStatAccu_bool; /* '<S7>/Multiport Switch1' */
    proPorts->CSCLTA_LimiterActivated_nu =
        CSCLTA_LimiterActivated_nu; /* '<S7>/Multiport Switch1' */
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */