#include "rtwtypes.h"
#ifndef MCTLFC_EXT_H
#define MCTLFC_EXT_H

typedef struct {
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
    uint8_T MCTLFC_DPLTVG_DeratingLevel_nu;         /* '<Root>/Inport262' */
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
    uint8_T MCTLFC_DPRTVG_DeratingLevel_nu;         /* '<Root>/Inport267' */
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
    uint8_T MCTLFC_DPOTVG_DeratingLevel_nu;         /* '<Root>/Inport263' */
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
    uint8_T MCTLFC_TJATVG_DeratingLevel_nu;         /* '<Root>/Inport264' */
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
    uint8_T MCTLFC_LCRTVG_DeratingLevel_nu;         /* '<Root>/Inport265' */
} sMCTLFCInReq_st;

typedef struct { uint8_T tmp; } sMCTLFCParam_st;

typedef struct {
    real32_T
        CSCLTA_LeCridrBndPosX0_met; /* '<S7>/Multiport Switch1'
                                                               * S_CSCLTA_LeCridrBndPosX0_met,
                                     * float32, the PosX0 value of left corridor
                                     * bound, [-300, 300]
                                                               */
    real32_T
        CSCLTA_LeCridrBndPosY0_met; /* '<S7>/Multiport Switch1'
                                                                       * S_CSCLTA_LeCridrBndPosY0_met,
                                     * float32, the PosY0 value of left corridor
                                     * bound, [-15, 15]
                                                                       */
    real32_T
        CSCLTA_LeCridrHeadAng_rad; /* '<S7>/Multiport Switch1'
                                                                  * S_CSCLTA_LeCridrBndHeadAng_rad,
                                    * float32, the heading angle value of left
                                    * corridor bound, [-0.78539816, 0.78539816]
                                                                  */
    real32_T
        CSCLTA_LeCridrBndCrv_1pm; /* '<S7>/Multiport Switch1'
                                                                 * S_CSCLTA_LeCridrBndCrv_1pm,
                                   * float32, the curve value of left corridor
                                   * bound, [-0.1, 0.1]
                                                                 */
    real32_T
        CSCLTA_LeCridrBndCrvChng_1pm2; /* '<S7>/Multiport Switch1'
                                                                          * S_CSCLTA_LeCridrBndCrvChng_1pm2,
                                        * the curve deviation value of left
                                        * corridor bound, [-0.001, 0.001]
                                                                          */
    real32_T
        CSCLTA_LeCridrBndLength_met; /* '<S7>/Multiport Switch1'
                                                                        * S_CSCLTA_LeCridrBndLength_met,
                                      * the length value of left corridor bound,
                                      * [0, 150]
                                                                        */
    real32_T
        CSCLTA_RiCridrBndPosX0_met; /* '<S7>/Multiport Switch1'
                                                                       * S_CSCLTA_RiCridrBndPosX0_met,
                                     * float32, the PosX0 value of right
                                     * corridor bound, [-300, 300]
                                                                       */
    real32_T
        CSCLTA_RiCridrBndPosY0_met; /* '<S7>/Multiport Switch1'
                                                                       * S_CSCLTA_RiCridrBndPosY0_met,
                                     * float32, the PosY0 value of right
                                     * corridor bound, [-15, 15]
                                                                       */
    real32_T
        CSCLTA_RiCridrHeadAng_rad; /* '<S7>/Multiport Switch1'
                                                                  * S_CSCLTA_RiCridrBndHeadAng_rad,
                                    * float32, the heading angle value of right
                                    * corridor bound, [-0.78539816, 0.78539816]
                                                                  */
    real32_T
        CSCLTA_RiCridrBndCrv_1pm; /* '<S7>/Multiport Switch1'
                                                                 * S_CSCLTA_RiCridrBndCrv_1pm,
                                   * float32, the curve value of right corridor
                                   * bound, [-0.1, 0.1]
                                                                 */
    real32_T
        CSCLTA_RiCridrBndCrvChng_1pm2; /* '<S7>/Multiport Switch1'
                                                                          * S_CSCLTA_RiCridrBndCrvChng_1pm2,
                                        * the curve deviation value of right
                                        * corridor bound, [-0.001, 0.001]
                                                                          */
    real32_T
        CSCLTA_RiCridrBndLength_met; /* '<S7>/Multiport Switch1'
                                                                        * S_CSCLTA_RiCridrBndLength_met,
                                      * the length value of right corridor
                                      * bound, [0, 150]
                                                                        */
    real32_T
        CSCLTA_TgtTrajPosX0_met; /* '<S7>/Multiport Switch1'
                                                                * S_CSCLTA_TgtTrajPosX0_met,
                                  * float32, the PosX0 value of target corridor
                                  * bound, [-300, 300]
                                                                */
    real32_T
        CSCLTA_TgtTrajPosY0_met; /* '<S7>/Multiport Switch1'
                                                                * S_CSCLTA_TgtTrajPosY0_met,
                                  * float32, the PosY0 value of target corridor
                                  * bound, [-15, 15]
                                                                */
    real32_T
        CSCLTA_TgtTrajHeadAng_rad; /* '<S7>/Multiport Switch1'
                                                                  * S_CSCLTA_TgtTrajHeadAng_rad,
                                    * float32, the heading angle value of target
                                    * corridor bound, [-0.78539816, 0.78539816]
                                                                  */
    real32_T
        CSCLTA_TgtTrajCrv_1pm; /* '<S7>/Multiport Switch1'
                                                              * S_CSCLTA_TgtTrajCrv_1pm,
                                * float32, the curve value of target corridor
                                * bound, [-0.1, 0.1]
                                                              */
    real32_T
        CSCLTA_TgtTrajCrvChng_1pm2; /* '<S7>/Multiport Switch1'
                                                                       * S_CSCLTA_TgtTrajCrvChng_1pm2,
                                     * the curve deviation value of target
                                     * corridor bound, [-0.001, 0.001]
                                                                       */
    real32_T
        CSCLTA_TgtTrajLength_met; /* '<S7>/Multiport Switch1'
                                                                 * S_CSCLTA_TgtTrajLength_met,
                                   * the length value of target corridor bound,
                                   * [0, 150]
                                                                 */
    real32_T
        CSCLTA_WeightTgtDistY_nu; /* '<S7>/Multiport Switch1'
                                                                 * S_CSCLTA_WeightTgtDistY_nu,
                                   * float32, The importance factor of the
                                   * lateral deviation, [0,1]
                                                                 */
    real32_T
        CSCLTA_WeightEndTime_nu; /* '<S7>/Multiport Switch1'
                                                                * S_CSCLTA_WeightEndTime_nu,
                                  * float32, The importance factor of the time
                                  * required for the planned trajectory, [0,1]
                                                                */
    real32_T
        CSCLTA_DistYToILeTgtArea_met; /* '<S7>/Multiport Switch1'
                                                                         * S_CSCLTA_DistYTolLeTgtArea_met,
                                       * float32, lateral tolerance left
                                       * boundary value , [0,10]
                                                                         */
    real32_T
        CSCLTA_DistYToIRiTgtArea_met; /* '<S7>/Multiport Switch1'
                                                                         * S_CSCLTA_DistYTolRiTgtArea_met,
                                       * float32, lateral tolerance right
                                       * boundary value , [0,10]
                                                                         */
    real32_T
        CSCLTA_FTireAclMax_mps2; /* '<S7>/Multiport Switch1'
                                                                * S_CSCLTA_FTireAclMax_mps2,
                                  * float32, lateral acceleration upper limiting
                                  * value, [-20,20]
                                                                */
    real32_T
        CSCLTA_FTireAclMin_mps2; /* '<S7>/Multiport Switch1'
                                                                * S_CSCLTA_FTireAclMin_mps2,
                                  * float32, lateral acceleration lower limiting
                                  * value, [-20,20]
                                                                */
    real32_T
        CSCLTA_PredTimeHeadAng_sec; /* '<S7>/Multiport Switch1'
                                                                       * S_CSCLTA_PredTimeHeadAng_sec,
                                     * float32， todo, [0，60]
                                                                       */
    real32_T
        CSCLTA_PredTimeCrv_sec; /* '<S7>/Multiport Switch1'
                                                               * S_CSCLTA_PredictionTimeCrv_sec,float32,
                                 * todo,[0，60]
                                                               */
    real32_T
        CSCLTA_PlanningHorzion_sec; /* '<S7>/Multiport Switch1'
                                                                       * S_CSCLTA_PlanningHorizon_sec,
                                     * float32, max Planning horizon(time) of
                                     * the trajectory, [0，60]
                                                                       */
    real32_T
        CSCLTA_ObstacleVelX_mps; /* '<S7>/Multiport Switch1'
                                                                * S_CSCLTA_ObstacleVelX_mps,
                                  * float32, the obstacle velocity X in target
                                  * trajectory if it is existed , [-20,150]
                                                                */
    real32_T
        CSCLTA_ObstacleAclX_mps2; /* '<S7>/Multiport Switch1'
                                                                 * S_CSCLTA_ObstacleAclX_mps2,
                                   * float32, the obstacle accel X in target
                                   * trajectory if it is existed , [-20,20]
                                                                 */
    real32_T
        CSCLTA_ObstacleWidth_met; /* '<S7>/Multiport Switch1'
                                                                 * S_CSCLTA_ObstacleWidth_met,
                                   * float32, the obstacle width in target
                                   * trajectory if it is existed , [0,150]
                                                                 */
    real32_T
        CSCLTA_ObstacleDistX_met; /* '<S7>/Multiport Switch1'
                                                                 * S_CSCLTA_ObstacleDistX_met,
                                   * float32, the obstacle distance X in target
                                   * trajectory if it is existed , [-1000,1000]
                                                                 */
    real32_T
        CSCLTA_ObstacleDistY_met; /* '<S7>/Multiport Switch1'
                                                                 * S_CSCLTA_ObstacleDistY_met,
                                   * float32, the obstacle distance X in target
                                   * trajectory if it is existed , [-1000,1000]
                                                                 */
    real32_T
        CSCLTA_SensorTStamp_sec;           /* '<S7>/Multiport Switch1'
                                                                          * S_CSCLTA_SensorTStamp_sec,
                                            * float32, time stamp of the camera signal
                                            * from camera sensor,[0,4295]
                                                                          */
    real32_T CSCLTA_MaxCrvTrajGuiCtrl_1pm; /* '<S7>/Multiport Switch1' */
    real32_T CSCLTA_MaxCrvGrdBuildup_1pms; /* '<S7>/Multiport Switch1' */
    real32_T CSCLTA_MaxCrvGrdRed_1pms;     /* '<S7>/Multiport Switch1' */
    real32_T CSCLTA_GrdLimitTgtCrvGC_1pms; /* '<S7>/Multiport Switch1' */
    real32_T CSCLTA_StrWhStifLimit_nu;     /* '<S7>/Multiport Switch1' */
    real32_T CSCLTA_StrWhStifGrad_1ps;     /* '<S7>/Multiport Switch1' */
    real32_T CSCLTA_TrqRampGrad_1ps;       /* '<S7>/Multiport Switch1' */
    real32_T CSCLTA_MaxTrqScalLimit_nu;    /* '<S7>/Multiport Switch1' */
    real32_T CSCLTA_MaxTrqScalGrad_nu;     /* '<S7>/Multiport Switch1' */
    real32_T CSCLTA_LimiterDuration_sec;   /* '<S7>/Multiport Switch1' */
    real32_T
        CSCLTA_MaxJerkAllowed_mps3;        /* '<S7>/Multiport Switch1'
                                                                              * S_CSCLTA_MaxJerkAllowed_mps3,
                                            * float32, Maximum Jerk Allowed in the
                                            * trajectory planning, [0,50]
                                                                              */
    uint8_T CSCLTA_DeratingLevel_nu;       /* '<S7>/Multiport Switch1' */
    uint8_T CSCLTA_ControllingFunction_nu; /* '<S5>/Switch' */
    uint8_T
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
    uint8_T
        CSCLTA_TgtPlanServQu_nu;        /* '<S7>/Multiport Switch1'
                                                                       * S_CSCLTA_TrajPlanServQu_nu,
                                         * uint8, todo,
                                         * [0,255],P_TJATVG_TrajPlanValServQu_nu
                                         * =
                                         * +/ 0x00,P_TJATVG_TrajPlanValSrvQuSALC_nu
                                         * = +/ 0x10,
                                                                       */
    uint8_T CSCLTA_TrajGuiQualifier_nu; /* '<S7>/Multiport Switch1'
                                                                       *
                                           S_CSCLTA_TrajGuiQualifier_nu, uint8,
                                           qualifier value of trajectory
                                           guidence, The qualifier indicates
                                           if/how the target curvature should be
                                           considered, [0,5] .E_LCF_TGQ_REQ_OFF=
                                           0, E_LCF_TGQ_REQ	= 1,
                                           E_LCF_TGQ_REQ_FREEZE= 3,
                                           E_LCF_TGQ_REQ_FFC	= 4,
                                           E_LCF_TGQ_REQ_REFCHNG= 5

                                                                       */
    boolean_T
        CSCLTA_TriggerReplan_nu; /* '<S7>/Multiport Switch1'
                                                                * S_CSCLTA_TriggerReplan_nu,uint8,trigger
                                  * replan signal from CSCLTA module,[0，1]
                                                                */
    boolean_T
        CSCLTA_LatencyCompActivated_bool; /* '<S7>/Multiport Switch1'
                                                                                 * S_CSCLTA_LatencyCompActivated_nu, uint8, the trigger flag for latency compensation function, [0,1] 1: latency compensation enable, 0: latency compensation disable
                                                                                 */
    boolean_T CSCLTA_HighStatAccu_bool;   /* '<S7>/Multiport Switch1' */
    boolean_T CSCLTA_LimiterActivated_nu; /* '<S7>/Multiport Switch1' */
} sMCTLFCOut_st;

#ifndef Rte_TypeDef_sMCTLFCDebug_st
#define Rte_TypeDef_sMCTLFCDebug_st
typedef struct {
    uint32_T uiVersionNum_nu;  // uint32 value example
    /* Definition for custom storage class: Global */
    uint8_T MCTLFC_t_prio_ALCA;  /* '<S5>/Selector' */
    uint8_T MCTLFC_t_prio_AOLC;  /* '<S5>/Selector1' */
    uint8_T MCTLFC_t_prio_ESA;   /* '<S5>/Selector2' */
    uint8_T MCTLFC_t_prio_LDP;   /* '<S5>/Selector3' */
    uint8_T MCTLFC_t_prio_LDPOC; /* '<S5>/Selector4' */
    uint8_T MCTLFC_t_prio_RDP;   /* '<S5>/Selector5' */
    uint8_T MCTLFC_t_prio_TJA;   /* '<S5>/Selector6' */
    uint8_T MCTLFC_t_prio_max;   /* '<S5>/Max' */
} sMCTLFCDebug_st;
#endif
extern void MCTLFC_Exec(const sMCTLFCInReq_st *reqPorts,
                        const sMCTLFCParam_st *params,
                        sMCTLFCOut_st *proPorts,
                        sMCTLFCDebug_st *debugInfo);
extern void MCTLFC_Init(void);

#endif