#include "LCCRA_ext.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

#define ASW_QM_CORE2_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/* Imported  block signals */
real32_T LBP_LeLnClthPosY0_met;      /* '<Root>/Inport' */
real32_T LBP_LeLnClthHeading_rad;    /* '<Root>/Inport1' */
real32_T LBP_LeLnClthCrv_1pm;        /* '<Root>/Inport2' */
real32_T LBP_LeLnClthCrvChng_1pm2;   /* '<Root>/Inport3' */
boolean_T LBP_LeLnValid_bool;        /* '<Root>/Inport4' */
real32_T LBP_RiLnClthPosY0_met;      /* '<Root>/Inport5' */
real32_T LBP_RiLnClthHeading_rad;    /* '<Root>/Inport6' */
real32_T LBP_RiLnClthCrv_1pm;        /* '<Root>/Inport7' */
real32_T LBP_RiLnClthCrvChng_1pm2;   /* '<Root>/Inport8' */
boolean_T LBP_RiLnValid_bool;        /* '<Root>/Inport9' */
real32_T ALP_LeLnClthPosY0_met;      /* '<Root>/Inport10' */
real32_T ALP_LeLnClthHeading_rad;    /* '<Root>/Inport11' */
real32_T ALP_LeLnClthCrv_1pm;        /* '<Root>/Inport12' */
real32_T ALP_LeLnClthCrvChng_1pm2;   /* '<Root>/Inport13' */
boolean_T ALP_LeLnValid_bool;        /* '<Root>/Inport14' */
real32_T ALP_RiLnClthPosY0_met;      /* '<Root>/Inport15' */
real32_T ALP_RiLnClthHeading_rad;    /* '<Root>/Inport16' */
real32_T ALP_RiLnClthCrv_1pm;        /* '<Root>/Inport17' */
real32_T ALP_RiLnClthCrvChng_1pm2;   /* '<Root>/Inport18' */
boolean_T ALP_RiLnValid_bool;        /* '<Root>/Inport19' */
BusObject* Fusion_TargetObject_str;  /* '<Root>/Inport26' */
real32_T VED_EgoVelocity_mps;        /* '<Root>/Inport31' */
real32_T VED_EgoYawRate_rps;         /* '<Root>/Inport35' */
real32_T VED_EgoClthCrv_1pm;         /* '<Root>/Inport36' */
real32_T VED_EgoClthCrvChng_1pm2;    /* '<Root>/Inport37' */
real32_T ABPR_LnWidth_met;           /* '<Root>/Inport38' */
boolean_T ABPR_LnWidthValid_bool;    /* '<Root>/Inport22' */
real32_T LCFRCV_SysCycleTimeSen_sec; /* '<Root>/Inport20' */
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  Functionname:     LCCRA_Reset                                  */
/*!

                                      @brief:           LCCRA function reset

                                      @description:     All global variables
   related to LCCRA are reset in this function when LCCRA executes for the first
   time, or system exception needs to be reset

                                      @param[in]:       void

                                      @return:          void
                                *****************************************************************************/
void LCCRA_Reset(void) { LCCRA_initialize(); }

/*****************************************************************************
  Functionname:     LCCRA_Exec                                  */
/*!

                                        @brief:           Execution entry of
   LCCRA function

                                        @description:     LCCRA main function

                                        @param[in]:       reqPorts   LCCRA input
                                                                              params
   LCCRA parameter input proPorts   LCCRA output debugInfo  LCCRA debug
   information

                                        @return:void
                                      *****************************************************************************/
void LCCRA_Exec(const sLCCRAInReq_t* reqPorts,
                const sLCCRAParam_t* param,
                sLCCRAOutPro_t* proPorts,
                sLCCRADebug_t* debug) {
    // printf("--------------------LCCRA--------------------\n");
    /* LCCRA input wrapper */
    LBP_LeLnClthPosY0_met =
        reqPorts->LBP_LeLnClthPosY0_met; /* '<Root>/Inport' */
    LBP_LeLnClthHeading_rad =
        reqPorts->LBP_LeLnClthHeading_rad;               /* '<Root>/Inport1' */
    LBP_LeLnClthCrv_1pm = reqPorts->LBP_LeLnClthCrv_1pm; /* '<Root>/Inport2' */
    LBP_LeLnClthCrvChng_1pm2 =
        reqPorts->LBP_LeLnClthCrvChng_1pm2;            /* '<Root>/Inport3' */
    LBP_LeLnValid_bool = reqPorts->LBP_LeLnValid_bool; /* '<Root>/Inport4' */
    LBP_RiLnClthPosY0_met =
        reqPorts->LBP_RiLnClthPosY0_met; /* '<Root>/Inport5' */
    LBP_RiLnClthHeading_rad =
        reqPorts->LBP_RiLnClthHeading_rad;               /* '<Root>/Inport6' */
    LBP_RiLnClthCrv_1pm = reqPorts->LBP_RiLnClthCrv_1pm; /* '<Root>/Inport7' */
    LBP_RiLnClthCrvChng_1pm2 =
        reqPorts->LBP_RiLnClthCrvChng_1pm2;            /* '<Root>/Inport8' */
    LBP_RiLnValid_bool = reqPorts->LBP_RiLnValid_bool; /* '<Root>/Inport9' */
    ALP_LeLnClthPosY0_met =
        reqPorts->ALP_LeLnClthPosY0_met; /* '<Root>/Inport10' */
    ALP_LeLnClthHeading_rad =
        reqPorts->ALP_LeLnClthHeading_rad;               /* '<Root>/Inport11' */
    ALP_LeLnClthCrv_1pm = reqPorts->ALP_LeLnClthCrv_1pm; /* '<Root>/Inport12' */
    ALP_LeLnClthCrvChng_1pm2 =
        reqPorts->ALP_LeLnClthCrvChng_1pm2;            /* '<Root>/Inport13' */
    ALP_LeLnValid_bool = reqPorts->ALP_LeLnValid_bool; /* '<Root>/Inport14' */
    ALP_RiLnClthPosY0_met =
        reqPorts->ALP_RiLnClthPosY0_met; /* '<Root>/Inport15' */
    ALP_RiLnClthHeading_rad =
        reqPorts->ALP_RiLnClthHeading_rad;               /* '<Root>/Inport16' */
    ALP_RiLnClthCrv_1pm = reqPorts->ALP_RiLnClthCrv_1pm; /* '<Root>/Inport17' */
    ALP_RiLnClthCrvChng_1pm2 =
        reqPorts->ALP_RiLnClthCrvChng_1pm2;            /* '<Root>/Inport18' */
    ALP_RiLnValid_bool = reqPorts->ALP_RiLnValid_bool; /* '<Root>/Inport19' */
    Fusion_TargetObject_str =
        reqPorts->Fusion_TargetObject_str;               /* '<Root>/Inport26' */
    VED_EgoVelocity_mps = reqPorts->VED_EgoVelocity_mps; /* '<Root>/Inport31' */
    VED_EgoYawRate_rps = reqPorts->VED_EgoYawRate_rps;   /* '<Root>/Inport35' */
    VED_EgoClthCrv_1pm = reqPorts->VED_EgoClthCrv_1pm;   /* '<Root>/Inport36' */
    VED_EgoClthCrvChng_1pm2 =
        reqPorts->VED_EgoClthCrvChng_1pm2;         /* '<Root>/Inport37' */
    ABPR_LnWidth_met = reqPorts->ABPR_LnWidth_met; /* '<Root>/Inport38' */
    ABPR_LnWidthValid_bool =
        reqPorts->ABPR_LnWidthValid_bool; /* '<Root>/Inport22' */

    LCFRCV_SysCycleTimeSen_sec = param->LCFRCV_SysCycleTimeSen_sec;
    /* LCCRA function */

    LCCRA_step();

    /* LCCRA output wrapper */
    proPorts->LCCRA_LeftSafeFlag_bool =
        LCCRA_LeftSafeFlag_bool; /* '<S1>/LCCRA_SafeFlag' */
    proPorts->LCCRA_RightSafeFlag_bool =
        LCCRA_RightSafeFlag_bool; /* '<S1>/LCCRA_SafeFlag' */
    proPorts->LCCRA_LeftFrontSafeFlag_bool =
        LCCRA_LeftFrontSafeFlag_bool; /* '<S1>/LCCRA_SafeFlag' */
    proPorts->LCCRA_LeftRearSafeFlag_bool =
        LCCRA_LeftRearSafeFlag_bool; /* '<S1>/LCCRA_SafeFlag' */
    proPorts->LCCRA_RightFrontSafeFlag_bool =
        LCCRA_RightFrontSafeFlag_bool; /* '<S1>/LCCRA_SafeFlag' */
    proPorts->LCCRA_RightRearSafeFlag_bool =
        LCCRA_RightRearSafeFlag_bool; /* '<S1>/LCCRA_SafeFlag' */
    proPorts->LCCRA_LeftHighLightID_nu = LCCRA_LeftDangerObjID_nu;
    proPorts->LCCRA_RightHighLightID_nu = LCCRA_RightDangerObjID_nu;
    proPorts->LCCRA_FrontSafeFlag_bool = LCCRA_FrontSafeFlag_bool;
    proPorts->LCCRA_FrontDangerObjID_nu = LCCRA_FrontDangerObjID_nu;

    debug->LCCRA_DebugDelaytime_str =
        LCCRA_DebugDelaytime_str; /* '<S1>/LCCRA_FindMIOs' */
    debug->LCCRA_DebugMIOs_str =
        LCCRA_DebugMIOs_str; /* '<S1>/LCCRA_FindMIOs' */
    debug->LCCRA_DebugWeight_str =
        LCCRA_DebugWeight_str; /* '<S1>/LCCRA_FindMIOs' */
    debug->LCCRA_bFrontTTCSafe = LCCRA_bFrontTTCSafe;
    debug->LCCRA_bLeftFrontTTCSafe = LCCRA_bLeftFrontTTCSafe;
    debug->LCCRA_bLeftFrontTGSafe = LCCRA_bLeftFrontTGSafe;
    debug->LCCRA_bLeftFrontRDSafe = LCCRA_bLeftFrontRDSafe;
    debug->LCCRA_bNextLeftFrontSafe = LCCRA_bNextLeftFrontSafe;
    debug->LCCRA_bLeftRearTTCSafe = LCCRA_bLeftRearTTCSafe;
    debug->LCCRA_bLeftRearTGSafe = LCCRA_bLeftRearTGSafe;
    debug->LCCRA_bLeftRearRDSafe = LCCRA_bLeftRearRDSafe;
    debug->LCCRA_bNextLeftRearSafe = LCCRA_bNextLeftRearSafe;
    debug->LCCRA_bRightFrontTTCSafe = LCCRA_bRightFrontTTCSafe;
    debug->LCCRA_bRightFrontTGSafe = LCCRA_bRightFrontTGSafe;
    debug->LCCRA_bRightFrontRDSafe = LCCRA_bRightFrontRDSafe;
    debug->LCCRA_bNextRightFrontSafe = LCCRA_bNextRightFrontSafe;
    debug->LCCRA_bRightRearTTCSafe = LCCRA_bRightRearTTCSafe;
    debug->LCCRA_bRightRearTGSafe = LCCRA_bRightRearTGSafe;
    debug->LCCRA_bRightRearRDSafe = LCCRA_bRightRearRDSafe;
    debug->LCCRA_bNextRightRearSafe = LCCRA_bNextRightRearSafe;
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */