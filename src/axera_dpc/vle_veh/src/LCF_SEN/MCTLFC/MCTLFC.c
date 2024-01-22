/*
 * File: MCTLFC.c
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

#include "MCTLFC.h"
#include "MCTLFC_private.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
/* Exported data definition */
// #define DLMU2_START_CODE
// #include "Mem_Map.h"
/* Named constants for Chart: '<S5>/MCT_StateMachinet' */
#define MCTLFC_IN_ALCA ((uint8_T)1U)
#define MCTLFC_IN_AOLC ((uint8_T)2U)
#define MCTLFC_IN_ESA ((uint8_T)3U)
#define MCTLFC_IN_LDP ((uint8_T)4U)
#define MCTLFC_IN_LDPOC ((uint8_T)5U)
#define MCTLFC_IN_OFF ((uint8_T)6U)
#define MCTLFC_IN_RDP ((uint8_T)7U)
#define MCTLFC_IN_TJA ((uint8_T)8U)

#define ASW_QM_CORE2_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/* Exported block signals */
real32_T CSCLTA_LeCridrBndPosX0_met;    /* '<S7>/Multiport Switch1'
                                         * S_CSCLTA_LeCridrBndPosX0_met, float32,
                                         * the PosX0 value of left corridor bound,
                                         * [-300, 300]
                                         */
real32_T CSCLTA_LeCridrBndPosY0_met;    /* '<S7>/Multiport Switch1'
                                         * S_CSCLTA_LeCridrBndPosY0_met, float32,
                                         * the PosY0 value of left corridor bound,
                                         * [-15, 15]
                                         */
real32_T CSCLTA_LeCridrHeadAng_rad;     /* '<S7>/Multiport Switch1'
                                         * S_CSCLTA_LeCridrBndHeadAng_rad, float32,
                                         * the heading angle value of left corridor
                                         * bound, [-0.78539816, 0.78539816]
                                         */
real32_T CSCLTA_LeCridrBndCrv_1pm;      /* '<S7>/Multiport Switch1'
                                         * S_CSCLTA_LeCridrBndCrv_1pm, float32, the
                                         * curve value of left corridor bound, [-0.1,
                                         * 0.1]
                                         */
real32_T CSCLTA_LeCridrBndCrvChng_1pm2; /* '<S7>/Multiport Switch1'
                                         * S_CSCLTA_LeCridrBndCrvChng_1pm2, the
                                         * curve deviation value of left
                                         * corridor bound, [-0.001, 0.001]
                                         */
real32_T CSCLTA_LeCridrBndLength_met;   /* '<S7>/Multiport Switch1'
                                         * S_CSCLTA_LeCridrBndLength_met, the
                                         * length value of left corridor bound,
                                         * [0, 150]
                                         */
real32_T CSCLTA_RiCridrBndPosX0_met;    /* '<S7>/Multiport Switch1'
                                         * S_CSCLTA_RiCridrBndPosX0_met, float32,
                                         * the PosX0 value of right corridor bound,
                                         * [-300, 300]
                                         */
real32_T CSCLTA_RiCridrBndPosY0_met;    /* '<S7>/Multiport Switch1'
                                         * S_CSCLTA_RiCridrBndPosY0_met, float32,
                                         * the PosY0 value of right corridor bound,
                                         * [-15, 15]
                                         */
real32_T CSCLTA_RiCridrHeadAng_rad;     /* '<S7>/Multiport Switch1'
                                         * S_CSCLTA_RiCridrBndHeadAng_rad, float32,
                                         * the heading angle value of right corridor
                                         * bound, [-0.78539816, 0.78539816]
                                         */
real32_T CSCLTA_RiCridrBndCrv_1pm;      /* '<S7>/Multiport Switch1'
                                         * S_CSCLTA_RiCridrBndCrv_1pm, float32, the
                                         * curve value of right corridor bound,
                                         * [-0.1, 0.1]
                                         */
real32_T CSCLTA_RiCridrBndCrvChng_1pm2; /* '<S7>/Multiport Switch1'
                                         * S_CSCLTA_RiCridrBndCrvChng_1pm2, the
                                         * curve deviation value of right
                                         * corridor bound, [-0.001, 0.001]
                                         */
real32_T CSCLTA_RiCridrBndLength_met;   /* '<S7>/Multiport Switch1'
                                         * S_CSCLTA_RiCridrBndLength_met, the
                                         * length value of right corridor bound,
                                         * [0, 150]
                                         */
real32_T CSCLTA_TgtTrajPosX0_met;       /* '<S7>/Multiport Switch1'
                                         * S_CSCLTA_TgtTrajPosX0_met, float32, the
                                         * PosX0 value of target corridor bound,
                                         * [-300, 300]
                                         */
real32_T CSCLTA_TgtTrajPosY0_met;       /* '<S7>/Multiport Switch1'
                                         * S_CSCLTA_TgtTrajPosY0_met, float32, the
                                         * PosY0 value of target corridor bound, [-15,
                                         * 15]
                                         */
real32_T CSCLTA_TgtTrajHeadAng_rad;     /* '<S7>/Multiport Switch1'
                                         * S_CSCLTA_TgtTrajHeadAng_rad, float32, the
                                         * heading angle value of target corridor
                                         * bound, [-0.78539816, 0.78539816]
                                         */
real32_T CSCLTA_TgtTrajCrv_1pm;         /* '<S7>/Multiport Switch1'
                                         * S_CSCLTA_TgtTrajCrv_1pm, float32, the curve
                                         * value of target corridor bound, [-0.1, 0.1]
                                         */
real32_T CSCLTA_TgtTrajCrvChng_1pm2;    /* '<S7>/Multiport Switch1'
                                         * S_CSCLTA_TgtTrajCrvChng_1pm2, the curve
                                         * deviation value of target corridor
                                         * bound, [-0.001, 0.001]
                                         */
real32_T CSCLTA_TgtTrajLength_met;      /* '<S7>/Multiport Switch1'
                                         * S_CSCLTA_TgtTrajLength_met, the length
                                         * value of target corridor bound, [0, 150]
                                         */
real32_T CSCLTA_WeightTgtDistY_nu;      /* '<S7>/Multiport Switch1'
                                         * S_CSCLTA_WeightTgtDistY_nu, float32, The
                                         * importance factor of the lateral
                                         * deviation, [0,1]
                                         */
real32_T CSCLTA_WeightEndTime_nu;       /* '<S7>/Multiport Switch1'
                                         * S_CSCLTA_WeightEndTime_nu, float32, The
                                         * importance factor of the time required for
                                         * the planned trajectory, [0,1]
                                         */
real32_T CSCLTA_DistYToILeTgtArea_met;  /* '<S7>/Multiport Switch1'
                                         * S_CSCLTA_DistYTolLeTgtArea_met,
                                         * float32, lateral tolerance left
                                         * boundary value , [0,10]
                                         */
real32_T CSCLTA_DistYToIRiTgtArea_met;  /* '<S7>/Multiport Switch1'
                                         * S_CSCLTA_DistYTolRiTgtArea_met,
                                         * float32, lateral tolerance right
                                         * boundary value , [0,10]
                                         */
real32_T CSCLTA_FTireAclMax_mps2;       /* '<S7>/Multiport Switch1'
                                         * S_CSCLTA_FTireAclMax_mps2, float32, lateral
                                         * acceleration upper limiting value, [-20,20]
                                         */
real32_T CSCLTA_FTireAclMin_mps2;       /* '<S7>/Multiport Switch1'
                                         * S_CSCLTA_FTireAclMin_mps2, float32, lateral
                                         * acceleration lower limiting value, [-20,20]
                                         */
real32_T CSCLTA_PredTimeHeadAng_sec;    /* '<S7>/Multiport Switch1'
                                         * S_CSCLTA_PredTimeHeadAng_sec, float32，
                                         * todo, [0，60]
                                         */
real32_T CSCLTA_PredTimeCrv_sec;        /* '<S7>/Multiport Switch1'
                                         * S_CSCLTA_PredictionTimeCrv_sec,float32,
                                         * todo,[0，60]
                                         */
real32_T CSCLTA_PlanningHorzion_sec;    /* '<S7>/Multiport Switch1'
                                         * S_CSCLTA_PlanningHorizon_sec, float32,
                                         * max Planning horizon(time) of the
                                         * trajectory, [0，60]
                                         */
real32_T CSCLTA_ObstacleVelX_mps;       /* '<S7>/Multiport Switch1'
                                         * S_CSCLTA_ObstacleVelX_mps, float32, the
                                         * obstacle velocity X in target trajectory if
                                         * it is existed , [-20,150]
                                         */
real32_T CSCLTA_ObstacleAclX_mps2;      /* '<S7>/Multiport Switch1'
                                         * S_CSCLTA_ObstacleAclX_mps2, float32, the
                                         * obstacle accel X in target trajectory if
                                         * it is existed , [-20,20]
                                         */
real32_T CSCLTA_ObstacleWidth_met;      /* '<S7>/Multiport Switch1'
                                         * S_CSCLTA_ObstacleWidth_met, float32, the
                                         * obstacle width in target trajectory if it
                                         * is existed , [0,150]
                                         */
real32_T CSCLTA_ObstacleDistX_met;      /* '<S7>/Multiport Switch1'
                                         * S_CSCLTA_ObstacleDistX_met, float32, the
                                         * obstacle distance X in target trajectory
                                         * if it is existed , [-1000,1000]
                                         */
real32_T CSCLTA_ObstacleDistY_met;      /* '<S7>/Multiport Switch1'
                                         * S_CSCLTA_ObstacleDistY_met, float32, the
                                         * obstacle distance X in target trajectory
                                         * if it is existed , [-1000,1000]
                                         */
real32_T CSCLTA_SensorTStamp_sec;       /* '<S7>/Multiport Switch1'
                                         * S_CSCLTA_SensorTStamp_sec, float32, time
                                         * stamp of the camera signal from camera
                                         * sensor,[0,4295]
                                         */
real32_T CSCLTA_MaxCrvTrajGuiCtrl_1pm;  /* '<S7>/Multiport Switch1' */
real32_T CSCLTA_MaxCrvGrdBuildup_1pms;  /* '<S7>/Multiport Switch1' */
real32_T CSCLTA_MaxCrvGrdRed_1pms;      /* '<S7>/Multiport Switch1' */
real32_T CSCLTA_GrdLimitTgtCrvGC_1pms;  /* '<S7>/Multiport Switch1' */
real32_T CSCLTA_StrWhStifLimit_nu;      /* '<S7>/Multiport Switch1' */
real32_T CSCLTA_StrWhStifGrad_1ps;      /* '<S7>/Multiport Switch1' */
real32_T CSCLTA_TrqRampGrad_1ps;        /* '<S7>/Multiport Switch1' */
real32_T CSCLTA_MaxTrqScalLimit_nu;     /* '<S7>/Multiport Switch1' */
real32_T CSCLTA_MaxTrqScalGrad_nu;      /* '<S7>/Multiport Switch1' */
real32_T CSCLTA_LimiterDuration_sec;    /* '<S7>/Multiport Switch1' */
real32_T CSCLTA_MaxJerkAllowed_mps3;    /* '<S7>/Multiport Switch1'
                                         * S_CSCLTA_MaxJerkAllowed_mps3, float32,
                                         * Maximum Jerk Allowed in the trajectory
                                         * planning, [0,50]
                                         */
uint8_T CSCLTA_ControllingFunction_nu;  /* '<S5>/Switch' */
uint8_T CSCLTA_SysStateLCF;             /* '<S7>/Multiport Switch1'
                                         * S_CSCLTA_SysStateLCF_enum, uint8, lateral control
                                         * function system state enum value,
                                         * [0,6],E_LCF_SYSSTATE_NOTPRESENT = 0;
                                         * E_LCF_SYSSTATE_DISABLED = 1;
                                         * E_LCF_SYSSTATE_PASSIVE = 2;
                                         * E_LCF_SYSSTATE_REQUEST = 3;
                                         * E_LCF_SYSSTATE_CONTROLLING = 4;
                                         * E_LCF_SYSSTATE_RAMPOUT = 5; E_LCF_SYSSTATE_ERROR
                                         * = 6;
                                         */
uint8_T
    CSCLTA_TgtPlanServQu_nu; /* '<S7>/Multiport Switch1'
                              * S_CSCLTA_TrajPlanServQu_nu, uint8, todo,
                              * [0,255],P_TJATVG_TrajPlanValServQu_nu	=
                              * +/ 0x00,P_TJATVG_TrajPlanValSrvQuSALC_nu
                              * = +/ 0x10,
                              */
uint8_T
    CSCLTA_TrajGuiQualifier_nu;             /* '<S7>/Multiport Switch1'
                                             * S_CSCLTA_TrajGuiQualifier_nu, uint8,
                                             qualifier value of trajectory guidence, The
                                             qualifier indicates if/how the target curvature
                                             should be considered, [0,5] .E_LCF_TGQ_REQ_OFF=
                                             0, E_LCF_TGQ_REQ	= 1,
                                             E_LCF_TGQ_REQ_FREEZE= 3, E_LCF_TGQ_REQ_FFC
                                             = 4, E_LCF_TGQ_REQ_REFCHNG= 5
            
                                             */
uint8_T CSCLTA_DeratingLevel_nu;            /* '<S7>/Multiport Switch1' */
boolean_T CSCLTA_TriggerReplan_nu;          /* '<S7>/Multiport Switch1'
                                             * S_CSCLTA_TriggerReplan_nu,uint8,trigger
                                             * replan signal from CSCLTA module,[0，1]
                                             */
boolean_T CSCLTA_LatencyCompActivated_bool; /* '<S7>/Multiport Switch1'
                                             * S_CSCLTA_LatencyCompActivated_nu,
                                             * uint8, the trigger flag for
                                             * latency compensation function,
                                             * [0,1] 1: latency compensation
                                             * enable, 0: latency compensation
                                             * disable
                                             */
boolean_T CSCLTA_HighStatAccu_bool;         /* '<S7>/Multiport Switch1' */
boolean_T CSCLTA_LimiterActivated_nu;       /* '<S7>/Multiport Switch1' */

/* Block signals (default storage) */
B_MCTLFC_T MCTLFC_B;

/* Block states (default storage) */
DW_MCTLFC_T MCTLFC_DW;
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/* Exported data definition */

/* Const memory section */
/* Definition for custom storage class: Const */
const uint8_T MCTLFC_ControllingFunction_nu =
    0U;                                   /* Referenced by: '<S5>/Constant' */
const uint8_T MCTLFC_E_LCF_ALCA_nu = 5U;  /* Referenced by:
                                           * '<S5>/Constant11'
                                           * '<S5>/Constant18'
                                           * '<S14>/Constant6'
                                           */
const uint8_T MCTLFC_E_LCF_AOLC_nu = 6U;  /* Referenced by:
                                           * '<S5>/Constant12'
                                           * '<S5>/Constant17'
                                           */
const uint8_T MCTLFC_E_LCF_ESA_nu = 7U;   /* Referenced by:
                                           * '<S5>/Constant13'
                                           * '<S5>/Constant19'
                                           */
const uint8_T MCTLFC_E_LCF_LDPOC_nu = 3U; /* Referenced by:
                                           * '<S5>/Constant15'
                                           * '<S5>/Constant21'
                                           * '<S14>/Constant5'
                                           */
const uint8_T MCTLFC_E_LCF_LDP_nu = 2U;   /* Referenced by:
                                           * '<S5>/Constant14'
                                           * '<S5>/Constant20'
                                           * '<S14>/Constant3'
                                           */
const uint8_T MCTLFC_E_LCF_OFF_nu = 0U;   /* Referenced by: '<S5>/Constant16' */
const uint8_T MCTLFC_E_LCF_RDP_nu = 4U;   /* Referenced by:
                                           * '<S5>/Constant22'
                                           * '<S5>/Constant24'
                                           * '<S14>/Constant4'
                                           */
const uint8_T MCTLFC_E_LCF_SYSSTATE_CONTROLLING_nu = 4U; /* Referenced by:
                                                          * '<S10>/Constant1'
                                                          * '<S10>/Constant5'
                                                          */
const uint8_T MCTLFC_E_LCF_SYSSTATE_NOTPRESENT_nu =
    0U; /* Referenced by: '<S15>/Constant52' */
const uint8_T MCTLFC_E_LCF_SYSSTATE_RAMPOUT_nu =
    5U; /* Referenced by: '<S10>/Constant2' */
const uint8_T MCTLFC_E_LCF_SYSSTATE_REQUEST_nu = 3U; /* Referenced by:
                                                      * '<S10>/Constant'
                                                      * '<S10>/Constant3'
                                                      */
const uint8_T MCTLFC_E_LCF_TGO_REQ_OFF =
    0U;                                 /* Referenced by: '<S15>/Constant71' */
const uint8_T MCTLFC_E_LCF_TJA_nu = 1U; /* Referenced by:
                                         * '<S5>/Constant1'
                                         * '<S5>/Constant23'
                                         * '<S14>/Constant'
                                         */
const uint8_T MCTLFC_ErrInjEnable_bool =
    0U; /* Referenced by: '<S5>/Constant2' */
const real32_T MCTLFC_FollowUpDurationVec_sec[7] = {
    0.0F, 0.0F, 3.0F, 3.0F,
    3.0F, 0.0F, 0.0F}; /* Referenced by: '<S11>/Constant1' */

const uint8_T MCTLFC_PriorityVector_nu[7] = {
    1U, 2U, 5U, 3U, 4U, 0U, 0U}; /* Referenced by: '<S10>/Constant9' */

const boolean_T MCTLFC_RampOutOverwrite_bool[7] = {
    0, 0, 0, 0, 0, 0, 0}; /* Referenced by: '<S10>/Constant4' */

/* Definition for custom storage class: Global */
uint8_T MCTLFC_t_prio_ALCA;  /* '<S5>/Selector' */
uint8_T MCTLFC_t_prio_AOLC;  /* '<S5>/Selector1' */
uint8_T MCTLFC_t_prio_ESA;   /* '<S5>/Selector2' */
uint8_T MCTLFC_t_prio_LDP;   /* '<S5>/Selector3' */
uint8_T MCTLFC_t_prio_LDPOC; /* '<S5>/Selector4' */
uint8_T MCTLFC_t_prio_RDP;   /* '<S5>/Selector5' */
uint8_T MCTLFC_t_prio_TJA;   /* '<S5>/Selector6' */
uint8_T MCTLFC_t_prio_max;   /* '<S5>/Max' */

/* Forward declaration for local functions */
static void MCTLFC_TJA(void);
static void MCTLFC_RDP(void);

/* Function for Chart: '<S5>/MCT_StateMachinet' */
static void MCTLFC_TJA(void) {
    if ((MCTLFC_t_prio_TJA < MCTLFC_t_prio_max) || (MCTLFC_t_prio_max == 0)) {
        if ((MCTLFC_t_prio_ESA == MCTLFC_t_prio_max) &&
            (MCTLFC_t_prio_max != 0)) {
            MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_ESA;

            /* Constant: '<S5>/Constant13' */
            MCTLFC_B.S_MCTLFC_ControllingFunction_nu = MCTLFC_E_LCF_ESA_nu;
        } else if ((MCTLFC_t_prio_AOLC == MCTLFC_t_prio_max) &&
                   (MCTLFC_t_prio_max != 0)) {
            MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_AOLC;

            /* Constant: '<S5>/Constant12' */
            MCTLFC_B.S_MCTLFC_ControllingFunction_nu = MCTLFC_E_LCF_AOLC_nu;
        } else if ((MCTLFC_t_prio_RDP == MCTLFC_t_prio_max) &&
                   (MCTLFC_t_prio_max != 0)) {
            MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_RDP;

            /* Constant: '<S5>/Constant24' */
            MCTLFC_B.S_MCTLFC_ControllingFunction_nu = MCTLFC_E_LCF_RDP_nu;
        } else if ((MCTLFC_t_prio_ALCA == MCTLFC_t_prio_max) &&
                   (MCTLFC_t_prio_max != 0)) {
            MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_ALCA;

            /* Constant: '<S5>/Constant11' */
            MCTLFC_B.S_MCTLFC_ControllingFunction_nu = MCTLFC_E_LCF_ALCA_nu;
        } else if ((MCTLFC_t_prio_LDPOC == MCTLFC_t_prio_max) &&
                   (MCTLFC_t_prio_max != 0)) {
            MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_LDPOC;

            /* Constant: '<S5>/Constant15' */
            MCTLFC_B.S_MCTLFC_ControllingFunction_nu = MCTLFC_E_LCF_LDPOC_nu;
        } else if ((MCTLFC_t_prio_LDP == MCTLFC_t_prio_max) &&
                   (MCTLFC_t_prio_max != 0)) {
            MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_LDP;

            /* Constant: '<S5>/Constant14' */
            MCTLFC_B.S_MCTLFC_ControllingFunction_nu = MCTLFC_E_LCF_LDP_nu;
        } else if ((MCTLFC_t_prio_TJA == MCTLFC_t_prio_max) &&
                   (MCTLFC_t_prio_max != 0)) {
            MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_TJA;

            /* Constant: '<S5>/Constant1' */
            MCTLFC_B.S_MCTLFC_ControllingFunction_nu = MCTLFC_E_LCF_TJA_nu;
        } else {
            MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_OFF;

            /* Constant: '<S5>/Constant16' */
            MCTLFC_B.S_MCTLFC_ControllingFunction_nu = MCTLFC_E_LCF_OFF_nu;
        }
    }
}

/* Function for Chart: '<S5>/MCT_StateMachinet' */
static void MCTLFC_RDP(void) {
    if ((MCTLFC_t_prio_RDP < MCTLFC_t_prio_max) || (MCTLFC_t_prio_max == 0)) {
        if ((MCTLFC_t_prio_ESA == MCTLFC_t_prio_max) &&
            (MCTLFC_t_prio_max != 0)) {
            MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_ESA;

            /* Constant: '<S5>/Constant13' */
            MCTLFC_B.S_MCTLFC_ControllingFunction_nu = MCTLFC_E_LCF_ESA_nu;
        } else if ((MCTLFC_t_prio_AOLC == MCTLFC_t_prio_max) &&
                   (MCTLFC_t_prio_max != 0)) {
            MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_AOLC;

            /* Constant: '<S5>/Constant12' */
            MCTLFC_B.S_MCTLFC_ControllingFunction_nu = MCTLFC_E_LCF_AOLC_nu;
        } else if ((MCTLFC_t_prio_RDP == MCTLFC_t_prio_max) &&
                   (MCTLFC_t_prio_max != 0)) {
            MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_RDP;

            /* Constant: '<S5>/Constant24' */
            MCTLFC_B.S_MCTLFC_ControllingFunction_nu = MCTLFC_E_LCF_RDP_nu;
        } else if ((MCTLFC_t_prio_ALCA == MCTLFC_t_prio_max) &&
                   (MCTLFC_t_prio_max != 0)) {
            MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_ALCA;

            /* Constant: '<S5>/Constant11' */
            MCTLFC_B.S_MCTLFC_ControllingFunction_nu = MCTLFC_E_LCF_ALCA_nu;
        } else if ((MCTLFC_t_prio_LDPOC == MCTLFC_t_prio_max) &&
                   (MCTLFC_t_prio_max != 0)) {
            MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_LDPOC;

            /* Constant: '<S5>/Constant15' */
            MCTLFC_B.S_MCTLFC_ControllingFunction_nu = MCTLFC_E_LCF_LDPOC_nu;
        } else if ((MCTLFC_t_prio_LDP == MCTLFC_t_prio_max) &&
                   (MCTLFC_t_prio_max != 0)) {
            MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_LDP;

            /* Constant: '<S5>/Constant14' */
            MCTLFC_B.S_MCTLFC_ControllingFunction_nu = MCTLFC_E_LCF_LDP_nu;
        } else if ((MCTLFC_t_prio_TJA == MCTLFC_t_prio_max) &&
                   (MCTLFC_t_prio_max != 0)) {
            MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_TJA;

            /* Constant: '<S5>/Constant1' */
            MCTLFC_B.S_MCTLFC_ControllingFunction_nu = MCTLFC_E_LCF_TJA_nu;
        } else {
            MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_OFF;

            /* Constant: '<S5>/Constant16' */
            MCTLFC_B.S_MCTLFC_ControllingFunction_nu = MCTLFC_E_LCF_OFF_nu;
        }
    }
}

/* Model step function */
void MCTLFC_step(void) {
    int32_T i;
    int32_T rtb_t_prio_LCF;
    uint8_T rtb_VectorConcatenate[7];
    uint8_T rtb_VectorConcatenate_f;
    boolean_T rtb_Equal3[7];
    boolean_T rtb_Equal3_f;
    boolean_T rtb_OR2;

    /* SignalConversion generated from: '<S10>/Vector Concatenate' incorporates:
     *  Inport: '<Root>/Inport1'
     */
    rtb_VectorConcatenate[0] = MCTLFC_LCFRCV_TJASTM_SysStateTJA_nu;

    /* SignalConversion generated from: '<S10>/Vector Concatenate' incorporates:
     *  Inport: '<Root>/Inport2'
     */
    rtb_VectorConcatenate[1] = MCTLFC_DPLSMI_SysStateLDP_nu;

    /* SignalConversion generated from: '<S10>/Vector Concatenate' incorporates:
     *  Inport: '<Root>/Inport3'
     */
    rtb_VectorConcatenate[2] = MCTLFC_DPOSTM_SysStateLDPOC_nu;

    /* SignalConversion generated from: '<S10>/Vector Concatenate' incorporates:
     *  Inport: '<Root>/Inport4'
     */
    rtb_VectorConcatenate[3] = MCTLFC_DPRSMI_SysStateRDP_nu;

    /* SignalConversion generated from: '<S10>/Vector Concatenate' incorporates:
     *  Inport: '<Root>/Inport5'
     */
    rtb_VectorConcatenate[4] = MCTLFC_LCRSMI_SysStateALCA_nu;

    /* SignalConversion generated from: '<S10>/Vector Concatenate' incorporates:
     *  Inport: '<Root>/Inport6'
     */
    rtb_VectorConcatenate[5] = MCTLFC_LCSSTM_SysStateAOLC_nu;

    /* SignalConversion generated from: '<S10>/Vector Concatenate' incorporates:
     *  Inport: '<Root>/Inport7'
     */
    rtb_VectorConcatenate[6] = MCTLFC_ESASTM_SysStateESA_nu;

    /* RelationalOperator: '<S10>/Equal3' incorporates:
     *  Constant: '<S10>/Constant3'
     */
    for (i = 0; i < 7; i++) {
        rtb_Equal3[i] =
            (rtb_VectorConcatenate[i] == MCTLFC_E_LCF_SYSSTATE_REQUEST_nu);
    }

    /* End of RelationalOperator: '<S10>/Equal3' */

    /* Logic: '<S10>/OR' */
    rtb_OR2 = rtb_Equal3[0];
    for (i = 0; i < 6; i++) {
        rtb_OR2 = (rtb_OR2 || rtb_Equal3[i + 1]);
    }

    /* RelationalOperator: '<S10>/Equal4' incorporates:
     *  Constant: '<S10>/Constant5'
     */
    for (i = 0; i < 7; i++) {
        rtb_Equal3[i] =
            (rtb_VectorConcatenate[i] == MCTLFC_E_LCF_SYSSTATE_CONTROLLING_nu);
    }

    /* End of RelationalOperator: '<S10>/Equal4' */

    /* Logic: '<S10>/OR1' */
    rtb_Equal3_f = rtb_Equal3[0];
    for (i = 0; i < 6; i++) {
        rtb_Equal3_f = (rtb_Equal3_f || rtb_Equal3[i + 1]);
    }

    /* Logic: '<S10>/OR2' incorporates:
     *  Logic: '<S10>/OR'
     *  Logic: '<S10>/OR1'
     */
    rtb_OR2 = (rtb_OR2 || rtb_Equal3_f);
    for (i = 0; i < 7; i++) {
        /* RelationalOperator: '<S10>/Equal' */
        rtb_VectorConcatenate_f = rtb_VectorConcatenate[i];

        /* Product: '<S10>/Product' incorporates:
         *  Constant: '<S10>/Constant'
         *  Constant: '<S10>/Constant1'
         *  Constant: '<S10>/Constant2'
         *  Constant: '<S10>/Constant4'
         *  Constant: '<S10>/Constant9'
         *  Logic: '<S10>/AND'
         *  Logic: '<S10>/AND1'
         *  Logic: '<S10>/NOT'
         *  Logic: '<S10>/OR3'
         *  RelationalOperator: '<S10>/Equal'
         *  RelationalOperator: '<S10>/Equal1'
         *  RelationalOperator: '<S10>/Equal2'
         */
        rtb_t_prio_LCF =
            ((rtb_VectorConcatenate_f == MCTLFC_E_LCF_SYSSTATE_REQUEST_nu) ||
             (rtb_VectorConcatenate_f ==
              MCTLFC_E_LCF_SYSSTATE_CONTROLLING_nu) ||
             ((rtb_VectorConcatenate_f == MCTLFC_E_LCF_SYSSTATE_RAMPOUT_nu) &&
              ((!MCTLFC_RampOutOverwrite_bool[i]) || (!rtb_OR2)))) *
            MCTLFC_PriorityVector_nu[i];

        /* RelationalOperator: '<S11>/Equal' incorporates:
         *  Constant: '<S11>/Constant'
         */
        rtb_Equal3_f = (rtb_t_prio_LCF == 0);

        /* Switch: '<S13>/Switch2' incorporates:
         *  Constant: '<S11>/Constant1'
         *  Constant: '<S13>/Constant1'
         *  Inport: '<Root>/Inport'
         *  Logic: '<S12>/AND'
         *  Logic: '<S12>/NOT'
         *  RelationalOperator: '<S13>/GreaterThan'
         *  Switch: '<S13>/Switch'
         *  UnitDelay: '<S12>/Unit Delay'
         *  UnitDelay: '<S13>/Unit Delay'
         */
        if (rtb_Equal3_f && (!MCTLFC_DW.UnitDelay_DSTATE_c[i])) {
            MCTLFC_DW.UnitDelay_DSTATE[i] = MCTLFC_FollowUpDurationVec_sec[i];
        } else if (MCTLFC_DW.UnitDelay_DSTATE[i] >
                   MCTLFC_LCFRCV_SysCycleTimeSen_sec) {
            /* Switch: '<S13>/Switch' incorporates:
             *  Inport: '<Root>/Inport'
             *  Sum: '<S13>/Subtract'
             *  UnitDelay: '<S13>/Unit Delay'
             */
            MCTLFC_DW.UnitDelay_DSTATE[i] -= MCTLFC_LCFRCV_SysCycleTimeSen_sec;
        } else {
            MCTLFC_DW.UnitDelay_DSTATE[i] = 0.0F;
        }

        /* End of Switch: '<S13>/Switch2' */

        /* Switch: '<S11>/Switch' incorporates:
         *  Constant: '<S13>/Constant2'
         *  RelationalOperator: '<S13>/GreaterThan1'
         *  UnitDelay: '<S11>/Unit Delay'
         *  UnitDelay: '<S13>/Unit Delay'
         */
        if (MCTLFC_DW.UnitDelay_DSTATE[i] <= 0.0F) {
            MCTLFC_DW.UnitDelay_DSTATE_e[i] = (uint8_T)rtb_t_prio_LCF;
        }

        /* End of Switch: '<S11>/Switch' */

        /* RelationalOperator: '<S11>/Equal' */
        rtb_Equal3[i] = rtb_Equal3_f;
    }

    /* Selector: '<S5>/Selector' incorporates:
     *  Constant: '<S5>/Constant18'
     *  UnitDelay: '<S11>/Unit Delay'
     */
    MCTLFC_t_prio_ALCA = MCTLFC_DW.UnitDelay_DSTATE_e[MCTLFC_E_LCF_ALCA_nu - 1];

    /* Selector: '<S5>/Selector1' incorporates:
     *  Constant: '<S5>/Constant17'
     *  UnitDelay: '<S11>/Unit Delay'
     */
    MCTLFC_t_prio_AOLC = MCTLFC_DW.UnitDelay_DSTATE_e[MCTLFC_E_LCF_AOLC_nu - 1];

    /* Selector: '<S5>/Selector2' incorporates:
     *  Constant: '<S5>/Constant19'
     *  UnitDelay: '<S11>/Unit Delay'
     */
    MCTLFC_t_prio_ESA = MCTLFC_DW.UnitDelay_DSTATE_e[MCTLFC_E_LCF_ESA_nu - 1];

    /* Selector: '<S5>/Selector3' incorporates:
     *  Constant: '<S5>/Constant20'
     *  UnitDelay: '<S11>/Unit Delay'
     */
    MCTLFC_t_prio_LDP = MCTLFC_DW.UnitDelay_DSTATE_e[MCTLFC_E_LCF_LDP_nu - 1];

    /* Selector: '<S5>/Selector4' incorporates:
     *  Constant: '<S5>/Constant21'
     *  UnitDelay: '<S11>/Unit Delay'
     */
    MCTLFC_t_prio_LDPOC =
        MCTLFC_DW.UnitDelay_DSTATE_e[MCTLFC_E_LCF_LDPOC_nu - 1];

    /* Selector: '<S5>/Selector5' incorporates:
     *  Constant: '<S5>/Constant22'
     *  UnitDelay: '<S11>/Unit Delay'
     */
    MCTLFC_t_prio_RDP = MCTLFC_DW.UnitDelay_DSTATE_e[MCTLFC_E_LCF_RDP_nu - 1];

    /* Selector: '<S5>/Selector6' incorporates:
     *  Constant: '<S5>/Constant23'
     *  UnitDelay: '<S11>/Unit Delay'
     */
    MCTLFC_t_prio_TJA = MCTLFC_DW.UnitDelay_DSTATE_e[MCTLFC_E_LCF_TJA_nu - 1];

    /* MinMax: '<S5>/Max' incorporates:
     *  UnitDelay: '<S11>/Unit Delay'
     */
    MCTLFC_t_prio_max = MCTLFC_DW.UnitDelay_DSTATE_e[0];
    for (i = 0; i < 6; i++) {
        rtb_VectorConcatenate_f = MCTLFC_DW.UnitDelay_DSTATE_e[i + 1];
        if (MCTLFC_t_prio_max <= rtb_VectorConcatenate_f) {
            MCTLFC_t_prio_max = rtb_VectorConcatenate_f;
        }
    }

    /* End of MinMax: '<S5>/Max' */

    /* Chart: '<S5>/MCT_StateMachinet' incorporates:
     *  Constant: '<S5>/Constant1'
     *  Constant: '<S5>/Constant11'
     *  Constant: '<S5>/Constant12'
     *  Constant: '<S5>/Constant13'
     *  Constant: '<S5>/Constant14'
     *  Constant: '<S5>/Constant15'
     *  Constant: '<S5>/Constant16'
     *  Constant: '<S5>/Constant24'
     */
    if (MCTLFC_DW.is_active_c3_MCTLFC == 0U) {
        MCTLFC_DW.is_active_c3_MCTLFC = 1U;
        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_OFF;
        MCTLFC_B.S_MCTLFC_ControllingFunction_nu = MCTLFC_E_LCF_OFF_nu;
    } else {
        switch (MCTLFC_DW.is_c3_MCTLFC) {
            case MCTLFC_IN_ALCA:
                if ((MCTLFC_t_prio_ALCA < MCTLFC_t_prio_max) ||
                    (MCTLFC_t_prio_max == 0)) {
                    if ((MCTLFC_t_prio_ESA == MCTLFC_t_prio_max) &&
                        (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_ESA;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_ESA_nu;
                    } else if ((MCTLFC_t_prio_AOLC == MCTLFC_t_prio_max) &&
                               (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_AOLC;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_AOLC_nu;
                    } else if ((MCTLFC_t_prio_RDP == MCTLFC_t_prio_max) &&
                               (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_RDP;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_RDP_nu;
                    } else if ((MCTLFC_t_prio_ALCA == MCTLFC_t_prio_max) &&
                               (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_ALCA;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_ALCA_nu;
                    } else if ((MCTLFC_t_prio_LDPOC == MCTLFC_t_prio_max) &&
                               (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_LDPOC;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_LDPOC_nu;
                    } else if ((MCTLFC_t_prio_LDP == MCTLFC_t_prio_max) &&
                               (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_LDP;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_LDP_nu;
                    } else if ((MCTLFC_t_prio_TJA == MCTLFC_t_prio_max) &&
                               (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_TJA;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_TJA_nu;
                    } else {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_OFF;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_OFF_nu;
                    }
                }
                break;

            case MCTLFC_IN_AOLC:
                if ((MCTLFC_t_prio_AOLC < MCTLFC_t_prio_max) ||
                    (MCTLFC_t_prio_max == 0)) {
                    if ((MCTLFC_t_prio_ESA == MCTLFC_t_prio_max) &&
                        (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_ESA;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_ESA_nu;
                    } else if ((MCTLFC_t_prio_AOLC == MCTLFC_t_prio_max) &&
                               (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_AOLC;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_AOLC_nu;
                    } else if ((MCTLFC_t_prio_RDP == MCTLFC_t_prio_max) &&
                               (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_RDP;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_RDP_nu;
                    } else if ((MCTLFC_t_prio_ALCA == MCTLFC_t_prio_max) &&
                               (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_ALCA;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_ALCA_nu;
                    } else if ((MCTLFC_t_prio_LDPOC == MCTLFC_t_prio_max) &&
                               (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_LDPOC;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_LDPOC_nu;
                    } else if ((MCTLFC_t_prio_LDP == MCTLFC_t_prio_max) &&
                               (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_LDP;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_LDP_nu;
                    } else if ((MCTLFC_t_prio_TJA == MCTLFC_t_prio_max) &&
                               (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_TJA;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_TJA_nu;
                    } else {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_OFF;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_OFF_nu;
                    }
                }
                break;

            case MCTLFC_IN_ESA:
                if ((MCTLFC_t_prio_ESA < MCTLFC_t_prio_max) ||
                    (MCTLFC_t_prio_max == 0)) {
                    if ((MCTLFC_t_prio_ESA == MCTLFC_t_prio_max) &&
                        (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_ESA;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_ESA_nu;
                    } else if ((MCTLFC_t_prio_AOLC == MCTLFC_t_prio_max) &&
                               (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_AOLC;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_AOLC_nu;
                    } else if ((MCTLFC_t_prio_RDP == MCTLFC_t_prio_max) &&
                               (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_RDP;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_RDP_nu;
                    } else if ((MCTLFC_t_prio_ALCA == MCTLFC_t_prio_max) &&
                               (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_ALCA;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_ALCA_nu;
                    } else if ((MCTLFC_t_prio_LDPOC == MCTLFC_t_prio_max) &&
                               (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_LDPOC;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_LDPOC_nu;
                    } else if ((MCTLFC_t_prio_LDP == MCTLFC_t_prio_max) &&
                               (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_LDP;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_LDP_nu;
                    } else if ((MCTLFC_t_prio_TJA == MCTLFC_t_prio_max) &&
                               (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_TJA;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_TJA_nu;
                    } else {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_OFF;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_OFF_nu;
                    }
                }
                break;

            case MCTLFC_IN_LDP:
                if ((MCTLFC_t_prio_LDP < MCTLFC_t_prio_max) ||
                    (MCTLFC_t_prio_max == 0)) {
                    if ((MCTLFC_t_prio_ESA == MCTLFC_t_prio_max) &&
                        (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_ESA;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_ESA_nu;
                    } else if ((MCTLFC_t_prio_AOLC == MCTLFC_t_prio_max) &&
                               (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_AOLC;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_AOLC_nu;
                    } else if ((MCTLFC_t_prio_RDP == MCTLFC_t_prio_max) &&
                               (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_RDP;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_RDP_nu;
                    } else if ((MCTLFC_t_prio_ALCA == MCTLFC_t_prio_max) &&
                               (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_ALCA;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_ALCA_nu;
                    } else if ((MCTLFC_t_prio_LDPOC == MCTLFC_t_prio_max) &&
                               (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_LDPOC;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_LDPOC_nu;
                    } else if ((MCTLFC_t_prio_LDP == MCTLFC_t_prio_max) &&
                               (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_LDP;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_LDP_nu;
                    } else if ((MCTLFC_t_prio_TJA == MCTLFC_t_prio_max) &&
                               (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_TJA;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_TJA_nu;
                    } else {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_OFF;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_OFF_nu;
                    }
                }
                break;

            case MCTLFC_IN_LDPOC:
                if ((MCTLFC_t_prio_LDPOC < MCTLFC_t_prio_max) ||
                    (MCTLFC_t_prio_max == 0)) {
                    if ((MCTLFC_t_prio_ESA == MCTLFC_t_prio_max) &&
                        (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_ESA;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_ESA_nu;
                    } else if ((MCTLFC_t_prio_AOLC == MCTLFC_t_prio_max) &&
                               (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_AOLC;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_AOLC_nu;
                    } else if ((MCTLFC_t_prio_RDP == MCTLFC_t_prio_max) &&
                               (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_RDP;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_RDP_nu;
                    } else if ((MCTLFC_t_prio_ALCA == MCTLFC_t_prio_max) &&
                               (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_ALCA;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_ALCA_nu;
                    } else if ((MCTLFC_t_prio_LDPOC == MCTLFC_t_prio_max) &&
                               (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_LDPOC;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_LDPOC_nu;
                    } else if ((MCTLFC_t_prio_LDP == MCTLFC_t_prio_max) &&
                               (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_LDP;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_LDP_nu;
                    } else if ((MCTLFC_t_prio_TJA == MCTLFC_t_prio_max) &&
                               (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_TJA;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_TJA_nu;
                    } else {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_OFF;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_OFF_nu;
                    }
                }
                break;

            case MCTLFC_IN_OFF:
                if (MCTLFC_t_prio_max != 0) {
                    if ((MCTLFC_t_prio_ESA == MCTLFC_t_prio_max) &&
                        (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_ESA;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_ESA_nu;
                    } else if ((MCTLFC_t_prio_AOLC == MCTLFC_t_prio_max) &&
                               (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_AOLC;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_AOLC_nu;
                    } else if ((MCTLFC_t_prio_RDP == MCTLFC_t_prio_max) &&
                               (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_RDP;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_RDP_nu;
                    } else if ((MCTLFC_t_prio_ALCA == MCTLFC_t_prio_max) &&
                               (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_ALCA;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_ALCA_nu;
                    } else if ((MCTLFC_t_prio_LDPOC == MCTLFC_t_prio_max) &&
                               (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_LDPOC;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_LDPOC_nu;
                    } else if ((MCTLFC_t_prio_LDP == MCTLFC_t_prio_max) &&
                               (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_LDP;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_LDP_nu;
                    } else if ((MCTLFC_t_prio_TJA == MCTLFC_t_prio_max) &&
                               (MCTLFC_t_prio_max != 0)) {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_TJA;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_TJA_nu;
                    } else {
                        MCTLFC_DW.is_c3_MCTLFC = MCTLFC_IN_OFF;
                        MCTLFC_B.S_MCTLFC_ControllingFunction_nu =
                            MCTLFC_E_LCF_OFF_nu;
                    }
                }
                break;

            case MCTLFC_IN_RDP:
                MCTLFC_RDP();
                break;

            default:
                /* case IN_TJA: */
                MCTLFC_TJA();
                break;
        }
    }

    /* End of Chart: '<S5>/MCT_StateMachinet' */

    /* Switch: '<S5>/Switch' incorporates:
     *  Constant: '<S5>/Constant2'
     */
    if (MCTLFC_ErrInjEnable_bool != 0) {
        /* Switch: '<S5>/Switch' incorporates:
         *  Constant: '<S5>/Constant'
         */
        CSCLTA_ControllingFunction_nu = MCTLFC_ControllingFunction_nu;
    } else {
        /* Switch: '<S5>/Switch' */
        CSCLTA_ControllingFunction_nu =
            MCTLFC_B.S_MCTLFC_ControllingFunction_nu;
    }

    /* End of Switch: '<S5>/Switch' */

    /* Switch: '<S14>/Switch' incorporates:
     *  Constant: '<S14>/Constant'
     *  Constant: '<S14>/Constant10'
     *  Constant: '<S14>/Constant3'
     *  Constant: '<S14>/Constant4'
     *  Constant: '<S14>/Constant5'
     *  Constant: '<S14>/Constant6'
     *  RelationalOperator: '<S14>/Equal'
     *  RelationalOperator: '<S14>/Equal1'
     *  RelationalOperator: '<S14>/Equal2'
     *  RelationalOperator: '<S14>/Equal3'
     *  RelationalOperator: '<S14>/Equal4'
     *  Switch: '<S14>/Switch2'
     *  Switch: '<S14>/Switch3'
     *  Switch: '<S14>/Switch4'
     */
    if (CSCLTA_ControllingFunction_nu == MCTLFC_E_LCF_ALCA_nu) {
        i = 5;
    } else if (CSCLTA_ControllingFunction_nu == MCTLFC_E_LCF_LDPOC_nu) {
        /* Switch: '<S14>/Switch4' incorporates:
         *  Constant: '<S14>/Constant9'
         */
        i = 4;
    } else if (CSCLTA_ControllingFunction_nu == MCTLFC_E_LCF_RDP_nu) {
        /* Switch: '<S14>/Switch3' incorporates:
         *  Constant: '<S14>/Constant7'
         *  Switch: '<S14>/Switch4'
         */
        i = 3;
    } else if (CSCLTA_ControllingFunction_nu == MCTLFC_E_LCF_LDP_nu) {
        /* Switch: '<S14>/Switch2' incorporates:
         *  Constant: '<S14>/Constant8'
         *  Switch: '<S14>/Switch3'
         *  Switch: '<S14>/Switch4'
         */
        i = 2;
    } else {
        i = (CSCLTA_ControllingFunction_nu == MCTLFC_E_LCF_TJA_nu);
    }

    /* End of Switch: '<S14>/Switch' */

    /* MultiPortSwitch generated from: '<S7>/Multiport Switch1' */
    switch (i) {
        case 1:
            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport146'
             */
            CSCLTA_SysStateLCF = MCTLFC_SysStateTJA_nu;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport147'
             */
            CSCLTA_LeCridrBndPosX0_met = MCTLFC_TJATTG_LeCridrBndPosX0_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport148'
             */
            CSCLTA_LeCridrBndPosY0_met = MCTLFC_TJATTG_LeCridrBndPosY0_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport149'
             */
            CSCLTA_LeCridrHeadAng_rad = MCTLFC_TJATTG_LeCridrBndHeadAng_rad;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport150'
             */
            CSCLTA_LeCridrBndCrv_1pm = MCTLFC_TJATTG_LeCridrBndCrv_1pm;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport151'
             */
            CSCLTA_LeCridrBndCrvChng_1pm2 =
                MCTLFC_TJATTG_LeCridrBndCrvChng_1pm2;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport152'
             */
            CSCLTA_LeCridrBndLength_met = MCTLFC_TJATTG_LeCridrBndLength_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport153'
             */
            CSCLTA_RiCridrBndPosX0_met = MCTLFC_TJATTG_RiCridrBndPosX0_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport154'
             */
            CSCLTA_RiCridrBndPosY0_met = MCTLFC_TJATTG_RiCridrBndPosY0_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport155'
             */
            CSCLTA_RiCridrHeadAng_rad = MCTLFC_TJATTG_RiCridrBndHeadAng_rad;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport156'
             */
            CSCLTA_RiCridrBndCrv_1pm = MCTLFC_TJATTG_RiCridrBndCrv_1pm;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport157'
             */
            CSCLTA_RiCridrBndCrvChng_1pm2 =
                MCTLFC_TJATTG_RiCridrBndCrvChng_1pm2;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport158'
             */
            CSCLTA_RiCridrBndLength_met = MCTLFC_TJATTG_RiCridrBndLength_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport159'
             */
            CSCLTA_TgtTrajPosX0_met = MCTLFC_TJATTG_TgtTrajPosX0_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport160'
             */
            CSCLTA_TgtTrajPosY0_met = MCTLFC_TJATTG_TgtTrajPosY0_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport161'
             */
            CSCLTA_TgtTrajHeadAng_rad = MCTLFC_TJATTG_TgtTrajHeadAng_rad;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport162'
             */
            CSCLTA_TgtTrajCrv_1pm = MCTLFC_TJATTG_TgtTrajCrv_1pm;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport163'
             */
            CSCLTA_TgtTrajCrvChng_1pm2 = MCTLFC_TJATTG_TgtTrajCrvChng_1pm2;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport164'
             */
            CSCLTA_TgtTrajLength_met = MCTLFC_TJATTG_TgtTrajLength_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport165'
             */
            CSCLTA_TgtPlanServQu_nu = MCTLFC_TJATVG_TrajPlanServQu_nu;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport166'
             */
            CSCLTA_WeightTgtDistY_nu = MCTLFC_TJATVG_WeightTgtDistY_nu;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport167'
             */
            CSCLTA_WeightEndTime_nu = MCTLFC_TJATVG_WeightEndTime_nu;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport168'
             */
            CSCLTA_DistYToILeTgtArea_met = MCTLFC_TJATVG_DistYToLeTgtArea_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport169'
             */
            CSCLTA_DistYToIRiTgtArea_met = MCTLFC_TJATVG_DistYToRiTgtArea_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport170'
             */
            CSCLTA_FTireAclMax_mps2 = MCTLFC_TJATVG_FTireAclMax_mps2;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport171'
             */
            CSCLTA_FTireAclMin_mps2 = MCTLFC_TJATVG_FTireAclMin_mps2;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport172'
             */
            CSCLTA_TrajGuiQualifier_nu = MCTLFC_TJATVG_TrajGuiQu_nu;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport173'
             */
            CSCLTA_TriggerReplan_nu = MCTLFC_TJATVG_TriggerReplan_bool;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport174'
             */
            CSCLTA_PredTimeHeadAng_sec = MCTLFC_TJATVG_PredTimeHeadAng_sec;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport175'
             */
            CSCLTA_PredTimeCrv_sec = MCTLFC_TJATVG_PredTimeCrv_sec;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport176'
             */
            CSCLTA_PlanningHorzion_sec = MCTLFC_TJATVG_PlanningHorzion_sec;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport177'
             */
            CSCLTA_ObstacleVelX_mps = MCTLFC_TJATVG_ObstacleVelX_mps;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport178'
             */
            CSCLTA_ObstacleAclX_mps2 = MCTLFC_TJATVG_ObstacleAclX_mps2;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport179'
             */
            CSCLTA_ObstacleWidth_met = MCTLFC_TJATVG_ObstacleWidth_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport180'
             */
            CSCLTA_ObstacleDistX_met = MCTLFC_TJATVG_ObstacleDistX_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport181'
             */
            CSCLTA_ObstacleDistY_met = MCTLFC_TJATVG_ObstacleDistY_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport182'
             */
            CSCLTA_LatencyCompActivated_bool =
                MCTLFC_TJATVG_LtcyCompActivated_bool;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport183'
             */
            CSCLTA_SensorTStamp_sec = MCTLFC_TJATVG_SensorTStamp_sec;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport184'
             */
            CSCLTA_MaxCrvTrajGuiCtrl_1pm = MCTLFC_TJATVG_MaxCrvTrajGuiCtrl_1pm;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport185'
             */
            CSCLTA_MaxCrvGrdBuildup_1pms = MCTLFC_TJATVG_MaxCrvGrdBuildup_1pms;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport186'
             */
            CSCLTA_MaxCrvGrdRed_1pms = MCTLFC_TJATVG_MaxCrvGrdRed_1pms;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport187'
             */
            CSCLTA_GrdLimitTgtCrvGC_1pms = MCTLFC_TJATVG_GrdLimitTgtCrvTGC_1pms;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport188'
             */
            CSCLTA_StrWhStifLimit_nu = MCTLFC_TJATVG_StrWhStifLimit_nu;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport240'
             */
            CSCLTA_StrWhStifGrad_1ps = MCTLFC_TJATVG_StrWhStifGrad_1ps;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport241'
             */
            CSCLTA_TrqRampGrad_1ps = MCTLFC_TJATVG_TrqRampGrad_1ps;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport189'
             */
            CSCLTA_MaxTrqScalLimit_nu = MCTLFC_TJATVG_MaxTrqScaILimit_nu;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport238'
             */
            CSCLTA_MaxTrqScalGrad_nu = MCTLFC_TJATVG_MaxTrqScalGrad_1ps;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport239'
             */
            CSCLTA_HighStatAccu_bool = MCTLFC_TJATVG_HighStatAccu_bool;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport190'
             */
            CSCLTA_LimiterActivated_nu = MCTLFC_TJATVG_LimiterActivated_bool;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport191'
             */
            CSCLTA_LimiterDuration_sec = MCTLFC_TJATVG_LimiterTimeDuration_sec;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport260'
             */
            CSCLTA_MaxJerkAllowed_mps3 = MCTLFC_TJATVG_MaxJerkAllowed_mps3;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport264'
             */
            CSCLTA_DeratingLevel_nu = MCTLFC_TJATVG_DeratingLevel_nu;
            break;

        case 2:
            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport8'
             */
            CSCLTA_SysStateLCF = MCTLFC_SysStateLDP_nu;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport9'
             */
            CSCLTA_LeCridrBndPosX0_met = MCTLFC_DPLTTG_LeCridrBndPosX0_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport10'
             */
            CSCLTA_LeCridrBndPosY0_met = MCTLFC_DPLTTG_LeCridrBndPosY0_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport11'
             */
            CSCLTA_LeCridrHeadAng_rad = MCTLFC_DPLTTG_LeCridrBndHeadAng_rad;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport12'
             */
            CSCLTA_LeCridrBndCrv_1pm = MCTLFC_DPLTTG_LeCridrBndCrv_1pm;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport13'
             */
            CSCLTA_LeCridrBndCrvChng_1pm2 =
                MCTLFC_DPLTTG_LeCridrBndCrvChng_1pm2;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport14'
             */
            CSCLTA_LeCridrBndLength_met = MCTLFC_DPLTTG_LeCridrBndLength_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport15'
             */
            CSCLTA_RiCridrBndPosX0_met = MCTLFC_DPLTTG_RiCridrBndPosX0_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport16'
             */
            CSCLTA_RiCridrBndPosY0_met = MCTLFC_DPLTTG_RiCridrBndPosY0_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport17'
             */
            CSCLTA_RiCridrHeadAng_rad = MCTLFC_DPLTTG_RiCridrBndHeadAng_rad;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport18'
             */
            CSCLTA_RiCridrBndCrv_1pm = MCTLFC_DPLTTG_RiCridrBndCrv_1pm;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport19'
             */
            CSCLTA_RiCridrBndCrvChng_1pm2 =
                MCTLFC_DPLTTG_RiCridrBndCrvChng_1pm2;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport20'
             */
            CSCLTA_RiCridrBndLength_met = MCTLFC_DPLTTG_RiCridrBndLength_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport21'
             */
            CSCLTA_TgtTrajPosX0_met = MCTLFC_DPLTTG_TgtTrajPosX0_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport22'
             */
            CSCLTA_TgtTrajPosY0_met = MCTLFC_DPLTTG_TgtTrajPosY0_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport23'
             */
            CSCLTA_TgtTrajHeadAng_rad = MCTLFC_DPLTTG_TgtTrajHeadAng_rad;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport24'
             */
            CSCLTA_TgtTrajCrv_1pm = MCTLFC_DPLTTG_TgtTrajCrv_1pm;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport25'
             */
            CSCLTA_TgtTrajCrvChng_1pm2 = MCTLFC_DPLTTG_TgtTrajCrvChng_1pm2;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport26'
             */
            CSCLTA_TgtTrajLength_met = MCTLFC_DPLTTG_TgtTrajLength_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport27'
             */
            CSCLTA_TgtPlanServQu_nu = MCTLFC_DPLTVG_TrajPlanServQu_nu;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport28'
             */
            CSCLTA_WeightTgtDistY_nu = MCTLFC_DPLTVG_WeightTgtDistY_nu;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport29'
             */
            CSCLTA_WeightEndTime_nu = MCTLFC_DPLTVG_WeightEndTime_nu;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport30'
             */
            CSCLTA_DistYToILeTgtArea_met = MCTLFC_DPLTVG_DistYToLeTgtArea_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport31'
             */
            CSCLTA_DistYToIRiTgtArea_met = MCTLFC_DPLTVG_DistYToRiTgtArea_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport32'
             */
            CSCLTA_FTireAclMax_mps2 = MCTLFC_DPLTVG_FTireAclMax_mps2;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport33'
             */
            CSCLTA_FTireAclMin_mps2 = MCTLFC_DPLTVG_FTireAclMin_mps2;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport34'
             */
            CSCLTA_TrajGuiQualifier_nu = MCTLFC_DPLTVG_TrajGuiQu_nu;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport35'
             */
            CSCLTA_TriggerReplan_nu = MCTLFC_DPLTVG_TriggerReplan_bool;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport36'
             */
            CSCLTA_PredTimeHeadAng_sec = MCTLFC_DPLTVG_PredTimeHeadAng_sec;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport37'
             */
            CSCLTA_PredTimeCrv_sec = MCTLFC_DPLTVG_PredTimeCrv_sec;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport38'
             */
            CSCLTA_PlanningHorzion_sec = MCTLFC_DPLTVG_PlanningHorzion_sec;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport39'
             */
            CSCLTA_ObstacleVelX_mps = MCTLFC_DPLTVG_ObstacleVelX_mps;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport40'
             */
            CSCLTA_ObstacleAclX_mps2 = MCTLFC_DPLTVG_ObstacleAclX_mps2;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport41'
             */
            CSCLTA_ObstacleWidth_met = MCTLFC_DPLTVG_ObstacleWidth_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport42'
             */
            CSCLTA_ObstacleDistX_met = MCTLFC_DPLTVG_ObstacleDistX_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport43'
             */
            CSCLTA_ObstacleDistY_met = MCTLFC_DPLTVG_ObstacleDistY_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport44'
             */
            CSCLTA_LatencyCompActivated_bool =
                MCTLFC_DPLTVG_LtcyCompActivated_bool;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport45'
             */
            CSCLTA_SensorTStamp_sec = MCTLFC_DPLTVG_SensorTStamp_sec;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport46'
             */
            CSCLTA_MaxCrvTrajGuiCtrl_1pm = MCTLFC_DPLTVG_MaxCrvTrajGuiCtrl_1pm;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport47'
             */
            CSCLTA_MaxCrvGrdBuildup_1pms = MCTLFC_DPLTVG_MaxCrvGrdBuildup_1pms;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport48'
             */
            CSCLTA_MaxCrvGrdRed_1pms = MCTLFC_DPLTVG_MaxCrvGrdRed_1pms;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport49'
             */
            CSCLTA_GrdLimitTgtCrvGC_1pms = MCTLFC_DPLTVG_GrdLimitTgtCrvTGC_1pms;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport50'
             */
            CSCLTA_StrWhStifLimit_nu = MCTLFC_DPLTVG_StrWhStifLimit_nu;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport252'
             */
            CSCLTA_StrWhStifGrad_1ps = MCTLFC_DPLTVG_StrWhStifGrad_1ps;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport253'
             */
            CSCLTA_TrqRampGrad_1ps = MCTLFC_DPLTVG_TrqRampGrad_1ps;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport51'
             */
            CSCLTA_MaxTrqScalLimit_nu = MCTLFC_DPLTVG_MaxTrqScaILimit_nu;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport250'
             */
            CSCLTA_MaxTrqScalGrad_nu = MCTLFC_DPLTVG_MaxTrqScalGrad_1ps;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport251'
             */
            CSCLTA_HighStatAccu_bool = MCTLFC_DPLTVG_HighStatAccu_bool;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport52'
             */
            CSCLTA_LimiterActivated_nu = MCTLFC_DPLTVG_LimiterActivated_bool;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport53'
             */
            CSCLTA_LimiterDuration_sec = MCTLFC_DPLTVG_LimiterTimeDuration_sec;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport258'
             */
            CSCLTA_MaxJerkAllowed_mps3 = MCTLFC_DPLTVG_MaxJerkAllowed_mps3;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport262'
             */
            CSCLTA_DeratingLevel_nu = MCTLFC_DPLTVG_DeratingLevel_nu;
            break;

        case 3:
            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport54'
             */
            CSCLTA_SysStateLCF = MCTLFC_SysStateRDP_nu;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport55'
             */
            CSCLTA_LeCridrBndPosX0_met = MCTLFC_DPRTTG_LeCridrBndPosX0_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport56'
             */
            CSCLTA_LeCridrBndPosY0_met = MCTLFC_DPRTTG_LeCridrBndPosY0_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport57'
             */
            CSCLTA_LeCridrHeadAng_rad = MCTLFC_DPRTTG_LeCridrBndHeadAng_rad;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport58'
             */
            CSCLTA_LeCridrBndCrv_1pm = MCTLFC_DPRTTG_LeCridrBndCrv_1pm;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport59'
             */
            CSCLTA_LeCridrBndCrvChng_1pm2 =
                MCTLFC_DPRTTG_LeCridrBndCrvChng_1pm2;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport60'
             */
            CSCLTA_LeCridrBndLength_met = MCTLFC_DPRTTG_LeCridrBndLength_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport61'
             */
            CSCLTA_RiCridrBndPosX0_met = MCTLFC_DPRTTG_RiCridrBndPosX0_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport62'
             */
            CSCLTA_RiCridrBndPosY0_met = MCTLFC_DPRTTG_RiCridrBndPosY0_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport63'
             */
            CSCLTA_RiCridrHeadAng_rad = MCTLFC_DPRTTG_RiCridrBndHeadAng_rad;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport64'
             */
            CSCLTA_RiCridrBndCrv_1pm = MCTLFC_DPRTTG_RiCridrBndCrv_1pm;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport65'
             */
            CSCLTA_RiCridrBndCrvChng_1pm2 =
                MCTLFC_DPRTTG_RiCridrBndCrvChng_1pm2;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport66'
             */
            CSCLTA_RiCridrBndLength_met = MCTLFC_DPRTTG_RiCridrBndLength_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport67'
             */
            CSCLTA_TgtTrajPosX0_met = MCTLFC_DPRTTG_TgtTrajPosX0_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport68'
             */
            CSCLTA_TgtTrajPosY0_met = MCTLFC_DPRTTG_TgtTrajPosY0_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport69'
             */
            CSCLTA_TgtTrajHeadAng_rad = MCTLFC_DPRTTG_TgtTrajHeadAng_rad;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport70'
             */
            CSCLTA_TgtTrajCrv_1pm = MCTLFC_DPRTTG_TgtTrajCrv_1pm;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport71'
             */
            CSCLTA_TgtTrajCrvChng_1pm2 = MCTLFC_DPRTTG_TgtTrajCrvChng_1pm2;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport72'
             */
            CSCLTA_TgtTrajLength_met = MCTLFC_DPRTTG_TgtTrajLength_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport73'
             */
            CSCLTA_TgtPlanServQu_nu = MCTLFC_DPRTVG_TrajPlanServQu_nu;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport74'
             */
            CSCLTA_WeightTgtDistY_nu = MCTLFC_DPRTVG_WeightTgtDistY_nu;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport75'
             */
            CSCLTA_WeightEndTime_nu = MCTLFC_DPRTVG_WeightEndTime_nu;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport76'
             */
            CSCLTA_DistYToILeTgtArea_met = MCTLFC_DPRTVG_DistYToLeTgtArea_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport77'
             */
            CSCLTA_DistYToIRiTgtArea_met = MCTLFC_DPRTVG_DistYToRiTgtArea_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport78'
             */
            CSCLTA_FTireAclMax_mps2 = MCTLFC_DPRTVG_FTireAclMax_mps2;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport79'
             */
            CSCLTA_FTireAclMin_mps2 = MCTLFC_DPRTVG_FTireAclMin_mps2;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport80'
             */
            CSCLTA_TrajGuiQualifier_nu = MCTLFC_DPRTVG_TrajGuiQu_nu;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport81'
             */
            CSCLTA_TriggerReplan_nu = MCTLFC_DPRTVG_TriggerReplan_bool;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport82'
             */
            CSCLTA_PredTimeHeadAng_sec = MCTLFC_DPRTVG_PredTimeHeadAng_sec;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport83'
             */
            CSCLTA_PredTimeCrv_sec = MCTLFC_DPRTVG_PredTimeCrv_sec;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport84'
             */
            CSCLTA_PlanningHorzion_sec = MCTLFC_DPRTVG_PlanningHorzion_sec;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport85'
             */
            CSCLTA_ObstacleVelX_mps = MCTLFC_DPRTVG_ObstacleVelX_mps;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport86'
             */
            CSCLTA_ObstacleAclX_mps2 = MCTLFC_DPRTVG_ObstacleAclX_mps2;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport87'
             */
            CSCLTA_ObstacleWidth_met = MCTLFC_DPRTVG_ObstacleWidth_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport88'
             */
            CSCLTA_ObstacleDistX_met = MCTLFC_DPRTVG_ObstacleDistX_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport89'
             */
            CSCLTA_ObstacleDistY_met = MCTLFC_DPRTVG_ObstacleDistY_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport90'
             */
            CSCLTA_LatencyCompActivated_bool =
                MCTLFC_DPRTVG_LtcyCompActivated_bool;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport91'
             */
            CSCLTA_SensorTStamp_sec = MCTLFC_DPRTVG_SensorTStamp_sec;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport92'
             */
            CSCLTA_MaxCrvTrajGuiCtrl_1pm = MCTLFC_DPRTVG_MaxCrvTrajGuiCtrl_1pm;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport93'
             */
            CSCLTA_MaxCrvGrdBuildup_1pms = MCTLFC_DPRTVG_MaxCrvGrdBuildup_1pms;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport94'
             */
            CSCLTA_MaxCrvGrdRed_1pms = MCTLFC_DPRTVG_MaxCrvGrdRed_1pms;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport95'
             */
            CSCLTA_GrdLimitTgtCrvGC_1pms = MCTLFC_DPRTVG_GrdLimitTgtCrvTGC_1pms;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport96'
             */
            CSCLTA_StrWhStifLimit_nu = MCTLFC_DPRTVG_StrWhStifLimit_nu;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport248'
             */
            CSCLTA_StrWhStifGrad_1ps = MCTLFC_DPRTVG_StrWhStifGrad_1ps;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport249'
             */
            CSCLTA_TrqRampGrad_1ps = MCTLFC_DPRTVG_TrqRampGrad_1ps;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport97'
             */
            CSCLTA_MaxTrqScalLimit_nu = MCTLFC_DPRTVG_MaxTrqScaILimit_nu;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport246'
             */
            CSCLTA_MaxTrqScalGrad_nu = MCTLFC_DPRTVG_MaxTrqScalGrad_1ps;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport247'
             */
            CSCLTA_HighStatAccu_bool = MCTLFC_DPRTVG_HighStatAccu_bool;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport98'
             */
            CSCLTA_LimiterActivated_nu = MCTLFC_DPRTVG_LimiterActivated_bool;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport99'
             */
            CSCLTA_LimiterDuration_sec = MCTLFC_DPRTVG_LimiterTimeDuration_sec;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport266'
             */
            CSCLTA_MaxJerkAllowed_mps3 = MCTLFC_DPRTVG_MaxJerkAllowed_mps3;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport267'
             */
            CSCLTA_DeratingLevel_nu = MCTLFC_DPRTVG_DeratingLevel_nu;
            break;

        case 4:
            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport100'
             */
            CSCLTA_SysStateLCF = MCTLFC_SysStateLDPOC_nu;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport101'
             */
            CSCLTA_LeCridrBndPosX0_met = MCTLFC_DPOTTG_LeCridrBndPosX0_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport102'
             */
            CSCLTA_LeCridrBndPosY0_met = MCTLFC_DPOTTG_LeCridrBndPosY0_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport103'
             */
            CSCLTA_LeCridrHeadAng_rad = MCTLFC_DPOTTG_LeCridrBndHeadAng_rad;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport104'
             */
            CSCLTA_LeCridrBndCrv_1pm = MCTLFC_DPOTTG_LeCridrBndCrv_1pm;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport105'
             */
            CSCLTA_LeCridrBndCrvChng_1pm2 =
                MCTLFC_DPOTTG_LeCridrBndCrvChng_1pm2;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport106'
             */
            CSCLTA_LeCridrBndLength_met = MCTLFC_DPOTTG_LeCridrBndLength_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport107'
             */
            CSCLTA_RiCridrBndPosX0_met = MCTLFC_DPOTTG_RiCridrBndPosX0_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport108'
             */
            CSCLTA_RiCridrBndPosY0_met = MCTLFC_DPOTTG_RiCridrBndPosY0_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport109'
             */
            CSCLTA_RiCridrHeadAng_rad = MCTLFC_DPOTTG_RiCridrBndHeadAng_rad;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport110'
             */
            CSCLTA_RiCridrBndCrv_1pm = MCTLFC_DPOTTG_RiCridrBndCrv_1pm;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport111'
             */
            CSCLTA_RiCridrBndCrvChng_1pm2 =
                MCTLFC_DPOTTG_RiCridrBndCrvChng_1pm2;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport112'
             */
            CSCLTA_RiCridrBndLength_met = MCTLFC_DPOTTG_RiCridrBndLength_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport113'
             */
            CSCLTA_TgtTrajPosX0_met = MCTLFC_DPOTTG_TgtTrajPosX0_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport114'
             */
            CSCLTA_TgtTrajPosY0_met = MCTLFC_DPOTTG_TgtTrajPosY0_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport115'
             */
            CSCLTA_TgtTrajHeadAng_rad = MCTLFC_DPOTTG_TgtTrajHeadAng_rad;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport116'
             */
            CSCLTA_TgtTrajCrv_1pm = MCTLFC_DPOTTG_TgtTrajCrv_1pm;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport117'
             */
            CSCLTA_TgtTrajCrvChng_1pm2 = MCTLFC_DPOTTG_TgtTrajCrvChng_1pm2;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport118'
             */
            CSCLTA_TgtTrajLength_met = MCTLFC_DPOTTG_TgtTrajLength_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport119'
             */
            CSCLTA_TgtPlanServQu_nu = MCTLFC_DPOTVG_TrajPlanServQu_nu;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport120'
             */
            CSCLTA_WeightTgtDistY_nu = MCTLFC_DPOTVG_WeightTgtDistY_nu;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport121'
             */
            CSCLTA_WeightEndTime_nu = MCTLFC_DPOTVG_WeightEndTime_nu;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport122'
             */
            CSCLTA_DistYToILeTgtArea_met = MCTLFC_DPOTVG_DistYToLeTgtArea_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport123'
             */
            CSCLTA_DistYToIRiTgtArea_met = MCTLFC_DPOTVG_DistYToRiTgtArea_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport124'
             */
            CSCLTA_FTireAclMax_mps2 = MCTLFC_DPOTVG_FTireAclMax_mps2;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport125'
             */
            CSCLTA_FTireAclMin_mps2 = MCTLFC_DPOTVG_FTireAclMin_mps2;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport126'
             */
            CSCLTA_TrajGuiQualifier_nu = MCTLFC_DPOTVG_TrajGuiQu_nu;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport127'
             */
            CSCLTA_TriggerReplan_nu = MCTLFC_DPOTVG_TriggerReplan_bool;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport128'
             */
            CSCLTA_PredTimeHeadAng_sec = MCTLFC_DPOTVG_PredTimeHeadAng_sec;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport129'
             */
            CSCLTA_PredTimeCrv_sec = MCTLFC_DPOTVG_PredTimeCrv_sec;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport130'
             */
            CSCLTA_PlanningHorzion_sec = MCTLFC_DPOTVG_PlanningHorzion_sec;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport131'
             */
            CSCLTA_ObstacleVelX_mps = MCTLFC_DPOTVG_ObstacleVelX_mps;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport132'
             */
            CSCLTA_ObstacleAclX_mps2 = MCTLFC_DPOTVG_ObstacleAclX_mps2;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport133'
             */
            CSCLTA_ObstacleWidth_met = MCTLFC_DPOTVG_ObstacleWidth_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport134'
             */
            CSCLTA_ObstacleDistX_met = MCTLFC_DPOTVG_ObstacleDistX_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport135'
             */
            CSCLTA_ObstacleDistY_met = MCTLFC_DPOTVG_ObstacleDistY_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport136'
             */
            CSCLTA_LatencyCompActivated_bool =
                MCTLFC_DPOTVG_LtcyCompActivated_bool;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport137'
             */
            CSCLTA_SensorTStamp_sec = MCTLFC_DPOTVG_SensorTStamp_sec;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport138'
             */
            CSCLTA_MaxCrvTrajGuiCtrl_1pm = MCTLFC_DPOTVG_MaxCrvTrajGuiCtrl_1pm;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport139'
             */
            CSCLTA_MaxCrvGrdBuildup_1pms = MCTLFC_DPOTVG_MaxCrvGrdBuildup_1pms;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport140'
             */
            CSCLTA_MaxCrvGrdRed_1pms = MCTLFC_DPOTVG_MaxCrvGrdRed_1pms;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport141'
             */
            CSCLTA_GrdLimitTgtCrvGC_1pms = MCTLFC_DPOTVG_GrdLimitTgtCrvTGC_1pms;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport142'
             */
            CSCLTA_StrWhStifLimit_nu = MCTLFC_DPOTVG_StrWhStifLimit_nu;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport244'
             */
            CSCLTA_StrWhStifGrad_1ps = MCTLFC_DPOTVG_StrWhStifGrad_1ps;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport245'
             */
            CSCLTA_TrqRampGrad_1ps = MCTLFC_DPOTVG_TrqRampGrad_1ps;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport143'
             */
            CSCLTA_MaxTrqScalLimit_nu = MCTLFC_DPOTVG_MaxTrqScaILimit_nu;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport242'
             */
            CSCLTA_MaxTrqScalGrad_nu = MCTLFC_DPOTVG_MaxTrqScalGrad_1ps;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport243'
             */
            CSCLTA_HighStatAccu_bool = MCTLFC_DPOTVG_HighStatAccu_bool;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport144'
             */
            CSCLTA_LimiterActivated_nu = MCTLFC_DPOTVG_LimiterActivated_bool;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport145'
             */
            CSCLTA_LimiterDuration_sec = MCTLFC_DPOTVG_LimiterTimeDuration_sec;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport259'
             */
            CSCLTA_MaxJerkAllowed_mps3 = MCTLFC_DPOTVG_MaxJerkAllowed_mps3;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport263'
             */
            CSCLTA_DeratingLevel_nu = MCTLFC_DPOTVG_DeratingLevel_nu;
            break;

        case 5:
            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport192'
             */
            CSCLTA_SysStateLCF = MCTLFC_SysStateALCA_nu;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport193'
             */
            CSCLTA_LeCridrBndPosX0_met = MCTLFC_LCRTTG_LeCridrBndPosX0_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport194'
             */
            CSCLTA_LeCridrBndPosY0_met = MCTLFC_LCRTTG_LeCridrBndPosY0_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport195'
             */
            CSCLTA_LeCridrHeadAng_rad = MCTLFC_LCRTTG_LeCridrBndHeadAng_rad;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport196'
             */
            CSCLTA_LeCridrBndCrv_1pm = MCTLFC_LCRTTG_LeCridrBndCrv_1pm;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport197'
             */
            CSCLTA_LeCridrBndCrvChng_1pm2 =
                MCTLFC_LCRTTG_LeCridrBndCrvChng_1pm2;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport198'
             */
            CSCLTA_LeCridrBndLength_met = MCTLFC_LCRTTG_LeCridrBndLength_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport199'
             */
            CSCLTA_RiCridrBndPosX0_met = MCTLFC_LCRTTG_RiCridrBndPosX0_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport200'
             */
            CSCLTA_RiCridrBndPosY0_met = MCTLFC_LCRTTG_RiCridrBndPosY0_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport201'
             */
            CSCLTA_RiCridrHeadAng_rad = MCTLFC_LCRTTG_RiCridrBndHeadAng_rad;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport202'
             */
            CSCLTA_RiCridrBndCrv_1pm = MCTLFC_LCRTTG_RiCridrBndCrv_1pm;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport203'
             */
            CSCLTA_RiCridrBndCrvChng_1pm2 =
                MCTLFC_LCRTTG_RiCridrBndCrvChng_1pm2;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport204'
             */
            CSCLTA_RiCridrBndLength_met = MCTLFC_LCRTTG_RiCridrBndLength_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport205'
             */
            CSCLTA_TgtTrajPosX0_met = MCTLFC_LCRTTG_TgtTrajPosX0_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport206'
             */
            CSCLTA_TgtTrajPosY0_met = MCTLFC_LCRTTG_TgtTrajPosY0_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport207'
             */
            CSCLTA_TgtTrajHeadAng_rad = MCTLFC_LCRTTG_TgtTrajHeadAng_rad;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport208'
             */
            CSCLTA_TgtTrajCrv_1pm = MCTLFC_LCRTTG_TgtTrajCrv_1pm;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport209'
             */
            CSCLTA_TgtTrajCrvChng_1pm2 = MCTLFC_LCRTTG_TgtTrajCrvChng_1pm2;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport210'
             */
            CSCLTA_TgtTrajLength_met = MCTLFC_LCRTTG_TgtTrajLength_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport211'
             */
            CSCLTA_TgtPlanServQu_nu = MCTLFC_LCRTVG_TrajPlanServQu_nu;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport212'
             */
            CSCLTA_WeightTgtDistY_nu = MCTLFC_LCRTVG_WeightTgtDistY_nu;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport213'
             */
            CSCLTA_WeightEndTime_nu = MCTLFC_LCRTVG_WeightEndTime_nu;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport214'
             */
            CSCLTA_DistYToILeTgtArea_met = MCTLFC_LCRTVG_DistYToLeTgtArea_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport215'
             */
            CSCLTA_DistYToIRiTgtArea_met = MCTLFC_LCRTVG_DistYToRiTgtArea_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport216'
             */
            CSCLTA_FTireAclMax_mps2 = MCTLFC_LCRTVG_FTireAclMax_mps2;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport217'
             */
            CSCLTA_FTireAclMin_mps2 = MCTLFC_LCRTVG_FTireAclMin_mps2;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport218'
             */
            CSCLTA_TrajGuiQualifier_nu = MCTLFC_LCRTVG_TrajGuiQu_nu;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport219'
             */
            CSCLTA_TriggerReplan_nu = MCTLFC_LCRTVG_TriggerReplan_bool;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport220'
             */
            CSCLTA_PredTimeHeadAng_sec = MCTLFC_LCRTVG_PredTimeHeadAng_sec;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport221'
             */
            CSCLTA_PredTimeCrv_sec = MCTLFC_LCRTVG_PredTimeCrv_sec;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport222'
             */
            CSCLTA_PlanningHorzion_sec = MCTLFC_LCRTVG_PlanningHorzion_sec;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport223'
             */
            CSCLTA_ObstacleVelX_mps = MCTLFC_LCRTVG_ObstacleVelX_mps;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport224'
             */
            CSCLTA_ObstacleAclX_mps2 = MCTLFC_LCRTVG_ObstacleAclX_mps2;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport225'
             */
            CSCLTA_ObstacleWidth_met = MCTLFC_LCRTVG_ObstacleWidth_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport226'
             */
            CSCLTA_ObstacleDistX_met = MCTLFC_LCRTVG_ObstacleDistX_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport227'
             */
            CSCLTA_ObstacleDistY_met = MCTLFC_LCRTVG_ObstacleDistY_met;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport228'
             */
            CSCLTA_LatencyCompActivated_bool =
                MCTLFC_LCRTVG_LtcyCompActivated_bool;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport229'
             */
            CSCLTA_SensorTStamp_sec = MCTLFC_LCRTVG_SensorTStamp_sec;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport230'
             */
            CSCLTA_MaxCrvTrajGuiCtrl_1pm = MCTLFC_LCRTVG_MaxCrvTrajGuiCtrl_1pm;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport231'
             */
            CSCLTA_MaxCrvGrdBuildup_1pms = MCTLFC_LCRTVG_MaxCrvGrdBuildup_1pms;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport232'
             */
            CSCLTA_MaxCrvGrdRed_1pms = MCTLFC_LCRTVG_MaxCrvGrdRed_1pms;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport233'
             */
            CSCLTA_GrdLimitTgtCrvGC_1pms = MCTLFC_LCRTVG_GrdLimitTgtCrvTGC_1pms;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport234'
             */
            CSCLTA_StrWhStifLimit_nu = MCTLFC_LCRTVG_StrWhStifLimit_nu;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport256'
             */
            CSCLTA_StrWhStifGrad_1ps = MCTLFC_LCRTVG_StrWhStifGrad_1ps;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport257'
             */
            CSCLTA_TrqRampGrad_1ps = MCTLFC_LCRTVG_TrqRampGrad_1ps;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport235'
             */
            CSCLTA_MaxTrqScalLimit_nu = MCTLFC_LCRTVG_MaxTrqScaILimit_nu;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport254'
             */
            CSCLTA_MaxTrqScalGrad_nu = MCTLFC_LCRTVG_MaxTrqScalGrad_1ps;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport255'
             */
            CSCLTA_HighStatAccu_bool = MCTLFC_LCRTVG_HighStatAccu_bool;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport236'
             */
            CSCLTA_LimiterActivated_nu = MCTLFC_LCRTVG_LimiterActivated_bool;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport237'
             */
            CSCLTA_LimiterDuration_sec = MCTLFC_LCRTVG_LimiterTimeDuration_sec;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport261'
             */
            CSCLTA_MaxJerkAllowed_mps3 = MCTLFC_LCRTVG_MaxJerkAllowed_mps3;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Inport: '<Root>/Inport265'
             */
            CSCLTA_DeratingLevel_nu = MCTLFC_LCRTVG_DeratingLevel_nu;
            break;

        default:
            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant52'
             */
            CSCLTA_SysStateLCF = MCTLFC_E_LCF_SYSSTATE_NOTPRESENT_nu;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant53'
             */
            CSCLTA_LeCridrBndPosX0_met = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant64'
             */
            CSCLTA_LeCridrBndPosY0_met = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant75'
             */
            CSCLTA_LeCridrHeadAng_rad = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant86'
             */
            CSCLTA_LeCridrBndCrv_1pm = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant97'
             */
            CSCLTA_LeCridrBndCrvChng_1pm2 = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant100'
             */
            CSCLTA_LeCridrBndLength_met = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant101'
             */
            CSCLTA_RiCridrBndPosX0_met = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant102'
             */
            CSCLTA_RiCridrBndPosY0_met = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant103'
             */
            CSCLTA_RiCridrHeadAng_rad = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant54'
             */
            CSCLTA_RiCridrBndCrv_1pm = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant55'
             */
            CSCLTA_RiCridrBndCrvChng_1pm2 = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant56'
             */
            CSCLTA_RiCridrBndLength_met = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant57'
             */
            CSCLTA_TgtTrajPosX0_met = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant61'
             */
            CSCLTA_TgtTrajPosY0_met = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant62'
             */
            CSCLTA_TgtTrajHeadAng_rad = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant63'
             */
            CSCLTA_TgtTrajCrv_1pm = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant65'
             */
            CSCLTA_TgtTrajCrvChng_1pm2 = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant66'
             */
            CSCLTA_TgtTrajLength_met = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant67'
             */
            CSCLTA_TgtPlanServQu_nu = 0U;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant68'
             */
            CSCLTA_WeightTgtDistY_nu = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant69'
             */
            CSCLTA_WeightEndTime_nu = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant58'
             */
            CSCLTA_DistYToILeTgtArea_met = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant59'
             */
            CSCLTA_DistYToIRiTgtArea_met = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant60'
             */
            CSCLTA_FTireAclMax_mps2 = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant70'
             */
            CSCLTA_FTireAclMin_mps2 = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant71'
             */
            CSCLTA_TrajGuiQualifier_nu = MCTLFC_E_LCF_TGO_REQ_OFF;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant72'
             */
            CSCLTA_TriggerReplan_nu = false;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant84'
             */
            CSCLTA_PredTimeHeadAng_sec = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant92'
             */
            CSCLTA_PredTimeCrv_sec = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant93'
             */
            CSCLTA_PlanningHorzion_sec = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant94'
             */
            CSCLTA_ObstacleVelX_mps = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant95'
             */
            CSCLTA_ObstacleAclX_mps2 = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant96'
             */
            CSCLTA_ObstacleWidth_met = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant98'
             */
            CSCLTA_ObstacleDistX_met = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant99'
             */
            CSCLTA_ObstacleDistY_met = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant73'
             */
            CSCLTA_LatencyCompActivated_bool = false;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant74'
             */
            CSCLTA_SensorTStamp_sec = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant76'
             */
            CSCLTA_MaxCrvTrajGuiCtrl_1pm = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant77'
             */
            CSCLTA_MaxCrvGrdBuildup_1pms = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant81'
             */
            CSCLTA_MaxCrvGrdRed_1pms = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant82'
             */
            CSCLTA_GrdLimitTgtCrvGC_1pms = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant83'
             */
            CSCLTA_StrWhStifLimit_nu = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant85'
             */
            CSCLTA_StrWhStifGrad_1ps = 100.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant87'
             */
            CSCLTA_TrqRampGrad_1ps = 100.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant88'
             */
            CSCLTA_MaxTrqScalLimit_nu = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant89'
             */
            CSCLTA_MaxTrqScalGrad_nu = 100.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant90'
             */
            CSCLTA_HighStatAccu_bool = false;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant78'
             */
            CSCLTA_LimiterActivated_nu = false;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant79'
             */
            CSCLTA_LimiterDuration_sec = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant80'
             */
            CSCLTA_MaxJerkAllowed_mps3 = 0.0F;

            /* MultiPortSwitch generated from: '<S7>/Multiport Switch1'
             * incorporates:
             *  Constant: '<S15>/Constant91'
             */
            CSCLTA_DeratingLevel_nu = 0U;
            break;
    }

    /* End of MultiPortSwitch generated from: '<S7>/Multiport Switch1' */

    /* Update for UnitDelay: '<S12>/Unit Delay' */
    for (i = 0; i < 7; i++) {
        MCTLFC_DW.UnitDelay_DSTATE_c[i] = rtb_Equal3[i];
    }

    /* End of Update for UnitDelay: '<S12>/Unit Delay' */
}

/* Model initialize function */
void MCTLFC_initialize(void) {
    {
        int32_T i;

        /* InitializeConditions for UnitDelay: '<S12>/Unit Delay' */
        for (i = 0; i < 7; i++) {
            MCTLFC_DW.UnitDelay_DSTATE_c[i] = true;
        }

        /* End of InitializeConditions for UnitDelay: '<S12>/Unit Delay' */
    }
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */