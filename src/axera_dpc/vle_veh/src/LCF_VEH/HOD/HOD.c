/**********************************Model
 *Property********************************
 *
 * Company             : SENSETIME
 *
 * Model               : HOD
 *
 ************************************Auto
 *Coder**********************************
 *
 * File                             : HOD.c
 *
 * FileType                         : Code Source File
 *
 * Real-Time Workshop file version  : 9.4 (R2020b) 29-Jul-2020
 *
 * TLC version                      : 9.4 (Aug 20 2020)
 *
 * C source code generated on       : Tue Nov 23 09:45:44 2021
 *******************************************************************************/

#include "HOD.h"
#include "HOD_private.h"
#include "look1_iflf_binlxpw.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
#define CAL_START_CODE
#include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/* Exported data definition */

/* ConstVolatile memory section */
/* Definition for custom storage class: ConstVolatile */
const volatile uint16_T HOD_BtfAbuseFct_P = 1U;    /* Referenced by:
                                                    * '<S6>/IAM_Ts_P6'
                                                    * '<S11>/IAM_Ts_P6'
                                                    */
const volatile uint8_T HOD_BtfAvailableFct_P = 2U; /* Referenced by:
                                                    * '<S15>/IAM_Ts_P5'
                                                    * '<S16>/IAM_Ts_P5'
                                                    */
const volatile real32_T HOD_CoeffVelXLowPass_M[8] = {
    0.6F, 0.6F, 0.6F, 0.6F,
    0.6F, 0.6F, 0.6F, 0.6F}; /* Referenced by: '<S40>/1-D Lookup Table1' */

const volatile boolean_T HOD_ConstHandOnCnfm_P =
    0; /* Referenced by: '<S9>/IAM_Ts_P2' */
const volatile boolean_T HOD_ConstHandsOffCnfm_P =
    1; /* Referenced by: '<S9>/IAM_Ts_P4' */
const volatile uint8_T HOD_CycleFirstAbuse_P =
    1U; /* Referenced by: '<S26>/IAM_Ts_P6' */
const volatile real32_T HOD_OfstEstHandTrqNeg_P =
    0.0F; /* Referenced by: '<S40>/IAM_Ts_P1' */
const volatile real32_T HOD_OfstEstHandTrqPos_P =
    0.0F; /* Referenced by: '<S40>/IAM_Ts_P7' */
const volatile real32_T HOD_RatDrvAttentionMax_P =
    100.0F; /* Referenced by: '<S16>/IAM_Ts_P2' */
const volatile real32_T HOD_SpdLimHys_P =
    0.0F; /* Referenced by: '<S13>/IAM_Ts_P1' */
const volatile real32_T HOD_SpdLimMin_P = 0.0F; /* Referenced by:
                                                 * '<S13>/IAM_Ts_P3'
                                                 * '<S13>/IAM_Ts_P6'
                                                 */
const volatile uint8_T HOD_StDegrTrig_P =
    3U;                                             /* Referenced by:
                                                     * '<S14>/DriverAttentionStateMachine'
                                                     * '<S16>/IAM_Ts_P1'
                                                     */
const volatile uint8_T HOD_StHandsOffSuspLf_P = 3U; /* Referenced by:
                                                     * '<S49>/IAM_Ts_P3'
                                                     * '<S57>/IAM_Ts_P6'
                                                     * '<S52>/IAM_Ts_P10'
                                                     * '<S55>/IAM_Ts_P27'
                                                     */
const volatile uint8_T HOD_StHandsOffSuspRi_P = 4U; /* Referenced by:
                                                     * '<S49>/IAM_Ts_P4'
                                                     * '<S57>/IAM_Ts_P8'
                                                     * '<S52>/IAM_Ts_P12'
                                                     * '<S55>/IAM_Ts_P28'
                                                     */
const volatile uint8_T HOD_StHandsOff_P =
    5U; /* Referenced by: '<S9>/IAM_Ts_P10' */
const volatile uint8_T HOD_StHandsOnLf_P = 0U; /* Referenced by:
                                                * '<S9>/IAM_Ts_P7'
                                                * '<S54>/IAM_Ts_P15'
                                                * '<S55>/IAM_Ts_P16'
                                                */
const volatile uint8_T HOD_StHandsOnReq_P =
    1U;                                          /* Referenced by:
                                                  * '<S14>/DriverAttentionStateMachine'
                                                  * '<S15>/IAM_Ts_P2'
                                                  */
const volatile uint8_T HOD_StHandsOnRi_P = 1U;   /* Referenced by:
                                                  * '<S9>/IAM_Ts_P5'
                                                  * '<S54>/IAM_Ts_P19'
                                                  * '<S55>/IAM_Ts_P18'
                                                  */
const volatile uint8_T HOD_StHandsOnSusp_P = 2U; /* Referenced by:
                                                  * '<S49>/IAM_Ts_P5'
                                                  * '<S57>/IAM_Ts_P9'
                                                  * '<S54>/IAM_Ts_P26'
                                                  */
const volatile uint8_T HOD_StNoWarning_P = 0U;
/* Referenced by: '<S14>/DriverAttentionStateMachine' */
const volatile uint8_T HOD_StTakeOverReq_P =
    2U; /* Referenced by:
         * '<S14>/DriverAttentionStateMachine'
         * '<S15>/IAM_Ts_P4'
         */
const volatile real32_T HOD_TiConsecAbusMax_P =
    180.0F; /* Referenced by: '<S27>/IAM_Ts_P1' */
const volatile real32_T HOD_TiFirstAbuse_P =
    4.0F; /* Referenced by: '<S28>/IAM_Ts_P2' */
const volatile real32_T HOD_TiHandsOffDelta_P =
    0.14F; /* Referenced by: '<S53>/IAM_Ts_P7' */
const volatile real32_T HOD_TiHandsOffQlfMax_P = 3.0F; /* Referenced by:
                                                        * '<S41>/IAM_Ts_P1'
                                                        * '<S53>/IAM_Ts_P9'
                                                        */
const volatile real32_T HOD_TiHandsOffQlf_P = 1.5F;    /* Referenced by:
                                                        * '<S41>/IAM_Ts_P4'
                                                        * '<S52>/IAM_Ts_P23'
                                                        * '<S52>/IAM_Ts_P5'
                                                        * '<S54>/IAM_Ts_P24'
                                                        * '<S55>/IAM_Ts_P8'
                                                        */
const volatile real32_T HOD_TiHandsOnQlf_P = 0.4F;     /* Referenced by:
                                                        * '<S57>/IAM_Ts_P2'
                                                        * '<S54>/IAM_Ts_P20'
                                                        */
const volatile real32_T HOD_TiHorHandsOff_P[10] = {
    40.0F, 12.0F, 3.0F, 13.0F, 3.0F,
    0.0F,  0.0F,  0.0F, 0.0F,  0.0F}; /* Referenced by: '<S15>/IAM_Ts_P6' */

const volatile real32_T HOD_TiIncNextWarn_P =
    10.0F; /* Referenced by: '<S28>/IAM_Ts_P5' */
const volatile real32_T HOD_TiTorHandsOff_P[10] = {
    5.0F, 7.0F, 6.0F, 6.0F, 6.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F}; /* Referenced by: '<S15>/IAM_Ts_P8' */

const volatile real32_T HOD_TiTorTrigDegr_P[10] = {
    2.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F}; /* Referenced by: '<S15>/IAM_Ts_P12' */

const volatile real32_T HOD_TrqHandsOffDelta_P = 100.0F; /* Referenced by:
                                                          * '<S53>/IAM_Ts_P14'
                                                          * '<S55>/IAM_Ts_P14'
                                                          */
const volatile real32_T HOD_TrqHandsOffLim_P =
    0.15F; /* Referenced by: '<S41>/IAM_Ts_P17' */
const volatile real32_T HOD_TrqHandsOffStepLim_P =
    0.06F; /* Referenced by: '<S52>/IAM_Ts_P4' */
const volatile real32_T HOD_TrqHandsOnLimAbs_P = 0.5F; /* Referenced by:
                                                        * '<S49>/IAM_Ts_P7'
                                                        * '<S57>/IAM_Ts_P7'
                                                        */
const volatile real32_T HOD_TrqtHandsOnLim_P = 0.2F;   /* Referenced by:
                                                        * '<S42>/IAM_Ts_P1'
                                                        * '<S42>/IAM_Ts_P2'
                                                        * '<S43>/IAM_Ts_P3'
                                                        * '<S43>/IAM_Ts_P4'
                                                        */
const volatile real32_T HOD_VelXVeh_X[8] = {
    0.0F,   8.33F, 16.66F, 25.0F, 33.33F,
    41.66F, 50.0F, 58.33F}; /* Referenced by: '<S40>/1-D Lookup Table1' */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
#define CAL_STOP_CODE
#include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/* Named constants for Chart: '<S14>/DriverAttentionStateMachine' */
#define HOD_IN_HandsOnRequest ((uint8_T)1U)
#define HOD_IN_NoWarning ((uint8_T)2U)
#define HOD_IN_TakeOverRequest ((uint8_T)3U)
#define HOD_IN_TriggerDegradation ((uint8_T)4U)

/* Named constants for Chart: '<S56>/HandTorqueStateMachine1' */
#define HOD_IN_HandsOff ((uint8_T)1U)
#define HOD_IN_HandsOnLeft ((uint8_T)2U)
#define HOD_IN_HandsOnRight ((uint8_T)3U)
#define HOD_IN_SuspHandsOffLeft ((uint8_T)4U)
#define HOD_IN_SuspHandsOffRight ((uint8_T)5U)
#define HOD_IN_SuspHandsOn ((uint8_T)6U)

#define ASW_QM_CORE2_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"

/* Exported block signals */
real32_T HOD_TiTrigDegrDelay;      /* '<S15>/Selector2' */
real32_T HOD_CntEstHandTrq;        /* '<S49>/Unit Delay' */
real32_T HOD_CntHandsOffLim;       /* '<S52>/Multiport Switch9' */
real32_T HOD_CoeffVelXFlt;         /* '<S40>/1-D Lookup Table1' */
real32_T HOD_TrqActManCorrd;       /* '<S40>/Subtract' */
real32_T HOD_TrqEstHandFlt;        /* '<S44>/Add' */
real32_T HOD_TrqHandsOffLim;       /* '<S41>/Multiport Switch1' */
real32_T HOD_TiHandsOnReqDelay;    /* '<S15>/Selector1' */
real32_T HOD_TiTakeOverReqDelay;   /* '<S15>/Selector' */
real32_T HOD_RatDrvAttentionMax;   /* '<S16>/IAM_Ts_P2' */
real32_T HOD_RatDrvAttention;      /* '<S16>/Multiport Switch2' */
real32_T HOD_CntHandsOffLimInt;    /* '<S54>/Multiport Switch7' */
real32_T HOD_TrqHandsOffSns;       /* '<S55>/Multiport Switch8' */
real32_T HOD_CntHandsOffLimBig;    /* '<S53>/Multiport Switch3' */
real32_T HOD_TrqEstHandStp;        /* '<S40>/Subtract1' */
real32_T HOD_TiAbuseWarn;          /* '<S28>/Add1' */
real32_T HOD_TiDisaDrvAbuse;       /* '<S27>/Add' */
uint8_T HOD_StEstHandTrq;          /* '<S56>/HandTorqueStateMachine1' */
uint8_T HOD_StHandsOffDtct;        /* '<S9>/Multiport Switch5' */
uint8_T HOD_StSysWarning;          /* '<S14>/DriverAttentionStateMachine' */
boolean_T HOD_EnaTrigDegr;         /* '<S24>/AND' */
boolean_T HOD_EnaHandsOffExcd;     /* '<S57>/Logical Operator8' */
boolean_T HOD_EnaHandsOnExcdRi;    /* '<S57>/Logical Operator14' */
boolean_T HOD_EnaHandsOnExcdLf;    /* '<S57>/Logical Operator12' */
boolean_T HOD_EnaLeftOn;           /* '<S43>/GreaterThanOrEqual' */
boolean_T HOD_EnaLeftOff;          /* '<S43>/Logical Operator2' */
boolean_T HOD_EnaRightOn;          /* '<S42>/Less Than' */
boolean_T HOD_EnaRightOff;         /* '<S42>/Logical Operator1' */
boolean_T HOD_EnaActFct;           /* '<S15>/Equal2' */
boolean_T HOD_EnaHandsOffCnfm;     /* '<S9>/Multiport Switch1' */
boolean_T HOD_VldVehSpd;           /* '<S38>/Switch' */
boolean_T HOD_EnaHandsOn;          /* '<S15>/Logical Operator1' */
boolean_T HOD_EnaHandsOnReq;       /* '<S22>/AND' */
boolean_T HOD_EnaTakeOverReq;      /* '<S23>/AND' */
boolean_T HOD_EnaActTorqueChk;     /* '<S11>/AND' */
boolean_T HOD_HandsOffAbuseWarn;   /* '<S6>/Signal Conversion' */
boolean_T HOD_EnaRightInt;         /* '<S42>/Logical Operator' */
boolean_T HOD_EnaLeftInt;          /* '<S43>/Logical Operator3' */
boolean_T HOD_TrigAbuseWarn;       /* '<S27>/AND1' */
boolean_T HOD_DisaDrvAbuse;        /* '<S27>/OR1' */
boolean_T HOD_EnaIncTrigger;       /* '<S26>/AND5' */
boolean_T HOD_HandsOffAbuseWarnTp; /* '<S25>/OR' */

/* Block states (default storage) */
DW_HOD_T HOD_DW;

/* Definition for custom storage class: Global */
boolean_T HOD_DisaAbuseWarnRSFlipFlop;    /* '<S30>/Unit Delay' */
boolean_T HOD_DisaAbuseWarnRSFlipFlop2;   /* '<S37>/Unit Delay' */
boolean_T HOD_EnaActTorqueChkEdgeFalling; /* '<S31>/Unit Delay' */
boolean_T HOD_EnaActTorqueChkEdgeRising;  /* '<S33>/Unit Delay' */
boolean_T HOD_EnaHandsOffChkEdgeFalling;  /* '<S32>/Unit Delay' */
boolean_T HOD_EnaHandsOffChkEdgeRising;   /* '<S34>/Unit Delay' */
real32_T HOD_EnaHandsOnReqTurnOnDelay;    /* '<S22>/Unit Delay' */
real32_T HOD_EnaTakeOverReqTurnOnDelay;   /* '<S23>/Unit Delay' */
real32_T HOD_EnaTrigDegrTurnOnDelay;      /* '<S24>/Unit Delay' */
real32_T HOD_PrevCntEstHandTrq;           /* '<S49>/Unit Delay' */
real32_T HOD_PrevCounterlimitUpdate;      /* '<S52>/Unit Delay' */
real32_T HOD_PrevCycleFirstAbuse;         /* '<S26>/Unit Delay3' */
real32_T HOD_PrevFollowUpTimer;           /* '<S25>/Unit Delay2' */
uint8_T HOD_PrevStEstHandTrqLast;         /* '<S8>/Unit Delay' */
uint8_T HOD_PrevStHandsOffDtct;           /* '<S9>/Unit Delay1' */
uint8_T HOD_PrevStSysWarning;             /* '<S10>/Unit Delay' */
real32_T HOD_PrevTiAbuseWarn;             /* '<S28>/Unit Delay2' */
real32_T HOD_PrevTiDisaDrvAbuse;          /* '<S27>/Unit Delay' */
real32_T HOD_PrevTrqEstHandFlt;           /* '<S40>/Unit Delay2' */
real32_T HOD_PrevTrqEstHandFlt2;          /* '<S44>/Unit Delay' */
real32_T HOD_PrevTrqHandsOffSns;          /* '<S55>/Unit Delay' */
boolean_T HOD_TrigAbuseWarnEdgeFalling;   /* '<S29>/Unit Delay' */
boolean_T HOD_TrigAbuseWarnEdgeFalling2;  /* '<S36>/Unit Delay' */
boolean_T HOD_TrigAbuseWarnRSFlipFlop;    /* '<S35>/Unit Delay' */
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"

/* Model step function */
void HOD_step(void) {
    int32_T rtb_BitwiseOperator1_tmp;
    real32_T rtb_Abs1;
    real32_T rtb_Divide;
    real32_T tmp;
    uint8_T rtb_Switch2;
    boolean_T rtb_Equal10;
    boolean_T rtb_HOD_EnaHandsOffChk;

    /* Switch: '<S18>/Switch2' incorporates:
     *  Constant: '<S15>/IAM_Ts_P1'
     *  Inport: '<Root>/HOD_StActFctLevel'
     *  RelationalOperator: '<S18>/LowerRelop1'
     */
    if (HOD_StActFctLevel > 9) {
        rtb_Switch2 = 9U;
    } else {
        rtb_Switch2 = HOD_StActFctLevel;
    }

    /* End of Switch: '<S18>/Switch2' */

    /* Selector: '<S15>/Selector2' incorporates:
     *  Constant: '<S15>/IAM_Ts_P12'
     */
    HOD_TiTrigDegrDelay = HOD_TiTorTrigDegr_P[rtb_Switch2];

    /* SignalConversion generated from: '<S14>/DriverAttentionStateMachine'
     * incorporates: UnitDelay: '<S10>/Unit Delay'
     */
    HOD_StSysWarning = HOD_PrevStSysWarning;

    /* RelationalOperator: '<S15>/Equal1' incorporates:
     *  Constant: '<S15>/IAM_Ts_P4'
     */
    rtb_Equal10 = (HOD_StSysWarning == HOD_StTakeOverReq_P);

    /* MultiPortSwitch: '<S24>/Multiport Switch' */
    if (!rtb_Equal10) {
        /* MultiPortSwitch: '<S24>/Multiport Switch1' incorporates:
         *  UnitDelay: '<S24>/Unit Delay'
         */
        HOD_EnaTrigDegrTurnOnDelay = HOD_TiTrigDegrDelay;
    } else {
        /* MultiPortSwitch: '<S24>/Multiport Switch1' incorporates:
         *  Inport: '<Root>/HOD_CycleTime'
         *  MinMax: '<S24>/Max'
         *  Sum: '<S24>/Subtract'
         *  UnaryMinus: '<S24>/Unary Minus'
         *  UnitDelay: '<S24>/Unit Delay'
         */
        HOD_EnaTrigDegrTurnOnDelay =
            fmaxf(-HOD_CycleTime, HOD_EnaTrigDegrTurnOnDelay) - HOD_CycleTime;
    }

    /* End of MultiPortSwitch: '<S24>/Multiport Switch' */

    /* Logic: '<S24>/AND' incorporates:
     *  Inport: '<Root>/HOD_CycleTime'
     *  RelationalOperator: '<S24>/LessThanOrEqual'
     *  UnaryMinus: '<S24>/Unary Minus1'
     *  UnitDelay: '<S24>/Unit Delay'
     */
    HOD_EnaTrigDegr =
        (rtb_Equal10 && (HOD_EnaTrigDegrTurnOnDelay <= -HOD_CycleTime));

    /* UnitDelay: '<S49>/Unit Delay' */
    HOD_CntEstHandTrq = HOD_PrevCntEstHandTrq;

    /* MultiPortSwitch: '<S52>/Multiport Switch9' incorporates:
     *  Constant: '<S52>/IAM_Ts_P17'
     *  RelationalOperator: '<S52>/Equal6'
     *  UnitDelay: '<S52>/Unit Delay'
     */
    if (HOD_PrevCounterlimitUpdate != 0.0F) {
        /* MultiPortSwitch: '<S52>/Multiport Switch9' */
        HOD_CntHandsOffLim = HOD_PrevCounterlimitUpdate;
    } else {
        /* MultiPortSwitch: '<S52>/Multiport Switch9' incorporates:
         *  Constant: '<S52>/IAM_Ts_P23'
         *  Inport: '<Root>/HOD_CycleTime'
         *  Product: '<S52>/Divide3'
         */
        HOD_CntHandsOffLim = HOD_TiHandsOffQlf_P / HOD_CycleTime;
    }

    /* End of MultiPortSwitch: '<S52>/Multiport Switch9' */

    /* Logic: '<S57>/Logical Operator8' incorporates:
     *  Constant: '<S57>/IAM_Ts_P6'
     *  Constant: '<S57>/IAM_Ts_P8'
     *  Logic: '<S57>/Logical Operator2'
     *  RelationalOperator: '<S57>/Equal'
     *  RelationalOperator: '<S57>/Equal1'
     *  RelationalOperator: '<S57>/GreaterThanOrEqual'
     *  UnitDelay: '<S8>/Unit Delay'
     */
    HOD_EnaHandsOffExcd =
        (((HOD_PrevStEstHandTrqLast == HOD_StHandsOffSuspLf_P) ||
          (HOD_PrevStEstHandTrqLast == HOD_StHandsOffSuspRi_P)) &&
         (HOD_CntEstHandTrq >= HOD_CntHandsOffLim));

    /* Sum: '<S44>/Add' incorporates:
     *  UnitDelay: '<S44>/Unit Delay'
     */
    HOD_TrqEstHandFlt = HOD_PrevTrqEstHandFlt2;

    /* Lookup_n-D: '<S40>/1-D Lookup Table1' incorporates:
     *  Inport: '<Root>/HOD_VelXVeh'
     */
    HOD_CoeffVelXFlt = look1_iflf_binlxpw(
        HOD_VelXVeh, ((const real32_T *)&(HOD_VelXVeh_X[0])),
        ((const real32_T *)&(HOD_CoeffVelXLowPass_M[0])), 7U);

    /* Product: '<S44>/Divide' incorporates:
     *  Inport: '<Root>/HOD_CycleTime'
     *  MinMax: '<S44>/Max1'
     *  Product: '<S40>/Divide'
     */
    rtb_Divide =
        HOD_CycleTime / fmaxf(HOD_CycleTime, HOD_CycleTime / HOD_CoeffVelXFlt);

    /* Switch: '<S45>/Switch2' incorporates:
     *  Constant: '<S40>/IAM_Ts_P1'
     *  Constant: '<S40>/IAM_Ts_P7'
     *  Inport: '<Root>/HOD_TrqActMan'
     *  RelationalOperator: '<S45>/LowerRelop1'
     *  RelationalOperator: '<S45>/UpperRelop'
     *  Switch: '<S45>/Switch'
     */
    if (HOD_TrqActMan > HOD_OfstEstHandTrqPos_P) {
        tmp = HOD_OfstEstHandTrqPos_P;
    } else if (HOD_TrqActMan < HOD_OfstEstHandTrqNeg_P) {
        /* Switch: '<S45>/Switch' incorporates:
         *  Constant: '<S40>/IAM_Ts_P1'
         */
        tmp = HOD_OfstEstHandTrqNeg_P;
    } else {
        tmp = HOD_TrqActMan;
    }

    /* End of Switch: '<S45>/Switch2' */

    /* Sum: '<S40>/Subtract' incorporates:
     *  Inport: '<Root>/HOD_TrqActMan'
     */
    HOD_TrqActManCorrd = HOD_TrqActMan - tmp;

    /* Switch: '<S46>/Switch2' incorporates:
     *  Constant: '<S44>/IAM_Ts_P1'
     *  Constant: '<S44>/IAM_Ts_P4'
     *  RelationalOperator: '<S46>/LowerRelop1'
     *  RelationalOperator: '<S46>/UpperRelop'
     *  Switch: '<S46>/Switch'
     */
    if (rtb_Divide > 1.0F) {
        rtb_Divide = 1.0F;
    } else {
        if (rtb_Divide < 0.0F) {
            /* Switch: '<S46>/Switch' incorporates:
             *  Constant: '<S44>/IAM_Ts_P1'
             */
            rtb_Divide = 0.0F;
        }
    }

    /* End of Switch: '<S46>/Switch2' */

    /* Sum: '<S44>/Add' incorporates:
     *  Product: '<S44>/Product'
     *  Sum: '<S44>/Subtract'
     */
    HOD_TrqEstHandFlt += (HOD_TrqActManCorrd - HOD_TrqEstHandFlt) * rtb_Divide;

    /* Abs: '<S57>/Abs' incorporates:
     *  Abs: '<S49>/Abs'
     */
    rtb_Divide = fabsf(HOD_TrqEstHandFlt);

    /* Product: '<S57>/Divide3' incorporates:
     *  Constant: '<S57>/IAM_Ts_P2'
     *  Inport: '<Root>/HOD_CycleTime'
     *  MultiPortSwitch: '<S54>/Multiport Switch6'
     *  MultiPortSwitch: '<S54>/Multiport Switch7'
     *  Product: '<S54>/Divide6'
     */
    HOD_CntHandsOffLimInt = HOD_TiHandsOnQlf_P / HOD_CycleTime;

    /* Logic: '<S57>/Logical Operator11' incorporates:
     *  Abs: '<S57>/Abs'
     *  Constant: '<S57>/IAM_Ts_P7'
     *  Constant: '<S57>/IAM_Ts_P9'
     *  Logic: '<S57>/OR'
     *  Product: '<S57>/Divide3'
     *  RelationalOperator: '<S57>/Equal3'
     *  RelationalOperator: '<S57>/GreaterThan'
     *  RelationalOperator: '<S57>/GreaterThanOrEqual1'
     *  UnitDelay: '<S8>/Unit Delay'
     */
    rtb_Equal10 = ((HOD_PrevStEstHandTrqLast == HOD_StHandsOnSusp_P) &&
                   ((HOD_CntEstHandTrq >= HOD_CntHandsOffLimInt) ||
                    (rtb_Divide > HOD_TrqHandsOnLimAbs_P)));

    /* Logic: '<S57>/Logical Operator14' incorporates:
     *  Constant: '<S57>/IAM_Ts_P10'
     *  RelationalOperator: '<S57>/GreaterThan2'
     */
    HOD_EnaHandsOnExcdRi = (rtb_Equal10 && (HOD_TrqEstHandFlt < 0.0F));

    /* Logic: '<S57>/Logical Operator12' incorporates:
     *  Logic: '<S57>/Logical Operator1'
     */
    HOD_EnaHandsOnExcdLf = (rtb_Equal10 && (!HOD_EnaHandsOnExcdRi));

    /* RelationalOperator: '<S43>/GreaterThanOrEqual' incorporates:
     *  Constant: '<S43>/IAM_Ts_P4'
     */
    HOD_EnaLeftOn = (HOD_TrqEstHandFlt >= HOD_TrqtHandsOnLim_P);

    /* Product: '<S41>/Divide3' incorporates:
     *  Constant: '<S41>/IAM_Ts_P1'
     *  Inport: '<Root>/HOD_CycleTime'
     *  Product: '<S53>/Divide1'
     */
    tmp = HOD_TiHandsOffQlfMax_P / HOD_CycleTime;

    /* MultiPortSwitch: '<S41>/Multiport Switch1' incorporates:
     *  Constant: '<S41>/IAM_Ts_P11'
     *  Product: '<S41>/Divide'
     *  Product: '<S41>/Divide3'
     *  RelationalOperator: '<S41>/GreaterThan'
     */
    if (HOD_CntEstHandTrq <= tmp / 2.0F) {
        /* MultiPortSwitch: '<S41>/Multiport Switch2' incorporates:
         *  Constant: '<S41>/IAM_Ts_P4'
         *  Inport: '<Root>/HOD_CycleTime'
         *  Product: '<S41>/Divide4'
         *  RelationalOperator: '<S41>/GreaterThan1'
         */
        if (HOD_CntEstHandTrq <= HOD_TiHandsOffQlf_P / HOD_CycleTime) {
            /* MultiPortSwitch: '<S41>/Multiport Switch1' incorporates:
             *  Constant: '<S41>/IAM_Ts_P17'
             */
            HOD_TrqHandsOffLim = HOD_TrqHandsOffLim_P;
        } else {
            /* MultiPortSwitch: '<S41>/Multiport Switch1' incorporates:
             *  Constant: '<S41>/IAM_Ts_P13'
             *  Constant: '<S41>/IAM_Ts_P17'
             *  Product: '<S41>/Divide2'
             */
            HOD_TrqHandsOffLim = HOD_TrqHandsOffLim_P / 2.0F;
        }

        /* End of MultiPortSwitch: '<S41>/Multiport Switch2' */
    } else {
        /* MultiPortSwitch: '<S41>/Multiport Switch1' incorporates:
         *  Constant: '<S41>/IAM_Ts_P12'
         *  Constant: '<S41>/IAM_Ts_P17'
         *  Product: '<S41>/Divide1'
         */
        HOD_TrqHandsOffLim = HOD_TrqHandsOffLim_P / 4.0F;
    }

    /* End of MultiPortSwitch: '<S41>/Multiport Switch1' */

    /* Logic: '<S43>/Logical Operator2' incorporates:
     *  Constant: '<S43>/IAM_Ts_P5'
     *  RelationalOperator: '<S43>/GreaterThan2'
     *  RelationalOperator: '<S43>/LessThanOrEqual2'
     */
    HOD_EnaLeftOff = ((HOD_TrqHandsOffLim >= HOD_TrqEstHandFlt) &&
                      (HOD_TrqEstHandFlt > 0.0F));

    /* RelationalOperator: '<S42>/Less Than' incorporates:
     *  Constant: '<S42>/IAM_Ts_P1'
     *  UnaryMinus: '<S42>/Unary Minus1'
     */
    HOD_EnaRightOn = (HOD_TrqEstHandFlt <= -HOD_TrqtHandsOnLim_P);

    /* Logic: '<S42>/Logical Operator1' incorporates:
     *  Constant: '<S42>/IAM_Ts_P11'
     *  RelationalOperator: '<S42>/GreaterThan1'
     *  RelationalOperator: '<S42>/LessThanOrEqual1'
     *  UnaryMinus: '<S42>/Unary Minus2'
     */
    HOD_EnaRightOff = ((-HOD_TrqHandsOffLim < HOD_TrqEstHandFlt) &&
                       (HOD_TrqEstHandFlt <= 0.0F));

    /* Chart: '<S56>/HandTorqueStateMachine1' */
    if (HOD_DW.is_active_c4_HOD == 0U) {
        HOD_DW.is_active_c4_HOD = 1U;
        HOD_DW.is_c4_HOD = HOD_IN_HandsOff;

        /* SignalConversion generated from: '<S56>/HandTorqueStateMachine1' */
        HOD_StEstHandTrq = 5U;
    } else {
        switch (HOD_DW.is_c4_HOD) {
            case HOD_IN_HandsOff:
                /* SignalConversion generated from:
                 * '<S56>/HandTorqueStateMachine1' */
                HOD_StEstHandTrq = 5U;
                if (((!HOD_EnaLeftOff) || (!HOD_EnaRightOff)) &&
                    (HOD_EnaLeftOn || HOD_EnaRightOn)) {
                    HOD_DW.is_c4_HOD = HOD_IN_SuspHandsOn;

                    /* SignalConversion generated from:
                     * '<S56>/HandTorqueStateMachine1' */
                    HOD_StEstHandTrq = 2U;
                }
                break;

            case HOD_IN_HandsOnLeft:
                /* SignalConversion generated from:
                 * '<S56>/HandTorqueStateMachine1' */
                HOD_StEstHandTrq = 0U;
                if (HOD_EnaRightOn) {
                    HOD_DW.is_c4_HOD = HOD_IN_HandsOnRight;

                    /* SignalConversion generated from:
                     * '<S56>/HandTorqueStateMachine1' */
                    HOD_StEstHandTrq = 1U;
                } else {
                    if (HOD_EnaLeftOff || HOD_EnaRightOff) {
                        HOD_DW.is_c4_HOD = HOD_IN_SuspHandsOffLeft;

                        /* SignalConversion generated from:
                         * '<S56>/HandTorqueStateMachine1' */
                        HOD_StEstHandTrq = 3U;
                    }
                }
                break;

            case HOD_IN_HandsOnRight:
                /* SignalConversion generated from:
                 * '<S56>/HandTorqueStateMachine1' */
                HOD_StEstHandTrq = 1U;
                if (HOD_EnaLeftOn) {
                    HOD_DW.is_c4_HOD = HOD_IN_HandsOnLeft;

                    /* SignalConversion generated from:
                     * '<S56>/HandTorqueStateMachine1' */
                    HOD_StEstHandTrq = 0U;
                } else {
                    if (HOD_EnaLeftOff || HOD_EnaRightOff) {
                        HOD_DW.is_c4_HOD = HOD_IN_SuspHandsOffRight;

                        /* SignalConversion generated from:
                         * '<S56>/HandTorqueStateMachine1' */
                        HOD_StEstHandTrq = 4U;
                    }
                }
                break;

            case HOD_IN_SuspHandsOffLeft:
                /* SignalConversion generated from:
                 * '<S56>/HandTorqueStateMachine1' */
                HOD_StEstHandTrq = 3U;
                if (HOD_EnaRightOn) {
                    HOD_DW.is_c4_HOD = HOD_IN_HandsOnRight;

                    /* SignalConversion generated from:
                     * '<S56>/HandTorqueStateMachine1' */
                    HOD_StEstHandTrq = 1U;
                } else if (HOD_EnaLeftOn) {
                    HOD_DW.is_c4_HOD = HOD_IN_HandsOnLeft;

                    /* SignalConversion generated from:
                     * '<S56>/HandTorqueStateMachine1' */
                    HOD_StEstHandTrq = 0U;
                } else {
                    if (HOD_EnaHandsOffExcd) {
                        HOD_DW.is_c4_HOD = HOD_IN_HandsOff;

                        /* SignalConversion generated from:
                         * '<S56>/HandTorqueStateMachine1' */
                        HOD_StEstHandTrq = 5U;
                    }
                }
                break;

            case HOD_IN_SuspHandsOffRight:
                /* SignalConversion generated from:
                 * '<S56>/HandTorqueStateMachine1' */
                HOD_StEstHandTrq = 4U;
                if (HOD_EnaLeftOn) {
                    HOD_DW.is_c4_HOD = HOD_IN_HandsOnLeft;

                    /* SignalConversion generated from:
                     * '<S56>/HandTorqueStateMachine1' */
                    HOD_StEstHandTrq = 0U;
                } else if (HOD_EnaRightOn) {
                    HOD_DW.is_c4_HOD = HOD_IN_HandsOnRight;

                    /* SignalConversion generated from:
                     * '<S56>/HandTorqueStateMachine1' */
                    HOD_StEstHandTrq = 1U;
                } else {
                    if (HOD_EnaHandsOffExcd) {
                        HOD_DW.is_c4_HOD = HOD_IN_HandsOff;

                        /* SignalConversion generated from:
                         * '<S56>/HandTorqueStateMachine1' */
                        HOD_StEstHandTrq = 5U;
                    }
                }
                break;

            default:
                /* SignalConversion generated from:
                 * '<S56>/HandTorqueStateMachine1' */
                /* case IN_SuspHandsOn: */
                HOD_StEstHandTrq = 2U;
                if (HOD_EnaLeftOff || HOD_EnaRightOff) {
                    HOD_DW.is_c4_HOD = HOD_IN_HandsOff;

                    /* SignalConversion generated from:
                     * '<S56>/HandTorqueStateMachine1' */
                    HOD_StEstHandTrq = 5U;
                } else if (HOD_EnaHandsOnExcdLf) {
                    HOD_DW.is_c4_HOD = HOD_IN_HandsOnLeft;

                    /* SignalConversion generated from:
                     * '<S56>/HandTorqueStateMachine1' */
                    HOD_StEstHandTrq = 0U;
                } else {
                    if (HOD_EnaHandsOnExcdRi) {
                        HOD_DW.is_c4_HOD = HOD_IN_HandsOnRight;

                        /* SignalConversion generated from:
                         * '<S56>/HandTorqueStateMachine1' */
                        HOD_StEstHandTrq = 1U;
                    }
                }
                break;
        }
    }

    /* End of Chart: '<S56>/HandTorqueStateMachine1' */

    /* S-Function (sfix_bitop): '<S15>/Bitwise AND' incorporates:
     *  Constant: '<S15>/IAM_Ts_P5'
     *  Inport: '<Root>/HOD_BtfAvailableFun'
     *  S-Function (sfix_bitop): '<S16>/Bitwise AND'
     */
    rtb_BitwiseOperator1_tmp = HOD_BtfAvailableFct & HOD_BtfAvailableFct_P;

    /* RelationalOperator: '<S15>/Equal2' incorporates:
     *  Constant: '<S15>/IAM_Ts_P11'
     *  S-Function (sfix_bitop): '<S15>/Bitwise AND'
     */
    HOD_EnaActFct = (0U != (uint8_T)rtb_BitwiseOperator1_tmp);

    /* MultiPortSwitch: '<S9>/Multiport Switch5' incorporates:
     *  Constant: '<S9>/IAM_Ts_P5'
     *  Constant: '<S9>/IAM_Ts_P7'
     *  Constant: '<S9>/IAM_Ts_P9'
     *  Logic: '<S9>/Logical Operator'
     *  MultiPortSwitch: '<S9>/Multiport Switch'
     *  RelationalOperator: '<S9>/Equal'
     *  RelationalOperator: '<S9>/Equal1'
     *  RelationalOperator: '<S9>/Equal5'
     *  UnitDelay: '<S9>/Unit Delay1'
     */
    if (HOD_PrevStHandsOffDtct != 3) {
        /* MultiPortSwitch: '<S9>/Multiport Switch2' incorporates:
         *  Constant: '<S9>/IAM_Ts_P10'
         *  RelationalOperator: '<S9>/Equal4'
         */
        if (HOD_StEstHandTrq != HOD_StHandsOff_P) {
            /* MultiPortSwitch: '<S9>/Multiport Switch5' */
            HOD_StHandsOffDtct = HOD_PrevStHandsOffDtct;
        } else {
            /* MultiPortSwitch: '<S9>/Multiport Switch5' incorporates:
             *  Constant: '<S9>/IAM_Ts_P14'
             */
            HOD_StHandsOffDtct = 3U;
        }

        /* End of MultiPortSwitch: '<S9>/Multiport Switch2' */
    } else if ((HOD_StEstHandTrq != HOD_StHandsOnLf_P) &&
               (HOD_StEstHandTrq != HOD_StHandsOnRi_P)) {
        /* MultiPortSwitch: '<S9>/Multiport Switch' incorporates:
         *  MultiPortSwitch: '<S9>/Multiport Switch5'
         */
        HOD_StHandsOffDtct = HOD_PrevStHandsOffDtct;
    } else {
        /* MultiPortSwitch: '<S9>/Multiport Switch5' incorporates:
         *  Constant: '<S9>/IAM_Ts_P6'
         *  MultiPortSwitch: '<S9>/Multiport Switch'
         */
        HOD_StHandsOffDtct = 0U;
    }

    /* End of MultiPortSwitch: '<S9>/Multiport Switch5' */

    /* MultiPortSwitch: '<S9>/Multiport Switch1' incorporates:
     *  Constant: '<S9>/IAM_Ts_P3'
     *  RelationalOperator: '<S9>/Equal2'
     */
    if (HOD_StHandsOffDtct != 0) {
        /* MultiPortSwitch: '<S9>/Multiport Switch1' incorporates:
         *  Constant: '<S9>/IAM_Ts_P4'
         */
        HOD_EnaHandsOffCnfm = HOD_ConstHandsOffCnfm_P;
    } else {
        /* MultiPortSwitch: '<S9>/Multiport Switch1' incorporates:
         *  Constant: '<S9>/IAM_Ts_P2'
         */
        HOD_EnaHandsOffCnfm = HOD_ConstHandOnCnfm_P;
    }

    /* End of MultiPortSwitch: '<S9>/Multiport Switch1' */

    /* Abs: '<S13>/Abs1' incorporates:
     *  Inport: '<Root>/HOD_VelXVeh'
     */
    rtb_Abs1 = fabsf(HOD_VelXVeh);

    /* Switch: '<S38>/Switch' incorporates:
     *  Constant: '<S13>/IAM_Ts_P6'
     *  RelationalOperator: '<S38>/Less Than'
     */
    if (HOD_SpdLimMin_P < rtb_Abs1) {
        /* Switch: '<S38>/Switch' incorporates:
         *  Constant: '<S38>/Constant'
         */
        HOD_VldVehSpd = true;
    } else {
        /* Switch: '<S38>/Switch' incorporates:
         *  Constant: '<S13>/IAM_Ts_P1'
         *  Constant: '<S13>/IAM_Ts_P3'
         *  RelationalOperator: '<S38>/Less Than1'
         *  Sum: '<S13>/Subtract1'
         *  UnitDelay: '<S38>/Unit Delay'
         */
        HOD_VldVehSpd =
            ((rtb_Abs1 >= HOD_SpdLimMin_P - HOD_SpdLimHys_P) && HOD_VldVehSpd);
    }

    /* End of Switch: '<S38>/Switch' */

    /* Switch: '<S13>/Switch' */
    rtb_HOD_EnaHandsOffChk = (HOD_VldVehSpd && HOD_EnaHandsOffCnfm);

    /* Outputs for Enabled SubSystem: '<S6>/DriverAbuseMonitoring' incorporates:
     *  EnablePort: '<S12>/Enable'
     */
    /* Logic: '<S15>/Logical Operator1' incorporates:
     *  Logic: '<S27>/Logical Operator1'
     *  Logic: '<S32>/NOT'
     */
    HOD_EnaHandsOn = !rtb_HOD_EnaHandsOffChk;

    /* End of Outputs for SubSystem: '<S6>/DriverAbuseMonitoring' */

    /* Logic: '<S15>/AND' incorporates:
     *  S-Function (sfix_bitop): '<S15>/Bitwise AND'
     */
    rtb_Equal10 =
        (rtb_HOD_EnaHandsOffChk && ((uint8_T)rtb_BitwiseOperator1_tmp != 0));

    /* Selector: '<S15>/Selector1' incorporates:
     *  Constant: '<S15>/IAM_Ts_P6'
     */
    HOD_TiHandsOnReqDelay = HOD_TiHorHandsOff_P[rtb_Switch2];

    /* MultiPortSwitch: '<S22>/Multiport Switch' */
    if (!rtb_Equal10) {
        /* MultiPortSwitch: '<S22>/Multiport Switch1' incorporates:
         *  UnitDelay: '<S22>/Unit Delay'
         */
        HOD_EnaHandsOnReqTurnOnDelay = HOD_TiHandsOnReqDelay;
    } else {
        /* MultiPortSwitch: '<S22>/Multiport Switch1' incorporates:
         *  Inport: '<Root>/HOD_CycleTime'
         *  MinMax: '<S22>/Max'
         *  Sum: '<S22>/Subtract'
         *  UnaryMinus: '<S22>/Unary Minus'
         *  UnitDelay: '<S22>/Unit Delay'
         */
        HOD_EnaHandsOnReqTurnOnDelay =
            fmaxf(-HOD_CycleTime, HOD_EnaHandsOnReqTurnOnDelay) - HOD_CycleTime;
    }

    /* End of MultiPortSwitch: '<S22>/Multiport Switch' */

    /* Logic: '<S22>/AND' incorporates:
     *  Inport: '<Root>/HOD_CycleTime'
     *  RelationalOperator: '<S22>/LessThanOrEqual'
     *  UnaryMinus: '<S22>/Unary Minus1'
     *  UnitDelay: '<S22>/Unit Delay'
     */
    HOD_EnaHandsOnReq =
        (rtb_Equal10 && (HOD_EnaHandsOnReqTurnOnDelay <= -HOD_CycleTime));

    /* RelationalOperator: '<S15>/Equal' incorporates:
     *  Constant: '<S15>/IAM_Ts_P2'
     */
    rtb_Equal10 = (HOD_StSysWarning == HOD_StHandsOnReq_P);

    /* Selector: '<S15>/Selector' incorporates:
     *  Constant: '<S15>/IAM_Ts_P8'
     */
    HOD_TiTakeOverReqDelay = HOD_TiTorHandsOff_P[rtb_Switch2];

    /* MultiPortSwitch: '<S23>/Multiport Switch' */
    if (!rtb_Equal10) {
        /* MultiPortSwitch: '<S23>/Multiport Switch1' incorporates:
         *  UnitDelay: '<S23>/Unit Delay'
         */
        HOD_EnaTakeOverReqTurnOnDelay = HOD_TiTakeOverReqDelay;
    } else {
        /* MultiPortSwitch: '<S23>/Multiport Switch1' incorporates:
         *  Inport: '<Root>/HOD_CycleTime'
         *  MinMax: '<S23>/Max'
         *  Sum: '<S23>/Subtract'
         *  UnaryMinus: '<S23>/Unary Minus'
         *  UnitDelay: '<S23>/Unit Delay'
         */
        HOD_EnaTakeOverReqTurnOnDelay =
            fmaxf(-HOD_CycleTime, HOD_EnaTakeOverReqTurnOnDelay) -
            HOD_CycleTime;
    }

    /* End of MultiPortSwitch: '<S23>/Multiport Switch' */

    /* Logic: '<S23>/AND' incorporates:
     *  Inport: '<Root>/HOD_CycleTime'
     *  RelationalOperator: '<S23>/LessThanOrEqual'
     *  UnaryMinus: '<S23>/Unary Minus1'
     *  UnitDelay: '<S23>/Unit Delay'
     */
    HOD_EnaTakeOverReq =
        (rtb_Equal10 && (HOD_EnaTakeOverReqTurnOnDelay <= -HOD_CycleTime));

    /* Chart: '<S14>/DriverAttentionStateMachine' */
    if (HOD_DW.is_active_c3_HOD == 0U) {
        HOD_DW.is_active_c3_HOD = 1U;
        HOD_DW.is_c3_HOD = HOD_IN_NoWarning;

        /* SignalConversion generated from: '<S14>/DriverAttentionStateMachine'
         */
        HOD_StSysWarning = HOD_StNoWarning_P;
    } else {
        switch (HOD_DW.is_c3_HOD) {
            case HOD_IN_HandsOnRequest:
                /* SignalConversion generated from:
                 * '<S14>/DriverAttentionStateMachine' */
                HOD_StSysWarning = HOD_StHandsOnReq_P;
                if ((!HOD_EnaActFct) || HOD_EnaHandsOn) {
                    HOD_DW.is_c3_HOD = HOD_IN_NoWarning;

                    /* SignalConversion generated from:
                     * '<S14>/DriverAttentionStateMachine' */
                    HOD_StSysWarning = HOD_StNoWarning_P;
                } else {
                    rtb_Equal10 = (HOD_EnaTakeOverReq && (!HOD_EnaHandsOn));
                    if (rtb_Equal10 && (HOD_TiTrigDegrDelay == 0.0F)) {
                        HOD_DW.is_c3_HOD = HOD_IN_TriggerDegradation;

                        /* SignalConversion generated from:
                         * '<S14>/DriverAttentionStateMachine' */
                        HOD_StSysWarning = HOD_StDegrTrig_P;
                    } else {
                        if (rtb_Equal10 && (HOD_TiTrigDegrDelay > 0.0F)) {
                            HOD_DW.is_c3_HOD = HOD_IN_TakeOverRequest;

                            /* SignalConversion generated from:
                             * '<S14>/DriverAttentionStateMachine' */
                            HOD_StSysWarning = HOD_StTakeOverReq_P;
                        }
                    }
                }
                break;

            case HOD_IN_NoWarning:
                /* SignalConversion generated from:
                 * '<S14>/DriverAttentionStateMachine' */
                HOD_StSysWarning = HOD_StNoWarning_P;
                if (HOD_EnaHandsOnReq && HOD_EnaActFct && (!HOD_EnaHandsOn)) {
                    HOD_DW.is_c3_HOD = HOD_IN_HandsOnRequest;

                    /* SignalConversion generated from:
                     * '<S14>/DriverAttentionStateMachine' */
                    HOD_StSysWarning = HOD_StHandsOnReq_P;
                }
                break;

            case HOD_IN_TakeOverRequest:
                /* SignalConversion generated from:
                 * '<S14>/DriverAttentionStateMachine' */
                HOD_StSysWarning = HOD_StTakeOverReq_P;
                if ((!HOD_EnaActFct) || HOD_EnaHandsOn) {
                    HOD_DW.is_c3_HOD = HOD_IN_NoWarning;

                    /* SignalConversion generated from:
                     * '<S14>/DriverAttentionStateMachine' */
                    HOD_StSysWarning = HOD_StNoWarning_P;
                } else {
                    if (HOD_EnaTrigDegr && (!HOD_EnaHandsOn)) {
                        HOD_DW.is_c3_HOD = HOD_IN_TriggerDegradation;

                        /* SignalConversion generated from:
                         * '<S14>/DriverAttentionStateMachine' */
                        HOD_StSysWarning = HOD_StDegrTrig_P;
                    }
                }
                break;

            default:
                /* SignalConversion generated from:
                 * '<S14>/DriverAttentionStateMachine' */
                /* case IN_TriggerDegradation: */
                HOD_StSysWarning = HOD_StDegrTrig_P;
                if ((!HOD_EnaActFct) || HOD_EnaHandsOn) {
                    HOD_DW.is_c3_HOD = HOD_IN_NoWarning;

                    /* SignalConversion generated from:
                     * '<S14>/DriverAttentionStateMachine' */
                    HOD_StSysWarning = HOD_StNoWarning_P;
                }
                break;
        }
    }

    /* End of Chart: '<S14>/DriverAttentionStateMachine' */

    /* SignalConversion generated from: '<S16>/IAM_Ts_P2' incorporates:
     *  Constant: '<S16>/IAM_Ts_P2'
     */
    HOD_RatDrvAttentionMax = HOD_RatDrvAttentionMax_P;

    /* MultiPortSwitch: '<S16>/Multiport Switch1' incorporates:
     *  Constant: '<S16>/IAM_Ts_P1'
     *  Constant: '<S16>/IAM_Ts_P3'
     *  RelationalOperator: '<S16>/Equal'
     *  RelationalOperator: '<S16>/Equal1'
     */
    if (rtb_BitwiseOperator1_tmp != 0) {
        rtb_BitwiseOperator1_tmp = rtb_HOD_EnaHandsOffChk;
    } else {
        rtb_BitwiseOperator1_tmp = (HOD_StSysWarning == HOD_StDegrTrig_P);
    }

    /* End of MultiPortSwitch: '<S16>/Multiport Switch1' */

    /* MultiPortSwitch: '<S16>/Multiport Switch2' */
    if (rtb_BitwiseOperator1_tmp == 0) {
        /* MultiPortSwitch: '<S16>/Multiport Switch2' */
        HOD_RatDrvAttention = HOD_RatDrvAttentionMax;
    } else {
        /* MultiPortSwitch: '<S16>/Multiport Switch2' incorporates:
         *  Constant: '<S16>/IAM_Ts_P6'
         */
        HOD_RatDrvAttention = -100.0F;
    }

    /* End of MultiPortSwitch: '<S16>/Multiport Switch2' */

    /* Logic: '<S11>/AND' incorporates:
     *  Constant: '<S11>/IAM_Ts_P6'
     *  Inport: '<Root>/HOD_BtfControlFct'
     *  Inport: '<Root>/HOD_EnaActTorque'
     *  RelationalOperator: '<S11>/GreaterThan'
     *  S-Function (sfix_bitop): '<S11>/Bitwise Operator'
     *  S-Function (sfix_bitop): '<S11>/Bitwise Operator1'
     */
    HOD_EnaActTorqueChk =
        (HOD_EnaActTorque && ((HOD_BtfControlFct & 1 & HOD_BtfAbuseFct_P) > 0));

    /* Outputs for Enabled SubSystem: '<S6>/DriverAbuseMonitoring' incorporates:
     *  EnablePort: '<S12>/Enable'
     */
    /* Constant: '<S6>/IAM_Ts_P6' */
    if (HOD_BtfAbuseFct_P > 0) {
        /* Switch: '<S35>/Switch' incorporates:
         *  Constant: '<S35>/Constant2'
         *  Logic: '<S27>/OR'
         *  Logic: '<S31>/AND'
         *  Logic: '<S31>/NOT'
         *  Logic: '<S32>/AND'
         *  Logic: '<S33>/AND'
         *  Logic: '<S33>/NOT'
         *  UnitDelay: '<S31>/Unit Delay'
         *  UnitDelay: '<S32>/Unit Delay'
         *  UnitDelay: '<S33>/Unit Delay'
         *  UnitDelay: '<S35>/Unit Delay'
         */
        if (((!HOD_EnaActTorqueChk) && HOD_EnaActTorqueChkEdgeFalling) ||
            (HOD_EnaHandsOn && HOD_EnaHandsOffChkEdgeFalling)) {
            HOD_TrigAbuseWarnRSFlipFlop = false;
        } else {
            HOD_TrigAbuseWarnRSFlipFlop =
                ((HOD_EnaActTorqueChk && (!HOD_EnaActTorqueChkEdgeRising)) ||
                 HOD_TrigAbuseWarnRSFlipFlop);
        }

        /* End of Switch: '<S35>/Switch' */

        /* Logic: '<S27>/AND2' */
        rtb_Equal10 = (HOD_EnaActTorqueChk && rtb_HOD_EnaHandsOffChk);

        /* Logic: '<S27>/AND1' incorporates:
         *  Logic: '<S34>/AND'
         *  Logic: '<S34>/NOT'
         *  UnitDelay: '<S34>/Unit Delay'
         *  UnitDelay: '<S35>/Unit Delay'
         */
        HOD_TrigAbuseWarn = (HOD_TrigAbuseWarnRSFlipFlop &&
                             (rtb_Equal10 && (!HOD_EnaHandsOffChkEdgeRising)));

        /* MultiPortSwitch: '<S27>/Multiport Switch' incorporates:
         *  Sum: '<S27>/Add'
         *  UnitDelay: '<S27>/Unit Delay'
         */
        HOD_TiDisaDrvAbuse = HOD_PrevTiDisaDrvAbuse;

        /* Logic: '<S27>/OR1' incorporates:
         *  Constant: '<S27>/IAM_Ts_P1'
         *  Logic: '<S27>/AND3'
         *  RelationalOperator: '<S27>/GreaterThan'
         */
        HOD_DisaDrvAbuse = ((HOD_EnaActTorqueChk && HOD_EnaHandsOn) ||
                            (HOD_TiDisaDrvAbuse > HOD_TiConsecAbusMax_P));

        /* MultiPortSwitch: '<S30>/Multiport Switch' incorporates:
         *  Constant: '<S30>/Constant2'
         *  Logic: '<S29>/AND'
         *  Logic: '<S29>/NOT'
         *  UnitDelay: '<S29>/Unit Delay'
         *  UnitDelay: '<S30>/Unit Delay'
         */
        if (HOD_TrigAbuseWarn || (!HOD_TrigAbuseWarnEdgeFalling)) {
            /* MultiPortSwitch: '<S30>/Multiport Switch1' */
            switch ((int32_T)HOD_DisaDrvAbuse) {
                case 0:
                    break;

                default:
                    /* UnitDelay: '<S30>/Unit Delay' incorporates:
                     *  Constant: '<S30>/Constant1'
                     */
                    HOD_DisaAbuseWarnRSFlipFlop = true;
                    break;
            }

            /* End of MultiPortSwitch: '<S30>/Multiport Switch1' */
        } else {
            HOD_DisaAbuseWarnRSFlipFlop = false;
        }

        /* End of MultiPortSwitch: '<S30>/Multiport Switch' */

        /* MultiPortSwitch: '<S26>/Multiport Switch4' incorporates:
         *  Constant: '<S26>/IAM_Ts_P8'
         *  Constant: '<S26>/IAM_Ts_P9'
         */
        if (!HOD_TrigAbuseWarn) {
            rtb_Abs1 = 0.0F;
        } else {
            rtb_Abs1 = 1.0F;
        }

        /* End of MultiPortSwitch: '<S26>/Multiport Switch4' */

        /* MultiPortSwitch: '<S26>/Multiport Switch3' incorporates:
         *  Constant: '<S26>/IAM_Ts_P7'
         *  Sum: '<S26>/Add2'
         *  UnitDelay: '<S26>/Unit Delay3'
         *  UnitDelay: '<S30>/Unit Delay'
         */
        if (HOD_DisaAbuseWarnRSFlipFlop) {
            HOD_PrevCycleFirstAbuse = 0.0F;
        }

        HOD_PrevCycleFirstAbuse = rtb_Abs1 + HOD_PrevCycleFirstAbuse;

        /* End of MultiPortSwitch: '<S26>/Multiport Switch3' */

        /* Logic: '<S26>/AND5' incorporates:
         *  Constant: '<S26>/IAM_Ts_P6'
         *  RelationalOperator: '<S26>/GreaterThan1'
         *  UnitDelay: '<S26>/Unit Delay3'
         */
        HOD_EnaIncTrigger = (HOD_TrigAbuseWarn && (HOD_PrevCycleFirstAbuse >=
                                                   HOD_CycleFirstAbuse_P));

        /* MultiPortSwitch: '<S37>/Multiport Switch' incorporates:
         *  Constant: '<S37>/Constant2'
         *  Logic: '<S36>/AND'
         *  Logic: '<S36>/NOT'
         *  UnitDelay: '<S36>/Unit Delay'
         *  UnitDelay: '<S37>/Unit Delay'
         */
        if (HOD_TrigAbuseWarn || (!HOD_TrigAbuseWarnEdgeFalling2)) {
            /* MultiPortSwitch: '<S37>/Multiport Switch1' */
            switch ((int32_T)HOD_DisaDrvAbuse) {
                case 0:
                    break;

                default:
                    /* UnitDelay: '<S37>/Unit Delay' incorporates:
                     *  Constant: '<S37>/Constant1'
                     */
                    HOD_DisaAbuseWarnRSFlipFlop2 = true;
                    break;
            }

            /* End of MultiPortSwitch: '<S37>/Multiport Switch1' */
        } else {
            HOD_DisaAbuseWarnRSFlipFlop2 = false;
        }

        /* End of MultiPortSwitch: '<S37>/Multiport Switch' */

        /* MultiPortSwitch: '<S28>/Multiport Switch1' incorporates:
         *  Sum: '<S28>/Add1'
         *  UnitDelay: '<S28>/Unit Delay2'
         */
        HOD_TiAbuseWarn = HOD_PrevTiAbuseWarn;

        /* MultiPortSwitch: '<S28>/Multiport Switch2' incorporates:
         *  Constant: '<S28>/IAM_Ts_P4'
         *  Constant: '<S28>/IAM_Ts_P5'
         */
        if (!HOD_EnaIncTrigger) {
            rtb_Abs1 = 0.0F;
        } else {
            rtb_Abs1 = HOD_TiIncNextWarn_P;
        }

        /* End of MultiPortSwitch: '<S28>/Multiport Switch2' */

        /* MultiPortSwitch: '<S28>/Multiport Switch1' incorporates:
         *  Constant: '<S28>/IAM_Ts_P2'
         *  Logic: '<S28>/Logical Operator1'
         *  Sum: '<S28>/Add1'
         *  UnitDelay: '<S37>/Unit Delay'
         */
        if (HOD_DisaAbuseWarnRSFlipFlop2) {
            HOD_TiAbuseWarn = HOD_TiFirstAbuse_P;
        }

        HOD_TiAbuseWarn += rtb_Abs1;

        /* MultiPortSwitch: '<S25>/Multiport Switch2' incorporates:
         *  Logic: '<S25>/Logical Operator1'
         *  MultiPortSwitch: '<S25>/Multiport Switch1'
         */
        if (HOD_TrigAbuseWarn) {
            /* MultiPortSwitch: '<S27>/Multiport Switch' incorporates:
             *  Inport: '<Root>/HOD_CycleTime'
             *  Sum: '<S25>/Add'
             */
            HOD_PrevFollowUpTimer = HOD_TiAbuseWarn + HOD_CycleTime;
        } else {
            if (HOD_DisaDrvAbuse) {
                /* MultiPortSwitch: '<S27>/Multiport Switch' incorporates:
                 *  Constant: '<S25>/IAM_Ts_P4'
                 *  MultiPortSwitch: '<S25>/Multiport Switch1'
                 */
                HOD_PrevFollowUpTimer = 0.0F;
            }
        }

        /* End of MultiPortSwitch: '<S25>/Multiport Switch2' */

        /* Logic: '<S25>/OR' incorporates:
         *  Constant: '<S25>/IAM_Ts_P1'
         *  RelationalOperator: '<S25>/GreaterThan'
         */
        HOD_HandsOffAbuseWarnTp =
            (HOD_TrigAbuseWarn || (HOD_PrevFollowUpTimer > 1.0E-5F));

        /* MultiPortSwitch: '<S27>/Multiport Switch' incorporates:
         *  Inport: '<Root>/HOD_CycleTime'
         *  Sum: '<S25>/Subtract'
         */
        HOD_PrevFollowUpTimer = HOD_PrevFollowUpTimer - HOD_CycleTime;

        /* MultiPortSwitch: '<S27>/Multiport Switch' incorporates:
         *  Constant: '<S27>/IAM_Ts_P11'
         *  Inport: '<Root>/HOD_CycleTime'
         *  Sum: '<S27>/Add'
         */
        if (HOD_DisaDrvAbuse) {
            HOD_TiDisaDrvAbuse = 0.0F;
        }

        HOD_TiDisaDrvAbuse += HOD_CycleTime;

        /* Update for UnitDelay: '<S31>/Unit Delay' */
        HOD_EnaActTorqueChkEdgeFalling = HOD_EnaActTorqueChk;

        /* Update for UnitDelay: '<S32>/Unit Delay' */
        HOD_EnaHandsOffChkEdgeFalling = rtb_HOD_EnaHandsOffChk;

        /* Update for UnitDelay: '<S33>/Unit Delay' */
        HOD_EnaActTorqueChkEdgeRising = HOD_EnaActTorqueChk;

        /* Update for UnitDelay: '<S34>/Unit Delay' */
        HOD_EnaHandsOffChkEdgeRising = rtb_Equal10;

        /* Update for UnitDelay: '<S29>/Unit Delay' */
        HOD_TrigAbuseWarnEdgeFalling = HOD_TrigAbuseWarn;

        /* Update for UnitDelay: '<S27>/Unit Delay' */
        HOD_PrevTiDisaDrvAbuse = HOD_TiDisaDrvAbuse;

        /* Update for UnitDelay: '<S36>/Unit Delay' */
        HOD_TrigAbuseWarnEdgeFalling2 = HOD_TrigAbuseWarn;

        /* Update for UnitDelay: '<S28>/Unit Delay2' */
        HOD_PrevTiAbuseWarn = HOD_TiAbuseWarn;
    }

    /* End of Constant: '<S6>/IAM_Ts_P6' */
    /* End of Outputs for SubSystem: '<S6>/DriverAbuseMonitoring' */

    /* SignalConversion: '<S6>/Signal Conversion' */
    HOD_HandsOffAbuseWarn = HOD_HandsOffAbuseWarnTp;

    /* Logic: '<S49>/Logical Operator12' incorporates:
     *  Constant: '<S49>/IAM_Ts_P3'
     *  Constant: '<S49>/IAM_Ts_P4'
     *  RelationalOperator: '<S49>/Equal'
     *  RelationalOperator: '<S49>/Equal1'
     *  UnitDelay: '<S8>/Unit Delay'
     */
    rtb_HOD_EnaHandsOffChk =
        ((HOD_PrevStEstHandTrqLast == HOD_StHandsOffSuspLf_P) ||
         (HOD_PrevStEstHandTrqLast == HOD_StHandsOffSuspRi_P));

    /* Logic: '<S49>/Logical Operator8' incorporates:
     *  Logic: '<S49>/Logical Operator'
     *  Logic: '<S49>/Logical Operator1'
     */
    rtb_Equal10 = ((!HOD_EnaRightOn) && (!HOD_EnaLeftOn));

    /* MultiPortSwitch: '<S49>/Multiport Switch' incorporates:
     *  Constant: '<S49>/IAM_Ts_P1'
     *  Constant: '<S49>/IAM_Ts_P5'
     *  Constant: '<S49>/IAM_Ts_P7'
     *  Constant: '<S49>/IAM_Ts_P8'
     *  Logic: '<S49>/Logical Operator'
     *  Logic: '<S49>/Logical Operator2'
     *  Logic: '<S49>/Logical Operator8'
     *  MultiPortSwitch: '<S49>/Multiport Switch1'
     *  Product: '<S49>/Divide'
     *  RelationalOperator: '<S49>/Equal2'
     *  RelationalOperator: '<S49>/GreaterThan'
     *  Sum: '<S49>/Add1'
     *  UnitDelay: '<S49>/Unit Delay'
     *  UnitDelay: '<S8>/Unit Delay'
     */
    if ((HOD_PrevStEstHandTrqLast != HOD_StHandsOnSusp_P) ||
        (rtb_Equal10 && (rtb_Divide <= HOD_TrqHandsOnLimAbs_P / 2.0F))) {
        /* Logic: '<S49>/Logical Operator7' incorporates:
         *  Logic: '<S49>/Logical Operator'
         */
        rtb_HOD_EnaHandsOffChk = !rtb_HOD_EnaHandsOffChk;

        /* MultiPortSwitch: '<S49>/Multiport Switch2' incorporates:
         *  Logic: '<S49>/Logical Operator'
         *  Logic: '<S49>/Logical Operator11'
         *  Logic: '<S49>/Logical Operator7'
         *  MultiPortSwitch: '<S49>/Multiport Switch1'
         */
        if (rtb_HOD_EnaHandsOffChk ||
            ((!HOD_EnaRightOff) && (!HOD_EnaLeftOff))) {
            /* UnitDelay: '<S49>/Unit Delay' incorporates:
             *  Constant: '<S49>/IAM_Ts_P2'
             */
            HOD_PrevCntEstHandTrq = 0.0F;
        } else {
            if ((!rtb_HOD_EnaHandsOffChk) || (!rtb_Equal10)) {
                /* UnitDelay: '<S49>/Unit Delay' incorporates:
                 *  Constant: '<S49>/IAM_Ts_P6'
                 *  MultiPortSwitch: '<S49>/Multiport Switch1'
                 *  Sum: '<S49>/Add'
                 */
                HOD_PrevCntEstHandTrq = HOD_CntEstHandTrq + 1.0F;
            }
        }

        /* End of MultiPortSwitch: '<S49>/Multiport Switch2' */
    } else {
        if ((!rtb_HOD_EnaHandsOffChk) && rtb_Equal10) {
            /* MultiPortSwitch: '<S49>/Multiport Switch1' */
            rtb_Divide = HOD_CntEstHandTrq;
        } else {
            /* MultiPortSwitch: '<S49>/Multiport Switch1' incorporates:
             *  Constant: '<S49>/IAM_Ts_P6'
             *  Sum: '<S49>/Add'
             */
            rtb_Divide = HOD_CntEstHandTrq + 1.0F;
        }

        HOD_PrevCntEstHandTrq = rtb_Divide + 1.0F;
    }

    /* End of MultiPortSwitch: '<S49>/Multiport Switch' */

    /* MultiPortSwitch: '<S54>/Multiport Switch7' incorporates:
     *  Constant: '<S54>/IAM_Ts_P15'
     *  Constant: '<S54>/IAM_Ts_P19'
     *  Logic: '<S54>/OR2'
     *  RelationalOperator: '<S54>/Equal1'
     *  RelationalOperator: '<S54>/Equal2'
     *  UnitDelay: '<S8>/Unit Delay'
     */
    if ((HOD_PrevStEstHandTrqLast != HOD_StHandsOnLf_P) &&
        (HOD_PrevStEstHandTrqLast != HOD_StHandsOnRi_P)) {
        /* MultiPortSwitch: '<S54>/Multiport Switch6' incorporates:
         *  Constant: '<S54>/IAM_Ts_P26'
         *  RelationalOperator: '<S54>/Equal5'
         */
        if (HOD_PrevStEstHandTrqLast != HOD_StHandsOnSusp_P) {
            /* Product: '<S57>/Divide3' incorporates:
             *  MultiPortSwitch: '<S54>/Multiport Switch7'
             */
            HOD_CntHandsOffLimInt = HOD_CntHandsOffLim;
        }
    } else {
        /* Product: '<S57>/Divide3' incorporates:
         *  Constant: '<S54>/IAM_Ts_P24'
         *  Inport: '<Root>/HOD_CycleTime'
         *  MultiPortSwitch: '<S54>/Multiport Switch7'
         *  Product: '<S54>/Divide5'
         */
        HOD_CntHandsOffLimInt = HOD_TiHandsOffQlf_P / HOD_CycleTime;
    }

    /* MultiPortSwitch: '<S55>/Multiport Switch8' incorporates:
     *  Constant: '<S55>/IAM_Ts_P16'
     *  Constant: '<S55>/IAM_Ts_P18'
     *  Logic: '<S55>/OR3'
     *  RelationalOperator: '<S55>/Equal7'
     *  RelationalOperator: '<S55>/Equal8'
     *  UnitDelay: '<S8>/Unit Delay'
     */
    if ((HOD_PrevStEstHandTrqLast != HOD_StHandsOnLf_P) &&
        (HOD_PrevStEstHandTrqLast != HOD_StHandsOnRi_P)) {
        /* MultiPortSwitch: '<S55>/Multiport Switch8' incorporates:
         *  UnitDelay: '<S55>/Unit Delay'
         */
        HOD_TrqHandsOffSns = HOD_PrevTrqHandsOffSns;
    } else {
        /* MultiPortSwitch: '<S55>/Multiport Switch8' incorporates:
         *  Inport: '<Root>/HOD_TrqActMan'
         */
        HOD_TrqHandsOffSns = HOD_TrqActMan;
    }

    /* End of MultiPortSwitch: '<S55>/Multiport Switch8' */

    /* MultiPortSwitch: '<S53>/Multiport Switch3' incorporates:
     *  Abs: '<S53>/Abs'
     *  Constant: '<S53>/IAM_Ts_P14'
     *  Inport: '<Root>/HOD_TrqActMan'
     *  Logic: '<S53>/AND1'
     *  RelationalOperator: '<S53>/GreaterThan'
     *  RelationalOperator: '<S53>/GreaterThanOrEqual'
     *  Sum: '<S53>/Subtract'
     */
    if ((fabsf(HOD_TrqHandsOffSns - HOD_TrqActMan) <= HOD_TrqHandsOffDelta_P) ||
        (HOD_CntHandsOffLimInt > tmp)) {
        /* MultiPortSwitch: '<S53>/Multiport Switch3' */
        HOD_CntHandsOffLimBig = HOD_CntHandsOffLimInt;
    } else {
        /* MultiPortSwitch: '<S53>/Multiport Switch3' incorporates:
         *  Constant: '<S53>/IAM_Ts_P7'
         *  Inport: '<Root>/HOD_CycleTime'
         *  Product: '<S53>/Divide2'
         *  Sum: '<S53>/Add1'
         */
        HOD_CntHandsOffLimBig =
            HOD_TiHandsOffDelta_P / HOD_CycleTime + HOD_CntHandsOffLimInt;
    }

    /* End of MultiPortSwitch: '<S53>/Multiport Switch3' */

    /* Sum: '<S40>/Subtract1' incorporates:
     *  UnitDelay: '<S40>/Unit Delay2'
     */
    HOD_TrqEstHandStp = HOD_PrevTrqEstHandFlt - HOD_TrqEstHandFlt;

    /* MultiPortSwitch: '<S52>/Multiport Switch2' incorporates:
     *  Abs: '<S52>/Abs2'
     *  Constant: '<S52>/IAM_Ts_P10'
     *  Constant: '<S52>/IAM_Ts_P12'
     *  Constant: '<S52>/IAM_Ts_P4'
     *  Logic: '<S52>/Logical Operator1'
     *  Logic: '<S52>/Logical Operator3'
     *  Logic: '<S52>/OR1'
     *  MultiPortSwitch: '<S52>/Multiport Switch5'
     *  RelationalOperator: '<S52>/Equal3'
     *  RelationalOperator: '<S52>/Equal4'
     *  RelationalOperator: '<S52>/Less Than1'
     *  UnitDelay: '<S52>/Unit Delay'
     *  UnitDelay: '<S8>/Unit Delay'
     */
    if ((HOD_PrevStEstHandTrqLast != HOD_StHandsOffSuspLf_P) &&
        (HOD_PrevStEstHandTrqLast != HOD_StHandsOffSuspRi_P) &&
        ((!HOD_EnaLeftOff) && (!HOD_EnaRightOff))) {
        HOD_PrevCounterlimitUpdate = HOD_CntHandsOffLimBig;
    } else if (fabsf(HOD_TrqEstHandStp) >= HOD_TrqHandsOffStepLim_P) {
        /* UnitDelay: '<S52>/Unit Delay' incorporates:
         *  Constant: '<S52>/IAM_Ts_P5'
         *  Inport: '<Root>/HOD_CycleTime'
         *  MultiPortSwitch: '<S52>/Multiport Switch5'
         *  Product: '<S52>/Divide1'
         */
        HOD_PrevCounterlimitUpdate = HOD_TiHandsOffQlf_P / HOD_CycleTime;
    } else {
        /* UnitDelay: '<S52>/Unit Delay' incorporates:
         *  Constant: '<S52>/IAM_Ts_P2'
         *  MultiPortSwitch: '<S52>/Multiport Switch5'
         *  Sum: '<S52>/Add3'
         */
        HOD_PrevCounterlimitUpdate = HOD_CntHandsOffLimBig - 1.0F;
    }

    /* End of MultiPortSwitch: '<S52>/Multiport Switch2' */

    /* Logic: '<S42>/Logical Operator' incorporates:
     *  Constant: '<S42>/IAM_Ts_P2'
     *  RelationalOperator: '<S42>/GreaterThan'
     *  RelationalOperator: '<S42>/LessThanOrEqual'
     *  UnaryMinus: '<S42>/Unary Minus2'
     *  UnaryMinus: '<S42>/Unary Minus3'
     */
    HOD_EnaRightInt = ((HOD_TrqEstHandFlt > -HOD_TrqtHandsOnLim_P) &&
                       (HOD_TrqEstHandFlt <= -HOD_TrqHandsOffLim));

    /* Logic: '<S43>/Logical Operator3' incorporates:
     *  Constant: '<S43>/IAM_Ts_P3'
     *  RelationalOperator: '<S43>/GreaterThan3'
     *  RelationalOperator: '<S43>/LessThanOrEqual3'
     */
    HOD_EnaLeftInt = ((HOD_TrqEstHandFlt <= HOD_TrqtHandsOnLim_P) &&
                      (HOD_TrqEstHandFlt > HOD_TrqHandsOffLim));

    /* MultiPortSwitch: '<S55>/Multiport Switch1' incorporates:
     *  Abs: '<S55>/Abs'
     *  Constant: '<S55>/IAM_Ts_P11'
     *  Constant: '<S55>/IAM_Ts_P14'
     *  Constant: '<S55>/IAM_Ts_P27'
     *  Constant: '<S55>/IAM_Ts_P28'
     *  Constant: '<S55>/IAM_Ts_P8'
     *  Inport: '<Root>/HOD_CycleTime'
     *  Inport: '<Root>/HOD_TrqActMan'
     *  Logic: '<S55>/AND1'
     *  Logic: '<S55>/AND2'
     *  Logic: '<S55>/Logical Operator2'
     *  Logic: '<S55>/Logical Operator4'
     *  Logic: '<S55>/OR'
     *  Product: '<S55>/Divide'
     *  Product: '<S55>/Divide4'
     *  RelationalOperator: '<S55>/Equal10'
     *  RelationalOperator: '<S55>/Equal9'
     *  RelationalOperator: '<S55>/GreaterThan'
     *  RelationalOperator: '<S55>/Less Than'
     *  Sum: '<S55>/Subtract'
     *  UnitDelay: '<S55>/Unit Delay'
     *  UnitDelay: '<S8>/Unit Delay'
     */
    if (((HOD_PrevStEstHandTrqLast != HOD_StHandsOffSuspLf_P) &&
         (HOD_PrevStEstHandTrqLast != HOD_StHandsOffSuspRi_P)) ||
        ((!HOD_EnaLeftInt) && (!HOD_EnaRightInt)) ||
        ((HOD_CntEstHandTrq >= HOD_TiHandsOffQlf_P / HOD_CycleTime / 4.0F) &&
         (fabsf(HOD_TrqActMan - HOD_TrqHandsOffSns) <=
          HOD_TrqHandsOffDelta_P))) {
        HOD_PrevTrqHandsOffSns = HOD_TrqHandsOffSns;
    } else {
        HOD_PrevTrqHandsOffSns = HOD_TrqActMan;
    }

    /* End of MultiPortSwitch: '<S55>/Multiport Switch1' */

    /* Update for UnitDelay: '<S10>/Unit Delay' */
    HOD_PrevStSysWarning = HOD_StSysWarning;

    /* Update for UnitDelay: '<S8>/Unit Delay' */
    HOD_PrevStEstHandTrqLast = HOD_StEstHandTrq;

    /* Update for UnitDelay: '<S44>/Unit Delay' */
    HOD_PrevTrqEstHandFlt2 = HOD_TrqEstHandFlt;

    /* Update for UnitDelay: '<S9>/Unit Delay1' */
    HOD_PrevStHandsOffDtct = HOD_StHandsOffDtct;

    /* Update for UnitDelay: '<S40>/Unit Delay2' */
    HOD_PrevTrqEstHandFlt = HOD_TrqEstHandFlt;
}

/* Model initialize function */
void HOD_initialize(void) {
    /* InitializeConditions for UnitDelay: '<S9>/Unit Delay1' */
    HOD_PrevStHandsOffDtct = 3U;

    /* SystemInitialize for Enabled SubSystem: '<S6>/DriverAbuseMonitoring' */
    /* InitializeConditions for UnitDelay: '<S33>/Unit Delay' */
    HOD_EnaActTorqueChkEdgeRising = true;

    /* InitializeConditions for UnitDelay: '<S34>/Unit Delay' */
    HOD_EnaHandsOffChkEdgeRising = true;

    /* End of SystemInitialize for SubSystem: '<S6>/DriverAbuseMonitoring' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
