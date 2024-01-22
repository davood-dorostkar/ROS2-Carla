/**********************************Model Property********************************
 *
 * Company             : SENSETIME
 *
 * Model               : HOD
 *
 ************************************Auto Coder**********************************
 *
 * File                             : HOD.h
 *
 * FileType                         : Code Header File
 *
 * Real-Time Workshop file version  : 9.4 (R2020b) 29-Jul-2020
 *
 * TLC version                      : 9.4 (Aug 20 2020)
 *
 * C source code generated on       : Tue Nov 23 09:45:44 2021
 *******************************************************************************/

#ifndef RTW_HEADER_HOD_h_
#define RTW_HEADER_HOD_h_
#include <math.h>
#ifndef HOD_COMMON_INCLUDES_
#define HOD_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* HOD_COMMON_INCLUDES_ */

#include "HOD_types.h"

/* Macros for accessing real-time model data structure */

/* Block states (default storage) for system '<Root>' */
typedef struct {
  uint8_T is_active_c4_HOD;            /* '<S56>/HandTorqueStateMachine1' */
  uint8_T is_c4_HOD;                   /* '<S56>/HandTorqueStateMachine1' */
  uint8_T is_active_c3_HOD;            /* '<S14>/DriverAttentionStateMachine' */
  uint8_T is_c3_HOD;                   /* '<S14>/DriverAttentionStateMachine' */
} DW_HOD_T;

/* Block states (default storage) */
extern DW_HOD_T HOD_DW;

/*
 * Exported Global Signals
 *
 * Note: Exported global signals are block signals with an exported global
 * storage class designation.  Code generation will declare the memory for
 * these signals and export their symbols.
 *
 */
extern real32_T HOD_TiTrigDegrDelay;   /* '<S15>/Selector2' */
extern real32_T HOD_CntEstHandTrq;     /* '<S49>/Unit Delay' */
extern real32_T HOD_CntHandsOffLim;    /* '<S52>/Multiport Switch9' */
extern real32_T HOD_CoeffVelXFlt;      /* '<S40>/1-D Lookup Table1' */
extern real32_T HOD_TrqActManCorrd;    /* '<S40>/Subtract' */
extern real32_T HOD_TrqEstHandFlt;     /* '<S44>/Add' */
extern real32_T HOD_TrqHandsOffLim;    /* '<S41>/Multiport Switch1' */
extern real32_T HOD_TiHandsOnReqDelay; /* '<S15>/Selector1' */
extern real32_T HOD_TiTakeOverReqDelay;/* '<S15>/Selector' */
extern real32_T HOD_RatDrvAttentionMax;/* '<S16>/IAM_Ts_P2' */
extern real32_T HOD_RatDrvAttention;   /* '<S16>/Multiport Switch2' */
extern real32_T HOD_CntHandsOffLimInt; /* '<S54>/Multiport Switch7' */
extern real32_T HOD_TrqHandsOffSns;    /* '<S55>/Multiport Switch8' */
extern real32_T HOD_CntHandsOffLimBig; /* '<S53>/Multiport Switch3' */
extern real32_T HOD_TrqEstHandStp;     /* '<S40>/Subtract1' */
extern real32_T HOD_TiAbuseWarn;       /* '<S28>/Add1' */
extern real32_T HOD_TiDisaDrvAbuse;    /* '<S27>/Add' */
extern uint8_T HOD_StEstHandTrq;       /* '<S56>/HandTorqueStateMachine1' */
extern uint8_T HOD_StHandsOffDtct;     /* '<S9>/Multiport Switch5' */
extern uint8_T HOD_StSysWarning;       /* '<S14>/DriverAttentionStateMachine' */
extern boolean_T HOD_EnaTrigDegr;      /* '<S24>/AND' */
extern boolean_T HOD_EnaHandsOffExcd;  /* '<S57>/Logical Operator8' */
extern boolean_T HOD_EnaHandsOnExcdRi; /* '<S57>/Logical Operator14' */
extern boolean_T HOD_EnaHandsOnExcdLf; /* '<S57>/Logical Operator12' */
extern boolean_T HOD_EnaLeftOn;        /* '<S43>/GreaterThanOrEqual' */
extern boolean_T HOD_EnaLeftOff;       /* '<S43>/Logical Operator2' */
extern boolean_T HOD_EnaRightOn;       /* '<S42>/Less Than' */
extern boolean_T HOD_EnaRightOff;      /* '<S42>/Logical Operator1' */
extern boolean_T HOD_EnaActFct;        /* '<S15>/Equal2' */
extern boolean_T HOD_EnaHandsOffCnfm;  /* '<S9>/Multiport Switch1' */
extern boolean_T HOD_VldVehSpd;        /* '<S38>/Switch' */
extern boolean_T HOD_EnaHandsOn;       /* '<S15>/Logical Operator1' */
extern boolean_T HOD_EnaHandsOnReq;    /* '<S22>/AND' */
extern boolean_T HOD_EnaTakeOverReq;   /* '<S23>/AND' */
extern boolean_T HOD_EnaActTorqueChk;  /* '<S11>/AND' */
extern boolean_T HOD_HandsOffAbuseWarn;/* '<S6>/Signal Conversion' */
extern boolean_T HOD_EnaRightInt;      /* '<S42>/Logical Operator' */
extern boolean_T HOD_EnaLeftInt;       /* '<S43>/Logical Operator3' */
extern boolean_T HOD_TrigAbuseWarn;    /* '<S27>/AND1' */
extern boolean_T HOD_DisaDrvAbuse;     /* '<S27>/OR1' */
extern boolean_T HOD_EnaIncTrigger;    /* '<S26>/AND5' */
extern boolean_T HOD_HandsOffAbuseWarnTp;/* '<S25>/OR' */

/* Model entry point functions */
extern void HOD_initialize(void);
extern void HOD_step(void);

/* Exported data declaration */

/* ConstVolatile memory section */
/* Declaration for custom storage class: ConstVolatile */
extern const volatile uint16_T HOD_BtfAbuseFct_P;/* Referenced by:
                                                  * '<S6>/IAM_Ts_P6'
                                                  * '<S11>/IAM_Ts_P6'
                                                  */
extern const volatile uint8_T HOD_BtfAvailableFct_P;/* Referenced by:
                                                     * '<S15>/IAM_Ts_P5'
                                                     * '<S16>/IAM_Ts_P5'
                                                     */
extern const volatile real32_T HOD_CoeffVelXLowPass_M[8];
                                  /* Referenced by: '<S40>/1-D Lookup Table1' */
extern const volatile boolean_T HOD_ConstHandOnCnfm_P;/* Referenced by: '<S9>/IAM_Ts_P2' */
extern const volatile boolean_T HOD_ConstHandsOffCnfm_P;/* Referenced by: '<S9>/IAM_Ts_P4' */
extern const volatile uint8_T HOD_CycleFirstAbuse_P;/* Referenced by: '<S26>/IAM_Ts_P6' */
extern const volatile real32_T HOD_OfstEstHandTrqNeg_P;/* Referenced by: '<S40>/IAM_Ts_P1' */
extern const volatile real32_T HOD_OfstEstHandTrqPos_P;/* Referenced by: '<S40>/IAM_Ts_P7' */
extern const volatile real32_T HOD_RatDrvAttentionMax_P;/* Referenced by: '<S16>/IAM_Ts_P2' */
extern const volatile real32_T HOD_SpdLimHys_P;/* Referenced by: '<S13>/IAM_Ts_P1' */
extern const volatile real32_T HOD_SpdLimMin_P;/* Referenced by:
                                                * '<S13>/IAM_Ts_P3'
                                                * '<S13>/IAM_Ts_P6'
                                                */
extern const volatile uint8_T HOD_StDegrTrig_P;/* Referenced by:
                                                * '<S14>/DriverAttentionStateMachine'
                                                * '<S16>/IAM_Ts_P1'
                                                */
extern const volatile uint8_T HOD_StHandsOffSuspLf_P;/* Referenced by:
                                                      * '<S49>/IAM_Ts_P3'
                                                      * '<S57>/IAM_Ts_P6'
                                                      * '<S52>/IAM_Ts_P10'
                                                      * '<S55>/IAM_Ts_P27'
                                                      */
extern const volatile uint8_T HOD_StHandsOffSuspRi_P;/* Referenced by:
                                                      * '<S49>/IAM_Ts_P4'
                                                      * '<S57>/IAM_Ts_P8'
                                                      * '<S52>/IAM_Ts_P12'
                                                      * '<S55>/IAM_Ts_P28'
                                                      */
extern const volatile uint8_T HOD_StHandsOff_P;/* Referenced by: '<S9>/IAM_Ts_P10' */
extern const volatile uint8_T HOD_StHandsOnLf_P;/* Referenced by:
                                                 * '<S9>/IAM_Ts_P7'
                                                 * '<S54>/IAM_Ts_P15'
                                                 * '<S55>/IAM_Ts_P16'
                                                 */
extern const volatile uint8_T HOD_StHandsOnReq_P;/* Referenced by:
                                                  * '<S14>/DriverAttentionStateMachine'
                                                  * '<S15>/IAM_Ts_P2'
                                                  */
extern const volatile uint8_T HOD_StHandsOnRi_P;/* Referenced by:
                                                 * '<S9>/IAM_Ts_P5'
                                                 * '<S54>/IAM_Ts_P19'
                                                 * '<S55>/IAM_Ts_P18'
                                                 */
extern const volatile uint8_T HOD_StHandsOnSusp_P;/* Referenced by:
                                                   * '<S49>/IAM_Ts_P5'
                                                   * '<S57>/IAM_Ts_P9'
                                                   * '<S54>/IAM_Ts_P26'
                                                   */
extern const volatile uint8_T HOD_StNoWarning_P;
                        /* Referenced by: '<S14>/DriverAttentionStateMachine' */
extern const volatile uint8_T HOD_StTakeOverReq_P;/* Referenced by:
                                                   * '<S14>/DriverAttentionStateMachine'
                                                   * '<S15>/IAM_Ts_P4'
                                                   */
extern const volatile real32_T HOD_TiConsecAbusMax_P;/* Referenced by: '<S27>/IAM_Ts_P1' */
extern const volatile real32_T HOD_TiFirstAbuse_P;/* Referenced by: '<S28>/IAM_Ts_P2' */
extern const volatile real32_T HOD_TiHandsOffDelta_P;/* Referenced by: '<S53>/IAM_Ts_P7' */
extern const volatile real32_T HOD_TiHandsOffQlfMax_P;/* Referenced by:
                                                       * '<S41>/IAM_Ts_P1'
                                                       * '<S53>/IAM_Ts_P9'
                                                       */
extern const volatile real32_T HOD_TiHandsOffQlf_P;/* Referenced by:
                                                    * '<S41>/IAM_Ts_P4'
                                                    * '<S52>/IAM_Ts_P23'
                                                    * '<S52>/IAM_Ts_P5'
                                                    * '<S54>/IAM_Ts_P24'
                                                    * '<S55>/IAM_Ts_P8'
                                                    */
extern const volatile real32_T HOD_TiHandsOnQlf_P;/* Referenced by:
                                                   * '<S57>/IAM_Ts_P2'
                                                   * '<S54>/IAM_Ts_P20'
                                                   */
extern const volatile real32_T HOD_TiHorHandsOff_P[10];/* Referenced by: '<S15>/IAM_Ts_P6' */
extern const volatile real32_T HOD_TiIncNextWarn_P;/* Referenced by: '<S28>/IAM_Ts_P5' */
extern const volatile real32_T HOD_TiTorHandsOff_P[10];/* Referenced by: '<S15>/IAM_Ts_P8' */
extern const volatile real32_T HOD_TiTorTrigDegr_P[10];/* Referenced by: '<S15>/IAM_Ts_P12' */
extern const volatile real32_T HOD_TrqHandsOffDelta_P;/* Referenced by:
                                                       * '<S53>/IAM_Ts_P14'
                                                       * '<S55>/IAM_Ts_P14'
                                                       */
extern const volatile real32_T HOD_TrqHandsOffLim_P;/* Referenced by: '<S41>/IAM_Ts_P17' */
extern const volatile real32_T HOD_TrqHandsOffStepLim_P;/* Referenced by: '<S52>/IAM_Ts_P4' */
extern const volatile real32_T HOD_TrqHandsOnLimAbs_P;/* Referenced by:
                                                       * '<S49>/IAM_Ts_P7'
                                                       * '<S57>/IAM_Ts_P7'
                                                       */
extern const volatile real32_T HOD_TrqtHandsOnLim_P;/* Referenced by:
                                                     * '<S42>/IAM_Ts_P1'
                                                     * '<S42>/IAM_Ts_P2'
                                                     * '<S43>/IAM_Ts_P3'
                                                     * '<S43>/IAM_Ts_P4'
                                                     */
extern const volatile real32_T HOD_VelXVeh_X[8];
                                  /* Referenced by: '<S40>/1-D Lookup Table1' */

/* Declaration for custom storage class: Global */
extern boolean_T HOD_DisaAbuseWarnRSFlipFlop;/* '<S30>/Unit Delay' */
extern boolean_T HOD_DisaAbuseWarnRSFlipFlop2;/* '<S37>/Unit Delay' */
extern boolean_T HOD_EnaActTorqueChkEdgeFalling;/* '<S31>/Unit Delay' */
extern boolean_T HOD_EnaActTorqueChkEdgeRising;/* '<S33>/Unit Delay' */
extern boolean_T HOD_EnaHandsOffChkEdgeFalling;/* '<S32>/Unit Delay' */
extern boolean_T HOD_EnaHandsOffChkEdgeRising;/* '<S34>/Unit Delay' */
extern real32_T HOD_EnaHandsOnReqTurnOnDelay;/* '<S22>/Unit Delay' */
extern real32_T HOD_EnaTakeOverReqTurnOnDelay;/* '<S23>/Unit Delay' */
extern real32_T HOD_EnaTrigDegrTurnOnDelay;/* '<S24>/Unit Delay' */
extern real32_T HOD_PrevCntEstHandTrq; /* '<S49>/Unit Delay' */
extern real32_T HOD_PrevCounterlimitUpdate;/* '<S52>/Unit Delay' */
extern real32_T HOD_PrevCycleFirstAbuse;/* '<S26>/Unit Delay3' */
extern real32_T HOD_PrevFollowUpTimer; /* '<S25>/Unit Delay2' */
extern uint8_T HOD_PrevStEstHandTrqLast;/* '<S8>/Unit Delay' */
extern uint8_T HOD_PrevStHandsOffDtct; /* '<S9>/Unit Delay1' */
extern uint8_T HOD_PrevStSysWarning;   /* '<S10>/Unit Delay' */
extern real32_T HOD_PrevTiAbuseWarn;   /* '<S28>/Unit Delay2' */
extern real32_T HOD_PrevTiDisaDrvAbuse;/* '<S27>/Unit Delay' */
extern real32_T HOD_PrevTrqEstHandFlt; /* '<S40>/Unit Delay2' */
extern real32_T HOD_PrevTrqEstHandFlt2;/* '<S44>/Unit Delay' */
extern real32_T HOD_PrevTrqHandsOffSns;/* '<S55>/Unit Delay' */
extern boolean_T HOD_TrigAbuseWarnEdgeFalling;/* '<S29>/Unit Delay' */
extern boolean_T HOD_TrigAbuseWarnEdgeFalling2;/* '<S36>/Unit Delay' */
extern boolean_T HOD_TrigAbuseWarnRSFlipFlop;/* '<S35>/Unit Delay' */

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S18>/Data Type Duplicate' : Unused code path elimination
 * Block '<S18>/Data Type Propagation' : Unused code path elimination
 * Block '<S46>/Data Type Duplicate' : Unused code path elimination
 * Block '<S46>/Data Type Propagation' : Unused code path elimination
 * Block '<S45>/Data Type Duplicate' : Unused code path elimination
 * Block '<S45>/Data Type Propagation' : Unused code path elimination
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('HOD_Model/HOD')    - opens subsystem HOD_Model/HOD
 * hilite_system('HOD_Model/HOD/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'HOD_Model'
 * '<S1>'   : 'HOD_Model/HOD'
 * '<S5>'   : 'HOD_Model/HOD/HOD_60ms'
 * '<S6>'   : 'HOD_Model/HOD/HOD_60ms/DAE'
 * '<S7>'   : 'HOD_Model/HOD/HOD_60ms/EnableCondition'
 * '<S8>'   : 'HOD_Model/HOD/HOD_60ms/EstimatedHandTorqueState'
 * '<S9>'   : 'HOD_Model/HOD/HOD_60ms/HandsOffComfirm'
 * '<S10>'  : 'HOD_Model/HOD/HOD_60ms/DAE/CalculateSystemWarning'
 * '<S11>'  : 'HOD_Model/HOD/HOD_60ms/DAE/CriticalSituationCheck'
 * '<S12>'  : 'HOD_Model/HOD/HOD_60ms/DAE/DriverAbuseMonitoring'
 * '<S13>'  : 'HOD_Model/HOD/HOD_60ms/DAE/VelocityCheck'
 * '<S14>'  : 'HOD_Model/HOD/HOD_60ms/DAE/CalculateSystemWarning/DriverAttentionState'
 * '<S15>'  : 'HOD_Model/HOD/HOD_60ms/DAE/CalculateSystemWarning/GenerateConditionsWarning'
 * '<S16>'  : 'HOD_Model/HOD/HOD_60ms/DAE/CalculateSystemWarning/SignalMapping'
 * '<S17>'  : 'HOD_Model/HOD/HOD_60ms/DAE/CalculateSystemWarning/DriverAttentionState/DriverAttentionStateMachine'
 * '<S18>'  : 'HOD_Model/HOD/HOD_60ms/DAE/CalculateSystemWarning/GenerateConditionsWarning/Saturation Dynamic'
 * '<S19>'  : 'HOD_Model/HOD/HOD_60ms/DAE/CalculateSystemWarning/GenerateConditionsWarning/TurnOnDelay'
 * '<S20>'  : 'HOD_Model/HOD/HOD_60ms/DAE/CalculateSystemWarning/GenerateConditionsWarning/TurnOnDelay1'
 * '<S21>'  : 'HOD_Model/HOD/HOD_60ms/DAE/CalculateSystemWarning/GenerateConditionsWarning/TurnOnDelay2'
 * '<S22>'  : 'HOD_Model/HOD/HOD_60ms/DAE/CalculateSystemWarning/GenerateConditionsWarning/TurnOnDelay/TurnOnDelay3'
 * '<S23>'  : 'HOD_Model/HOD/HOD_60ms/DAE/CalculateSystemWarning/GenerateConditionsWarning/TurnOnDelay1/TurnOnDelay3'
 * '<S24>'  : 'HOD_Model/HOD/HOD_60ms/DAE/CalculateSystemWarning/GenerateConditionsWarning/TurnOnDelay2/TurnOnDelay3'
 * '<S25>'  : 'HOD_Model/HOD/HOD_60ms/DAE/DriverAbuseMonitoring/FollowUpTimer'
 * '<S26>'  : 'HOD_Model/HOD/HOD_60ms/DAE/DriverAbuseMonitoring/IncreasedTrigger'
 * '<S27>'  : 'HOD_Model/HOD/HOD_60ms/DAE/DriverAbuseMonitoring/TriggerConditions'
 * '<S28>'  : 'HOD_Model/HOD/HOD_60ms/DAE/DriverAbuseMonitoring/WarningDurationCounter'
 * '<S29>'  : 'HOD_Model/HOD/HOD_60ms/DAE/DriverAbuseMonitoring/IncreasedTrigger/EdgeFalling'
 * '<S30>'  : 'HOD_Model/HOD/HOD_60ms/DAE/DriverAbuseMonitoring/IncreasedTrigger/RSFlipFlop'
 * '<S31>'  : 'HOD_Model/HOD/HOD_60ms/DAE/DriverAbuseMonitoring/TriggerConditions/EdgeFalling'
 * '<S32>'  : 'HOD_Model/HOD/HOD_60ms/DAE/DriverAbuseMonitoring/TriggerConditions/EdgeFalling1'
 * '<S33>'  : 'HOD_Model/HOD/HOD_60ms/DAE/DriverAbuseMonitoring/TriggerConditions/EdgeRising'
 * '<S34>'  : 'HOD_Model/HOD/HOD_60ms/DAE/DriverAbuseMonitoring/TriggerConditions/EdgeRising1'
 * '<S35>'  : 'HOD_Model/HOD/HOD_60ms/DAE/DriverAbuseMonitoring/TriggerConditions/RSFlipFlop'
 * '<S36>'  : 'HOD_Model/HOD/HOD_60ms/DAE/DriverAbuseMonitoring/WarningDurationCounter/EdgeFalling'
 * '<S37>'  : 'HOD_Model/HOD/HOD_60ms/DAE/DriverAbuseMonitoring/WarningDurationCounter/RSFlipFlop'
 * '<S38>'  : 'HOD_Model/HOD/HOD_60ms/DAE/VelocityCheck/Hysteresis1'
 * '<S39>'  : 'HOD_Model/HOD/HOD_60ms/EnableCondition/ EstimatedHandTorqueIntervals'
 * '<S40>'  : 'HOD_Model/HOD/HOD_60ms/EnableCondition/HandTorqueEstimaton'
 * '<S41>'  : 'HOD_Model/HOD/HOD_60ms/EnableCondition/HandsOffTorque'
 * '<S42>'  : 'HOD_Model/HOD/HOD_60ms/EnableCondition/ EstimatedHandTorqueIntervals/LeftIntervals'
 * '<S43>'  : 'HOD_Model/HOD/HOD_60ms/EnableCondition/ EstimatedHandTorqueIntervals/RightIntervals'
 * '<S44>'  : 'HOD_Model/HOD/HOD_60ms/EnableCondition/HandTorqueEstimaton/LowPassFilter'
 * '<S45>'  : 'HOD_Model/HOD/HOD_60ms/EnableCondition/HandTorqueEstimaton/Saturation Dynamic3'
 * '<S46>'  : 'HOD_Model/HOD/HOD_60ms/EnableCondition/HandTorqueEstimaton/LowPassFilter/Saturation Dynamic1'
 * '<S47>'  : 'HOD_Model/HOD/HOD_60ms/EstimatedHandTorqueState/CounterAndHandsOffLim'
 * '<S48>'  : 'HOD_Model/HOD/HOD_60ms/EstimatedHandTorqueState/HandTorqueState'
 * '<S49>'  : 'HOD_Model/HOD/HOD_60ms/EstimatedHandTorqueState/CounterAndHandsOffLim/CounterIncrementAndReset'
 * '<S50>'  : 'HOD_Model/HOD/HOD_60ms/EstimatedHandTorqueState/CounterAndHandsOffLim/HandsOffCounterLimit'
 * '<S51>'  : 'HOD_Model/HOD/HOD_60ms/EstimatedHandTorqueState/CounterAndHandsOffLim/HandsOffCounterLimit/BigGradientTorqueAfterInt'
 * '<S52>'  : 'HOD_Model/HOD/HOD_60ms/EstimatedHandTorqueState/CounterAndHandsOffLim/HandsOffCounterLimit/CounterlimitUpdate'
 * '<S53>'  : 'HOD_Model/HOD/HOD_60ms/EstimatedHandTorqueState/CounterAndHandsOffLim/HandsOffCounterLimit/BigGradientTorqueAfterInt/BigGradientTorque'
 * '<S54>'  : 'HOD_Model/HOD/HOD_60ms/EstimatedHandTorqueState/CounterAndHandsOffLim/HandsOffCounterLimit/BigGradientTorqueAfterInt/CounterLimitInitialisation'
 * '<S55>'  : 'HOD_Model/HOD/HOD_60ms/EstimatedHandTorqueState/CounterAndHandsOffLim/HandsOffCounterLimit/BigGradientTorqueAfterInt/HandsOffSensedTorque'
 * '<S56>'  : 'HOD_Model/HOD/HOD_60ms/EstimatedHandTorqueState/HandTorqueState/CalculateHandTorqueState'
 * '<S57>'  : 'HOD_Model/HOD/HOD_60ms/EstimatedHandTorqueState/HandTorqueState/SuspiciousState'
 * '<S58>'  : 'HOD_Model/HOD/HOD_60ms/EstimatedHandTorqueState/HandTorqueState/CalculateHandTorqueState/HandTorqueStateMachine1'
 */
#endif                                 /* RTW_HEADER_HOD_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
