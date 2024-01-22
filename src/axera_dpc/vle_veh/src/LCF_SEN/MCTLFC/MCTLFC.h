/*
 * File: MCTLFC.h
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

#ifndef RTW_HEADER_MCTLFC_h_
#define RTW_HEADER_MCTLFC_h_
#ifndef MCTLFC_COMMON_INCLUDES_
#define MCTLFC_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif /* MCTLFC_COMMON_INCLUDES_ */

#include "MCTLFC_types.h"

/* Macros for accessing real-time model data structure */

/* Block signals (default storage) */
typedef struct {
    uint8_T S_MCTLFC_ControllingFunction_nu; /* '<S5>/MCT_StateMachinet' */
} B_MCTLFC_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
    real32_T UnitDelay_DSTATE[7];    /* '<S13>/Unit Delay' */
    uint8_T UnitDelay_DSTATE_e[7];   /* '<S11>/Unit Delay' */
    boolean_T UnitDelay_DSTATE_c[7]; /* '<S12>/Unit Delay' */
    uint8_T is_active_c3_MCTLFC;     /* '<S5>/MCT_StateMachinet' */
    uint8_T is_c3_MCTLFC;            /* '<S5>/MCT_StateMachinet' */
} DW_MCTLFC_T;

/* Block signals (default storage) */
extern B_MCTLFC_T MCTLFC_B;

/* Block states (default storage) */
extern DW_MCTLFC_T MCTLFC_DW;

/*
 * Exported Global Signals
 *
 * Note: Exported global signals are block signals with an exported global
 * storage class designation.  Code generation will declare the memory for
 * these signals and export their symbols.
 *
 */
extern real32_T CSCLTA_LeCridrBndPosX0_met; /* '<S7>/Multiport Switch1'
                                             * S_CSCLTA_LeCridrBndPosX0_met,
                                             * float32, the PosX0 value of left
                                             * corridor bound, [-300, 300]
                                             */
extern real32_T CSCLTA_LeCridrBndPosY0_met; /* '<S7>/Multiport Switch1'
                                             * S_CSCLTA_LeCridrBndPosY0_met,
                                             * float32, the PosY0 value of left
                                             * corridor bound, [-15, 15]
                                             */
extern real32_T CSCLTA_LeCridrHeadAng_rad;  /* '<S7>/Multiport Switch1'
                                             * S_CSCLTA_LeCridrBndHeadAng_rad,
                                             * float32, the heading angle value
                                             * of left corridor bound,
                                             * [-0.78539816, 0.78539816]
                                             */
extern real32_T CSCLTA_LeCridrBndCrv_1pm;   /* '<S7>/Multiport Switch1'
                                             * S_CSCLTA_LeCridrBndCrv_1pm,
                                             * float32, the curve value of left
                                             * corridor bound, [-0.1, 0.1]
                                             */
extern real32_T
    CSCLTA_LeCridrBndCrvChng_1pm2;           /* '<S7>/Multiport Switch1'
                                              * S_CSCLTA_LeCridrBndCrvChng_1pm2, the curve
                                              * deviation value of left corridor bound,
                                              * [-0.001, 0.001]
                                              */
extern real32_T CSCLTA_LeCridrBndLength_met; /* '<S7>/Multiport Switch1'
                                              * S_CSCLTA_LeCridrBndLength_met,
                                              * the length value of left
                                              * corridor bound, [0, 150]
                                              */
extern real32_T CSCLTA_RiCridrBndPosX0_met;  /* '<S7>/Multiport Switch1'
                                              * S_CSCLTA_RiCridrBndPosX0_met,
                                              * float32, the PosX0 value of right
                                              * corridor bound, [-300, 300]
                                              */
extern real32_T CSCLTA_RiCridrBndPosY0_met;  /* '<S7>/Multiport Switch1'
                                              * S_CSCLTA_RiCridrBndPosY0_met,
                                              * float32, the PosY0 value of right
                                              * corridor bound, [-15, 15]
                                              */
extern real32_T CSCLTA_RiCridrHeadAng_rad;   /* '<S7>/Multiport Switch1'
                                              * S_CSCLTA_RiCridrBndHeadAng_rad,
                                              * float32, the heading angle value
                                              * of right corridor bound,
                                              * [-0.78539816, 0.78539816]
                                              */
extern real32_T CSCLTA_RiCridrBndCrv_1pm;    /* '<S7>/Multiport Switch1'
                                              * S_CSCLTA_RiCridrBndCrv_1pm,
                                              * float32, the curve value of right
                                              * corridor bound, [-0.1, 0.1]
                                              */
extern real32_T
    CSCLTA_RiCridrBndCrvChng_1pm2;            /* '<S7>/Multiport Switch1'
                                               * S_CSCLTA_RiCridrBndCrvChng_1pm2, the curve
                                               * deviation value of right corridor bound,
                                               * [-0.001, 0.001]
                                               */
extern real32_T CSCLTA_RiCridrBndLength_met;  /* '<S7>/Multiport Switch1'
                                               * S_CSCLTA_RiCridrBndLength_met,
                                               * the length value of right
                                               * corridor bound, [0, 150]
                                               */
extern real32_T CSCLTA_TgtTrajPosX0_met;      /* '<S7>/Multiport Switch1'
                                               * S_CSCLTA_TgtTrajPosX0_met, float32,
                                               * the PosX0 value of target corridor
                                               * bound, [-300, 300]
                                               */
extern real32_T CSCLTA_TgtTrajPosY0_met;      /* '<S7>/Multiport Switch1'
                                               * S_CSCLTA_TgtTrajPosY0_met, float32,
                                               * the PosY0 value of target corridor
                                               * bound, [-15, 15]
                                               */
extern real32_T CSCLTA_TgtTrajHeadAng_rad;    /* '<S7>/Multiport Switch1'
                                               * S_CSCLTA_TgtTrajHeadAng_rad,
                                               * float32, the heading angle value
                                               * of target corridor bound,
                                               * [-0.78539816, 0.78539816]
                                               */
extern real32_T CSCLTA_TgtTrajCrv_1pm;        /* '<S7>/Multiport Switch1'
                                               * S_CSCLTA_TgtTrajCrv_1pm, float32, the
                                               * curve value of target corridor bound,
                                               * [-0.1, 0.1]
                                               */
extern real32_T CSCLTA_TgtTrajCrvChng_1pm2;   /* '<S7>/Multiport Switch1'
                                               * S_CSCLTA_TgtTrajCrvChng_1pm2, the
                                               * curve deviation value of target
                                               * corridor bound, [-0.001, 0.001]
                                               */
extern real32_T CSCLTA_TgtTrajLength_met;     /* '<S7>/Multiport Switch1'
                                               * S_CSCLTA_TgtTrajLength_met, the
                                               * length value of target corridor
                                               * bound, [0, 150]
                                               */
extern real32_T CSCLTA_WeightTgtDistY_nu;     /* '<S7>/Multiport Switch1'
                                               * S_CSCLTA_WeightTgtDistY_nu,
                                               * float32, The importance factor of
                                               * the lateral deviation, [0,1]
                                               */
extern real32_T CSCLTA_WeightEndTime_nu;      /* '<S7>/Multiport Switch1'
                                               * S_CSCLTA_WeightEndTime_nu, float32,
                                               * The importance factor of the time
                                               * required for the planned trajectory,
                                               * [0,1]
                                               */
extern real32_T CSCLTA_DistYToILeTgtArea_met; /* '<S7>/Multiport Switch1'
                                               * S_CSCLTA_DistYTolLeTgtArea_met,
                                               * float32, lateral tolerance left
                                               * boundary value , [0,10]
                                               */
extern real32_T CSCLTA_DistYToIRiTgtArea_met; /* '<S7>/Multiport Switch1'
                                               * S_CSCLTA_DistYTolRiTgtArea_met,
                                               * float32, lateral tolerance
                                               * right boundary value , [0,10]
                                               */
extern real32_T CSCLTA_FTireAclMax_mps2;      /* '<S7>/Multiport Switch1'
                                               * S_CSCLTA_FTireAclMax_mps2, float32,
                                               * lateral acceleration upper limiting
                                               * value, [-20,20]
                                               */
extern real32_T CSCLTA_FTireAclMin_mps2;      /* '<S7>/Multiport Switch1'
                                               * S_CSCLTA_FTireAclMin_mps2, float32,
                                               * lateral acceleration lower limiting
                                               * value, [-20,20]
                                               */
extern real32_T CSCLTA_PredTimeHeadAng_sec;   /* '<S7>/Multiport Switch1'
                                               * S_CSCLTA_PredTimeHeadAng_sec,
                                               * float32， todo, [0，60]
                                               */
extern real32_T
    CSCLTA_PredTimeCrv_sec;                   /* '<S7>/Multiport Switch1'
                                               * S_CSCLTA_PredictionTimeCrv_sec,float32,
                                               * todo,[0，60]
                                               */
extern real32_T CSCLTA_PlanningHorzion_sec;   /* '<S7>/Multiport Switch1'
                                               * S_CSCLTA_PlanningHorizon_sec,
                                               * float32, max Planning
                                               * horizon(time) of the trajectory,
                                               * [0，60]
                                               */
extern real32_T CSCLTA_ObstacleVelX_mps;      /* '<S7>/Multiport Switch1'
                                               * S_CSCLTA_ObstacleVelX_mps, float32,
                                               * the obstacle velocity X in target
                                               * trajectory if it is existed ,
                                               * [-20,150]
                                               */
extern real32_T CSCLTA_ObstacleAclX_mps2;     /* '<S7>/Multiport Switch1'
                                               * S_CSCLTA_ObstacleAclX_mps2,
                                               * float32, the obstacle accel X in
                                               * target trajectory if it is existed
                                               * , [-20,20]
                                               */
extern real32_T CSCLTA_ObstacleWidth_met;     /* '<S7>/Multiport Switch1'
                                               * S_CSCLTA_ObstacleWidth_met,
                                               * float32, the obstacle width in
                                               * target trajectory if it is existed
                                               * , [0,150]
                                               */
extern real32_T CSCLTA_ObstacleDistX_met;     /* '<S7>/Multiport Switch1'
                                               * S_CSCLTA_ObstacleDistX_met,
                                               * float32, the obstacle distance X in
                                               * target trajectory if it is existed
                                               * , [-1000,1000]
                                               */
extern real32_T CSCLTA_ObstacleDistY_met;     /* '<S7>/Multiport Switch1'
                                               * S_CSCLTA_ObstacleDistY_met,
                                               * float32, the obstacle distance X in
                                               * target trajectory if it is existed
                                               * , [-1000,1000]
                                               */
extern real32_T CSCLTA_SensorTStamp_sec;      /* '<S7>/Multiport Switch1'
                                               * S_CSCLTA_SensorTStamp_sec, float32,
                                               * time stamp of the camera signal from
                                               * camera sensor,[0,4295]
                                               */
extern real32_T CSCLTA_MaxCrvTrajGuiCtrl_1pm; /* '<S7>/Multiport Switch1' */
extern real32_T CSCLTA_MaxCrvGrdBuildup_1pms; /* '<S7>/Multiport Switch1' */
extern real32_T CSCLTA_MaxCrvGrdRed_1pms;     /* '<S7>/Multiport Switch1' */
extern real32_T CSCLTA_GrdLimitTgtCrvGC_1pms; /* '<S7>/Multiport Switch1' */
extern real32_T CSCLTA_StrWhStifLimit_nu;     /* '<S7>/Multiport Switch1' */
extern real32_T CSCLTA_StrWhStifGrad_1ps;     /* '<S7>/Multiport Switch1' */
extern real32_T CSCLTA_TrqRampGrad_1ps;       /* '<S7>/Multiport Switch1' */
extern real32_T CSCLTA_MaxTrqScalLimit_nu;    /* '<S7>/Multiport Switch1' */
extern real32_T CSCLTA_MaxTrqScalGrad_nu;     /* '<S7>/Multiport Switch1' */
extern real32_T CSCLTA_LimiterDuration_sec;   /* '<S7>/Multiport Switch1' */
extern real32_T CSCLTA_MaxJerkAllowed_mps3;   /* '<S7>/Multiport Switch1'
                                               * S_CSCLTA_MaxJerkAllowed_mps3,
                                               * float32, Maximum Jerk Allowed in
                                               * the trajectory planning, [0,50]
                                               */
extern uint8_T CSCLTA_ControllingFunction_nu; /* '<S5>/Switch' */
extern uint8_T CSCLTA_SysStateLCF;            /* '<S7>/Multiport Switch1'
                                               * S_CSCLTA_SysStateLCF_enum, uint8, lateral
                                               * control function system state enum value,
                                               * [0,6],E_LCF_SYSSTATE_NOTPRESENT = 0;
                                               * E_LCF_SYSSTATE_DISABLED = 1;
                                               * E_LCF_SYSSTATE_PASSIVE = 2;
                                               * E_LCF_SYSSTATE_REQUEST = 3;
                                               * E_LCF_SYSSTATE_CONTROLLING = 4;
                                               * E_LCF_SYSSTATE_RAMPOUT = 5;
                                               * E_LCF_SYSSTATE_ERROR = 6;
                                               */
extern uint8_T CSCLTA_TgtPlanServQu_nu;       /* '<S7>/Multiport Switch1'
                                               * S_CSCLTA_TrajPlanServQu_nu, uint8,
                                               * todo, [0,255],P_TJATVG_TrajPlanValServQu_nu	= +/ 0x00,P_TJATVG_TrajPlanValSrvQuSALC_nu	= +/ 0x10,
                                               */
extern uint8_T CSCLTA_TrajGuiQualifier_nu;    /* '<S7>/Multiport Switch1'
                                               * S_CSCLTA_TrajGuiQualifier_nu,
                                               uint8, qualifier value of trajectory
                                               guidence, The qualifier indicates
                                               if/how the target curvature should
                                               be considered, [0,5]
                                               .E_LCF_TGQ_REQ_OFF= 0, E_LCF_TGQ_REQ
                                               = 1, E_LCF_TGQ_REQ_FREEZE= 3,
                                               E_LCF_TGQ_REQ_FFC	= 4,
                                               E_LCF_TGQ_REQ_REFCHNG= 5
   
                                               */
extern uint8_T CSCLTA_DeratingLevel_nu;       /* '<S7>/Multiport Switch1' */
extern boolean_T
    CSCLTA_TriggerReplan_nu; /* '<S7>/Multiport Switch1'
                              * S_CSCLTA_TriggerReplan_nu,uint8,trigger replan
                              * signal from CSCLTA module,[0，1]
                              */
extern boolean_T
    CSCLTA_LatencyCompActivated_bool;        /* '<S7>/Multiport Switch1'
                                              * S_CSCLTA_LatencyCompActivated_nu,
                                              * uint8, the trigger flag for latency
                                              * compensation function, [0,1] 1: latency
                                              * compensation enable, 0: latency
                                              * compensation disable
                                              */
extern boolean_T CSCLTA_HighStatAccu_bool;   /* '<S7>/Multiport Switch1' */
extern boolean_T CSCLTA_LimiterActivated_nu; /* '<S7>/Multiport Switch1' */

/* Model entry point functions */
extern void MCTLFC_initialize(void);
extern void MCTLFC_step(void);

/* Exported data declaration */

/* Const memory section */
/* Declaration for custom storage class: Const */
extern const uint8_T
    MCTLFC_ControllingFunction_nu;          /* Referenced by: '<S5>/Constant' */
extern const uint8_T MCTLFC_E_LCF_ALCA_nu;  /* Referenced by:
                                             * '<S5>/Constant11'
                                             * '<S5>/Constant18'
                                             * '<S14>/Constant6'
                                             */
extern const uint8_T MCTLFC_E_LCF_AOLC_nu;  /* Referenced by:
                                             * '<S5>/Constant12'
                                             * '<S5>/Constant17'
                                             */
extern const uint8_T MCTLFC_E_LCF_ESA_nu;   /* Referenced by:
                                             * '<S5>/Constant13'
                                             * '<S5>/Constant19'
                                             */
extern const uint8_T MCTLFC_E_LCF_LDPOC_nu; /* Referenced by:
                                             * '<S5>/Constant15'
                                             * '<S5>/Constant21'
                                             * '<S14>/Constant5'
                                             */
extern const uint8_T MCTLFC_E_LCF_LDP_nu;   /* Referenced by:
                                             * '<S5>/Constant14'
                                             * '<S5>/Constant20'
                                             * '<S14>/Constant3'
                                             */
extern const uint8_T MCTLFC_E_LCF_OFF_nu; /* Referenced by: '<S5>/Constant16' */
extern const uint8_T MCTLFC_E_LCF_RDP_nu; /* Referenced by:
                                           * '<S5>/Constant22'
                                           * '<S5>/Constant24'
                                           * '<S14>/Constant4'
                                           */
extern const uint8_T MCTLFC_E_LCF_SYSSTATE_CONTROLLING_nu; /* Referenced by:
                                                            * '<S10>/Constant1'
                                                            * '<S10>/Constant5'
                                                            */
extern const uint8_T
    MCTLFC_E_LCF_SYSSTATE_NOTPRESENT_nu; /* Referenced by: '<S15>/Constant52' */
extern const uint8_T
    MCTLFC_E_LCF_SYSSTATE_RAMPOUT_nu; /* Referenced by: '<S10>/Constant2' */
extern const uint8_T MCTLFC_E_LCF_SYSSTATE_REQUEST_nu; /* Referenced by:
                                                        * '<S10>/Constant'
                                                        * '<S10>/Constant3'
                                                        */
extern const uint8_T
    MCTLFC_E_LCF_TGO_REQ_OFF; /* Referenced by: '<S15>/Constant71' */
extern const uint8_T MCTLFC_E_LCF_TJA_nu; /* Referenced by:
                                           * '<S5>/Constant1'
                                           * '<S5>/Constant23'
                                           * '<S14>/Constant'
                                           */
extern const uint8_T
    MCTLFC_ErrInjEnable_bool; /* Referenced by: '<S5>/Constant2' */
extern const real32_T
    MCTLFC_FollowUpDurationVec_sec[7]; /* Referenced by: '<S11>/Constant1' */
extern const uint8_T
    MCTLFC_PriorityVector_nu[7]; /* Referenced by: '<S10>/Constant9' */
extern const boolean_T
    MCTLFC_RampOutOverwrite_bool[7]; /* Referenced by: '<S10>/Constant4' */

/* Declaration for custom storage class: Global */
extern uint8_T MCTLFC_t_prio_ALCA;  /* '<S5>/Selector' */
extern uint8_T MCTLFC_t_prio_AOLC;  /* '<S5>/Selector1' */
extern uint8_T MCTLFC_t_prio_ESA;   /* '<S5>/Selector2' */
extern uint8_T MCTLFC_t_prio_LDP;   /* '<S5>/Selector3' */
extern uint8_T MCTLFC_t_prio_LDPOC; /* '<S5>/Selector4' */
extern uint8_T MCTLFC_t_prio_RDP;   /* '<S5>/Selector5' */
extern uint8_T MCTLFC_t_prio_TJA;   /* '<S5>/Selector6' */
extern uint8_T MCTLFC_t_prio_max;   /* '<S5>/Max' */

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S1>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S1>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S1>/Data Type Conversion10' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion11' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion12' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion13' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion14' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion15' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion16' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion17' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion18' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion19' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion2' : Eliminate redundant data type conversion
 * Block '<S1>/Data Type Conversion20' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion21' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion22' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion23' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion24' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion25' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion26' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion27' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion28' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion29' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion3' : Eliminate redundant data type conversion
 * Block '<S1>/Data Type Conversion30' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion31' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion32' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion33' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion34' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion35' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion36' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion37' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion38' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion39' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion4' : Eliminate redundant data type conversion
 * Block '<S1>/Data Type Conversion40' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion41' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion42' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion43' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion44' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion45' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion46' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion47' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion48' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion49' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion5' : Eliminate redundant data type conversion
 * Block '<S1>/Data Type Conversion50' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion51' : Eliminate redundant data type
 * conversion
 * Block '<S1>/Data Type Conversion6' : Eliminate redundant data type conversion
 * Block '<S1>/Data Type Conversion7' : Eliminate redundant data type conversion
 * Block '<S1>/Data Type Conversion8' : Eliminate redundant data type conversion
 * Block '<S1>/Data Type Conversion9' : Eliminate redundant data type conversion
 * Block '<S11>/Constant2' : Unused code path elimination
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
 * hilite_system('MCTLFC_model/MCTLFC')    - opens subsystem MCTLFC_model/MCTLFC
 * hilite_system('MCTLFC_model/MCTLFC/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'MCTLFC_model'
 * '<S1>'   : 'MCTLFC_model/MCTLFC'
 * '<S2>'   : 'MCTLFC_model/MCTLFC/ALCA'
 * '<S3>'   : 'MCTLFC_model/MCTLFC/LDP'
 * '<S4>'   : 'MCTLFC_model/MCTLFC/LDPOC'
 * '<S5>'   : 'MCTLFC_model/MCTLFC/MCTLFC'
 * '<S6>'   : 'MCTLFC_model/MCTLFC/RDP'
 * '<S7>'   : 'MCTLFC_model/MCTLFC/SignalProcess'
 * '<S8>'   : 'MCTLFC_model/MCTLFC/TJA'
 * '<S9>'   : 'MCTLFC_model/MCTLFC/MCTLFC/MCT_StateMachinet'
 * '<S10>'  : 'MCTLFC_model/MCTLFC/MCTLFC/RequestLCF'
 * '<S11>'  : 'MCTLFC_model/MCTLFC/MCTLFC/RequestLCF/FollowUpTimer'
 * '<S12>'  : 'MCTLFC_model/MCTLFC/MCTLFC/RequestLCF/FollowUpTimer/EdgeRising'
 * '<S13>'  :
 * 'MCTLFC_model/MCTLFC/MCTLFC/RequestLCF/FollowUpTimer/TimerRetrigger'
 * '<S14>'  : 'MCTLFC_model/MCTLFC/SignalProcess/SignalSelection'
 * '<S15>'  :
 * 'MCTLFC_model/MCTLFC/SignalProcess/Signal_Preparation_for_SwitchDefault'
 */
#endif /* RTW_HEADER_MCTLFC_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
