/**********************************Model
 Property********************************
 *
 * Company             : SENSETIME
 *
 * Tool Version        : Ver2.0
 *
 * Model Name          : LCCRA
 *
 * Model Long Name     : LCCRA

 *

 * Model Advisor       : Not Check

 *

 * Model Version       : Ver1.0

 *

 * Model Author        : ZhuHe

 *

 * Model Reviewer      :

 *

 * Model Review Data   :

 *

 * Model Cycle Time    : 50ms


 ************************************Auto
 Coder**********************************
 *
 * File                             : LCCRA.h
 *
 * FileType                         : Code Header File
 *
 * Real-Time Workshop file version  : 9.4 (R2020b) 29-Jul-2020
 *
 * TLC version                      : 9.4 (Aug 20 2020)
 *
 * C source code generated on       : Fri Feb  3 11:44:07 2023
 *
 * Copyright (C) by SenseTime Group Limited. All rights reserved.
 *******************************************************************************/

#ifndef RTW_HEADER_LCCRA_h_
#define RTW_HEADER_LCCRA_h_
#include <math.h>
#include <string.h>
#ifndef LCCRA_COMMON_INCLUDES_
#define LCCRA_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif /* LCCRA_COMMON_INCLUDES_ */

#include "LCCRA_types.h"

/* Macros for accessing real-time model data structure */

/* Block states (default storage) for system '<Root>' */
typedef struct {
    BusVehicle LCCRA_TargetVehicleUnitDelay_st; /* '<S1>/Unit Delay2' */
    BusMIOs LCCRA_MIOUnitDelay_str;             /* '<S1>/Unit Delay1' */
    real32_T LCCRA_egofrontdelaytime_sec;       /* '<S1>/LCCRA_FindMIOs' */
    real32_T LCCRA_egoreardelaytime_sec;        /* '<S1>/LCCRA_FindMIOs' */
    real32_T LCCRA_leftfrontdelaytime_sec;      /* '<S1>/LCCRA_FindMIOs' */
    real32_T LCCRA_leftreardelaytime_sec;       /* '<S1>/LCCRA_FindMIOs' */
    real32_T LCCRA_rightfrontdelaytime_sec;     /* '<S1>/LCCRA_FindMIOs' */
    real32_T LCCRA_rightreardelaytime_sec;      /* '<S1>/LCCRA_FindMIOs' */
    real32_T LCCRA_nextleftfrontdelaytime_se;   /* '<S1>/LCCRA_FindMIOs' */
    real32_T LCCRA_nextleftreardelaytime_sec;   /* '<S1>/LCCRA_FindMIOs' */
    real32_T LCCRA_nextrightfrontdelaytime_s;   /* '<S1>/LCCRA_FindMIOs' */
    real32_T LCCRA_nextrightreardelaytime_se;   /* '<S1>/LCCRA_FindMIOs' */
    uint8_T LCCRA_egofrontweight_nu;            /* '<S1>/LCCRA_FindMIOs' */
    uint8_T LCCRA_egorearweight_nu;             /* '<S1>/LCCRA_FindMIOs' */
    uint8_T LCCRA_leftfrontweight_nu;           /* '<S1>/LCCRA_FindMIOs' */
    uint8_T LCCRA_leftrearweight_nu;            /* '<S1>/LCCRA_FindMIOs' */
    uint8_T LCCRA_rightfrontweight_nu;          /* '<S1>/LCCRA_FindMIOs' */
    uint8_T LCCRA_rightrearweight_nu;           /* '<S1>/LCCRA_FindMIOs' */
    uint8_T LCCRA_nextleftfrontweight_nu;       /* '<S1>/LCCRA_FindMIOs' */
    uint8_T LCCRA_nextleftrearweight_nu;        /* '<S1>/LCCRA_FindMIOs' */
    uint8_T LCCRA_nextrightfrontweight_nu;      /* '<S1>/LCCRA_FindMIOs' */
    uint8_T LCCRA_nextrightrearweight_nu;       /* '<S1>/LCCRA_FindMIOs' */
} DW_LCCRA_T;

/* Constant parameters (default storage) */
typedef struct {
    /* Expression: UnitDelayInitialValue
     * Referenced by: '<S1>/Unit Delay2'
     */
    BusVehicle UnitDelay2_InitialCondition;

    /* Expression: UnitDelayInitialValue
     * Referenced by: '<S1>/Unit Delay1'
     */
    BusMIOs UnitDelay1_InitialCondition;
} ConstP_LCCRA_T;

/* Block states (default storage) */
extern DW_LCCRA_T LCCRA_DW;

/* External data declarations for dependent source files */
extern const BusVehicle LCCRA_rtZBusVehicle;     /* BusVehicle ground */
extern const BusDebugMIOs LCCRA_rtZBusDebugMIOs; /* BusDebugMIOs ground */
extern const BusWeight LCCRA_rtZBusWeight;       /* BusWeight ground */
extern const BusDelayTime LCCRA_rtZBusDelayTime; /* BusDelayTime ground */

/* Constant parameters (default storage) */
extern const ConstP_LCCRA_T LCCRA_ConstP;

/*
 * Exported Global Signals
 *
 * Note: Exported global signals are block signals with an exported global
 * storage class designation.  Code generation will declare the memory for
 * these signals and export their symbols.
 *
 */
extern int32_T LCCRA_FrontDangerObjID_nu;       /* '<S14>/Switch1' */
extern int32_T LCCRA_RightDangerObjID_nu;       /* '<S16>/MATLAB Function' */
extern int32_T LCCRA_LeftDangerObjID_nu;        /* '<S15>/MATLAB Function' */
extern boolean_T LCCRA_bLeftRearTTCSafe;        /* '<S32>/Switch' */
extern boolean_T LCCRA_bLeftRearTGSafe;         /* '<S31>/Switch' */
extern boolean_T LCCRA_bLeftRearRDSafe;         /* '<S30>/Switch' */
extern boolean_T LCCRA_bLeftFrontTTCSafe;       /* '<S29>/Switch' */
extern boolean_T LCCRA_bLeftFrontTGSafe;        /* '<S28>/Switch' */
extern boolean_T LCCRA_bLeftFrontRDSafe;        /* '<S27>/Switch' */
extern boolean_T LCCRA_bNextLeftFrontSafe;      /* '<S33>/Switch' */
extern boolean_T LCCRA_bNextLeftRearSafe;       /* '<S34>/Switch' */
extern boolean_T LCCRA_LeftFrontSafeFlag_bool;  /* '<S15>/AND2' */
extern boolean_T LCCRA_LeftRearSafeFlag_bool;   /* '<S15>/AND3' */
extern boolean_T LCCRA_bRightRearTTCSafe;       /* '<S51>/Switch' */
extern boolean_T LCCRA_bRightRearTGSafe;        /* '<S50>/Switch' */
extern boolean_T LCCRA_bRightRearRDSafe;        /* '<S49>/Switch' */
extern boolean_T LCCRA_bRightFrontTTCSafe;      /* '<S48>/Switch' */
extern boolean_T LCCRA_bRightFrontTGSafe;       /* '<S47>/Switch' */
extern boolean_T LCCRA_bRightFrontRDSafe;       /* '<S46>/Switch' */
extern boolean_T LCCRA_bNextRightFrontSafe;     /* '<S44>/Switch' */
extern boolean_T LCCRA_bNextRightRearSafe;      /* '<S45>/Switch' */
extern boolean_T LCCRA_RightFrontSafeFlag_bool; /* '<S16>/AND2' */
extern boolean_T LCCRA_RightRearSafeFlag_bool;  /* '<S16>/AND3' */
extern boolean_T LCCRA_bFrontTTCSafe;           /* '<S17>/Switch' */
extern boolean_T LCCRA_FrontSafeFlag_bool;      /* '<S14>/Switch' */
extern boolean_T LCCRA_RightSafeFlag_bool;      /* '<S16>/MATLAB Function' */
extern boolean_T LCCRA_LeftSafeFlag_bool;       /* '<S15>/MATLAB Function' */

/* Model entry point functions */
extern void LCCRA_initialize(void);
extern void LCCRA_step(void);

/* Exported data declaration */

/* ConstVolatile memory section */
/* Declaration for custom storage class: ConstVolatile */
extern const volatile real32_T
    LCCRA_CamExistThreshold_C_nu; /* Referenced by: '<S1>/Constant15' */
extern const volatile real32_T
    LCCRA_DefaultLaneWidth_C_met; /* Referenced by: '<S1>/Constant5' */
extern const volatile real32_T
    LCCRA_DelayTimeValue_C_nu[10]; /* Referenced by: '<S1>/Constant12' */
extern const volatile real32_T
    LCCRA_EgoFrontTTCThresholdBus_C_sec; /* Referenced by: '<S1>/Constant35' */
extern const volatile real32_T
    LCCRA_EgoFrontTTCThreshold_C_sec; /* Referenced by: '<S1>/Constant31' */
extern const volatile real32_T
    LCCRA_EgoLineOffset_C_met; /* Referenced by: '<S1>/Constant29' */
extern const volatile real32_T
    LCCRA_EgoRearHangDist_C_met; /* Referenced by: '<S1>/Constant32' */
extern const volatile real32_T
    LCCRA_FrontObjVelX_Bx_kph[10]; /* Referenced by:
                                    * '<S19>/1-D Lookup Table2'
                                    * '<S22>/1-D Lookup Table2'
                                    * '<S25>/1-D Lookup Table2'
                                    * '<S26>/1-D Lookup Table2'
                                    * '<S36>/1-D Lookup Table2'
                                    * '<S37>/1-D Lookup Table2'
                                    * '<S39>/1-D Lookup Table2'
                                    * '<S42>/1-D Lookup Table2'
                                    */
extern const volatile real32_T
    LCCRA_FrontTTCThresholdBus_C_sec; /* Referenced by: '<S1>/Constant33' */
extern const volatile real32_T
    LCCRA_FrontTTCThreshold_C_sec; /* Referenced by: '<S1>/Constant' */
extern const volatile real32_T
    LCCRA_FrontTimeGapBus_Mp_sec[12]; /* Referenced by:
                                       * '<S1>/1-D Lookup Table3'
                                       * '<S19>/1-D Lookup Table3'
                                       * '<S25>/1-D Lookup Table3'
                                       * '<S36>/1-D Lookup Table3'
                                       * '<S39>/1-D Lookup Table3'
                                       */
extern const volatile real32_T
    LCCRA_FrontTimeGap_Mp_sec[12]; /* Referenced by:
                                    * '<S1>/1-D Lookup Table1'
                                    * '<S19>/1-D Lookup Table1'
                                    * '<S25>/1-D Lookup Table1'
                                    * '<S36>/1-D Lookup Table1'
                                    * '<S39>/1-D Lookup Table1'
                                    */
extern const volatile real32_T
    LCCRA_LidarExistThreshold_C_nu; /* Referenced by: '<S1>/Constant16' */
extern const volatile real32_T
    LCCRA_MaxFlagRiskTime_C_sec; /* Referenced by: '<S1>/Constant14' */
extern const volatile real32_T
    LCCRA_MaxFlagTime_C_sec; /* Referenced by: '<S1>/Constant13' */
extern const volatile real32_T
    LCCRA_MaxHeadingAngle_C_rad; /* Referenced by: '<S1>/Constant7' */
extern const volatile real32_T
    LCCRA_MaxPredictionTime_C_sec; /* Referenced by: '<S1>/Constant20' */
extern const volatile uint8_T
    LCCRA_MaxWeightThreshold_C_nu; /* Referenced by: '<S1>/Constant11' */
extern const volatile real32_T
    LCCRA_MinHeadingAngle_C_rad; /* Referenced by: '<S1>/Constant8' */
extern const volatile real32_T
    LCCRA_MinPredictionTime_C_sec; /* Referenced by: '<S1>/Constant21' */
extern const volatile real32_T
    LCCRA_NextTTCThreshold_C_sec; /* Referenced by: '<S1>/Constant2' */
extern const volatile real32_T
    LCCRA_PredictionTime_K_nu; /* Referenced by: '<S1>/Constant18' */
extern const volatile real32_T
    LCCRA_PredictionTime_M_nu; /* Referenced by: '<S1>/Constant19' */
extern const volatile real32_T
    LCCRA_RadarExistThreshold_C_nu; /* Referenced by: '<S1>/Constant10' */
extern const volatile real32_T
    LCCRA_RearTTCThresholdBus_C_sec; /* Referenced by: '<S1>/Constant34' */
extern const volatile real32_T
    LCCRA_RearTTCThreshold_C_sec; /* Referenced by: '<S1>/Constant1' */
extern const volatile real32_T
    LCCRA_RearTimeGapBus_Mp_sec[12]; /* Referenced by:
                                      * '<S1>/1-D Lookup Table4'
                                      * '<S22>/1-D Lookup Table5'
                                      * '<S26>/1-D Lookup Table5'
                                      * '<S37>/1-D Lookup Table5'
                                      * '<S42>/1-D Lookup Table5'
                                      */
extern const volatile real32_T
    LCCRA_RearTimeGap_Mp_sec[12]; /* Referenced by:
                                   * '<S1>/1-D Lookup Table2'
                                   * '<S22>/1-D Lookup Table4'
                                   * '<S26>/1-D Lookup Table4'
                                   * '<S37>/1-D Lookup Table4'
                                   * '<S42>/1-D Lookup Table4'
                                   */
extern const volatile real32_T
    LCCRA_RiskDistThresholdHys_C_met; /* Referenced by: '<S1>/Constant28' */
extern const volatile real32_T
    LCCRA_RiskDistanceX_C_met; /* Referenced by: '<S1>/Constant23' */
extern const volatile real32_T
    LCCRA_RiskDistance_C_met; /* Referenced by: '<S1>/Constant22' */
extern const volatile real32_T
    LCCRA_SelectMIOMaxDis_C_met; /* Referenced by: '<S1>/Constant17' */
extern const volatile real32_T
    LCCRA_TTCSafeDistance_C_met; /* Referenced by: '<S1>/Constant9' */
extern const volatile real32_T
    LCCRA_TTCThresholdHys_C_sec; /* Referenced by: '<S1>/Constant26' */
extern const volatile real32_T
    LCCRA_TimeGapThresholdHys_C_sec; /* Referenced by: '<S1>/Constant27' */
extern const volatile real32_T
    LCCRA_TimeGapThreshold_C_sec; /* Referenced by: '<S1>/Constant3' */
extern const volatile real32_T
    LCCRA_TimeThreshold_C_sec; /* Referenced by: '<S1>/Constant4' */
extern const volatile real32_T
    LCCRA_TmGpVelXFac_Mp_nu[10]; /* Referenced by:
                                  * '<S19>/1-D Lookup Table2'
                                  * '<S22>/1-D Lookup Table2'
                                  * '<S25>/1-D Lookup Table2'
                                  * '<S26>/1-D Lookup Table2'
                                  * '<S36>/1-D Lookup Table2'
                                  * '<S37>/1-D Lookup Table2'
                                  * '<S39>/1-D Lookup Table2'
                                  * '<S42>/1-D Lookup Table2'
                                  */
extern const volatile boolean_T
    LCCRA_UseObjLength_bool; /* Referenced by: '<S1>/Constant30' */
extern const volatile real32_T
    LCCRA_VehVelXTimeGap_Bx_kph[12]; /* Referenced by:
                                      * '<S1>/1-D Lookup Table1'
                                      * '<S1>/1-D Lookup Table2'
                                      * '<S1>/1-D Lookup Table3'
                                      * '<S1>/1-D Lookup Table4'
                                      * '<S19>/1-D Lookup Table1'
                                      * '<S19>/1-D Lookup Table3'
                                      * '<S22>/1-D Lookup Table4'
                                      * '<S22>/1-D Lookup Table5'
                                      * '<S25>/1-D Lookup Table1'
                                      * '<S25>/1-D Lookup Table3'
                                      * '<S26>/1-D Lookup Table4'
                                      * '<S26>/1-D Lookup Table5'
                                      * '<S36>/1-D Lookup Table1'
                                      * '<S36>/1-D Lookup Table3'
                                      * '<S37>/1-D Lookup Table4'
                                      * '<S37>/1-D Lookup Table5'
                                      * '<S39>/1-D Lookup Table1'
                                      * '<S39>/1-D Lookup Table3'
                                      * '<S42>/1-D Lookup Table4'
                                      * '<S42>/1-D Lookup Table5'
                                      */
extern const volatile real32_T
    LCCRA_VelocityThreshold_C_mps; /* Referenced by: '<S1>/Constant6' */

/* Declaration for custom storage class: Global */
extern BusDelayTime LCCRA_DebugDelaytime_str;    /* '<S1>/LCCRA_FindMIOs' */
extern BusDebugMIOs LCCRA_DebugMIOs_str;         /* '<S1>/LCCRA_FindMIOs' */
extern BusWeight LCCRA_DebugWeight_str;          /* '<S1>/LCCRA_FindMIOs' */
extern real32_T LCCRA_LeftFrontSafeDistTmGp_met; /* '<S19>/Product2' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
extern real32_T LCCRA_LeftRearSafeDistTmGp_met; /* '<S22>/Product2' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
extern real32_T LCCRA_NextLeftFrontSfDistTmGp_met; /* '<S25>/Product2' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
extern real32_T LCCRA_NextLeftRearSfDistTmGp_met; /* '<S26>/Product2' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
extern real32_T LCCRA_NextRightFrontSfDistTmGp_met; /* '<S36>/Product2' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
extern real32_T LCCRA_NextRightRearSfDistTmGp_met; /* '<S37>/Product2' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
extern real32_T LCCRA_RightFrontSafeDistTmGp_met; /* '<S39>/Product2' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
extern real32_T LCCRA_RightRearSafeDistTmGp_met; /* '<S42>/Product2' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
extern BusVehicle LCCRA_TargetVehicle_str; /* '<S1>/LCCRA_ObjectPreProcess' */
extern boolean_T LCCRA_bLastFrontTTCSafe;  /* '<S17>/Unit Delay' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
extern boolean_T LCCRA_bLastLeftFrontRDSafe; /* '<S27>/Unit Delay' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
extern boolean_T LCCRA_bLastLeftFrontTGSafe; /* '<S28>/Unit Delay' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
extern boolean_T LCCRA_bLastLeftFrontTTCSafe; /* '<S29>/Unit Delay' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
extern boolean_T LCCRA_bLastLeftRearRDSafe; /* '<S30>/Unit Delay' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
extern boolean_T LCCRA_bLastLeftRearTGSafe; /* '<S31>/Unit Delay' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
extern boolean_T LCCRA_bLastLeftRearTTCSafe; /* '<S32>/Unit Delay' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
extern boolean_T LCCRA_bLastNextLeftFrontSafe; /* '<S33>/Unit Delay' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
extern boolean_T LCCRA_bLastNextLeftRearSafe; /* '<S34>/Unit Delay' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
extern boolean_T LCCRA_bLastNextRightFrontSafe; /* '<S44>/Unit Delay' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
extern boolean_T LCCRA_bLastNextRightRearSafe; /* '<S45>/Unit Delay' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
extern boolean_T LCCRA_bLastRightFrontRDSafe; /* '<S46>/Unit Delay' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
extern boolean_T LCCRA_bLastRightFrontTGSafe; /* '<S47>/Unit Delay' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
extern boolean_T LCCRA_bLastRightFrontTTCSafe; /* '<S48>/Unit Delay' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
extern boolean_T LCCRA_bLastRightRearRDSafe; /* '<S49>/Unit Delay' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
extern boolean_T LCCRA_bLastRightRearTGSafe; /* '<S50>/Unit Delay' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */
extern boolean_T LCCRA_bLastRightRearTTCSafe; /* '<S51>/Unit Delay' */

/* SLC lane width min hystereis flag--Used in TJASLC module
   DT:boolean */

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S1>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S1>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S1>/Data Type Conversion10' : Eliminate redundant data type
 * conversion Block '<S1>/Data Type Conversion11' : Eliminate redundant data
 * type conversion Block '<S1>/Data Type Conversion12' : Eliminate redundant
 * data type conversion Block '<S1>/Data Type Conversion13' : Eliminate
 * redundant data type conversion Block '<S1>/Data Type Conversion14' :
 * Eliminate redundant data type conversion Block '<S1>/Data Type Conversion15'
 * : Eliminate redundant data type conversion Block '<S1>/Data Type
 * Conversion16' : Eliminate redundant data type conversion Block '<S1>/Data
 * Type Conversion17' : Eliminate redundant data type conversion Block
 * '<S1>/Data Type Conversion18' : Eliminate redundant data type conversion
 * Block '<S1>/Data Type Conversion19' : Eliminate redundant data type
 * conversion Block '<S1>/Data Type Conversion2' : Eliminate redundant data type
 * conversion Block '<S1>/Data Type Conversion20' : Eliminate redundant data
 * type conversion Block '<S1>/Data Type Conversion21' : Eliminate redundant
 * data type conversion Block '<S1>/Data Type Conversion22' : Eliminate
 * redundant data type conversion Block '<S1>/Data Type Conversion23' :
 * Eliminate redundant data type conversion Block '<S1>/Data Type Conversion3' :
 * Eliminate redundant data type conversion Block '<S1>/Data Type Conversion4' :
 * Eliminate redundant data type conversion Block '<S1>/Data Type Conversion5' :
 * Eliminate redundant data type conversion Block '<S1>/Data Type Conversion6' :
 * Eliminate redundant data type conversion Block '<S1>/Data Type Conversion7' :
 * Eliminate redundant data type conversion Block '<S1>/Data Type Conversion8' :
 * Eliminate redundant data type conversion Block '<S1>/Data Type Conversion9' :
 * Eliminate redundant data type conversion Block '<S1>/Signal Conversion' :
 * Eliminate redundant signal conversion block Block '<S1>/Signal Conversion1' :
 * Eliminate redundant signal conversion block Block '<S1>/Signal Conversion2' :
 * Eliminate redundant signal conversion block Block '<S1>/Signal Conversion3' :
 * Eliminate redundant signal conversion block
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
 * hilite_system('LCCRA_model/LCCRA')    - opens subsystem LCCRA_model/LCCRA
 * hilite_system('LCCRA_model/LCCRA/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'LCCRA_model'
 * '<S1>'   : 'LCCRA_model/LCCRA'
 * '<S10>'  : 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag'
 * '<S11>'  : 'LCCRA_model/LCCRA/LCCRA_FindMIOs'
 * '<S12>'  : 'LCCRA_model/LCCRA/LCCRA_LanePreProcess'
 * '<S13>'  : 'LCCRA_model/LCCRA/LCCRA_ObjectPreProcess'
 * '<S14>'  : 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag/FrontSafeFlag'
 * '<S15>'  : 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag/LeftSafeFlag'
 * '<S16>'  : 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag/RightSafeFlag'
 * '<S17>'  : 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag/FrontSafeFlag/Hysteresis1'
 * '<S18>'  :
 * 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag/LeftSafeFlag/LeftFrontRiskDistSafe'
 * '<S19>'  :
 * 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag/LeftSafeFlag/LeftFrontTGSafe'
 * '<S20>'  :
 * 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag/LeftSafeFlag/LeftFrontTTCSafe'
 * '<S21>'  :
 * 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag/LeftSafeFlag/LeftRearRiskDistSafe'
 * '<S22>'  :
 * 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag/LeftSafeFlag/LeftRearTGSafe'
 * '<S23>'  :
 * 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag/LeftSafeFlag/LeftRearTTCSafe'
 * '<S24>'  : 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag/LeftSafeFlag/MATLAB
 * Function'
 * '<S25>'  :
 * 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag/LeftSafeFlag/NextLeftFrontSafe'
 * '<S26>'  :
 * 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag/LeftSafeFlag/NextLeftRearSafe'
 * '<S27>'  :
 * 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag/LeftSafeFlag/LeftFrontRiskDistSafe/Hysteresis1'
 * '<S28>'  :
 * 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag/LeftSafeFlag/LeftFrontTGSafe/Hysteresis1'
 * '<S29>'  :
 * 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag/LeftSafeFlag/LeftFrontTTCSafe/Hysteresis1'
 * '<S30>'  :
 * 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag/LeftSafeFlag/LeftRearRiskDistSafe/Hysteresis1'
 * '<S31>'  :
 * 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag/LeftSafeFlag/LeftRearTGSafe/Hysteresis1'
 * '<S32>'  :
 * 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag/LeftSafeFlag/LeftRearTTCSafe/Hysteresis1'
 * '<S33>'  :
 * 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag/LeftSafeFlag/NextLeftFrontSafe/Hysteresis1'
 * '<S34>'  :
 * 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag/LeftSafeFlag/NextLeftRearSafe/Hysteresis1'
 * '<S35>'  : 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag/RightSafeFlag/MATLAB
 * Function'
 * '<S36>'  :
 * 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag/RightSafeFlag/NextRightFrontSafe'
 * '<S37>'  :
 * 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag/RightSafeFlag/NextRightRearSafe'
 * '<S38>'  :
 * 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag/RightSafeFlag/RightFrontRiskDistSafe'
 * '<S39>'  :
 * 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag/RightSafeFlag/RightFrontTGSafe'
 * '<S40>'  :
 * 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag/RightSafeFlag/RightFrontTTCSafe'
 * '<S41>'  :
 * 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag/RightSafeFlag/RightRearRiskDistSafe'
 * '<S42>'  :
 * 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag/RightSafeFlag/RightRearTGSafe'
 * '<S43>'  :
 * 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag/RightSafeFlag/RightRearTTCSafe'
 * '<S44>'  :
 * 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag/RightSafeFlag/NextRightFrontSafe/Hysteresis1'
 * '<S45>'  :
 * 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag/RightSafeFlag/NextRightRearSafe/Hysteresis1'
 * '<S46>'  :
 * 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag/RightSafeFlag/RightFrontRiskDistSafe/Hysteresis1'
 * '<S47>'  :
 * 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag/RightSafeFlag/RightFrontTGSafe/Hysteresis1'
 * '<S48>'  :
 * 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag/RightSafeFlag/RightFrontTTCSafe/Hysteresis1'
 * '<S49>'  :
 * 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag/RightSafeFlag/RightRearRiskDistSafe/Hysteresis1'
 * '<S50>'  :
 * 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag/RightSafeFlag/RightRearTGSafe/Hysteresis1'
 * '<S51>'  :
 * 'LCCRA_model/LCCRA/LCCRA_Dtrm_SafeFlag/RightSafeFlag/RightRearTTCSafe/Hysteresis1'
 */

/*-
 * Requirements for '<Root>': LCCRA
 */
#endif /* RTW_HEADER_LCCRA_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
