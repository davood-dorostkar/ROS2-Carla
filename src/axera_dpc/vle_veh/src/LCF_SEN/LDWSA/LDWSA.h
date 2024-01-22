/**********************************Model Property********************************
 *
 * Company             : SENSETIME
 *
 * Tool Version        : Ver2.0
 *
 * Model Name          : LDWSA
 *
 * Model Long Name     : Lane Departure Warning

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


 ************************************Auto Coder**********************************
 *
 * File                             : LDWSA.h
 *
 * FileType                         : Code Header File
 *
 * Real-Time Workshop file version  : 9.4 (R2020b) 29-Jul-2020
 *
 * TLC version                      : 9.4 (Aug 20 2020)
 *
 * C source code generated on       : Sun Jan 15 13:14:35 2023
 *
 * Copyright (C) by SenseTime Group Limited. All rights reserved.
 *******************************************************************************/

#ifndef RTW_HEADER_LDWSA_h_
#define RTW_HEADER_LDWSA_h_
#include <math.h>
#include <string.h>
#ifndef LDWSA_COMMON_INCLUDES_
#define LDWSA_COMMON_INCLUDES_
#include "Sfun_Set_Bit.h"
#include "rtwtypes.h"
#endif                                 /* LDWSA_COMMON_INCLUDES_ */

#include "LDWSA_types.h"

/* Macros for accessing real-time model data structure */

/* Block states (default storage) for system '<Root>' */
typedef struct {
  uint8_T is_active_c1_LDWSA;          /* '<S5>/LDW_State' */
  uint8_T is_c1_LDWSA;                 /* '<S5>/LDW_State' */
  uint8_T is_LDW_ON;                   /* '<S5>/LDW_State' */
} DW_LDWSA_T;

/* Block states (default storage) */
extern DW_LDWSA_T LDWSA_DW;

/*
 * Exported Global Signals
 *
 * Note: Exported global signals are block signals with an exported global
 * storage class designation.  Code generation will declare the memory for
 * these signals and export their symbols.
 *
 */
extern real32_T LDVSE_NVRAMVehStartupSpd_kmph;/* '<S1>/LDW' */
extern real32_T LDWC_CrvSensiDecayRi_Mi;/* '<S106>/Switch6' */
extern real32_T LDDT_CrvThdMaxRi_ReMi; /* '<S11>/1-D Lookup Table'
                                        * Curve of maximum threshold of right clothiod curvature
                                        */
extern real32_T LDDT_CrvThdHystRi_ReMi;/* '<S11>/1-D Lookup Table1'
                                        * Curve of offset threshold of right clothiod curvature
                                        */
extern real32_T LDDT_LnCltdCurvRi_ReMi;/* '<S6>/Switch5'
                                        * Clothoid curvature of right lane
                                        */
extern real32_T LDDT_LnHeadRi_Rad;     /* '<S6>/Switch1'
                                        * Heading angle of rifht lane
                                        */
extern real32_T LDDT_RawLatVehSpdRi_Mps;/* '<S8>/Product1'
                                         * Raw right lateral vehicle speed
                                         */
extern real32_T LDDT_LatVehSpdRi_Mps;  /* '<S8>/Switch2'
                                        * right lateral vehicle speed
                                        */
extern real32_T LDWC_CrrctByLnWidth_Fct;/* '<S106>/1-D Lookup Table3'
                                         * DLC threshold corretction factor
                                         */
extern real32_T LDWC_DstcToLnTrsdCrvCpstnRi_Mi;/* '<S106>/1-D Lookup Table4'
                                                * Curvature compensation of threshold distance to right lane crossing
                                                */
extern real32_T LDWC_DstcToLnTrsdRi_Mi;/* '<S106>/Add3'
                                        *  Threshold distance to right lane crossing
                                        */
extern real32_T LDDT_LnPstnRi_Mi;      /* '<S6>/Switch3'
                                        * Position of  of right lane
                                        */
extern real32_T LDDT_RawDstcToLnRi_Mi; /* '<S8>/Subtract1'
                                        * Raw Distance of between vehicel and right lane
                                        */
extern real32_T LDDT_DstcToLnRi_Mi;    /* '<S8>/Switch4'
                                        * Distance of between vehicel and right lane
                                        */
extern real32_T LDVSE_MaxLatVel_Mps;   /* '<S22>/Lookup Table'
                                        * Maximum lateral velocity
                                        */
extern real32_T LDDT_CrvThdMaxLf_ReMi; /* '<S10>/1-D Lookup Table'
                                        * Curve of maximum threshold of left clothiod curvature
                                        */
extern real32_T LDDT_CrvThdHystLf_ReMi;/* '<S10>/1-D Lookup Table1'
                                        * Curve of offset threshold of left clothiod curvature
                                        */
extern real32_T LDDT_LnCltdCurvLf_ReMi;/* '<S6>/Switch4'
                                        * Clothoid curvature of left lane
                                        */
extern real32_T LDDT_LnHeadLf_Rad;     /* '<S6>/Switch'
                                        * Heading angle of left lane
                                        */
extern real32_T LDDT_RawLatVehSpdLf_Mps;/* '<S8>/Product'
                                         * Raw Left lateral vehicle speed
                                         */
extern real32_T LDDT_LatVehSpdLf_Mps;  /* '<S8>/Switch1'
                                        * Left lateral vehicle speed
                                        */
extern real32_T LDVSE_MaxCrvBySpd_ReMi;/* '<S20>/Lookup Table'
                                        * Maximum curvature for LDW invalid condition
                                        */
extern real32_T LDVSE_HystCrvBySpd_ReMi;/* '<S20>/Lookup Table1'
                                         * Curvature hysteresis for LDW invalid condition
                                         */
extern real32_T LDDT_TiToLnRi_Sec;     /* '<S8>/Switch6'
                                        * Time of vehicle to right lane
                                        */
extern real32_T LDWC_TiToLnTrsd_Sec;   /* '<S106>/Product1'
                                        * Threshold time of time to lane crossing
                                        */
extern real32_T LDDT_LnPstnLf_Mi;      /* '<S6>/Switch2'
                                        * Position of  of left lane
                                        */
extern real32_T LDDT_RawDstcToLnLf_Mi; /* '<S8>/Subtract'
                                        * Raw Distance of between vehicel and left lane
                                        */
extern real32_T LDDT_DstcToLnLf_Mi;    /* '<S8>/Switch3'
                                        * Distance of between vehicel and left lane
                                        */
extern real32_T LDWC_DstcToLnTrsdCrvCpstnLf_Mi;/* '<S106>/1-D Lookup Table7'
                                                * Curvature compensation of threshold distance to left lane crossing
                                                */
extern real32_T LDWC_CrvSensiDecayLe_Mi;/* '<S106>/Switch5' */
extern real32_T LDWC_DstcToLnTrsdLf_Mi;/* '<S106>/Add2'
                                        *  Threshold distance to left lane crossing
                                        */
extern real32_T LDDT_TiToLnLf_Sec;     /* '<S8>/Switch5'
                                        * Time of vehicle to left lane
                                        */
extern real32_T LDWC_WRBlockTime_Sec;  /* '<S64>/Switch' */
extern real32_T LDWC_DlcThdMode2_Mi;   /* '<S106>/1-D Lookup Table2'
                                        * DLC threshold at LDW mode 2
                                        */
extern real32_T LDWC_DlcThdMode1_Mi;   /* '<S106>/1-D Lookup Table'
                                        * DLC threshold at LDW mode 1
                                        */
extern real32_T LDWC_DlcThdMode3_Mi;   /* '<S106>/1-D Lookup Table1'
                                        * DLC threshold at LDW mode 3
                                        */
extern real32_T LDWC_DstcToLnTrsd_Mi;  /* '<S106>/Product'
                                        *  Threshold distance to  lane crossing
                                        */
extern uint16_T LDWC_SuppValid_Debug;  /* '<S62>/Signal Conversion8' */
extern uint8_T LDWC_DgrSide_St;        /* '<S1>/LDW'
                                        * State of danger side
                                        */
extern uint8_T LDDT_CurveTypeRi_St;    /* '<S9>/Switch2'
                                        * Curve Type of right Lane
                                        */
extern uint8_T LDVSE_SidCdtnLDWRi_St;  /* '<S22>/Signal Conversion4'
                                        * State of right side at LDW
                                        */
extern uint8_T LDDT_CurveTypeLe_St;    /* '<S9>/Switch'
                                        * Curve Type of left Lane
                                        */
extern uint8_T LDVSE_SidCdtnLDWLf_St;  /* '<S22>/Signal Conversion3'
                                        * State of left side at LDW
                                        */
extern uint8_T LDVSE_IvldLDW_St;       /* '<S20>/Signal Conversion1'
                                        * Invalid state of LDW
                                        */
extern uint8_T ELDWTriggerDgrForHMI;   /* '<S105>/Switch6'
                                        * for HMI
                                        */
extern boolean_T LDWC_NVRAMLDWSwitch_B;/* '<S1>/LDW' */
extern boolean_T LDWC_RdyToTrig_B;     /* '<S1>/LDW'
                                        * Condition of Ready to trigger state
                                        */
extern boolean_T LDDT_RdyTrigLDW_B;    /* '<S6>/Equal'
                                        * Condition of ready to trigger LDW state
                                        */
extern boolean_T LDDT_EnaSafety_B;     /* '<S6>/AND'
                                        * Enable flag for data from the safety interface
                                        */
extern boolean_T LDDT_EnaByInVldQlfrRi_B;/* '<S11>/Relational Operator4'
                                          * Enable flag for right lane validity by left lane invalid qualifier
                                          */
extern boolean_T LDDT_EnaByInVldQlfrSfRi_B;/* '<S11>/Relational Operator1'
                                            * Enable flag for right lane validity by left lane invalid qualifier for safety interface
                                            */
extern boolean_T LDDT_LnTrigVldRi_B;   /* '<S11>/Logical Operator2'
                                        * Condition validity of right lane marker at LDW trigger
                                        */
extern boolean_T LDDT_CclByInVldQlfrRi_B;/* '<S11>/Relational Operator5'
                                          * Enable flag for right lane validity by left lane invalid qualifier for safety interface when cancel the function
                                          */
extern boolean_T LDDT_LnCclVldRi_B;    /* '<S11>/Logical Operator5'
                                        * Condition validity of right lane marker at LDW cancel
                                        */
extern boolean_T LDDT_LnMakVldRi_B;    /* '<S11>/Switch2'
                                        * Condition validity of right lane marker
                                        */
extern boolean_T LDWC_RawTrigByDlcRi_B;/* '<S108>/Relational Operator3'
                                        * Raw trigger flag by DLC for right lane
                                        */
extern boolean_T LDVSE_RdyTrigLDW_B;   /* '<S22>/Equal'
                                        * Ready Trigger flag for LDW
                                        */
extern boolean_T LDDT_EnaByCstruSiteLf_B;/* '<S10>/NOT'
                                          * Enable flag for left lane validity by construction site detected
                                          */
extern boolean_T LDDT_EnaByInVldQlfrLf_B;/* '<S10>/Relational Operator4'
                                          * Enable flag for left lane validity by left lane invalid qualifier
                                          */
extern boolean_T LDDT_EnaByInVldQlfrSfLf_B;/* '<S10>/Relational Operator1'
                                            * Enable flag for left lane validity by left lane invalid qualifier for safety interface
                                            */
extern boolean_T LDDT_LnTrigVldLf_B;   /* '<S10>/Logical Operator2'
                                        * Condition validity of left lane marker at LDW trigger
                                        */
extern boolean_T LDDT_CclByInVldQlfrLf_B;/* '<S10>/Relational Operator5'
                                          * Enable flag for left lane validity by left lane invalid qualifier for safety interface when cancel the function
                                          */
extern boolean_T LDDT_LnCclVldLf_B;    /* '<S10>/Logical Operator5'
                                        * Condition validity of left lane marker at LDW cancel
                                        */
extern boolean_T LDDT_LnMakVldLf_B;    /* '<S10>/Switch'
                                        * Condition validity of left lane marker
                                        */
extern boolean_T LDVSE_VehLatSpdVldLf_B;/* '<S37>/Switch1'
                                         * Validity of left lateral vehicle speed
                                         */
extern boolean_T LDVSE_TrnSglLf_B;     /* '<S21>/Signal Conversion3'
                                        * Condition of  left turn signal
                                        */
extern boolean_T LDVSE_VehLatSpdVldRi_B;/* '<S40>/Switch1'
                                         * Validity of right lateral vehicle speed
                                         */
extern boolean_T LDVSE_TrnSglRi_B;     /* '<S21>/Signal Conversion4'
                                        * Condition of  right turn signal
                                        */
extern boolean_T LDWC_EnaTlcTrigRi_B;  /* '<S108>/Relational Operator1'
                                        * Enable flag for Raw trigger flag by TLC for right lane
                                        */
extern boolean_T LDWC_RawTrigByTlcRi_B;/* '<S108>/Relational Operator2'
                                        * Raw trigger flag by TLC for right lane
                                        */
extern boolean_T LDWC_DlyTrigByTlcRi_B;/* '<S134>/AND' */
extern boolean_T LDWC_EnaLdwTrigRi_B;  /* '<S123>/AND'
                                        * Enable flag for LDW function trigger
                                        */
extern boolean_T LDWC_RstTlcTrigRi_B;  /* '<S108>/Logical Operator15'
                                        * Reset flag for Raw trigger flag by TLC for right lane
                                        */
extern boolean_T LDWC_ResetForSafeRi_B;/* '<S108>/Logical Operator4'
                                        * Reset flag for the safe situation condition of right lane
                                        */
extern boolean_T LDWC_SetForSafeRi_B;  /* '<S108>/Relational Operator6'
                                        * Set flag for the safe situation condition of right lane
                                        */
extern boolean_T LDWC_SetForContinTrigRi_B;/* '<S108>/Logical Operator13' */
extern boolean_T LDWC_ResetForContinTrigRi_B;/* '<S108>/Logical Operator9' */
extern boolean_T LDWC_TrigBySideCondRi_B;/* '<S108>/Relational Operator7'
                                          * LDW function trigger flag by  side condition of right lane
                                          */
extern boolean_T LDWC_TrigByPrjSpecRi_B;/* '<S108>/Equal'
                                         * LDW function trigger flag by customer projects of right lane
                                         */
extern boolean_T LDWC_TrigRi_B;        /* '<S108>/Logical Operator3'
                                        * Condition of right trigger
                                        */
extern boolean_T LDWC_RawTrigByDlcLf_B;/* '<S107>/Relational Operator3'
                                        * Raw trigger flag by DLC for left lane
                                        */
extern boolean_T LDWC_EnaTlcTrigLf_B;  /* '<S107>/Relational Operator1'
                                        * Enable flag for Raw trigger flag by TLC for left lane
                                        */
extern boolean_T LDWC_RawTrigByTlcLf_B;/* '<S107>/Relational Operator2'
                                        * Raw trigger flag by TLC for left lane
                                        */
extern boolean_T LDWC_DlyTrigByTlcLf_B;/* '<S122>/AND' */
extern boolean_T LDWC_EnaLdwTrigLf_B;  /* '<S111>/AND'
                                        * Enable flag for LDW function trigger
                                        */
extern boolean_T LDWC_RstTlcTrigLf_B;  /* '<S107>/OR'
                                        * Reset flag for Raw trigger flag by TLC for left lane
                                        */
extern boolean_T LDWC_ResetForSafeLf_B;/* '<S107>/Logical Operator4'
                                        * Reset flag for the safe situation condition of left lane
                                        */
extern boolean_T LDWC_SetForSafeLf_B;  /* '<S107>/Relational Operator6'
                                        * Set flag for the safe situation condition of left lane
                                        */
extern boolean_T LDWC_ResetForContinTrigLf_B;/* '<S107>/Logical Operator9' */
extern boolean_T LDWC_SetForContinTrigLf_B;/* '<S107>/Logical Operator13' */
extern boolean_T LDWC_TrigBySideCondLf_B;/* '<S107>/Relational Operator7'
                                          * LDW function trigger flag by  side condition of left lane
                                          */
extern boolean_T LDWC_TrigByPrjSpecLf_B;/* '<S107>/Equal'
                                         * LDW function trigger flag by customer projects of left lane
                                         */
extern boolean_T LDWC_TrigLf_B;        /* '<S107>/Logical Operator3'
                                        * Condition of left trigger
                                        */
extern boolean_T LDWC_EnaDgrSide_B;    /* '<S104>/Logical Operator6'
                                        * Enable flag for Degerous side state
                                        */
extern boolean_T LDWC_FnsByDgrStLf_B;  /* '<S60>/Relational Operator7' */
extern boolean_T LDWC_FnsByLatDistLf_B;/* '<S60>/Logical Operator8' */
extern boolean_T LDWC_FnsByHeadingLf_B;/* '<S60>/Logical Operator9' */
extern boolean_T LDWC_FnsByLatSpdLf_B; /* '<S60>/Logical Operator10' */
extern boolean_T LDWC_DgrFnsLf_B;      /* '<S60>/Logical Operator6' */
extern boolean_T LDWC_FnsByDgrStRi_B;  /* '<S60>/Relational Operator9' */
extern boolean_T LDWC_FnsByLatDistRi_B;/* '<S60>/Logical Operator14' */
extern boolean_T LDWC_FnsByHeadingRi_B;/* '<S60>/Logical Operator15' */
extern boolean_T LDWC_FnsByLatSpdRi_B; /* '<S60>/Logical Operator13' */
extern boolean_T LDWC_DgrFnsRi_B;      /* '<S60>/Logical Operator12' */
extern boolean_T LDWC_MinLdwBySysSt_B; /* '<S60>/Relational Operator8' */
extern boolean_T LDWC_EdgeRiseForMinLdw_B;/* '<S87>/AND' */
extern boolean_T LDWC_HoldForMinLdw_B; /* '<S103>/GreaterThan1' */
extern boolean_T LDWC_FlagMinTimeLDW_B;/* '<S60>/Logical Operator11' */
extern boolean_T LDWC_DgrFns_B;        /* '<S96>/AND'
                                        * Condition of danger finish
                                        */
extern boolean_T LDWC_CancelBySpecific_B;/* '<S63>/Relational Operator38'
                                          * LDW cancel conditions by LDW specific bitfield
                                          */
extern boolean_T LDWC_CancelByVehSt_B; /* '<S63>/Relational Operator37'
                                        * LDW cancel conditions by vehicle state
                                        */
extern boolean_T LDWC_CancelByDrvSt_B; /* '<S63>/Relational Operator32'
                                        * LDW cancel conditions by drive state
                                        */
extern boolean_T LDWC_CancelByCtrlSt_B;/* '<S63>/Relational Operator33'
                                        * LDW cancel conditions by active control state
                                        */
extern boolean_T LDWC_CancelBySysSt_B; /* '<S63>/Relational Operator34'
                                        * LDW cancel conditions by system state
                                        */
extern boolean_T LDWC_CancelByAvlSt_B; /* '<S63>/Relational Operator35'
                                        * LDW cancel conditions by no available state
                                        */
extern boolean_T LDWC_CancelByPrjSpec_B;/* '<S63>/Relational Operator36'
                                         * LDW cancel conditions by customer projects
                                         */
extern boolean_T LDWC_MaxDurationBySysSt_B;/* '<S67>/Relational Operator47' */
extern boolean_T LDWC_EdgRiseForSysSt_B;/* '<S68>/AND' */
extern boolean_T LDWC_MaxDurationByStDly_B;/* '<S67>/Logical Operator22' */
extern boolean_T LDWC_TiWarmMx_B;      /* '<S67>/Logical Operator23'
                                        * Condition of warming max time
                                        */
extern boolean_T LDWC_ErrSideByTrigLf_B;/* '<S66>/Logical Operator19' */
extern boolean_T LDWC_ErrSideBySideCondLf_B;/* '<S66>/Relational Operator42' */
extern boolean_T LDWC_ErrSidByPrjSpecLf_B;/* '<S66>/Relational Operator43' */
extern boolean_T LDWC_ErrSidCdtnLf_B;  /* '<S66>/Logical Operator18'
                                        * Error condition of left side
                                        */
extern boolean_T LDWC_SideCondByDgrLf_B;/* '<S66>/Relational Operator41' */
extern boolean_T LDWC_CanelBySideLf_B; /* '<S66>/Logical Operator15' */
extern boolean_T LDWC_SideCondByDgrRi_B;/* '<S66>/Relational Operator44' */
extern boolean_T LDWC_ErrSideByTrigRi_B;/* '<S66>/Logical Operator21' */
extern boolean_T LDWC_ErrSideBySideCondRi_B;/* '<S66>/Relational Operator45' */
extern boolean_T LDWC_ErrSidByPrjSpecRi_B;/* '<S66>/Relational Operator46' */
extern boolean_T LDWC_ErrSidCdtnRi_B;  /* '<S66>/Logical Operator2'
                                        * Error condition of right side
                                        */
extern boolean_T LDWC_CanelBySideRi_B; /* '<S66>/Logical Operator1' */
extern boolean_T LDWC_ErrSidCdtn_B;    /* '<S66>/Logical Operator16'
                                        * Error condition of side
                                        */
extern boolean_T LDWC_CLatDevByDlcLf_B;/* '<S65>/Logical Operator24' */
extern boolean_T LDWC_CLatDevByDgrLf_B;/* '<S65>/Relational Operator39' */
extern boolean_T LDWC_CclLatDevLf_B;   /* '<S65>/Logical Operator12'
                                        * Cancel condition of left lane deviation
                                        */
extern boolean_T LDWC_CLatDevByDlcRi_B;/* '<S65>/Relational Operator40' */
extern boolean_T LDWC_CLatDevByDgrRi_B;/* '<S65>/Logical Operator25' */
extern boolean_T LDWC_CclLatDevRi_B;   /* '<S65>/Logical Operator14'
                                        * Cancel condition of right lane deviation
                                        */
extern boolean_T LDWC_CclLatDev_B;     /* '<S65>/Logical Operator13'
                                        * Cancel condition of lane deviation
                                        */
extern boolean_T LDWC_Cancel_B;        /* '<S63>/Logical Operator11' */
extern boolean_T LDWC_AbortBySpecific_B;/* '<S64>/Relational Operator2'
                                         * LDW abort conditions by LDW specific bitfield
                                         */
extern boolean_T LDWC_AbortByVehSt_B;  /* '<S64>/Relational Operator1'
                                        * LDW abort conditions by vehicle state
                                        */
extern boolean_T LDWC_AbortByDrvSt_B;  /* '<S64>/Relational Operator3'
                                        * LDW abort conditions by drive state
                                        */
extern boolean_T LDWC_AbortByCtrlSt_B; /* '<S64>/Relational Operator4'
                                        * LDW abort conditions by active control state
                                        */
extern boolean_T LDWC_AbortBySysSt_B;  /* '<S64>/Relational Operator5'
                                        * LDW abort conditions by system state
                                        */
extern boolean_T LDWC_AbortByAvlSt_B;  /* '<S64>/Relational Operator6'
                                        * LDW abort conditions by no available state
                                        */
extern boolean_T LDWC_AbortByPrjSpec_B;/* '<S64>/Relational Operator7'
                                        * LDW abort conditions by customer projects
                                        */
extern boolean_T LDWC_Abort_B;         /* '<S64>/Logical Operator6'
                                        * Condition of LDW abort state
                                        */
extern boolean_T LDWC_StrgRdyBySpecific_B;/* '<S64>/Relational Operator9'
                                           * LDW strong ready conditions by LDW specific bitfield
                                           */
extern boolean_T LDWC_StrgRdyByVehSt_B;/* '<S64>/Relational Operator8'
                                        * LDW strong ready conditions by vehicle state
                                        */
extern boolean_T LDWC_StrgRdyByDrvSt_B;/* '<S64>/Relational Operator10'
                                        * LDW strong ready conditions by drive state
                                        */
extern boolean_T LDWC_StrgRdyByCtrlSt_B;/* '<S64>/Relational Operator11'
                                         * LDW strong ready conditions by active control state
                                         */
extern boolean_T LDWC_StrgRdyBySysSt_B;/* '<S64>/Relational Operator12'
                                        * LDW strong ready conditions by system state
                                        */
extern boolean_T LDWC_StrgRdyByAvlSt_B;/* '<S64>/Relational Operator13'
                                        * LDW strong ready conditions by no available state
                                        */
extern boolean_T LDWC_StrgRdyByPrjSpec_B;/* '<S64>/Relational Operator14'
                                          * LDW strong ready conditions by customer projects
                                          */
extern boolean_T LDWC_StrgRdy_B;       /* '<S64>/Logical Operator1'
                                        * Condition of LDW strong ready state
                                        */
extern boolean_T LDWC_Degradation_B;   /* '<S59>/Logical Operator1' */
extern boolean_T LDWC_DegradationEdgeRise_B;/* '<S85>/AND' */
extern boolean_T LDWC_Degr_B;          /* '<S59>/Logical Operator2'
                                        * Condition of degradation
                                        */
extern boolean_T LDWC_SuppBySpecific_B;/* '<S64>/Relational Operator21'
                                        * LDW suppresion conditions by LDW specific bitfield
                                        */
extern boolean_T LDWC_SuppByVehSt_B;   /* '<S64>/Relational Operator20'
                                        * LDW suppresion conditions by vehicle state
                                        */
extern boolean_T LDWC_SuppByDrvSt_B;   /* '<S64>/Relational Operator15'
                                        * LDW suppresion conditions by drive state
                                        */
extern boolean_T LDWC_SuppByCtrlSt_B;  /* '<S64>/Relational Operator16'
                                        * LDW suppresion conditions by active control state
                                        */
extern boolean_T LDWC_SuppBySysSt_B;   /* '<S64>/Relational Operator17'
                                        * LDW suppresion conditions by system state
                                        */
extern boolean_T LDWC_SuppyByAvlSt_B;  /* '<S64>/Relational Operator18'
                                        * LDW suppresion conditions by no available state
                                        */
extern boolean_T LDWC_SuppPrjSpec_B;   /* '<S64>/Relational Operator19'
                                        * LDW suppresion conditions by customer projects
                                        */
extern boolean_T LDWC_Suppresion_B;    /* '<S64>/Logical Operator3' */
extern boolean_T LDWC_WeakRdyBySpecific_B;/* '<S64>/Relational Operator28'
                                           * LDW weak ready conditions by LDW specific bitfield
                                           */
extern boolean_T LDWC_WeakRdyByVehSt_B;/* '<S64>/Relational Operator27'
                                        * LDW weak ready conditions by vehicle state
                                        */
extern boolean_T LDWC_WeakRdyByDrvSt_B;/* '<S64>/Relational Operator22'
                                        * LDW weak ready conditions by drive state
                                        */
extern boolean_T LDWC_WeakRdyByCtrlSt_B;/* '<S64>/Relational Operator23'
                                         * LDW strong weak conditions by active control state
                                         */
extern boolean_T LDWC_WeakRdyBySysSt_B;/* '<S64>/Relational Operator24'
                                        * LDW weak ready conditions by system state
                                        */
extern boolean_T LDWC_WeakRdyByAvlSt_B;/* '<S64>/Relational Operator25'
                                        * LDW weak weak conditions by no available state
                                        */
extern boolean_T LDWC_WeakRdyByPrjSpec_B;/* '<S64>/Relational Operator26'
                                          * LDW weak weak conditions by customer projects
                                          */
extern boolean_T LDWC_WkRdy_B;         /* '<S64>/Logical Operator4'
                                        * Condition of LDW weak ready state
                                        */
extern boolean_T LDWC_BlockTimeBySysOut_B;/* '<S64>/Logical Operator9' */
extern boolean_T LDWC_RawBlockTimeByRampOut_B;/* '<S71>/AND' */
extern boolean_T LDWC_BlockTimeByRampOut_B;/* '<S83>/GreaterThan1' */
extern boolean_T LDWC_BlockTime_B;     /* '<S64>/Logical Operator10' */
extern boolean_T LDWC_Suppression_B;   /* '<S62>/OR'
                                        * Suppresion condition
                                        */
extern boolean_T LDWC_Trig_B;          /* '<S104>/Logical Operator1'
                                        * Condition of trigger
                                        */
extern E_LDWState_nu LDWC_SysOut_St;   /* '<S48>/Switch1'
                                        * Actual state of LDW
                                        */

/*
 * Exported States
 *
 * Note: Exported states are block states with an exported global
 * storage class designation.  Code generation will declare the memory for these
 * states and exports their symbols.
 *
 */
extern real32_T LDVSE_HodTiTrnSglLf_Sec;/* '<S35>/Unit Delay'
                                         * Holding time of left turn signal
                                         */
extern real32_T LDVSE_HodTiTrnSglRi_Sec;/* '<S36>/Unit Delay'
                                         * Holding time of right turn signal
                                         */
extern real32_T LDWC_DlyTiOfTiToLnRiMn_Sec;/* '<S134>/Unit Delay'
                                            * Delay time of time to right lane crossing
                                            */
extern real32_T LDWC_HdTiTrigRi_Sec;   /* '<S132>/Unit Delay'
                                        * holding time right trigger
                                        */
extern real32_T LDWC_ContinWarmTimesOldRi_Count;/* '<S108>/Unit Delay2'
                                                 * The number of consecutive alarms in the previous cycle
                                                 */
extern real32_T LDWC_SuppTimeOldRi_Sec;/* '<S133>/Unit Delay'
                                        * Holding time of left turn signal
                                        */
extern real32_T LDWC_DlyTiOfTiToLnLfMn_Sec;/* '<S122>/Unit Delay'
                                            * Delay time of time to left lane crossing
                                            */
extern real32_T LDWC_HdTiTrigLf_Sec;   /* '<S120>/Unit Delay'
                                        *  holding time left trigger
                                        */
extern real32_T LDWC_ContinWarmTimesOldLf_Count;/* '<S107>/Unit Delay2'
                                                 * The number of consecutive alarms in the previous cycle
                                                 */
extern real32_T LDWC_SuppTimeOldLf_Sec;/* '<S121>/Unit Delay'
                                        * Holding time of left turn signal
                                        */
extern real32_T LDWC_HdTiWarming_Sec;  /* '<S103>/Unit Delay'
                                        * Holding time of warming state start
                                        */
extern real32_T LDWC_DlyTiTgtFns_Sec;  /* '<S96>/Unit Delay'
                                        * Delay time of LDW finish state
                                        */
extern real32_T LDWC_HdTiWarmMx_Sec;   /* '<S70>/Unit Delay'
                                        * Holding time of warming state start
                                        */
extern real32_T LDWC_HdTiDegr_Sec;     /* '<S86>/Unit Delay'
                                        * Holding time of degradation
                                        */
extern real32_T LDWC_HdTiFns_Sec;      /* '<S83>/Unit Delay'
                                        * Holding time of finish state end
                                        */
extern real32_T LDWC_ActiveStopWatch_Ri_sec;/* '<S82>/Unit Delay' */
extern real32_T LDWC_HdTiFns_Ri_Sec;   /* '<S84>/Unit Delay' */
extern uint8_T LDWC_DgrSideOld_St;     /* '<S104>/UnitDelay1'
                                        * Old state of danger side
                                        */
extern boolean_T LDDT_UHysCltdCurvVldRi_B;/* '<S17>/Unit Delay'
                                           * Validity of right lane Clothoid curvature
                                           */
extern boolean_T LDDT_BHysHeadAglTrigVldRi_B;/* '<S16>/Unit Delay'
                                              * Valid trigger of right heading angle
                                              */
extern boolean_T LDDT_UHysHeadAglCclVldRi_B;/* '<S18>/Unit Delay'
                                             * Valid cancel of right heading angle
                                             */
extern boolean_T LDVSE_BHysLatVehSpdVldLf_B;/* '<S41>/Unit Delay'
                                             * Validity of left lateral vehicle speed before trigger state
                                             */
extern boolean_T LDDT_UHysCltdCurvVldLf_B;/* '<S13>/Unit Delay'
                                           * Validity of left lane Clothoid curvature
                                           */
extern boolean_T LDDT_BHysHeadAglTrigVldLf_B;/* '<S12>/Unit Delay'
                                              * Valid trigger of left heading angle
                                              */
extern boolean_T LDDT_UHysHeadAglCclVldLf_B;/* '<S14>/Unit Delay'
                                             * Valid cancel of left heading angle
                                             */
extern boolean_T LDVSE_UHysLatVehSpdVldLf_B;/* '<S42>/Unit Delay'
                                             * Validity of left lateral vehicle speed after trigger state
                                             */
extern boolean_T LDVSE_EdgeRisTrnSglRi_B;/* '<S33>/Unit Delay'
                                          * Edge rise of right turn signal
                                          */
extern boolean_T LDVSE_BHysLatVehSpdVldRi_B;/* '<S45>/Unit Delay'
                                             * Validity of right lateral vehicle speed before trigger state
                                             */
extern boolean_T LDVSE_UHysLatVehSpdVldRi_B;/* '<S46>/Unit Delay'
                                             * Validity of right lateral vehicle speed after trigger state
                                             */
extern boolean_T LDVSE_EdgeRisTrnSglLf_B;/* '<S34>/Unit Delay'
                                          * Edge rise of left turn signal
                                          */
extern boolean_T LDVSE_UHysSteAgl_B;   /* '<S27>/Unit Delay'
                                        * Validity of steering wheel angle
                                        */
extern boolean_T LDVSE_BHysSpdVeh_B;   /* '<S23>/Unit Delay'
                                        * Validity of displayed longitudinal speed
                                        */
extern boolean_T LDVSE_UHysSteAglSpd_B;/* '<S28>/Unit Delay'
                                        * Validity of steering wheel angle speed
                                        */
extern boolean_T LDVSE_BHysAccVehX_B;  /* '<S24>/Unit Delay'
                                        * Validity of  longitudinal Acceleration
                                        */
extern boolean_T LDVSE_BHysAccVehY_B;  /* '<S29>/Unit Delay'
                                        * Validity of  lateral Acceleration
                                        */
extern boolean_T LDVSE_UHysVehCurv_B;  /* '<S30>/Unit Delay'
                                        * Validity of  vehicle curvature
                                        */
extern boolean_T LDVSE_BHysLnWid_B;    /* '<S25>/Unit Delay'
                                        * Validity of lane width
                                        */
extern boolean_T LDWC_HdTiTrigRiEn_B;  /* '<S123>/Unit Delay'
                                        * Enable condition of holding time right trigger
                                        */
extern boolean_T LDWC_DisTrigRi_B;     /* '<S130>/Unit Delay'
                                        * Disable condition of right trigger
                                        */
extern boolean_T LDWC_SuppFlagOldRi_B; /* '<S108>/Unit Delay'
                                        * Suppression signal of continuous alarm in previous period
                                        */
extern boolean_T LDWC_PreActiveEdgeRi; /* '<S126>/Unit Delay'
                                        * Enable condition of continue left trigger
                                        */
extern boolean_T LDWC_ContinTrigRiEn_B;/* '<S127>/Unit Delay'
                                        * Enable condition of continue left trigger
                                        */
extern boolean_T LDWC_DisContinTrigRi_B;/* '<S131>/Unit Delay'
                                         * Disable condition of continue left trigger
                                         */
extern boolean_T LDWC_HdTiTrigLfEn_B;  /* '<S111>/Unit Delay'
                                        * Enable condition of holding time left trigger
                                        */
extern boolean_T LDWC_DisTrigLf_B;     /* '<S118>/Unit Delay'
                                        * Disable condition of left trigger
                                        */
extern boolean_T LDWC_SuppFlagOldLf_B; /* '<S107>/Unit Delay'
                                        * Suppression signal of continuous alarm in previous period
                                        */
extern boolean_T LDWC_PreActiveEdgeLf; /* '<S114>/Unit Delay'
                                        * Enable condition of continue left trigger
                                        */
extern boolean_T LDWC_ContinTrigLfEn_B;/* '<S115>/Unit Delay'
                                        * Enable condition of continue left trigger
                                        */
extern boolean_T LDWC_DisContinTrigLf_B;/* '<S119>/Unit Delay'
                                         * Disable condition of continue left trigger
                                         */
extern boolean_T LDWC_EdgeRisWarming_B;/* '<S87>/Unit Delay'
                                        * Edge rise of warming state
                                        */
extern boolean_T LDWC_EdgeRisWarmMx_B; /* '<S68>/Unit Delay'
                                        * Edge rise of warming state
                                        */
extern boolean_T LDWC_EdgeRisDegr_B;   /* '<S85>/Unit Delay'
                                        * Edge rise of degradation
                                        */
extern boolean_T LDWC_DegrOld_B;       /* '<S59>/UnitDelay'
                                        * UnitDelay condition of degradation
                                        */
extern boolean_T LDWC_EdgeRisFns_B;    /* '<S71>/Unit Delay'
                                        * Edge rise of fginish and cancel state
                                        */
extern boolean_T LDWC_EdgeRisActive_Ri_B;/* '<S74>/Unit Delay' */
extern boolean_T LDWC_EdgeRisFns_Ri_B; /* '<S72>/Unit Delay' */

/* Model entry point functions */
extern void LDWSA_initialize(void);
extern void LDWSA_step(void);

/* Exported data declaration */

/* Declaration for custom storage class: Global */
extern real32_T LDWC_LnLatVeh_BX_Mps[9];/* Referenced by:
                                         * '<S106>/1-D Lookup Table10'
                                         * '<S106>/1-D Lookup Table11'
                                         */
extern real32_T LDWC_LnLatVeh_Lf_Mps[9];
                                /* Referenced by: '<S106>/1-D Lookup Table10' */
extern real32_T LDWC_LnLatVeh_Ri_Mps[9];
                                /* Referenced by: '<S106>/1-D Lookup Table11' */
extern uint8_T LDWSA_SetBit_BS_Param_1[8];
                                    /* Referenced by: '<S32>/ex_sfun_set_bit' */
extern uint8_T LDWSA_SetBit_BS_Param_2[2];/* Referenced by:
                                           * '<S43>/ex_sfun_set_bit'
                                           * '<S44>/ex_sfun_set_bit'
                                           */
extern uint8_T LDWSA_SetBit_BS_Param_3[9];
                                   /* Referenced by: '<S144>/ex_sfun_set_bit' */

/* ConstVolatile memory section */
/* Declaration for custom storage class: ConstVolatile */
extern const volatile boolean_T LDDT_CstruSiteLDW_C_B;/* Referenced by:
                                                       * '<S10>/V_Parameter7'
                                                       * '<S11>/V_Parameter7'
                                                       */

/* Switch of consturction side */
extern const volatile uint8_T LDDT_CurveInner_C_St;/* Referenced by:
                                                    * '<S9>/V_Parameter11'
                                                    * '<S9>/V_Parameter2'
                                                    * '<S106>/V_Parameter2'
                                                    * '<S106>/V_Parameter3'
                                                    */

/* Constant of inner curve lane */
extern const volatile uint8_T LDDT_CurveNone_C_St;/* Referenced by:
                                                   * '<S9>/V_Parameter13'
                                                   * '<S9>/V_Parameter5'
                                                   */

/* Constant of Straight lane */
extern const volatile uint8_T LDDT_CurveOuter_C_St;/* Referenced by:
                                                    * '<S9>/V_Parameter12'
                                                    * '<S9>/V_Parameter4'
                                                    * '<S106>/V_Parameter4'
                                                    * '<S106>/V_Parameter5'
                                                    */

/* Constant of Outer curve lane */
extern const volatile real32_T LDDT_CurveThd_C_St;/* Referenced by: '<S9>/V_Parameter1' */

/* Curve threshold of lane */
extern const volatile uint16_T LDDT_LnIvldCclLf_C_St;
                                      /* Referenced by: '<S10>/V_Parameter13' */

/* Invalid cancel state of left lane
   ctrl for 4095
   safety for 15
 */
extern const volatile uint16_T LDDT_LnIvldCclRi_C_St;/* Referenced by: '<S11>/V_Parameter2' */

/* Invalid cancel state of right lane
   ctrl for 4095
   safety for 15 */
extern const volatile uint16_T LDDT_LnIvldLf_C_St;/* Referenced by: '<S10>/V_Parameter4' */

/* Invalid state of left lane
   ctrl for 20479
   safety for 15 */
extern const volatile uint16_T LDDT_LnIvldRi_C_St;/* Referenced by: '<S11>/V_Parameter4' */

/* Invalid state of right lane
   ctrl for 20479
   safety for 15 */
extern const volatile uint8_T LDDT_LnIvldSfLf_C_St;/* Referenced by: '<S10>/V_Parameter3' */

/* Invalid safety state of left lane */
extern const volatile uint8_T LDDT_LnIvldSfRi_C_St;/* Referenced by: '<S11>/V_Parameter3' */

/* Invalid safety state of right lane */
extern const volatile uint8_T LDDT_NoDgrSide_C_St;/* Referenced by: '<S6>/V_Parameter' */

/* State value of no danger of lane */
extern const volatile boolean_T LDDT_SfFcLDWOn_C_B;/* Referenced by:
                                                    * '<S6>/V_Parameter1'
                                                    * '<S10>/V_Parameter1'
                                                    * '<S11>/V_Parameter1'
                                                    */

/* LDW switch of safety face */
extern const volatile real32_T LDDT_TLCHeadAglTrsd_C_Rad;/* Referenced by:
                                                          * '<S8>/V_Parameter1'
                                                          * '<S8>/V_Parameter2'
                                                          */

/* Threshold of heading angle at TLC */
extern const volatile real32_T LDDT_TrsdHeadAglMn_C_Rad;/* Referenced by:
                                                         * '<S10>/V_Parameter11'
                                                         * '<S11>/V_Parameter11'
                                                         */

/* Minimum threshold of heading angle */
extern const volatile real32_T LDDT_TrsdHeadAglMx_C_Rad;/* Referenced by:
                                                         * '<S10>/V_Parameter10'
                                                         * '<S10>/V_Parameter8'
                                                         * '<S11>/V_Parameter10'
                                                         * '<S11>/V_Parameter8'
                                                         */

/* Maximum threshold of heading angle */
extern const volatile real32_T LDDT_TrsdHeadAglOfst_C_Rad;/* Referenced by:
                                                           * '<S10>/V_Parameter12'
                                                           * '<S10>/V_Parameter2'
                                                           * '<S11>/V_Parameter12'
                                                           * '<S11>/V_Parameter5'
                                                           */

/* Offset of heading angle threshold */
extern const volatile real32_T LDDT_TrsdLnCltdCurvLfMx_Cr_Mps[8];
                                   /* Referenced by: '<S10>/1-D Lookup Table' */

/* Curve of maximum threshold of left clothiod curvature */
extern const volatile real32_T LDDT_TrsdLnCltdCurvLfOfst_Cr_Mps[8];
                                  /* Referenced by: '<S10>/1-D Lookup Table1' */

/* Curve of offset threshold of left clothiod curvature */
extern const volatile real32_T LDDT_TrsdLnCltdCurvRiMx_Cr_Mps[8];
                                   /* Referenced by: '<S11>/1-D Lookup Table' */

/* Curve of maximum threshold of rifht clothiod curvature */
extern const volatile real32_T LDDT_TrsdLnCltdCurvRiOfst_Cr_Mps[8];
                                  /* Referenced by: '<S11>/1-D Lookup Table1' */

/* Curve of offset threshold of right clothiod curvature */
extern const volatile real32_T LDDT_VehSpdX_BX_Mps[8];/* Referenced by:
                                                       * '<S10>/1-D Lookup Table'
                                                       * '<S10>/1-D Lookup Table1'
                                                       * '<S11>/1-D Lookup Table'
                                                       * '<S11>/1-D Lookup Table1'
                                                       */

/* Breakpoint of vehicle speed */
extern const volatile real32_T LDVSE_HodTiTrnSgl_C_Sec;/* Referenced by:
                                                        * '<S21>/V_Parameter3'
                                                        * '<S21>/V_Parameter7'
                                                        */

/* Value of turn signal holding time */
extern const volatile real32_T LDVSE_LnWidTrsdMn_C_Mi;/* Referenced by: '<S20>/V_Parameter8' */

/* Minimum threshold of lane width */
extern const volatile real32_T LDVSE_LnWidTrsdMx_C_Mi;/* Referenced by: '<S20>/V_Parameter7' */

/* Maximum threshold of lane width */
extern const volatile real32_T LDVSE_LnWidTrsdOfst_C_Mi;/* Referenced by: '<S20>/V_Parameter9' */

/* Offset of lane width */
extern const volatile uint8_T LDVSE_NoDgrSide_C_St;/* Referenced by: '<S22>/V_Parameter' */

/* State value of no danger of lane */
extern const volatile real32_T LDVSE_SteAglSpdTrsdMx_C_Dgpm;/* Referenced by: '<S20>/V_Parameter5' */

/* Maximum threshold of steering wheel angle speed */
extern const volatile real32_T LDVSE_SteAglSpdTrsdOfst_C_Dgpm;/* Referenced by: '<S20>/V_Parameter6' */

/* Offset of steering wheel angle speed */
extern const volatile real32_T LDVSE_SteAglTrsdMx_C_Dgr;/* Referenced by: '<S20>/V_Parameter3' */

/* Maximum threshold of steering wheel angle */
extern const volatile real32_T LDVSE_SteAglTrsdOfst_C_Dgr;/* Referenced by: '<S20>/V_Parameter4' */

/* Offset of steering wheel angle */
extern const volatile uint8_T LDVSE_TrnSglLf_C_St;/* Referenced by:
                                                   * '<S21>/V_Parameter1'
                                                   * '<S21>/V_Parameter4'
                                                   */

/* State value of left turn signal  */
extern const volatile uint8_T LDVSE_TrnSglRi_C_St;/* Referenced by:
                                                   * '<S21>/V_Parameter'
                                                   * '<S21>/V_Parameter5'
                                                   */

/* State value of right turn signal  */
extern const volatile boolean_T LDVSE_TrnSglRstLfEn_C_B;/* Referenced by: '<S21>/V_Parameter2' */

/* Enable signal of left turn reset */
extern const volatile boolean_T LDVSE_TrnSglRstRiEn_C_B;/* Referenced by: '<S21>/V_Parameter6' */

/* Enable signal of right turn reset */
extern const volatile real32_T LDVSE_TrsdLnCltdCurvMx_Cr_Mps[8];/* Referenced by: '<S20>/Lookup Table' */

/* Curve of maximum threshold of clothiod curvature  */
extern const volatile real32_T LDVSE_TrsdLnCltdCurvOfst_Cr_Mps[8];
                                      /* Referenced by: '<S20>/Lookup Table1' */

/* Curve of offset threshold of clothiod curvature  */
extern const volatile real32_T LDVSE_VehAccSpdTrsdXMn_C_Npkg;
                                      /* Referenced by: '<S20>/V_Parameter11' */

/* Minimum threshold of longitudinal Acceleration */
extern const volatile real32_T LDVSE_VehAccSpdTrsdXMx_C_Npkg;
                                      /* Referenced by: '<S20>/V_Parameter10' */

/* Maximum threshold of longitudinal Acceleration */
extern const volatile real32_T LDVSE_VehAccSpdTrsdXOfst_C_Npkg;
                                      /* Referenced by: '<S20>/V_Parameter12' */

/* Offset of longitudinal Acceleration */
extern const volatile real32_T LDVSE_VehAccSpdTrsdYMx_C_Npkg;
                                      /* Referenced by: '<S20>/V_Parameter13' */

/* Maximum threshold of lateral Acceleration */
extern const volatile real32_T LDVSE_VehAccSpdTrsdYOfst_C_Npkg;
                                      /* Referenced by: '<S20>/V_Parameter14' */

/* Offset of lateral Acceleration */
extern const volatile real32_T LDVSE_VehLatTrsdLDWMn_C_Msp;/* Referenced by:
                                                            * '<S37>/V_Parameter4'
                                                            * '<S40>/V_Parameter10'
                                                            */

/* Minimum threshold of Lateral vehicle speed */
extern const volatile real32_T LDVSE_VehLatTrsdLDWMx_C_Msp;/* Referenced by:
                                                            * '<S37>/V_Parameter1'
                                                            * '<S40>/V_Parameter7'
                                                            */

/* Maximum threshold of Lateral vehicle speed */
extern const volatile real32_T LDVSE_VehLatTrsdLDWOfst_C_Msp;/* Referenced by:
                                                              * '<S37>/V_Parameter2'
                                                              * '<S37>/V_Parameter5'
                                                              * '<S40>/V_Parameter11'
                                                              * '<S40>/V_Parameter8'
                                                              */

/* Offset of Lateral vehicle speed */
extern const volatile real32_T LDVSE_VehLatTrsd_Cr_Msp[8];/* Referenced by: '<S22>/Lookup Table' */

/* Curve of maximum threshold of lateral velocity */
extern const volatile real32_T LDVSE_VehSpdTrsdMn_C_Kmph;/* Referenced by: '<S31>/V_Parameter1' */

/* Minimum threshold of displayed longitudinal speed */
extern const volatile real32_T LDVSE_VehSpdTrsdMx_C_Kmph;/* Referenced by: '<S20>/V_Parameter' */

/* Maximum threshold of displayed longitudinal speed */
extern const volatile real32_T LDVSE_VehSpdTrsdOfst_C_Kmph;/* Referenced by: '<S20>/V_Parameter2' */

/* Offset of displayed longitudinal speed */
extern const volatile real32_T LDVSE_VehSpdX_BX_Mps[8];/* Referenced by:
                                                        * '<S20>/Lookup Table'
                                                        * '<S20>/Lookup Table1'
                                                        * '<S22>/Lookup Table'
                                                        */

/* Breakpoint of vehicle speed  */
extern const volatile uint8_T LDWC_AbtDrvActCtrl_C_St;
                                      /* Referenced by: '<S64>/V_Parameter11' */

/* Abort state of active control */
extern const volatile uint8_T LDWC_AbtDrvIVld_C_St;
                                      /* Referenced by: '<S64>/V_Parameter10' */

/* Abort state of invalid driver */
extern const volatile uint8_T LDWC_AbtErrSpcLDW_C_St;
                                      /* Referenced by: '<S64>/V_Parameter17' */

/* Abort state of error specific */
extern const volatile uint8_T LDWC_AbtFctCstm_C_St;
                                      /* Referenced by: '<S64>/V_Parameter14' */

/* Abort state of customer specific */
extern const volatile uint8_T LDWC_AbtNoAvlbVehSys_C_St;
                                      /* Referenced by: '<S64>/V_Parameter13' */

/* Abort state of no availible vehicle system signals */
extern const volatile uint16_T LDWC_AbtVehIvld_C_St;/* Referenced by: '<S64>/V_Parameter9' */

/* Abort state of invalid vehicle */
extern const volatile uint8_T LDWC_AbtVehSysErr_C_St;
                                      /* Referenced by: '<S64>/V_Parameter12' */

/* Abort state of vehicle system errors */
extern const volatile uint8_T LDWC_CclDrvActCtrl_C_St;
                                      /* Referenced by: '<S63>/V_Parameter42' */

/* Cancel state of active control */
extern const volatile uint8_T LDWC_CclDrvIVld_C_St;
                                      /* Referenced by: '<S63>/V_Parameter41' */

/* Cancel state of invalid driver */
extern const volatile uint8_T LDWC_CclErrSpcLDW_C_St;
                                      /* Referenced by: '<S63>/V_Parameter46' */

/* Cancel state of error specific */
extern const volatile uint8_T LDWC_CclFctCstm_St;
                                      /* Referenced by: '<S63>/V_Parameter45' */

/* Cancel state of customer specific */
extern const volatile uint8_T LDWC_CclNoAvlbVehSys_C_St;
                                      /* Referenced by: '<S63>/V_Parameter44' */

/* Cancel state of no availible vehicle system signals */
extern const volatile uint16_T LDWC_CclVehIvld_C_St;
                                      /* Referenced by: '<S63>/V_Parameter47' */

/* Cancel state of invalid vehicle */
extern const volatile uint8_T LDWC_CclVehSysErr_C_St;
                                      /* Referenced by: '<S63>/V_Parameter43' */

/* Cancel state of vehicle system errors */
extern const volatile real32_T LDWC_ContiActiveTiFns_C_Sec;/* Referenced by:
                                                            * '<S64>/V_Parameter3'
                                                            * '<S64>/V_Parameter6'
                                                            */

/* Maximum time of quit state */
extern const volatile real32_T LDWC_ContinWarmSupp_C_Sec;/* Referenced by:
                                                          * '<S107>/V_Parameter3'
                                                          * '<S108>/V_Parameter2'
                                                          */

/* the time of continous warning suppression */
extern const volatile real32_T LDWC_ContinWarmTimes_C_Count;/* Referenced by:
                                                             * '<S107>/V_Parameter4'
                                                             * '<S108>/V_Parameter4'
                                                             */

/* The number of consecutive alarms allowed
 */
extern const volatile real32_T LDWC_ContinuActiveTi_C_Sec;/* Referenced by:
                                                           * '<S64>/V_Parameter1'
                                                           * '<S64>/V_Parameter4'
                                                           */
extern const volatile real32_T LDWC_CrvSensiAdvance_BX_ReMi[4];
                                 /* Referenced by: '<S106>/1-D Lookup Table9' */

/* Breakpoint of detected right lane curvature */
extern const volatile real32_T LDWC_CrvSensiAdvance_BY_Mi[4];
                                 /* Referenced by: '<S106>/1-D Lookup Table9' */

/* Breakpoint of detected right lane curvature */
extern const volatile real32_T LDWC_CrvSensiDecay_BX_ReMi[4];
                                 /* Referenced by: '<S106>/1-D Lookup Table8' */

/* Breakpoint of detected right lane curvature */
extern const volatile real32_T LDWC_CrvSensiDecay_BY_Mi[4];
                                 /* Referenced by: '<S106>/1-D Lookup Table8' */

/* Breakpoint of detected right lane curvature */
extern const volatile real32_T LDWC_DTCFctLnWid_Cr_Fct[5];
                                 /* Referenced by: '<S106>/1-D Lookup Table3' */

/* Lane width factor of DTL    */
extern const volatile real32_T LDWC_DgrCclOfst_C_Mi;/* Referenced by:
                                                     * '<S65>/V_Parameter1'
                                                     * '<S65>/V_Parameter53'
                                                     */

/* Danger offset distance of cancel state */
extern const volatile real32_T LDWC_DgrFnsHeadAng_C_Rad;/* Referenced by:
                                                         * '<S60>/V_Parameter23'
                                                         * '<S60>/V_Parameter38'
                                                         */

/* Danger of heading angle  */
extern const volatile real32_T LDWC_DgrFnsOfst_C_Mi;/* Referenced by:
                                                     * '<S60>/V_Parameter19'
                                                     * '<S60>/V_Parameter34'
                                                     */

/* Danger offset distance of finish state */
extern const volatile real32_T LDWC_DgrFnsSpdVelLat_C_Mps;/* Referenced by:
                                                           * '<S60>/V_Parameter25'
                                                           * '<S60>/V_Parameter40'
                                                           */

/* Danger of lateral speed */
extern const volatile uint8_T LDWC_DgrSideLf_C_St;/* Referenced by:
                                                   * '<S60>/V_Parameter17'
                                                   * '<S104>/V_Parameter6'
                                                   * '<S105>/V_Parameter8'
                                                   * '<S136>/V_Parameter32'
                                                   * '<S137>/V_Parameter32'
                                                   * '<S65>/V_Parameter48'
                                                   * '<S66>/V_Parameter56'
                                                   */

/* Constant of left side danger */
extern const volatile uint8_T LDWC_DgrSideRi_C_St;/* Referenced by:
                                                   * '<S60>/V_Parameter32'
                                                   * '<S104>/V_Parameter7'
                                                   * '<S136>/V_Parameter2'
                                                   * '<S137>/V_Parameter1'
                                                   * '<S65>/V_Parameter49'
                                                   * '<S66>/V_Parameter59'
                                                   */

/* Constant of right side danger */
extern const volatile real32_T LDWC_DlyTiFns_C_Sec;/* Referenced by:
                                                    * '<S64>/V_Parameter2'
                                                    * '<S64>/V_Parameter5'
                                                    */

/* Maximum time of quit state */
extern const volatile real32_T LDWC_DlyTiOfTiToLnMn_C_Sec;/* Referenced by:
                                                           * '<S107>/V_Parameter12'
                                                           * '<S108>/V_Parameter12'
                                                           */

/* Delay time of time to lane crossing */
extern const volatile real32_T LDWC_DlyTiTgtFns_C_Sec;/* Referenced by: '<S60>/V_Parameter9' */

/* Delay time of target finish  */
extern const volatile uint8_T LDWC_DrvMod2_C_St;
                                      /* Referenced by: '<S106>/V_Parameter9' */

/* Driver control mode of LDW £º2 mode */
extern const volatile uint8_T LDWC_DrvMod3_C_St;
                                     /* Referenced by: '<S106>/V_Parameter10' */

/* Driver control mode of LDW £º3 mode */
extern const volatile real32_T LDWC_DstcOfDiscToLnLmtMn_C_Mi;/* Referenced by:
                                                              * '<S107>/V_Parameter1'
                                                              * '<S107>/V_Parameter14'
                                                              * '<S108>/V_Parameter14'
                                                              * '<S108>/V_Parameter3'
                                                              */

/* Minimum distance limiting value of distance to lane crossing */
extern const volatile real32_T LDWC_DstcOfTiToLnMn_C_Mi;/* Referenced by:
                                                         * '<S107>/V_Parameter10'
                                                         * '<S108>/V_Parameter10'
                                                         */

/* Minimum distance of distance to lane crossing */
extern const volatile real32_T LDWC_DstcOfstSafeSitu_C_Mi;/* Referenced by:
                                                           * '<S107>/V_Parameter13'
                                                           * '<S108>/V_Parameter13'
                                                           */

/* Offset distance of safe situation */
extern const volatile real32_T LDWC_DstcToLnTrsdOfstLf_Cr_Mi[17];
                                 /* Referenced by: '<S106>/1-D Lookup Table7' */

/* Curve table of left offset distance to lane crossing threshold      */
extern const volatile real32_T LDWC_DstcToLnTrsdOfstRi_Cr_Mi[17];
                                 /* Referenced by: '<S106>/1-D Lookup Table4' */

/* Curve table of right offset distance to lane crossing threshold      */
extern const volatile real32_T LDWC_DstcTrsdVehSpdXDTL1_Cr_Mi[9];
                                  /* Referenced by: '<S106>/1-D Lookup Table' */

/* Curve table of distance to lane crossing threshold at mode 1     */
extern const volatile real32_T LDWC_DstcTrsdVehSpdXDTL2_Cr_Mi[9];
                                 /* Referenced by: '<S106>/1-D Lookup Table2' */

/* Curve table of distance to lane crossing threshold at mode 2     */
extern const volatile real32_T LDWC_DstcTrsdVehSpdXDTL3_Cr_Mi[9];
                                 /* Referenced by: '<S106>/1-D Lookup Table1' */

/* Curve table of distance to lane crossing threshold at mode 3     */
extern const volatile uint8_T LDWC_ErrCstmCclLf_C_St;
                                      /* Referenced by: '<S66>/V_Parameter58' */

/* Cancel state of left customer specific */
extern const volatile uint8_T LDWC_ErrCstmCclRi_C_St;
                                      /* Referenced by: '<S66>/V_Parameter61' */

/* Cancel state of right customer specific */
extern const volatile uint8_T LDWC_FnsCdsnEn_C_St;/* Referenced by:
                                                   * '<S60>/V_Parameter28'
                                                   * '<S60>/V_Parameter29'
                                                   * '<S60>/V_Parameter30'
                                                   * '<S60>/V_Parameter31'
                                                   * '<S60>/V_Parameter41'
                                                   * '<S60>/V_Parameter42'
                                                   * '<S60>/V_Parameter43'
                                                   */

/* State switch of finish condition  */
extern const volatile real32_T LDWC_FnsDuraMn_C_Sec;
                                      /* Referenced by: '<S60>/V_Parameter27' */

/* Minimum duration of finish state */
extern const volatile real32_T LDWC_HdTiTrigLf_C_Sec;/* Referenced by:
                                                      * '<S107>/V_Parameter15'
                                                      * '<S108>/V_Parameter15'
                                                      */

/* Holding time of left warming trigger */
extern const volatile real32_T LDWC_LDPSensiDecay_C_Mi;
                                      /* Referenced by: '<S106>/V_Parameter1' */

/* LDP sensitivity decay */
extern const volatile real32_T LDWC_LaneWidth_BX_Mi[5];/* Referenced by:
                                                        * '<S106>/1-D Lookup Table3'
                                                        * '<S106>/1-D Lookup Table6'
                                                        */

/* breakpoint of lane width */
extern const volatile real32_T LDWC_LnDectCrvLf_BX_ReMi[17];
                                 /* Referenced by: '<S106>/1-D Lookup Table7' */

/* Breakpoint of detected left lane curvature */
extern const volatile real32_T LDWC_LnDectCrvRi_BX_ReMi[17];
                                 /* Referenced by: '<S106>/1-D Lookup Table4' */

/* Breakpoint of detected right lane curvature */
extern const volatile real32_T LDWC_NoDgrCclOfst_C_Mi;/* Referenced by:
                                                       * '<S65>/V_Parameter51'
                                                       * '<S65>/V_Parameter54'
                                                       */

/* No danger offset distance of cancel state */
extern const volatile real32_T LDWC_NoDgrFnsHeadAng_C_Rad;/* Referenced by:
                                                           * '<S60>/V_Parameter22'
                                                           * '<S60>/V_Parameter37'
                                                           */

/*  No danger of heading angle      */
extern const volatile real32_T LDWC_NoDgrFnsOfst_C_Mi;/* Referenced by:
                                                       * '<S60>/V_Parameter21'
                                                       * '<S60>/V_Parameter36'
                                                       */

/* No danger offset distance of finish state */
extern const volatile real32_T LDWC_NoDgrFnsSpdVelLat_C_Mps;/* Referenced by:
                                                             * '<S60>/V_Parameter24'
                                                             * '<S60>/V_Parameter39'
                                                             */

/* No danger of lateral speed */
extern const volatile uint8_T LDWC_NoDgrSide_C_St;/* Referenced by:
                                                   * '<S104>/V_Parameter4'
                                                   * '<S104>/V_Parameter5'
                                                   * '<S105>/V_Parameter5'
                                                   */

/* Constant of no danger */
extern const volatile uint8_T LDWC_PrjSpecQu_C_St;/* Referenced by:
                                                   * '<S107>/V_Parameter'
                                                   * '<S108>/V_Parameter'
                                                   */
extern const volatile real32_T LDWC_SafetyFuncMaxTime_sec;/* Referenced by:
                                                           * '<S139>/V_Parameter1'
                                                           * '<S140>/V_Parameter1'
                                                           */

/* safety function active or error maximum */
extern const volatile uint8_T LDWC_SidCdtnCclLf_C_St;
                                      /* Referenced by: '<S66>/V_Parameter57' */

/* Cancel constant of left side condition */
extern const volatile uint8_T LDWC_SidCdtnCclRi_C_St;
                                      /* Referenced by: '<S66>/V_Parameter60' */

/* Cancel constant of right side condition */
extern const volatile uint8_T LDWC_StrgRdyDrvActCtrl_C_St;
                                      /* Referenced by: '<S64>/V_Parameter16' */

/* Strong ready state of active control */
extern const volatile uint8_T LDWC_StrgRdyDrvIVld_C_St;
                                      /* Referenced by: '<S64>/V_Parameter15' */

/* Strong ready state of invalid driver */
extern const volatile uint8_T LDWC_StrgRdyErrSpcLDW_C_St;
                                      /* Referenced by: '<S64>/V_Parameter21' */

/* Strong ready state of error specific */
extern const volatile uint8_T LDWC_StrgRdyFctCstm_C_St;
                                      /* Referenced by: '<S64>/V_Parameter20' */

/* Strong ready state of customer specific */
extern const volatile uint8_T LDWC_StrgRdyNoAvlbVehSys_C_St;
                                      /* Referenced by: '<S64>/V_Parameter19' */

/* Strong ready state of no availible vehicle system signals */
extern const volatile uint16_T LDWC_StrgRdyVehIvld_C_St;
                                      /* Referenced by: '<S64>/V_Parameter22' */

/* Strong ready state of invalid vehicle */
extern const volatile uint8_T LDWC_StrgRdyVehSysErr_C_St;
                                      /* Referenced by: '<S64>/V_Parameter18' */

/* Strong ready state of vehicle system errors */
extern const volatile uint8_T LDWC_SuppDrvActCtrl_C_St;
                                      /* Referenced by: '<S64>/V_Parameter24' */

/* Suppresion state of active control */
extern const volatile uint8_T LDWC_SuppDrvIvld_C_St;
                                      /* Referenced by: '<S64>/V_Parameter23' */

/* Suppresion state of invalid driver */
extern const volatile uint8_T LDWC_SuppErrSpcLDW_C_St;
                                      /* Referenced by: '<S64>/V_Parameter28' */

/* Suppresion state of error specific */
extern const volatile uint8_T LDWC_SuppFctCstm_C_St;
                                      /* Referenced by: '<S64>/V_Parameter27' */

/* Suppresion state of customer specific */
extern const volatile uint8_T LDWC_SuppNoAvlbVehSys_C_St;
                                      /* Referenced by: '<S64>/V_Parameter26' */

/* Suppresion state of no availible vehicle system signals */
extern const volatile uint16_T LDWC_SuppVehIvld_C_St;
                                      /* Referenced by: '<S64>/V_Parameter29' */

/* Suppresion state of invalid vehicle */
extern const volatile uint8_T LDWC_SuppVehSysErr_C_St;
                                      /* Referenced by: '<S64>/V_Parameter25' */

/* Suppresion state of vehicle system errors */
extern const volatile uint8_T LDWC_SuppVehicleInvalid_C_St;
                                     /* Referenced by: '<S141>/V_Parameter28' */

/* Suppresion state of vehicle */
extern const volatile boolean_T LDWC_Switch_C_B;
                                      /* Referenced by: '<S50>/V_Parameter11' */

/* Value of LDW disable switch */
extern const volatile real32_T LDWC_TgtTrajPstnY_C_Mi;/* Referenced by:
                                                       * '<S60>/V_Parameter18'
                                                       * '<S60>/V_Parameter20'
                                                       * '<S60>/V_Parameter33'
                                                       * '<S60>/V_Parameter35'
                                                       * '<S65>/V_Parameter50'
                                                       * '<S65>/V_Parameter52'
                                                       */

/* Target trajectory lateral position  */
extern const volatile real32_T LDWC_TiAbtDegr_C_Sec;
                                      /* Referenced by: '<S59>/V_Parameter63' */

/* Degradation time of abort state */
extern const volatile real32_T LDWC_TiCclDegr_C_Sec;/* Referenced by: '<S59>/V_Parameter3' */

/* Degradation time of cancel state */
extern const volatile real32_T LDWC_TiDgrFnsDegr_C_Sec;/* Referenced by: '<S59>/V_Parameter2' */

/* Degradation time of finish state */
extern const volatile real32_T LDWC_TiStrgRdyDegr_C_Sec;/* Referenced by: '<S59>/V_Parameter1' */

/* Degradation time of no strong ready state */
extern const volatile real32_T LDWC_TiToLnTrsdSpd_Cr_Sec[9];
                                 /* Referenced by: '<S106>/1-D Lookup Table5' */

/* Map table of time to lane crossing threshold */
extern const volatile real32_T LDWC_TiToLnTrsdWdh_Cr_Sec[5];
                                 /* Referenced by: '<S106>/1-D Lookup Table6' */

/* Map table of time to lane crossing threshold */
extern const volatile uint8_T LDWC_TrigCdtnEn_C_St;/* Referenced by:
                                                    * '<S107>/V_Parameter11'
                                                    * '<S107>/V_Parameter9'
                                                    * '<S108>/V_Parameter11'
                                                    * '<S108>/V_Parameter9'
                                                    */

/* Switch state of choose threshold  */
extern const volatile real32_T LDWC_VehSpdXDTL_BX_Mps[9];/* Referenced by:
                                                          * '<S106>/1-D Lookup Table'
                                                          * '<S106>/1-D Lookup Table1'
                                                          * '<S106>/1-D Lookup Table2'
                                                          */

/* DTL breakpoint of vehicle speed */
extern const volatile real32_T LDWC_VehSpdXTTL_BX_Mps[9];
                                 /* Referenced by: '<S106>/1-D Lookup Table5' */

/* TTL breakpoint of vehicle speed */
extern const volatile real32_T LDWC_VehYawRateHyst_C_rps;/* Referenced by: '<S143>/Constant10' */

/* Vehicle yaw rate hysteresis */
extern const volatile real32_T LDWC_VehYawRateMax_C_rps;/* Referenced by: '<S143>/Constant9' */

/* Vehicle yaw rate maximum */
extern const volatile real32_T LDWC_WarmMxTi_C_Sec;/* Referenced by:
                                                    * '<S107>/V_Parameter2'
                                                    * '<S108>/V_Parameter1'
                                                    * '<S67>/V_Parameter63'
                                                    */

/* Maximum time of warming state */
extern const volatile uint8_T LDWC_WkRdyDrvActCtrl_C_St;
                                      /* Referenced by: '<S64>/V_Parameter31' */

/* Weak ready state of active control */
extern const volatile uint8_T LDWC_WkRdyDrvIVld_C_St;
                                      /* Referenced by: '<S64>/V_Parameter30' */

/* Weak ready state of invalid driver */
extern const volatile uint8_T LDWC_WkRdyErrSpcLDW_C_St;
                                      /* Referenced by: '<S64>/V_Parameter35' */

/* Weak ready state of error specific */
extern const volatile uint8_T LDWC_WkRdyFctCstm_C_St;
                                      /* Referenced by: '<S64>/V_Parameter34' */

/* Weak ready state of customer specific */
extern const volatile uint8_T LDWC_WkRdyNoAvlbVehSys_C_St;
                                      /* Referenced by: '<S64>/V_Parameter33' */

/* Weak ready state of no availible vehicle system signals */
extern const volatile uint16_T LDWC_WkRdyVehIvld_C_St;
                                      /* Referenced by: '<S64>/V_Parameter36' */

/* Weak ready state of invalid vehicle */
extern const volatile uint8_T LDWC_WkRdyVehSysErr_C_St;
                                      /* Referenced by: '<S64>/V_Parameter32' */

/* Weak ready state of vehicle system errors */
extern const volatile real32_T VehicleSpeedThresholdHMI_Kph;/* Referenced by:
                                                             * '<S105>/V_Parameter1'
                                                             * '<S105>/V_Parameter3'
                                                             */

/* vehicle speed threshold for HMI */

/* Declaration for custom storage class: Global */
extern boolean_T LDDT_LnLengthLf_B;    /* '<S15>/Unit Delay' */
extern boolean_T LDDT_LnLengthRi_B;    /* '<S19>/Unit Delay' */
extern uint8_T LDVSE_PrevVehStartupSpd_Kmph;/* '<S31>/Unit Delay' */

/* Yawrate hysteresis--Used in LDWC module */
extern real32_T LDWC_ActiveStopWatch_sec;/* '<S81>/Unit Delay' */
extern boolean_T LDWC_EdgeRisActive_B; /* '<S73>/Unit Delay' */
extern uint8_T LDWC_PrevDgrSide_St;    /* '<S2>/Unit Delay' */

/* Yawrate hysteresis--Used in LDWC module */
extern boolean_T LDWC_PrevSwitchUnitDelay_bool;/* '<S50>/Unit Delay' */
extern E_LDWState_nu LDWC_PrevSysOutIn_St;/* '<S53>/Unit Delay' */

/* Yawrate hysteresis--Used in LDWC module */
extern boolean_T LDWC_RampTimeExpiredRSFF_bool;/* '<S56>/Unit Delay' */
extern real32_T LDWC_SafeFuncActiveTurnOnDelay_sec;/* '<S145>/Unit Delay' */
extern real32_T LDWC_SafeFuncErrorTurnOnDelay_sec;/* '<S146>/Unit Delay' */
extern real32_T LDWC_SusTimeExpiredTimerRetrigger_sec;/* '<S57>/Unit Delay' */

/* Yawrate hysteresis--Used in LDWC module */
extern E_LDWState_nu LDWC_SysOld_St;   /* '<S5>/UnitDelay' */

/* Yawrate hysteresis--Used in LDWC module */
extern boolean_T LDWC_VehYawRateHyst_bool;/* '<S147>/Unit Delay' */

/* Yawrate hysteresis--Used in LDPSC module */

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S21>/Signal Conversion1' : Eliminate redundant signal conversion block
 * Block '<S21>/Signal Conversion2' : Eliminate redundant signal conversion block
 * Block '<S5>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S59>/Signal Conversion1' : Eliminate redundant signal conversion block
 * Block '<S59>/Signal Conversion2' : Eliminate redundant signal conversion block
 * Block '<S107>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S107>/Signal Conversion1' : Eliminate redundant signal conversion block
 * Block '<S107>/Signal Conversion2' : Eliminate redundant signal conversion block
 * Block '<S107>/Signal Conversion3' : Eliminate redundant signal conversion block
 * Block '<S107>/Signal Conversion4' : Eliminate redundant signal conversion block
 * Block '<S108>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S108>/Signal Conversion1' : Eliminate redundant signal conversion block
 * Block '<S108>/Signal Conversion2' : Eliminate redundant signal conversion block
 * Block '<S108>/Signal Conversion3' : Eliminate redundant signal conversion block
 * Block '<S108>/Signal Conversion4' : Eliminate redundant signal conversion block
 * Block '<S20>/V_Parameter15' : Unused code path elimination
 * Block '<S20>/V_Parameter16' : Unused code path elimination
 * Block '<S53>/Constant1' : Unused code path elimination
 * Block '<S67>/Constant40' : Unused code path elimination
 * Block '<S64>/Constant30' : Unused code path elimination
 * Block '<S64>/Constant40' : Unused code path elimination
 * Block '<S59>/Constant' : Unused code path elimination
 * Block '<S86>/Constant3' : Unused code path elimination
 * Block '<S60>/Constant40' : Unused code path elimination
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
 * hilite_system('LDWSA_model/LDWSA')    - opens subsystem LDWSA_model/LDWSA
 * hilite_system('LDWSA_model/LDWSA/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'LDWSA_model'
 * '<S1>'   : 'LDWSA_model/LDWSA'
 * '<S2>'   : 'LDWSA_model/LDWSA/LDW'
 * '<S3>'   : 'LDWSA_model/LDWSA/LDW/LDDT'
 * '<S4>'   : 'LDWSA_model/LDWSA/LDW/LDVSE'
 * '<S5>'   : 'LDWSA_model/LDWSA/LDW/LDWC'
 * '<S6>'   : 'LDWSA_model/LDWSA/LDW/LDDT/JudgeSignal'
 * '<S7>'   : 'LDWSA_model/LDWSA/LDW/LDDT/LnMarkVld'
 * '<S8>'   : 'LDWSA_model/LDWSA/LDW/LDDT/TLC&DLC_Calc'
 * '<S9>'   : 'LDWSA_model/LDWSA/LDW/LDDT/JudgeSignal/LaneCurveType'
 * '<S10>'  : 'LDWSA_model/LDWSA/LDW/LDDT/LnMarkVld/LnMarkLf'
 * '<S11>'  : 'LDWSA_model/LDWSA/LDW/LDDT/LnMarkVld/LnMarkRi'
 * '<S12>'  : 'LDWSA_model/LDWSA/LDW/LDDT/LnMarkVld/LnMarkLf/Bilateral hysteresis'
 * '<S13>'  : 'LDWSA_model/LDWSA/LDW/LDDT/LnMarkVld/LnMarkLf/Unilateral hysteresis '
 * '<S14>'  : 'LDWSA_model/LDWSA/LDW/LDDT/LnMarkVld/LnMarkLf/Unilateral hysteresis 1'
 * '<S15>'  : 'LDWSA_model/LDWSA/LDW/LDDT/LnMarkVld/LnMarkLf/Unilateral hysteresis 2'
 * '<S16>'  : 'LDWSA_model/LDWSA/LDW/LDDT/LnMarkVld/LnMarkRi/Bilateral hysteresis'
 * '<S17>'  : 'LDWSA_model/LDWSA/LDW/LDDT/LnMarkVld/LnMarkRi/Unilateral hysteresis '
 * '<S18>'  : 'LDWSA_model/LDWSA/LDW/LDDT/LnMarkVld/LnMarkRi/Unilateral hysteresis 1'
 * '<S19>'  : 'LDWSA_model/LDWSA/LDW/LDDT/LnMarkVld/LnMarkRi/Unilateral hysteresis 2'
 * '<S20>'  : 'LDWSA_model/LDWSA/LDW/LDVSE/InVldLDW'
 * '<S21>'  : 'LDWSA_model/LDWSA/LDW/LDVSE/JudgeTrn'
 * '<S22>'  : 'LDWSA_model/LDWSA/LDW/LDVSE/SidCdtnLDW'
 * '<S23>'  : 'LDWSA_model/LDWSA/LDW/LDVSE/InVldLDW/Bilateral hysteresis'
 * '<S24>'  : 'LDWSA_model/LDWSA/LDW/LDVSE/InVldLDW/Bilateral hysteresis1'
 * '<S25>'  : 'LDWSA_model/LDWSA/LDW/LDVSE/InVldLDW/Bilateral hysteresis2'
 * '<S26>'  : 'LDWSA_model/LDWSA/LDW/LDVSE/InVldLDW/MappingUint2'
 * '<S27>'  : 'LDWSA_model/LDWSA/LDW/LDVSE/InVldLDW/Unilateral hysteresis '
 * '<S28>'  : 'LDWSA_model/LDWSA/LDW/LDVSE/InVldLDW/Unilateral hysteresis 1'
 * '<S29>'  : 'LDWSA_model/LDWSA/LDW/LDVSE/InVldLDW/Unilateral hysteresis 2'
 * '<S30>'  : 'LDWSA_model/LDWSA/LDW/LDVSE/InVldLDW/Unilateral hysteresis 3'
 * '<S31>'  : 'LDWSA_model/LDWSA/LDW/LDVSE/InVldLDW/VehStartupSpd'
 * '<S32>'  : 'LDWSA_model/LDWSA/LDW/LDVSE/InVldLDW/MappingUint2/Set_bit'
 * '<S33>'  : 'LDWSA_model/LDWSA/LDW/LDVSE/JudgeTrn/ Edge'
 * '<S34>'  : 'LDWSA_model/LDWSA/LDW/LDVSE/JudgeTrn/ Edge1'
 * '<S35>'  : 'LDWSA_model/LDWSA/LDW/LDVSE/JudgeTrn/FollowUpTimer'
 * '<S36>'  : 'LDWSA_model/LDWSA/LDW/LDVSE/JudgeTrn/FollowUpTimer1'
 * '<S37>'  : 'LDWSA_model/LDWSA/LDW/LDVSE/SidCdtnLDW/LfVehLatSpdVld'
 * '<S38>'  : 'LDWSA_model/LDWSA/LDW/LDVSE/SidCdtnLDW/MappingUint1'
 * '<S39>'  : 'LDWSA_model/LDWSA/LDW/LDVSE/SidCdtnLDW/MappingUint8'
 * '<S40>'  : 'LDWSA_model/LDWSA/LDW/LDVSE/SidCdtnLDW/RiVehLatSpdVld'
 * '<S41>'  : 'LDWSA_model/LDWSA/LDW/LDVSE/SidCdtnLDW/LfVehLatSpdVld/Bilateral hysteresis'
 * '<S42>'  : 'LDWSA_model/LDWSA/LDW/LDVSE/SidCdtnLDW/LfVehLatSpdVld/Unilateral hysteresis '
 * '<S43>'  : 'LDWSA_model/LDWSA/LDW/LDVSE/SidCdtnLDW/MappingUint1/Set_bit'
 * '<S44>'  : 'LDWSA_model/LDWSA/LDW/LDVSE/SidCdtnLDW/MappingUint8/Set_bit'
 * '<S45>'  : 'LDWSA_model/LDWSA/LDW/LDVSE/SidCdtnLDW/RiVehLatSpdVld/Bilateral hysteresis'
 * '<S46>'  : 'LDWSA_model/LDWSA/LDW/LDVSE/SidCdtnLDW/RiVehLatSpdVld/Unilateral hysteresis '
 * '<S47>'  : 'LDWSA_model/LDWSA/LDW/LDWC/LDW_State'
 * '<S48>'  : 'LDWSA_model/LDWSA/LDW/LDWC/RAMPOUTStChk'
 * '<S49>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput'
 * '<S50>'  : 'LDWSA_model/LDWSA/LDW/LDWC/SwitchCheck'
 * '<S51>'  : 'LDWSA_model/LDWSA/LDW/LDWC/RAMPOUTStChk/Enumerated Constant'
 * '<S52>'  : 'LDWSA_model/LDWSA/LDW/LDWC/RAMPOUTStChk/Enumerated Constant1'
 * '<S53>'  : 'LDWSA_model/LDWSA/LDW/LDWC/RAMPOUTStChk/RampoutWatchDog'
 * '<S54>'  : 'LDWSA_model/LDWSA/LDW/LDWC/RAMPOUTStChk/RampoutWatchDog/Enumerated Constant'
 * '<S55>'  : 'LDWSA_model/LDWSA/LDW/LDWC/RAMPOUTStChk/RampoutWatchDog/Enumerated Constant1'
 * '<S56>'  : 'LDWSA_model/LDWSA/LDW/LDWC/RAMPOUTStChk/RampoutWatchDog/RSFlipFlop'
 * '<S57>'  : 'LDWSA_model/LDWSA/LDW/LDWC/RAMPOUTStChk/RampoutWatchDog/TimerRetrigger'
 * '<S58>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Cancel&Ready'
 * '<S59>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Degradation'
 * '<S60>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrFns'
 * '<S61>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrTrig'
 * '<S62>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Suppression'
 * '<S63>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Cancel&Ready/Cancel'
 * '<S64>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Cancel&Ready/Ready'
 * '<S65>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Cancel&Ready/Cancel/LnDev'
 * '<S66>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Cancel&Ready/Cancel/SideCondition'
 * '<S67>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Cancel&Ready/Cancel/TiWarmMx'
 * '<S68>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Cancel&Ready/Cancel/TiWarmMx/ Edge1'
 * '<S69>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Cancel&Ready/Cancel/TiWarmMx/Enumerated Constant1'
 * '<S70>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Cancel&Ready/Cancel/TiWarmMx/TimerRetrigger'
 * '<S71>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Cancel&Ready/Ready/ Edge'
 * '<S72>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Cancel&Ready/Ready/ Edge1'
 * '<S73>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Cancel&Ready/Ready/EdgeRising'
 * '<S74>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Cancel&Ready/Ready/EdgeRising1'
 * '<S75>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Cancel&Ready/Ready/Enumerated Constant1'
 * '<S76>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Cancel&Ready/Ready/Enumerated Constant2'
 * '<S77>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Cancel&Ready/Ready/Enumerated Constant3'
 * '<S78>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Cancel&Ready/Ready/Enumerated Constant4'
 * '<S79>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Cancel&Ready/Ready/Enumerated Constant5'
 * '<S80>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Cancel&Ready/Ready/Enumerated Constant6'
 * '<S81>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Cancel&Ready/Ready/Stopwatch'
 * '<S82>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Cancel&Ready/Ready/Stopwatch1'
 * '<S83>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Cancel&Ready/Ready/TimerRetrigger'
 * '<S84>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Cancel&Ready/Ready/TimerRetrigger1'
 * '<S85>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Degradation/ Edge'
 * '<S86>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Degradation/FollowUpTimer'
 * '<S87>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrFns/ Edge1'
 * '<S88>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrFns/Bit Get3'
 * '<S89>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrFns/Bit Get4'
 * '<S90>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrFns/Bit Get5'
 * '<S91>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrFns/Bit Get6'
 * '<S92>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrFns/Bit Get7'
 * '<S93>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrFns/Bit Get8'
 * '<S94>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrFns/Bit Get9'
 * '<S95>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrFns/Enumerated Constant1'
 * '<S96>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrFns/RiseDelay2'
 * '<S97>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrFns/Saturation'
 * '<S98>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrFns/Saturation1'
 * '<S99>'  : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrFns/Saturation2'
 * '<S100>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrFns/Saturation3'
 * '<S101>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrFns/Saturation4'
 * '<S102>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrFns/Saturation5'
 * '<S103>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrFns/TimerRetrigger'
 * '<S104>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrTrig/DangerSide'
 * '<S105>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrTrig/DangerSideHMI'
 * '<S106>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrTrig/Threshold'
 * '<S107>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrTrig/TrgrLf'
 * '<S108>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrTrig/TrgrRi'
 * '<S109>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrTrig/DangerSide/Enumerated Constant1'
 * '<S110>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrTrig/DangerSide/Enumerated Constant2'
 * '<S111>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrTrig/TrgrLf/ Edge'
 * '<S112>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrTrig/TrgrLf/Bit Get'
 * '<S113>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrTrig/TrgrLf/Bit Get1'
 * '<S114>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrTrig/TrgrLf/EdgeRising'
 * '<S115>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrTrig/TrgrLf/EdgeRising1'
 * '<S116>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrTrig/TrgrLf/Enumerated Constant1'
 * '<S117>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrTrig/TrgrLf/Enumerated Constant2'
 * '<S118>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrTrig/TrgrLf/FF'
 * '<S119>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrTrig/TrgrLf/FF1'
 * '<S120>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrTrig/TrgrLf/FollowUpTimer'
 * '<S121>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrTrig/TrgrLf/FollowUpTimer1'
 * '<S122>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrTrig/TrgrLf/RiseDelay'
 * '<S123>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrTrig/TrgrRi/ Edge'
 * '<S124>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrTrig/TrgrRi/Bit Get'
 * '<S125>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrTrig/TrgrRi/Bit Get1'
 * '<S126>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrTrig/TrgrRi/EdgeRising'
 * '<S127>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrTrig/TrgrRi/EdgeRising1'
 * '<S128>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrTrig/TrgrRi/Enumerated Constant1'
 * '<S129>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrTrig/TrgrRi/Enumerated Constant2'
 * '<S130>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrTrig/TrgrRi/FF'
 * '<S131>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrTrig/TrgrRi/FF1'
 * '<S132>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrTrig/TrgrRi/FollowUpTimer'
 * '<S133>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrTrig/TrgrRi/FollowUpTimer1'
 * '<S134>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/DgrTrig/TrgrRi/RiseDelay'
 * '<S135>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Suppression/DriverStInvalid'
 * '<S136>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Suppression/LaneCurveInvalid'
 * '<S137>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Suppression/LateralVelocityInvalid'
 * '<S138>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Suppression/MappingUint16'
 * '<S139>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Suppression/SafetyFunctionActive'
 * '<S140>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Suppression/SafetyFunctionError'
 * '<S141>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Suppression/VehicleInvalid'
 * '<S142>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Suppression/VehicleStInvalid'
 * '<S143>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Suppression/YawRateInvalid'
 * '<S144>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Suppression/MappingUint16/Set_bit'
 * '<S145>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Suppression/SafetyFunctionActive/RiseDelay2'
 * '<S146>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Suppression/SafetyFunctionError/RiseDelay2'
 * '<S147>' : 'LDWSA_model/LDWSA/LDW/LDWC/StateInput/Suppression/YawRateInvalid/Hysteresis3'
 */

/*-
 * Requirements for '<Root>': LDWSA
 */
#endif                                 /* RTW_HEADER_LDWSA_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
