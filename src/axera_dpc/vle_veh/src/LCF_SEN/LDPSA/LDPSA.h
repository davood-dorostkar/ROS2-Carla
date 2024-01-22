/**********************************Model Property********************************
 *
 * Company             : SENSETIME
 *
 * Tool Version        : Ver2.0
 *
 * Model Name          : LDPSA
 *
 * Model Long Name     : Lane Departure Prevention

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
 * File                             : LDPSA.h
 *
 * FileType                         : Code Header File
 *
 * Real-Time Workshop file version  : 9.4 (R2020b) 29-Jul-2020
 *
 * TLC version                      : 9.4 (Aug 20 2020)
 *
 * C source code generated on       : Sun Jan 15 16:16:18 2023
 *
 * Copyright (C) by SenseTime Group Limited. All rights reserved.
 *******************************************************************************/

#ifndef RTW_HEADER_LDPSA_h_
#define RTW_HEADER_LDPSA_h_
#include <math.h>
#include <string.h>
#ifndef LDPSA_COMMON_INCLUDES_
#define LDPSA_COMMON_INCLUDES_
#include "Sfun_Set_Bit.h"
#include "rtwtypes.h"
#endif                                 /* LDPSA_COMMON_INCLUDES_ */

#include "LDPSA_types.h"

/* Macros for accessing real-time model data structure */

/* Block states (default storage) for system '<Root>' */
typedef struct {
  uint8_T is_active_c2_LDPSA;          /* '<S4>/LDP_State' */
  uint8_T is_c2_LDPSA;                 /* '<S4>/LDP_State' */
  uint8_T is_LDP_ON;                   /* '<S4>/LDP_State' */
} DW_LDPSA_T;

/* Block states (default storage) */
extern DW_LDPSA_T LDPSA_DW;

/*
 * Exported Global Signals
 *
 * Note: Exported global signals are block signals with an exported global
 * storage class designation.  Code generation will declare the memory for
 * these signals and export their symbols.
 *
 */
extern real32_T LDPTT_LnBdryPstnXLf_Mi;/* '<S1>/LDP'
                                        * Filtered left lane clothoid X0 position from LDP
                                        */
extern real32_T LDPTT_LnBdryPstnYLf_Mi;/* '<S1>/LDP'
                                        * Filtered left lane clothoid Y0 position from LDP
                                        */
extern real32_T LDPTT_LnBdryHeadAglLf_Rad;/* '<S1>/LDP'
                                           * Filtered left lane clothoid heading angle from LDP
                                           */
extern real32_T LDPTT_LnBdryCurvLf_ReMi;/* '<S1>/LDP'
                                         * Filtered left lane clothoid curvature from LDP
                                         */
extern real32_T LDPTT_LnBdryCurvRateLf_ReMi2;/* '<S1>/LDP'
                                              * Filtered left lane clothoid change of curvature from LDP
                                              */
extern real32_T LDPTT_LnBdryVldLengLf_Mi;/* '<S1>/LDP'
                                          * Filtered left lane clothoid length from LDP
                                          */
extern real32_T LDPTT_LnBdryPstnXRi_Mi;/* '<S1>/LDP'
                                        * Filtered right lane clothoid X0 position from LDP
                                        */
extern real32_T LDPTT_LnBdryPstnYRi_Mi;/* '<S1>/LDP'
                                        * Filtered right lane clothoid Y0 position from LDP
                                        */
extern real32_T LDPTT_LnBdryHeadAglRi_Rad;/* '<S1>/LDP'
                                           * Filtered right lane clothoid heading angle from LDP
                                           */
extern real32_T LDPTT_LnBdryCurvRi_ReMi;/* '<S1>/LDP'
                                         * Filtered right lane clothoid curvature from LDP
                                         */
extern real32_T LDPTT_LnBdryCurvRateRi_ReMi2;/* '<S1>/LDP'
                                              * Filtered right lane clothoid change of curvature from LDP
                                              */
extern real32_T LDPTT_LnBdryVldLengRi_Mi;/* '<S1>/LDP'
                                          * Filtered right lane clothoid length from LDP
                                          */
extern real32_T LDPTT_LnBdryPstnXCent_Mi;/* '<S1>/LDP'
                                          * Filtered center lane clothoid X0 position from LDP
                                          */
extern real32_T LDPTT_LnBdryPstnYCent_Mi;/* '<S1>/LDP'
                                          * Filtered center lane clothoid Y0 position from LDP
                                          */
extern real32_T LDPTT_LnBdryHeadAglCent_Rad;/* '<S1>/LDP'
                                             * Filtered center lane clothoid heading angle from LDP
                                             */
extern real32_T LDPTT_LnBdryCurvCent_ReMi;/* '<S1>/LDP'
                                           * Filtered center lane clothoid curvature from LDP
                                           */
extern real32_T LDPTT_LnBdryCurvRateCent_ReMi2;/* '<S1>/LDP'
                                                * Filtered center lane clothoid change of curvature from LDP
                                                */
extern real32_T LDPTT_LnBdryVldLengCent_Mi;/* '<S1>/LDP'
                                            * Filtered center lane clothoid length from LDP
                                            */
extern real32_T LDPTT_TgtPstnYLf_Mi;   /* '<S1>/LDP'
                                        * Lateral distance to the target when a dangerous situation on the left side takes place
                                        */
extern real32_T LDPTT_TgtPstnYRi_Mi;   /* '<S1>/LDP'
                                        * Lateral distance to the target when a dangerous situation on the right side takes place
                                        */
extern real32_T LDPTV_FTireAccMx_Mps2; /* '<S1>/LDP' */
extern real32_T LDPTV_FTireAccMn_Mps2; /* '<S1>/LDP' */
extern real32_T LDPTV_DstcYTgtAreaLf_Mi;/* '<S1>/LDP'
                                         * Y-axis distance of host vehicle to left target area
                                         */
extern real32_T LDPTV_DstcYTgtAreaRi_Mi;/* '<S1>/LDP'
                                         * Y-axis distance of host vehicle to right target area
                                         */
extern real32_T LDPTV_FctTgtDistY_Fct; /* '<S1>/LDP' */
extern real32_T LDPTV_TrajPlanServQu_Fct;/* '<S1>/LDP' */
extern real32_T LDPTV_PredTiCurv_Sec;  /* '<S1>/LDP' */
extern real32_T LDPTV_PredTiAgl_Sec;   /* '<S1>/LDP' */
extern real32_T LDPTV_TiLmtEnDura_Sec; /* '<S1>/LDP' */
extern real32_T LDPTV_JerkLmtMx_Mps3;  /* '<S1>/LDP'
                                        * Maximum Jerk Allowed in the trajectory planning
                                        */
extern real32_T LDPTV_VeloXObst_Mps;   /* '<S1>/LDP'
                                        * X-axis velocity of obstacle
                                        */
extern real32_T LDPTV_AccXObst_Mps2;   /* '<S1>/LDP'
                                        * X-axis acceleration of obstacle
                                        */
extern real32_T LDPTV_DstcXObst_Mi;    /* '<S1>/LDP'
                                        * X-axis distance of obstacle
                                        */
extern real32_T LDPTV_DstcYObst_Mi;    /* '<S1>/LDP'
                                        * Y-axis distance of obstacle
                                        */
extern real32_T LDPTV_LmtCurvMx_ReMi;  /* '<S1>/LDP'
                                        * Maximal limiter curvature allowed.
                                        */
extern real32_T LDPTV_LmtCurvGradIncMx_ReMps;/* '<S1>/LDP'
                                              * Maximal limiter curvature gradient allowed.
                                              */
extern real32_T LDPTV_LmtCurvGradDecMx_ReMps;/* '<S1>/LDP'
                                              * Maximal limiter curvature gradient allowed.
                                              */
extern real32_T LDPTV_SnsTiStamp_Sec;  /* '<S1>/LDP'
                                        * Sensor time stamp in seconds
                                        */
extern real32_T LDPTV_SteWhlGradLmt_Fct;/* '<S1>/LDP'
                                         * Steering Wheel Stiffness Limiter
                                         */
extern real32_T LDPTV_SteWhlGrad_ReS;  /* '<S1>/LDP'
                                        * Steering Wheel Stiffness Gradient
                                        */
extern real32_T LDPTV_TrqRampGrad_ReS; /* '<S1>/LDP'
                                        * Torque Ramp Gradient
                                        */
extern real32_T LDPTV_MxTrqScalGradLmt_Fct;/* '<S1>/LDP'
                                            * Maximum Torque Scaling Limiter (Torque saturation)
                                            */
extern real32_T LDPTV_MxTrqScalGrad_ReS;/* '<S1>/LDP'
                                         * Maximum Torque Scaling Gradient
                                         */
extern real32_T LDPTV_WeightEndTi_Fct; /* '<S1>/LDP'
                                        * Weight of the end time for the calculation in the TRJPLN
                                        */
extern real32_T LDPTV_PlanningHorizon_Sec;/* '<S1>/LDP' */
extern real32_T LDPTV_DMCDeraLvl_Fct;  /* '<S1>/LDP'
                                        * DMC Derating Level of the LDP function
                                        */
extern real32_T LDPTV_WidObst_Mi;      /* '<S1>/LDP'
                                        * Width of obstacle
                                        */
extern real32_T LDPTV_LmtCurvGradCtrlMx_ReMps;/* '<S1>/LDP'
                                               * Maximal limiter curvature gradient allowed.
                                               */
extern real32_T LDPVSE_NVRAMVehStartupSpd_Kmph;/* '<S1>/LDP' */
extern real32_T LDPSC_CrvSensiDecayRi_Mi;/* '<S78>/Switch6' */
extern real32_T LDPDT_CrvThdMaxRi_ReMi;/* '<S13>/1-D Lookup Table'
                                        * Curve of maximum threshold of right clothiod curvature
                                        */
extern real32_T LDPDT_CrvThdHystRi_ReMi;/* '<S13>/1-D Lookup Table1'
                                         * Curve of offset threshold of right clothiod curvature
                                         */
extern real32_T LDPDT_LnCltdCurvRi_ReMi;/* '<S8>/Switch5'
                                         * Clothoid curvature of right lane
                                         */
extern real32_T LDPDT_LnHeadRi_Rad;    /* '<S8>/Switch1'
                                        * Heading angle of rifht lane
                                        */
extern real32_T LDPDT_RawLatVehSpdRi_Mps;/* '<S10>/Product1'
                                          * Raw right lateral vehicle speed
                                          */
extern real32_T LDPDT_LatVehSpdRi_Mps; /* '<S10>/Switch2'
                                        * right lateral vehicle speed
                                        */
extern real32_T LDPSC_DstcToLnTrsdCrvCpstnRi_Mi;/* '<S78>/1-D Lookup Table4'
                                                 * Curvature compensation of threshold distance to right lane crossing
                                                 */
extern real32_T LDPSC_DstcToLnTrsdRi_Mi;/* '<S78>/Add3'
                                         *  Threshold distance to right lane crossing
                                         */
extern real32_T LDPDT_LnPstnRi_Mi;     /* '<S8>/Switch3'
                                        * Position of  of right lane
                                        */
extern real32_T LDPDT_RawDstcToLnRi_Mi;/* '<S10>/Subtract1'
                                        * Raw Distance of between vehicel and right lane
                                        */
extern real32_T LDPDT_DstcToLnRi_Mi;   /* '<S10>/Switch5'
                                        * Distance of between vehicel and right lane
                                        */
extern real32_T LDPDT_TiToLnRi_Sec;    /* '<S10>/Switch4'
                                        * Time of vehicle to right lane
                                        */
extern real32_T LDPSC_TiToLnTrsd_Sec;  /* '<S78>/Product1'
                                        * Threshold time of time to lane crossing
                                        */
extern real32_T LDPVSE_MaxLatVel_Mps;  /* '<S162>/Lookup Table'
                                        * Maximum lateral velocity
                                        */
extern real32_T LDPDT_CrvThdMaxLf_ReMi;/* '<S12>/1-D Lookup Table'
                                        * Curve of maximum threshold of left clothiod curvature
                                        */
extern real32_T LDPDT_CrvThdHystLf_ReMi;/* '<S12>/1-D Lookup Table1'
                                         * Curve of offset threshold of left clothiod curvature
                                         */
extern real32_T LDPDT_LnCltdCurvLf_ReMi;/* '<S8>/Switch4'
                                         * Clothoid curvature of left lane
                                         */
extern real32_T LDPDT_LnHeadLf_Rad;    /* '<S8>/Switch'
                                        * Heading angle of left lane
                                        */
extern real32_T LDPDT_LnPstnLf_Mi;     /* '<S8>/Switch2'
                                        * Position of  of left lane
                                        */
extern real32_T LDPDT_RawDstcToLnLf_Mi;/* '<S10>/Subtract'
                                        * Raw Distance of between vehicel and left lane
                                        */
extern real32_T LDPDT_DstcToLnLf_Mi;   /* '<S10>/Switch'
                                        * Distance of between vehicel and left lane
                                        */
extern real32_T LDPSC_DstcToLnTrsdCrvCpstnLf_Mi;/* '<S78>/1-D Lookup Table5'
                                                 * Curvature compensation of threshold distance to left lane crossing
                                                 */
extern real32_T LDPDT_RawLatVehSpdLf_Mps;/* '<S10>/Product'
                                          * Raw Left lateral vehicle speed
                                          */
extern real32_T LDPDT_LatVehSpdLf_Mps; /* '<S10>/Switch1'
                                        * Left lateral vehicle speed
                                        */
extern real32_T LDPSC_CrvSensiDecayLe_Mi;/* '<S78>/Switch5' */
extern real32_T LDPSC_DstcToLnTrsdLf_Mi;/* '<S78>/Add2'
                                         *  Threshold distance to left lane crossing
                                         */
extern real32_T LDPDT_TiToLnLf_Sec;    /* '<S10>/Switch3'
                                        * Time of vehicle to left lane
                                        */
extern real32_T LDPVSE_MaxCrvBySpd_ReMi;/* '<S160>/Lookup Table'
                                         * Maximum curvature for LDW invalid condition
                                         */
extern real32_T LDPVSE_HystCrvBySpd_ReMi;/* '<S160>/Lookup Table1'
                                          * Curvature hysteresis for LDW invalid condition
                                          */
extern real32_T LDPSC_RampoutTime_Sec; /* '<S32>/Switch'
                                        * Rampout time
                                        */
extern real32_T LDPSC_WRBlockTime_Sec; /* '<S37>/Switch' */
extern real32_T LDPTT_RawLnBdryPstnYLf_Mi;/* '<S121>/Switch'
                                           * Raw Filtered left lane clothoid Y0 position from LDP
                                           */
extern real32_T LDPTT_RawLnBdryPstnYRi_Mi;/* '<S121>/Switch1'
                                           * Raw Filtered right lane clothoid Y0 position from LDP
                                           */
extern real32_T LDPTT_TgtLatDstcRi_Mi; /* '<S121>/Switch9'
                                        * Distance between the hazardous lane marking and the planned target.
                                        */
extern real32_T LDPTT_TgtLatDstcLf_Mi; /* '<S121>/Switch2'
                                        * Distance between the hazardous lane marking and the planned target.
                                        */
extern real32_T LDPTT_RawBdryPstnYCent_Mi;/* '<S121>/Switch16'
                                           * Raw Filtered center lane clothoid Y0 position from LDP
                                           */
extern real32_T LDPTV_LatVel_Mps;      /* '<S152>/Switch'
                                        * Lateral Velocity For LDPTV
                                        */
extern real32_T LDPSC_DlcThdMode2_Mi;  /* '<S78>/1-D Lookup Table1'
                                        * DLC threshold at LDW mode 2
                                        */
extern real32_T LDPSC_DlcThdMode1_Mi;  /* '<S78>/1-D Lookup Table'
                                        * DLC threshold at LDW mode 1
                                        */
extern real32_T LDPSC_DlcThdMode3_Mi;  /* '<S78>/1-D Lookup Table2'
                                        * DLC threshold at LDW mode 3
                                        */
extern real32_T LDPSC_DstcToLnTrsd_Mi; /* '<S78>/Product'
                                        *  Threshold distance to  lane crossing
                                        */
extern uint16_T LDPSC_SuppValid_Debug; /* '<S35>/Signal Conversion8'
                                        * Suppression debug
                                        */
extern uint8_T LDPSC_DgrSide_St;       /* '<S1>/LDP'
                                        * State of danger side
                                        */
extern uint8_T LDPTV_TrajCtrlSt_St;    /* '<S1>/LDP'
                                        * Trajectory Guidance Qualifier Output
                                        */
extern uint8_T LDPDT_CurveTypeRi_St;   /* '<S11>/Switch2'
                                        * Curve Type of right Lane
                                        */
extern uint8_T LDPVSE_SidCdtnLDPRi_St; /* '<S162>/Signal Conversion1'
                                        * State of right side at LDP
                                        */
extern uint8_T LDPDT_CurveTypeLe_St;   /* '<S11>/Switch'
                                        * Curve Type of left Lane
                                        */
extern uint8_T LDPVSE_SidCdtnLDPLf_St; /* '<S162>/Signal Conversion'
                                        * State of left side at LDP
                                        */
extern uint8_T LDPVSE_IvldLDP_St;      /* '<S160>/Signal Conversion2'
                                        * Invalid state of LDP
                                        */
extern boolean_T LDPSC_RdyToTrig_B;    /* '<S1>/LDP'
                                        * Condition of Ready to trigger state
                                        */
extern boolean_T LDPTV_HighStatReq_B;  /* '<S1>/LDP'
                                        * High Stationary Accuracy required
                                        */
extern boolean_T LDPTV_LatCpstnEn_B;   /* '<S1>/LDP'
                                        * Switch for the latency compensation in trajectory plan
                                        */
extern boolean_T LDPTV_LmtEn_B;        /* '<S1>/LDP'
                                        * Switch to limit the target curvature in trajectory controller
                                        */
extern boolean_T LDPTV_TrigReplan_B;   /* '<S1>/LDP'
                                        * It has to be 1 for trajectory planning to calculate a trajectory
                                        */
extern boolean_T LDPSC_NVRAMLDPSwitch_B;/* '<S1>/LDP' */
extern boolean_T LDPDT_RdyTrigLDP_B;   /* '<S8>/Equal'
                                        * Condition of ready to trigger LDP state
                                        */
extern boolean_T LDPDT_EnaSafety_B;    /* '<S8>/AND'
                                        * Enable flag for data from the safety interface
                                        */
extern boolean_T LDPDT_EnaByInVldQlfrRi_B;/* '<S13>/Relational Operator4'
                                           * Enable flag for right lane validity by left lane invalid qualifier
                                           */
extern boolean_T LDPDT_EnaByInVldQlfrSfRi_B;/* '<S13>/Relational Operator1'
                                             * Enable flag for right lane validity by left lane invalid qualifier for safety interface
                                             */
extern boolean_T LDPDT_LnTrigVldRi_B;  /* '<S13>/Logical Operator2'
                                        * Condition validity of right lane marker at LDP trigger
                                        */
extern boolean_T LDPDT_CclByInVldQlfrRi_B;/* '<S13>/Relational Operator5'
                                           * Enable flag for right lane validity by left lane invalid qualifier for safety interface when cancel the function
                                           */
extern boolean_T LDPDT_LnCclVldRi_B;   /* '<S13>/Logical Operator5'
                                        * Condition validity of right lane marker at LDP cancel
                                        */
extern boolean_T LDPDT_LnMakVldRi_B;   /* '<S13>/Switch'
                                        * Condition validity of right lane marker
                                        */
extern boolean_T LDPSC_RawTrigByDlcRi_B;/* '<S80>/Relational Operator3'
                                         * Raw trigger flag by DLC for right lane
                                         */
extern boolean_T LDPSC_EnaTlcTrigRi_B; /* '<S80>/Relational Operator1'
                                        * Enable flag for Raw trigger flag by TLC for right lane
                                        */
extern boolean_T LDPSC_RawTrigByTlcRi_B;/* '<S80>/Relational Operator2'
                                         * Raw trigger flag by TLC for right lane
                                         */
extern boolean_T LDPSC_DlyTrigByTlcRi_B;/* '<S106>/AND'
                                         * Trigger flag by TLC for right lane
                                         */
extern boolean_T LDPSC_EnaLdwTrigRi_B; /* '<S95>/AND'
                                        * Enable flag for LDW function trigger
                                        */
extern boolean_T LDPSC_RstLdwTrigRi_B; /* '<S80>/OR'
                                        * Reset flag for LDW function trigger
                                        */
extern boolean_T LDPSC_HoldLdwTrigRi_B;/* '<S80>/Signal Conversion'
                                        * Enable flag for LDW function trigger after time holding
                                        */
extern boolean_T LDPSC_ResetForSafeRi_B;/* '<S80>/Logical Operator4'
                                         * Reset flag for the safe situation condition of right lane
                                         */
extern boolean_T LDPSC_SetForSafeRi_B; /* '<S80>/Relational Operator6'
                                        * Set flag for the safe situation condition of right lane
                                        */
extern boolean_T LDPSC_SetForContinTrigRi_B;/* '<S80>/Logical Operator13' */
extern boolean_T LDPSC_ResetForContinTrigRi_B;/* '<S80>/Logical Operator18' */
extern boolean_T LDPVSE_EdgeRiseTurnSglRi_B;/* '<S174>/AND' */
extern boolean_T LDPVSE_TrnSglRi_B;    /* '<S161>/Signal Conversion2'
                                        * Condition of  right turn signal
                                        */
extern boolean_T LDPVSE_RdyTrigLDW_B;  /* '<S162>/Equal'
                                        * Ready Trigger flag for LDW
                                        */
extern boolean_T LDPVSE_VehLatSpdVldRi_B;/* '<S180>/Switch'
                                          * Validity of right lateral vehicle speed
                                          */
extern boolean_T LDPSC_TrigBySideCondRi_B;/* '<S80>/Relational Operator7'
                                           * LDW function trigger flag by  side condition of right lane
                                           */
extern boolean_T LDPSC_TrigByPrjSpecRi_B;/* '<S80>/Equal'
                                          * LDW function trigger flag by customer projects of right lane
                                          */
extern boolean_T LDPSC_TrigRi_B;       /* '<S80>/Logical Operator3'
                                        * Condition of right trigger
                                        */
extern boolean_T LDPDT_EnaByCstruSiteLf_B;/* '<S12>/AND'
                                           * Enable flag for left lane validity by construction site detected
                                           */
extern boolean_T LDPDT_EnaByInVldQlfrLf_B;/* '<S12>/Relational Operator4'
                                           * Enable flag for left lane validity by left lane invalid qualifier
                                           */
extern boolean_T LDPDT_EnaByInVldQlfrSfLf_B;/* '<S12>/Relational Operator1'
                                             * Enable flag for left lane validity by left lane invalid qualifier for safety interface
                                             */
extern boolean_T LDPDT_LnTrigVldLf_B;  /* '<S12>/Logical Operator2'
                                        * Condition validity of left lane marker at LDP trigger
                                        */
extern boolean_T LDPDT_CclByInVldQlfrLf_B;/* '<S12>/Relational Operator5'
                                           * Enable flag for left lane validity by left lane invalid qualifier for safety interface when cancel the function
                                           */
extern boolean_T LDPDT_LnCclVldLf_B;   /* '<S12>/Logical Operator5'
                                        * Condition validity of left lane marker at LDP cancel
                                        */
extern boolean_T LDPDT_LnMakVldLf_B;   /* '<S12>/Switch'
                                        * Condition validity of left lane marker
                                        */
extern boolean_T LDPSC_RawTrigByDlcLf_B;/* '<S79>/Relational Operator3'
                                         * Raw trigger flag by DLC for left lane
                                         */
extern boolean_T LDPSC_EnaTlcTrigLf_B; /* '<S79>/Relational Operator1'
                                        * Enable flag for Raw trigger flag by TLC for left lane
                                        */
extern boolean_T LDPSC_RawTrigByTlcLf_B;/* '<S79>/Relational Operator2'
                                         * Raw trigger flag by TLC for left lane
                                         */
extern boolean_T LDPSC_DlyTrigByTlcLf_B;/* '<S94>/AND'
                                         * Trigger flag by TLC for left lane
                                         */
extern boolean_T LDPSC_EnaLdwTrigLf_B; /* '<S83>/AND'
                                        * Enable flag for LDW function trigger
                                        */
extern boolean_T LDPSC_RstLdwTrigLf_B; /* '<S79>/Logical Operator15'
                                        * Reset flag for LDW function trigger
                                        */
extern boolean_T LDPSC_HoldLdwTrigLf_B;/* '<S79>/Signal Conversion'
                                        * Enable flag for LDW function trigger after time holding
                                        */
extern boolean_T LDPSC_ResetForSafeLf_B;/* '<S79>/Logical Operator4'
                                         * Reset flag for the safe situation condition of left lane
                                         */
extern boolean_T LDPSC_SetForSafeLf_B; /* '<S79>/Relational Operator6'
                                        * Set flag for the safe situation condition of left lane
                                        */
extern boolean_T LDPSC_ResetForContinTrigLf_B;/* '<S79>/Logical Operator9' */
extern boolean_T LDPSC_SetForContinTrigLf_B;/* '<S79>/Logical Operator13' */
extern boolean_T LDPVSE_EdgeRiseTurnSglLf_B;/* '<S173>/AND' */
extern boolean_T LDPVSE_TrnSglLf_B;    /* '<S161>/Signal Conversion'
                                        * Condition of  left turn signal
                                        */
extern boolean_T LDPVSE_VehLatSpdVldLf_B;/* '<S177>/Switch'
                                          * Validity of left lateral vehicle speed
                                          */
extern boolean_T LDPSC_TrigBySideCondLf_B;/* '<S79>/Relational Operator7'
                                           * LDW function trigger flag by  side condition of left lane
                                           */
extern boolean_T LDPSC_TrigByPrjSpecLf_B;/* '<S79>/Equal'
                                          * LDW function trigger flag by customer projects of left lane
                                          */
extern boolean_T LDPSC_TrigLf_B;       /* '<S79>/Logical Operator3'
                                        * Condition of left trigger
                                        */
extern boolean_T LDPSC_EnaDgrSide_B;   /* '<S77>/Logical Operator6'
                                        * Enable flag for Degerous side state
                                        */
extern boolean_T LDPSC_FnsByDgrStLf_B; /* '<S33>/Relational Operator7' */
extern boolean_T LDPSC_FnsByLatDistLf_B;/* '<S33>/Logical Operator8' */
extern boolean_T LDPSC_FnsByHeadingLf_B;/* '<S33>/Logical Operator9' */
extern boolean_T LDPSC_FnsByLatSpdLf_B;/* '<S33>/Logical Operator10' */
extern boolean_T LDPSC_DgrFnsLf_B;     /* '<S33>/Logical Operator6' */
extern boolean_T LDPSC_FnsByDgrStRi_B; /* '<S33>/Relational Operator9' */
extern boolean_T LDPSC_FnsByLatDistRi_B;/* '<S33>/Logical Operator14' */
extern boolean_T LDPSC_FnsByHeadingRi_B;/* '<S33>/Logical Operator15' */
extern boolean_T LDPSC_FnsByLatSpdRi_B;/* '<S33>/Logical Operator13' */
extern boolean_T LDPSC_DgrFnsRi_B;     /* '<S33>/Logical Operator12' */
extern boolean_T LDPSC_MinLdwBySysSt_B;/* '<S33>/Relational Operator8' */
extern boolean_T LDPSC_EdgeRiseForMinLdw_B;/* '<S60>/AND' */
extern boolean_T LDPSC_HoldForMinLdw_B;/* '<S76>/GreaterThan1' */
extern boolean_T LDPSC_FlagMinTimeLDW_B;/* '<S33>/Logical Operator11' */
extern boolean_T LDPSC_DgrFns_B;       /* '<S69>/AND'
                                        * Condition of danger finish
                                        */
extern boolean_T LDPSC_CancelBySpecific_B;/* '<S36>/Relational Operator38'
                                           * LDW cancel conditions by LDW specific bitfield
                                           */
extern boolean_T LDPSC_CancelByVehSt_B;/* '<S36>/Relational Operator37'
                                        * LDW cancel conditions by vehicle state
                                        */
extern boolean_T LDPSC_CancelByDrvSt_B;/* '<S36>/Relational Operator32'
                                        * LDW cancel conditions by drive state
                                        */
extern boolean_T LDPSC_CancelByCtrlSt_B;/* '<S36>/Relational Operator33'
                                         * LDW cancel conditions by active control state
                                         */
extern boolean_T LDPSC_CancelBySysSt_B;/* '<S36>/Relational Operator34'
                                        * LDW cancel conditions by system state
                                        */
extern boolean_T LDPSC_CancelByAvlSt_B;/* '<S36>/Relational Operator35'
                                        * LDW cancel conditions by no available state
                                        */
extern boolean_T LDPSC_CancelByPrjSpec_B;/* '<S36>/Relational Operator36'
                                          * LDW cancel conditions by customer projects
                                          */
extern boolean_T LDPSC_MaxDurationBySysSt_B;/* '<S40>/Relational Operator47' */
extern boolean_T LDPSC_EdgRiseForSysSt_B;/* '<S41>/AND' */
extern boolean_T LDPSC_MaxDurationByStDly_B;/* '<S40>/Logical Operator22' */
extern boolean_T LDPSC_TiWarmMx_B;     /* '<S40>/Logical Operator23'
                                        * Condition of warming max time
                                        */
extern boolean_T LDPSC_ErrSideByTrigLf_B;/* '<S39>/Logical Operator19' */
extern boolean_T LDPSC_ErrSideBySideCondLf_B;/* '<S39>/Relational Operator42' */
extern boolean_T LDPSC_ErrSidByPrjSpecLf_B;/* '<S39>/Relational Operator43' */
extern boolean_T LDPSC_ErrSidCdtnLf_B; /* '<S39>/Logical Operator18'
                                        * Error condition of left side
                                        */
extern boolean_T LDPSC_SideCondByDgrLf_B;/* '<S39>/Relational Operator41' */
extern boolean_T LDPSC_CanelBySideLf_B;/* '<S39>/Logical Operator15' */
extern boolean_T LDPSC_SideCondByDgrRi_B;/* '<S39>/Relational Operator44' */
extern boolean_T LDPSC_ErrSideByTrigRi_B;/* '<S39>/Logical Operator21' */
extern boolean_T LDPSC_ErrSideBySideCondRi_B;/* '<S39>/Relational Operator45' */
extern boolean_T LDPSC_ErrSidByPrjSpecRi_B;/* '<S39>/Relational Operator46' */
extern boolean_T LDPSC_ErrSidCdtnRi_B; /* '<S39>/Logical Operator2'
                                        * Error condition of right side
                                        */
extern boolean_T LDPSC_CanelBySideRi_B;/* '<S39>/Logical Operator1' */
extern boolean_T LDPSC_ErrSidCdtn_B;   /* '<S39>/Logical Operator16'
                                        * Error condition of side
                                        */
extern boolean_T LDPSC_CLatDevByDlcLf_B;/* '<S38>/Logical Operator24' */
extern boolean_T LDPSC_CLatDevByDgrLf_B;/* '<S38>/Relational Operator39' */
extern boolean_T LDPSC_CclLatDevLf_B;  /* '<S38>/Logical Operator12'
                                        * Cancel condition of left lane deviation
                                        */
extern boolean_T LDPSC_CLatDevByDlcRi_B;/* '<S38>/Relational Operator40' */
extern boolean_T LDPSC_CLatDevByDgrRi_B;/* '<S38>/Logical Operator25' */
extern boolean_T LDPSC_CclLatDevRi_B;  /* '<S38>/Logical Operator14'
                                        * Cancel condition of right lane deviation
                                        */
extern boolean_T LDPSC_CclLatDev_B;    /* '<S38>/Logical Operator13'
                                        * Cancel condition of lane deviation
                                        */
extern boolean_T LDPSC_Cancel_B;       /* '<S36>/Logical Operator11' */
extern boolean_T LDPSC_AbortBySpecific_B;/* '<S37>/Relational Operator2'
                                          * LDW abort conditions by LDW specific bitfield
                                          */
extern boolean_T LDPSC_AbortByVehSt_B; /* '<S37>/Relational Operator1'
                                        * LDW abort conditions by vehicle state
                                        */
extern boolean_T LDPSC_AbortByDrvSt_B; /* '<S37>/Relational Operator3'
                                        * LDW abort conditions by drive state
                                        */
extern boolean_T LDPSC_AbortByCtrlSt_B;/* '<S37>/Relational Operator4'
                                        * LDW abort conditions by active control state
                                        */
extern boolean_T LDPSC_AbortBySysSt_B; /* '<S37>/Relational Operator5'
                                        * LDW abort conditions by system state
                                        */
extern boolean_T LDPSC_AbortByAvlSt_B; /* '<S37>/Relational Operator6'
                                        * LDW abort conditions by no available state
                                        */
extern boolean_T LDPSC_AbortByPrjSpec_B;/* '<S37>/Relational Operator7'
                                         * LDW abort conditions by customer projects
                                         */
extern boolean_T LDPSC_Abort_B;        /* '<S37>/Logical Operator6'
                                        * Condition of LDP abort state
                                        */
extern boolean_T LDPSC_StrgRdy_B;      /* '<S37>/Logical Operator1'
                                        * Condition of LDP strong ready state
                                        */
extern boolean_T LDPSC_Degradation_B;  /* '<S32>/Logical Operator1' */
extern boolean_T LDPSC_DegradationEdgeRise_B;/* '<S58>/AND' */
extern boolean_T LDPSC_Degr_B;         /* '<S32>/Logical Operator2'
                                        * Condition of degradation
                                        */
extern boolean_T LDPSC_SuppBySpecific_B;/* '<S37>/Relational Operator21'
                                         * LDW suppresion conditions by LDW specific bitfield
                                         */
extern boolean_T LDPSC_SuppByVehSt_B;  /* '<S37>/Relational Operator20'
                                        * LDW suppresion conditions by vehicle state
                                        */
extern boolean_T LDPSC_SuppByDrvSt_B;  /* '<S37>/Relational Operator15'
                                        * LDW suppresion conditions by drive state
                                        */
extern boolean_T LDPSC_SuppByCtrlSt_B; /* '<S37>/Relational Operator16'
                                        * LDW suppresion conditions by active control state
                                        */
extern boolean_T LDPSC_SuppBySysSt_B;  /* '<S37>/Relational Operator17'
                                        * LDW suppresion conditions by system state
                                        */
extern boolean_T LDPSC_SuppyByAvlSt_B; /* '<S37>/Relational Operator18'
                                        * LDW suppresion conditions by no available state
                                        */
extern boolean_T LDPSC_SuppPrjSpec_B;  /* '<S37>/Relational Operator19'
                                        * LDW suppresion conditions by customer projects
                                        */
extern boolean_T LDPSC_Suppresion_B;   /* '<S37>/Logical Operator3' */
extern boolean_T LDPSC_WeakRdyBySpecific_B;/* '<S37>/Relational Operator28'
                                            * LDW weak ready conditions by LDW specific bitfield
                                            */
extern boolean_T LDPSC_WeakRdyByVehSt_B;/* '<S37>/Relational Operator27'
                                         * LDW weak ready conditions by vehicle state
                                         */
extern boolean_T LDPSC_WeakRdyByDrvSt_B;/* '<S37>/Relational Operator22'
                                         * LDW weak ready conditions by drive state
                                         */
extern boolean_T LDPSC_WeakRdyByCtrlSt_B;/* '<S37>/Relational Operator23'
                                          * LDW strong weak conditions by active control state
                                          */
extern boolean_T LDPSC_WeakRdyBySysSt_B;/* '<S37>/Relational Operator24'
                                         * LDW weak ready conditions by system state
                                         */
extern boolean_T LDPSC_WeakRdyByAvlSt_B;/* '<S37>/Relational Operator25'
                                         * LDW weak weak conditions by no available state
                                         */
extern boolean_T LDPSC_WeakRdyByPrjSpec_B;/* '<S37>/Relational Operator26'
                                           * LDW weak weak conditions by customer projects
                                           */
extern boolean_T LDPSC_WkRdy_B;        /* '<S37>/Logical Operator4'
                                        * Condition of LDP weak ready state
                                        */
extern boolean_T LDPSC_BlockTimeBySysOut_B;/* '<S37>/Logical Operator9' */
extern boolean_T LDPSC_RawBlockTimeByRampOut_B;/* '<S44>/AND' */
extern boolean_T LDPSC_BlockTimeByRampOut_B;/* '<S56>/GreaterThan1' */
extern boolean_T LDPSC_BlockTime_B;    /* '<S37>/Logical Operator7' */
extern boolean_T LDPSC_Suppression_B;  /* '<S35>/OR'
                                        * Suppression condition
                                        */
extern boolean_T LDPVSE_TgtCntrByLnWidth_B;/* '<S160>/Less Than' */
extern boolean_T LDPVSE_TgtCntrLnEn_B; /* '<S160>/Logical Operator11'
                                        *  Enable the target in the center of the lane.
                                        */
extern boolean_T LDPTV_CurvInner_B;    /* '<S157>/AND2'
                                        * The flag for the left or right lane marking is an inner curve
                                        */
extern boolean_T LDPTV_CurvOuter_B;    /* '<S157>/AND1'
                                        * The flag for the left or right lane marking is an outer curve
                                        */
extern boolean_T LDPSC_Trig_B;         /* '<S77>/Logical Operator1'
                                        * Condition of trigger
                                        */
extern E_LDPState_nu LDPSC_SysOut_St;  /* '<S23>/Switch1'
                                        * Actual state of LDP
                                        */

/*
 * Exported States
 *
 * Note: Exported states are block states with an exported global
 * storage class designation.  Code generation will declare the memory for these
 * states and exports their symbols.
 *
 */
extern real32_T LDPSC_DlyTiOfTiToLnRiMn_Sec;/* '<S106>/Unit Delay'
                                             * Delay time of time to right lane crossing
                                             */
extern real32_T LDPSC_HdTiTrigRi_Sec;  /* '<S104>/Unit Delay'
                                        * holding time right trigger
                                        */
extern real32_T LDPSC_ContinWarmTimesOldRi_Count;/* '<S80>/Unit Delay2'
                                                  * The number of consecutive alarms in the previous cycle
                                                  */
extern real32_T LDPSC_SuppTimeOldRi_Sec;/* '<S105>/Unit Delay'
                                         * Holding time of left turn signal
                                         */
extern real32_T LDPVSE_HodTiTrnSglRi_Sec;/* '<S176>/Unit Delay'
                                          * Holding time of right turn signal
                                          */
extern real32_T LDPSC_DlyTiOfTiToLnLfMn_Sec;/* '<S94>/Unit Delay'
                                             * Delay time of time to left lane crossing
                                             */
extern real32_T LDPSC_HdTiTrigLf_Sec;  /* '<S92>/Unit Delay'
                                        *  holding time left trigger
                                        */
extern real32_T LDPSC_ContinWarmTimesOldLf_Count;/* '<S79>/Unit Delay2'
                                                  * The number of consecutive alarms in the previous cycle
                                                  */
extern real32_T LDPSC_SuppTimeOldLf_Sec;/* '<S93>/Unit Delay'
                                         * Holding time of left turn signal
                                         */
extern real32_T LDPVSE_HodTiTrnSglLf_Sec;/* '<S175>/Unit Delay'
                                          * Holding time of left turn signal
                                          */
extern real32_T LDPSC_HdTiWarming_Sec; /* '<S76>/Unit Delay'
                                        * Holding time of warming state start
                                        */
extern real32_T LDPSC_DlyTiTgtFns_Sec; /* '<S69>/Unit Delay'
                                        * Delay time of LDP finish state
                                        */
extern real32_T LDPSC_HdTiWarmMx_Sec;  /* '<S43>/Unit Delay'
                                        * Holding time of warming state start
                                        */
extern real32_T LDPSC_HdTiDegr_Sec;    /* '<S59>/Unit Delay'
                                        * Holding time of degradation
                                        */
extern real32_T LDPSC_HdTiFns_Sec;     /* '<S56>/Unit Delay'
                                        * Holding time of finish state end
                                        */
extern real32_T LDPSC_ActiveStopWatch_Ri_sec;/* '<S55>/Unit Delay' */
extern real32_T LDPSC_HdTiFns_Ri_Sec;  /* '<S57>/Unit Delay' */
extern real32_T LDPTT_LwLnBdryPstnXLf_Mi;/* '<S129>/Unit Delay'
                                          * Low filtered left lane clothoid X0 position from LDP
                                          */
extern real32_T LDPTT_LstLnBdryPstnXLf_Mi;/* '<S126>/UnitDelay'
                                           * Last filtered left lane clothoid Y0 position from LDP
                                           */
extern real32_T LDPTT_LstLnBdryVldLengLf_Mi;/* '<S126>/UnitDelay5'
                                             * Last filtered left lane clothoid length from LDP
                                             */
extern real32_T LDPTT_LwLnBdryVldLengLf_Mi;/* '<S134>/Unit Delay'
                                            * Low filtered left lane clothoid length from LDP
                                            */
extern real32_T LDPTT_LstLnBdryPstnYLf_Mi;/* '<S126>/UnitDelay1'
                                           * Last filtered left lane clothoid Y0 position from LDP
                                           */
extern real32_T LDPTT_LstLnWidCalc_Mi; /* '<S121>/UnitDelay'
                                        * Calculation ego lane width
                                        */
extern real32_T LDPTT_LwLnBdryPstnYLf_Mi;/* '<S130>/Unit Delay'
                                          * Low filtered left lane clothoid Y0 position from LDP
                                          */
extern real32_T LDPTT_LwLnBdryPstnYRi_Mi;/* '<S136>/Unit Delay'
                                          * Low filtered right lane clothoid Y0 position from LDP
                                          */
extern real32_T LDPTT_LwLnBdryPstnXRi_Mi;/* '<S135>/Unit Delay'
                                          * Low filtered right lane clothoid X0 position from LDP
                                          */
extern real32_T LDPTT_LwLnBdryHeadAglRi_Rad;/* '<S137>/Unit Delay'
                                             * Low filtered right lane clothoid heading angle from LDP
                                             */
extern real32_T LDPTT_LwLnBdryCurvRi_ReMi;/* '<S138>/Unit Delay'
                                           * Low filtered right lane clothoid curvature from LDP
                                           */
extern real32_T LDPTT_LwLnBdryCurvRateRi_ReMi2;/* '<S139>/Unit Delay'
                                                * Low filtered right lane clothoid change of curvature from LDP
                                                */
extern real32_T LDPTT_LwLnBdryVldLengRi_Mi;/* '<S140>/Unit Delay'
                                            * Low filtered right lane clothoid length from LDP
                                            */
extern real32_T LDPTT_LstTgtLatDstcRi_Mi;/* '<S121>/UnitDelay4'
                                          * Last distance between the hazardous lane marking and the planned target.
                                          */
extern real32_T LDPTT_LstMxTgtLatDevRi_Mi;/* '<S121>/UnitDelay6'
                                           * Last maximal distance between the middle of the vehicle and the planned target.
                                           */
extern real32_T LDPTT_LstTgtLatDevRi_Mi;/* '<S121>/UnitDelay7'
                                         * Last Distance between the middle of the vehicle and the planned target.
                                         */
extern real32_T LDPTT_LstTgtLatDevLf_Mi;/* '<S121>/UnitDelay3'
                                         * Last Distance between the middle of the vehicle and the planned target.
                                         */
extern real32_T LDPTT_LstTgtLatDstcLf_Mi;/* '<S121>/UnitDelay1'
                                          * Last distance between the hazardous lane marking and the planned target.
                                          */
extern real32_T LDPTT_LstMxTgtLatDevLf_Mi;/* '<S121>/UnitDelay2'
                                           * Last maximal distance between the middle of the vehicle and the planned target.
                                           */
extern real32_T LDPTT_LwLnBdryPstnYCent_Mi;/* '<S141>/Unit Delay'
                                            * Low filtered center lane clothoid Y0 position from LDP
                                            */
extern real32_T LDPTT_LstLnBdryPstnYCent_Mi;/* '<S128>/UnitDelay1'
                                             * Last filtered center lane clothoid Y0 position from LDP
                                             */
extern real32_T LDPTT_LstLnBdryCurvCent_ReMi;/* '<S128>/UnitDelay3'
                                              * Last filtered center lane clothoid curvature from LDP
                                              */
extern real32_T LDPTT_LwLnBdryCurvCent_ReMi;/* '<S143>/Unit Delay'
                                             * Low filtered center lane clothoid curvature from LDP
                                             */
extern real32_T LDPTV_LstPlanningHorizon_Sec;/* '<S150>/UnitDelay1'
                                              * Planning Horizon Scaling Factor for the calculation in the TRJPLN
                                              */
extern real32_T LDPTV_LstWeightEndTi_Fct;/* '<S150>/UnitDelay'
                                          * Last weight of the end time for the calculation in the TRJPLN
                                          */
extern real32_T LDPTT_LwLnBdryHeadAglCent_Rad;/* '<S142>/Unit Delay'
                                               * Low filtered center lane clothoid heading angle from LDP
                                               */
extern real32_T LDPTT_LstLnBdryHeadAglCent_Rad;/* '<S128>/UnitDelay2'
                                                * Last filtered center lane clothoid heading angle from LDP
                                                */
extern real32_T LDPTT_LwLnBdryCurvRateCent_ReMi2;/* '<S144>/Unit Delay'
                                                  * Low filtered center lane clothoid change of curvature from LDP
                                                  */
extern real32_T LDPTT_LstLnBdryCurvRateCent_ReMi2;/* '<S128>/UnitDelay4'
                                                   * Last filtered center lane clothoid change of curvature from LDP
                                                   */
extern real32_T LDPTT_LwLnBdryPstnXCent_Mi;/* '<S145>/Unit Delay'
                                            * Low filtered center lane clothoid X0 position from LDP
                                            */
extern real32_T LDPTT_LstLnBdryPstnXCent_Mi;/* '<S128>/UnitDelay'
                                             * Last filtered center lane clothoid X0 position from LDP
                                             */
extern real32_T LDPTT_LwLnBdryVldLengCent_Mi;/* '<S146>/Unit Delay'
                                              * Low filtered center lane clothoid length from LDP
                                              */
extern real32_T LDPTT_LstLnBdryVldLengCent_Mi;/* '<S128>/UnitDelay5'
                                               * Last filtered center lane clothoid length from LDP
                                               */
extern real32_T LDPTV_LstSteWhlGrad_ReS;/* '<S148>/UnitDelay1'
                                         * Last Steering Wheel Stiffness Gradient by Lateral Velocity
                                         */
extern real32_T LDPTV_LstDMCDeraLvl_Fct;/* '<S148>/UnitDelay'
                                         * Last DMC Derating Level of the LDP function
                                         */
extern real32_T LDPTT_LstLnBdryHeadAglLf_Rad;/* '<S126>/UnitDelay2'
                                              * Last filtered left lane clothoid heading angle from LDP
                                              */
extern real32_T LDPTT_LwLnBdryHeadAglLf_Rad;/* '<S131>/Unit Delay'
                                             * Low filtered left lane clothoid heading angle from LDP
                                             */
extern real32_T LDPTT_LstLnBdryCurvLf_ReMi;/* '<S126>/UnitDelay3'
                                            * Last filtered left lane clothoid curvature from LDP
                                            */
extern real32_T LDPTT_LwLnBdryCurvLf_ReMi;/* '<S132>/Unit Delay'
                                           * Low filtered left lane clothoid curvature from LDP
                                           */
extern real32_T LDPTT_LstLnBdryCurvRateLf_ReMi2;/* '<S126>/UnitDelay4'
                                                 * Last filtered left lane clothoid change of curvature from LDP
                                                 */
extern real32_T LDPTT_LwLnBdryCurvRateLf_ReMi2;/* '<S133>/Unit Delay'
                                                * Low filtered left lane clothoid change of curvature from LDP
                                                */
extern uint8_T LDPSC_LstPrevDgrSide_St;/* '<S2>/Unit Delay' */
extern uint8_T LDPSC_DgrSideOld_St;    /* '<S77>/UnitDelay1'
                                        * Old state of danger side
                                        */
extern boolean_T LDPDT_UHysCltdCurvVldRi_B;/* '<S19>/Unit Delay'
                                            * Validity of right lane Clothoid curvature
                                            */
extern boolean_T LDPDT_BHysHeadAglTrigVldRi_B;/* '<S18>/Unit Delay'
                                               * Valid trigger of right heading angle
                                               */
extern boolean_T LDPDT_UHysHeadAglCclVldRi_B;/* '<S20>/Unit Delay'
                                              * Valid cancel of right heading angle
                                              */
extern boolean_T LDPSC_HdTiTrigRiEn_B; /* '<S95>/Unit Delay'
                                        * Enable condition of holding time right trigger
                                        */
extern boolean_T LDPSC_DisTrigRi_B;    /* '<S102>/Unit Delay'
                                        * Disable condition of right trigger
                                        */
extern boolean_T LDPSC_SuppFlagOldRi_B;/* '<S80>/Unit Delay'
                                        * Suppression signal of continuous alarm in previous period
                                        */
extern boolean_T LDPSC_PreActiveEdgeRi;/* '<S98>/Unit Delay'
                                        * Enable condition of continue left trigger
                                        */
extern boolean_T LDPSC_ContinTrigRiEn_B;/* '<S99>/Unit Delay'
                                         * Enable condition of continue left trigger
                                         */
extern boolean_T LDPSC_DisContinTrigRi_B;/* '<S103>/Unit Delay'
                                          * Disable condition of continue left trigger
                                          */
extern boolean_T LDPVSE_EdgeRisTrnSglLf_B;/* '<S174>/Unit Delay'
                                           * Edge rise of left turn signal
                                           */
extern boolean_T LDPVSE_BHysLatVehSpdVldRi_B;/* '<S185>/Unit Delay'
                                              * Validity of right lateral vehicle speed before trigger state
                                              */
extern boolean_T LDPVSE_UHysLatVehSpdVldRi_B;/* '<S186>/Unit Delay'
                                              * Validity of right lateral vehicle speed after trigger state
                                              */
extern boolean_T LDPDT_UHysCltdCurvVldLf_B;/* '<S15>/Unit Delay'
                                            * Validity of left lane Clothoid curvature
                                            */
extern boolean_T LDPDT_BHysHeadAglTrigVldLf_B;/* '<S14>/Unit Delay'
                                               * Valid trigger of left heading angle
                                               */
extern boolean_T LDPDT_UHysHeadAglCclVldLf_B;/* '<S16>/Unit Delay'
                                              * Valid cancel of left heading angle
                                              */
extern boolean_T LDPSC_HdTiTrigLfEn_B; /* '<S83>/Unit Delay'
                                        * Enable condition of holding time left trigger
                                        */
extern boolean_T LDPSC_DisTrigLf_B;    /* '<S90>/Unit Delay'
                                        * Disable condition of left trigger
                                        */
extern boolean_T LDPSC_SuppFlagOldLf_B;/* '<S79>/Unit Delay'
                                        * Suppression signal of continuous alarm in previous period
                                        */
extern boolean_T LDPSC_PreActiveEdgeLf;/* '<S86>/Unit Delay'
                                        * Enable condition of continue left trigger
                                        */
extern boolean_T LDPSC_ContinTrigLfEn_B;/* '<S87>/Unit Delay'
                                         * Enable condition of continue left trigger
                                         */
extern boolean_T LDPSC_DisContinTrigLf_B;/* '<S91>/Unit Delay'
                                          * Disable condition of continue left trigger
                                          */
extern boolean_T LDPVSE_EdgeRisTrnSglRi_B;/* '<S173>/Unit Delay'
                                           * Edge rise of right turn signal
                                           */
extern boolean_T LDPVSE_BHysLatVehSpdVldLf_B;/* '<S181>/Unit Delay'
                                              * Validity of left lateral vehicle speed before trigger state
                                              */
extern boolean_T LDPVSE_UHysLatVehSpdVldLf_B;/* '<S182>/Unit Delay'
                                              * Validity of left lateral vehicle speed after trigger state
                                              */
extern boolean_T LDPSC_EdgeRisWarming_B;/* '<S60>/Unit Delay'
                                         * Edge rise of warming state
                                         */
extern boolean_T LDPVSE_BHysSpdVeh_B;  /* '<S163>/Unit Delay'
                                        * Validity of displayed longitudinal speed
                                        */
extern boolean_T LDPVSE_UHysSteAgl_B;  /* '<S167>/Unit Delay'
                                        * Validity of steering wheel angle
                                        */
extern boolean_T LDPVSE_UHysSteAglSpd_B;/* '<S168>/Unit Delay'
                                         * Validity of steering wheel angle speed
                                         */
extern boolean_T LDPVSE_BHysAccVehX_B; /* '<S164>/Unit Delay'
                                        * Validity of  longitudinal Acceleration
                                        */
extern boolean_T LDPVSE_BHysAccVehY_B; /* '<S169>/Unit Delay'
                                        * Validity of  lateral Acceleration
                                        */
extern boolean_T LDPVSE_UHysVehCurv_B; /* '<S170>/Unit Delay'
                                        * Validity of  vehicle curvature
                                        */
extern boolean_T LDPVSE_BHysLnWid_B;   /* '<S165>/Unit Delay'
                                        * Validity of lane width
                                        */
extern boolean_T LDPSC_EdgeRisWarmMx_B;/* '<S41>/Unit Delay'
                                        * Edge rise of warming state
                                        */
extern boolean_T LDPSC_EdgeRisDegr_B;  /* '<S58>/Unit Delay'
                                        * Edge rise of degradation
                                        */
extern boolean_T LDPSC_DegrOld_B;      /* '<S32>/UnitDelay'
                                        * UnitDelay condition of degradation
                                        */
extern boolean_T LDPSC_EdgeRisFns_B;   /* '<S44>/Unit Delay'
                                        * Edge rise of fginish and cancel state
                                        */
extern boolean_T LDPSC_EdgeRisActive_Ri_B;/* '<S47>/Unit Delay' */
extern boolean_T LDPSC_EdgeRisFns_Ri_B;/* '<S45>/Unit Delay' */
extern boolean_T LDPTT_CtrlIniEn_B;    /* '<S122>/Unit Delay'
                                        * Control initenable flag for LDP
                                        */
extern boolean_T LDPTT_LstControl_B;   /* '<S121>/UnitDelay5'
                                        * Control Sate of LDP
                                        */
extern boolean_T LDPTV_LstCtrl_St;     /* '<S148>/UnitDelay4'
                                        * Last Control State for DetermineStiffnessAndStatAccu
                                        */
extern E_LDPState_nu LDPSC_SysOld_St;  /* '<S4>/UnitDelay'
                                        * Old state of LDP
                                        */

/* Model entry point functions */
extern void LDPSA_initialize(void);
extern void LDPSA_step(void);

/* Exported data declaration */

/* Declaration for custom storage class: Global */
extern uint8_T LDPSA_SetBit_BS_Param_1[9];
                                   /* Referenced by: '<S116>/ex_sfun_set_bit' */
extern uint8_T LDPSA_SetBit_BS_Param_2[8];
                                   /* Referenced by: '<S172>/ex_sfun_set_bit' */
extern uint8_T LDPSA_SetBit_BS_Param_3[2];/* Referenced by:
                                           * '<S183>/ex_sfun_set_bit'
                                           * '<S184>/ex_sfun_set_bit'
                                           */
extern real32_T LDPSC_LnLatVeh_BX_Mps[9];/* Referenced by:
                                          * '<S78>/1-D Lookup Table10'
                                          * '<S78>/1-D Lookup Table11'
                                          */
extern real32_T LDPSC_LnLatVeh_Lf_Mps[9];
                                 /* Referenced by: '<S78>/1-D Lookup Table10' */
extern real32_T LDPSC_LnLatVeh_Ri_Mps[9];
                                 /* Referenced by: '<S78>/1-D Lookup Table11' */

/* ConstVolatile memory section */
/* Declaration for custom storage class: ConstVolatile */
extern const volatile boolean_T LDPDT_CstruSiteLDP_C_B;/* Referenced by:
                                                        * '<S12>/V_Parameter7'
                                                        * '<S13>/V_Parameter7'
                                                        */

/* Switch of consturction side */
extern const volatile uint8_T LDPDT_CurveInner_C_St;/* Referenced by:
                                                     * '<S11>/V_Parameter11'
                                                     * '<S11>/V_Parameter2'
                                                     * '<S78>/V_Parameter2'
                                                     * '<S78>/V_Parameter3'
                                                     */

/* Constant of inner curve lane */
extern const volatile uint8_T LDPDT_CurveNone_C_St;/* Referenced by:
                                                    * '<S11>/V_Parameter13'
                                                    * '<S11>/V_Parameter5'
                                                    */

/* Constant of Straight lane */
extern const volatile uint8_T LDPDT_CurveOuter_C_St;/* Referenced by:
                                                     * '<S11>/V_Parameter12'
                                                     * '<S11>/V_Parameter4'
                                                     * '<S78>/V_Parameter1'
                                                     * '<S78>/V_Parameter4'
                                                     */

/* Constant of Outer curve lane */
extern const volatile real32_T LDPDT_CurveThd_C_St;/* Referenced by: '<S11>/V_Parameter1' */

/* Curve threshold of lane */
extern const volatile uint16_T LDPDT_LnIvldCclLf_C_St;
                                      /* Referenced by: '<S12>/V_Parameter13' */

/* Invalid cancel state of left lane
   ctrl for 4095
   safety for 15 */
extern const volatile uint16_T LDPDT_LnIvldCclRi_C_St;/* Referenced by: '<S13>/V_Parameter2' */

/* Invalid cancel state of right lane */
extern const volatile uint16_T LDPDT_LnIvldLf_C_St;/* Referenced by: '<S12>/V_Parameter4' */

/* Invalid state of left lane
   ctrl for 20479
   safety for 15 */
extern const volatile uint16_T LDPDT_LnIvldRi_C_St;/* Referenced by: '<S13>/V_Parameter4' */

/* Invalid state of right lane */
extern const volatile uint8_T LDPDT_LnIvldSfLf_C_St;/* Referenced by: '<S12>/V_Parameter3' */

/* Invalid safety state of left lane */
extern const volatile uint8_T LDPDT_LnIvldSfRi_C_St;/* Referenced by: '<S13>/V_Parameter3' */

/* Invalid safety state of right lane */
extern const volatile uint8_T LDPDT_NoDgrSide_C_St;/* Referenced by: '<S8>/V_Parameter' */

/* State value of no danger of lane */
extern const volatile boolean_T LDPDT_SfFcLDPOn_C_B;/* Referenced by:
                                                     * '<S8>/V_Parameter1'
                                                     * '<S12>/V_Parameter1'
                                                     * '<S13>/V_Parameter1'
                                                     */

/* LDP switch of safety face */
extern const volatile real32_T LDPDT_TLCHeadAglTrsd_C_Rad;/* Referenced by:
                                                           * '<S10>/V_Parameter1'
                                                           * '<S10>/V_Parameter2'
                                                           */

/* Threshold of heading angle at TLC */
extern const volatile real32_T LDPDT_TrsdHeadAglMn_C_Rad;/* Referenced by:
                                                          * '<S12>/V_Parameter11'
                                                          * '<S13>/V_Parameter11'
                                                          */

/* Minimum threshold of heading angle */
extern const volatile real32_T LDPDT_TrsdHeadAglMx_C_Rad;/* Referenced by:
                                                          * '<S12>/V_Parameter10'
                                                          * '<S12>/V_Parameter8'
                                                          * '<S13>/V_Parameter10'
                                                          * '<S13>/V_Parameter8'
                                                          */

/* Maximum threshold of heading angle */
extern const volatile real32_T LDPDT_TrsdHeadAglOfst_C_Rad;/* Referenced by:
                                                            * '<S12>/V_Parameter12'
                                                            * '<S12>/V_Parameter2'
                                                            * '<S13>/V_Parameter12'
                                                            * '<S13>/V_Parameter5'
                                                            */

/* Offset of heading angle threshold */
extern const volatile real32_T LDPDT_TrsdLnCltdCurvLfMx_Cr_Mps[8];
                                   /* Referenced by: '<S12>/1-D Lookup Table' */

/* Curve of maximum threshold of left clothiod curvature */
extern const volatile real32_T LDPDT_TrsdLnCltdCurvLfOfst_Cr_Mps[8];
                                  /* Referenced by: '<S12>/1-D Lookup Table1' */

/* Curve of offset threshold of left clothiod curvature */
extern const volatile real32_T LDPDT_TrsdLnCltdCurvRiMx_Cr_Mps[8];
                                   /* Referenced by: '<S13>/1-D Lookup Table' */

/* Curve of maximum threshold of rifht clothiod curvature */
extern const volatile real32_T LDPDT_TrsdLnCltdCurvRiOfst_Cr_Mps[8];
                                  /* Referenced by: '<S13>/1-D Lookup Table1' */

/* Curve of offset threshold of right clothiod curvature */
extern const volatile real32_T LDPDT_VehSpdX_BX_Mps[8];/* Referenced by:
                                                        * '<S12>/1-D Lookup Table'
                                                        * '<S12>/1-D Lookup Table1'
                                                        * '<S13>/1-D Lookup Table'
                                                        * '<S13>/1-D Lookup Table1'
                                                        */

/* Breakpoint of vehicle speed */
extern const volatile uint8_T LDPSC_AbtDrvActCtrl_C_St;
                                      /* Referenced by: '<S37>/V_Parameter11' */

/* Abort state of active control */
extern const volatile uint8_T LDPSC_AbtDrvIVld_C_St;
                                      /* Referenced by: '<S37>/V_Parameter10' */

/* Abort state of invalid driver */
extern const volatile uint8_T LDPSC_AbtErrSpcLDP_C_St;
                                      /* Referenced by: '<S37>/V_Parameter17' */

/* Abort state of error specific */
extern const volatile uint8_T LDPSC_AbtFctCstm_C_St;
                                      /* Referenced by: '<S37>/V_Parameter14' */

/* Abort state of customer specific */
extern const volatile uint8_T LDPSC_AbtNoAvlbVehSys_C_St;
                                      /* Referenced by: '<S37>/V_Parameter13' */

/* Abort state of no availible vehicle system signals */
extern const volatile uint16_T LDPSC_AbtVehIvld_C_St;/* Referenced by: '<S37>/V_Parameter9' */

/* Abort state of invalid vehicle */
extern const volatile uint8_T LDPSC_AbtVehSysErr_C_St;
                                      /* Referenced by: '<S37>/V_Parameter12' */

/* Abort state of vehicle system errors */
extern const volatile uint8_T LDPSC_CclDrvActCtrl_C_St;
                                      /* Referenced by: '<S36>/V_Parameter42' */

/* Cancel state of active control */
extern const volatile uint8_T LDPSC_CclDrvIVld_C_St;
                                      /* Referenced by: '<S36>/V_Parameter41' */

/* Cancel state of invalid driver */
extern const volatile uint8_T LDPSC_CclErrSpcLDP_C_St;
                                      /* Referenced by: '<S36>/V_Parameter46' */

/* Cancel state of error specific */
extern const volatile uint8_T LDPSC_CclFctCstm_St;
                                      /* Referenced by: '<S36>/V_Parameter45' */

/* Cancel state of customer specific */
extern const volatile uint8_T LDPSC_CclNoAvlbVehSys_C_St;
                                      /* Referenced by: '<S36>/V_Parameter44' */

/* Cancel state of no availible vehicle system signals */
extern const volatile uint16_T LDPSC_CclVehIvld_C_St;
                                      /* Referenced by: '<S36>/V_Parameter47' */

/* Cancel state of invalid vehicle */
extern const volatile uint8_T LDPSC_CclVehSysErr_C_St;
                                      /* Referenced by: '<S36>/V_Parameter43' */

/* Cancel state of vehicle system errors */
extern const volatile real32_T LDPSC_ContiActiveTiFns_C_Sec;/* Referenced by:
                                                             * '<S37>/V_Parameter4'
                                                             * '<S37>/V_Parameter6'
                                                             */

/* Maximum time of warming state */
extern const volatile real32_T LDPSC_ContinWarmSupp_C_Sec;/* Referenced by:
                                                           * '<S79>/V_Parameter3'
                                                           * '<S80>/V_Parameter3'
                                                           */

/* the time of continous warning suppression */
extern const volatile real32_T LDPSC_ContinWarmTimes_C_Count;/* Referenced by:
                                                              * '<S79>/V_Parameter4'
                                                              * '<S80>/V_Parameter5'
                                                              */

/* The number of consecutive alarms allowed
 */
extern const volatile real32_T LDPSC_ContinuActiveTi_C_Sec;/* Referenced by:
                                                            * '<S37>/V_Parameter1'
                                                            * '<S37>/V_Parameter2'
                                                            */

/* Maximum time of warming state */
extern const volatile real32_T LDPSC_CrvSensiAdvance_BX_ReMi[4];
                                  /* Referenced by: '<S78>/1-D Lookup Table9' */

/* Breakpoint of detected right lane curvature */
extern const volatile real32_T LDPSC_CrvSensiAdvance_BY_Mi[4];
                                  /* Referenced by: '<S78>/1-D Lookup Table9' */

/* Breakpoint of detected right lane curvature */
extern const volatile real32_T LDPSC_CrvSensiDecay_BX_ReMi[4];
                                  /* Referenced by: '<S78>/1-D Lookup Table8' */

/* Breakpoint of detected right lane curvature */
extern const volatile real32_T LDPSC_CrvSensiDecay_BY_Mi[4];
                                  /* Referenced by: '<S78>/1-D Lookup Table8' */

/* Breakpoint of detected right lane curvature */
extern const volatile real32_T LDPSC_DTCFctLnWid_Cr_Fct[5];
                                  /* Referenced by: '<S78>/1-D Lookup Table3' */

/* Lane width factor of DTL    */
extern const volatile real32_T LDPSC_DgrCclOfst_C_Mi;/* Referenced by:
                                                      * '<S38>/V_Parameter1'
                                                      * '<S38>/V_Parameter53'
                                                      */

/* Danger offset distance of cancel state */
extern const volatile real32_T LDPSC_DgrFnsHeadAng_C_Rad;/* Referenced by:
                                                          * '<S33>/V_Parameter23'
                                                          * '<S33>/V_Parameter38'
                                                          */

/* Danger of heading angle  */
extern const volatile real32_T LDPSC_DgrFnsOfst_C_Mi;/* Referenced by:
                                                      * '<S33>/V_Parameter19'
                                                      * '<S33>/V_Parameter34'
                                                      */

/* Danger offset distance of finish state */
extern const volatile real32_T LDPSC_DgrFnsSpdVelLat_C_Mps;/* Referenced by:
                                                            * '<S33>/V_Parameter25'
                                                            * '<S33>/V_Parameter40'
                                                            */

/* Danger of lateral speed */
extern const volatile uint8_T LDPSC_DgrSideLf_C_St;/* Referenced by:
                                                    * '<S33>/V_Parameter17'
                                                    * '<S77>/V_Parameter6'
                                                    * '<S108>/V_Parameter32'
                                                    * '<S109>/V_Parameter32'
                                                    * '<S38>/V_Parameter48'
                                                    * '<S39>/V_Parameter56'
                                                    */

/* Constant of left side danger */
extern const volatile uint8_T LDPSC_DgrSideRi_C_St;/* Referenced by:
                                                    * '<S33>/V_Parameter32'
                                                    * '<S77>/V_Parameter7'
                                                    * '<S108>/V_Parameter2'
                                                    * '<S109>/V_Parameter1'
                                                    * '<S38>/V_Parameter49'
                                                    * '<S39>/V_Parameter59'
                                                    */

/* Constant of right side danger */
extern const volatile real32_T LDPSC_DlyTiOfTiToLnMn_C_Sec;/* Referenced by:
                                                            * '<S79>/V_Parameter12'
                                                            * '<S80>/V_Parameter12'
                                                            */

/* Delay time of time to lane crossing */
extern const volatile real32_T LDPSC_DlyTiTgtFns_C_Sec;/* Referenced by: '<S33>/V_Parameter9' */

/* Delay time of target finish  */
extern const volatile uint8_T LDPSC_DrvMod2_C_St;/* Referenced by: '<S78>/V_Parameter9' */

/* Driver control mode of LDP £º2 mode */
extern const volatile uint8_T LDPSC_DrvMod3_C_St;
                                      /* Referenced by: '<S78>/V_Parameter10' */

/* Driver control mode of LDP £º3 mode */
extern const volatile real32_T LDPSC_DstcOfDiscToLnLmtMn_C_Mi;/* Referenced by:
                                                               * '<S79>/V_Parameter14'
                                                               * '<S79>/V_Parameter5'
                                                               * '<S80>/V_Parameter14'
                                                               * '<S80>/V_Parameter4'
                                                               */

/* Minimum distance limiting value of distance to lane crossing */
extern const volatile real32_T LDPSC_DstcOfTiToLnMn_C_Mi;/* Referenced by:
                                                          * '<S79>/V_Parameter10'
                                                          * '<S80>/V_Parameter10'
                                                          */

/* Minimum distance of distance to lane crossing */
extern const volatile real32_T LDPSC_DstcOfstSafeSitu_C_Mi;/* Referenced by:
                                                            * '<S79>/V_Parameter13'
                                                            * '<S80>/V_Parameter13'
                                                            */

/* Offset distance of safe situation */
extern const volatile real32_T LDPSC_DstcToLnTrsdOfstLf_Cr_Mi[17];
                                  /* Referenced by: '<S78>/1-D Lookup Table5' */

/* Curve table of left offset distance to lane crossing threshold      */
extern const volatile real32_T LDPSC_DstcToLnTrsdOfstRi_Cr_Mi[17];
                                  /* Referenced by: '<S78>/1-D Lookup Table4' */

/* Curve table of right offset distance to lane crossing threshold      */
extern const volatile real32_T LDPSC_DstcTrsdVehSpdXDTL1_Cr_Mi[9];
                                   /* Referenced by: '<S78>/1-D Lookup Table' */

/* Curve table of distance to lane crossing threshold at mode 1     */
extern const volatile real32_T LDPSC_DstcTrsdVehSpdXDTL2_Cr_Mi[9];
                                  /* Referenced by: '<S78>/1-D Lookup Table1' */

/* Curve table of distance to lane crossing threshold at mode 2     */
extern const volatile real32_T LDPSC_DstcTrsdVehSpdXDTL3_Cr_Mi[9];
                                  /* Referenced by: '<S78>/1-D Lookup Table2' */

/* Curve table of distance to lane crossing threshold at mode 3     */
extern const volatile uint8_T LDPSC_ErrCstmCclLf_C_St;
                                      /* Referenced by: '<S39>/V_Parameter58' */

/* Cancel state of left customer specific */
extern const volatile uint8_T LDPSC_ErrCstmCclRi_C_St;
                                      /* Referenced by: '<S39>/V_Parameter61' */

/* Cancel state of right customer specific */
extern const volatile uint8_T LDPSC_FnsCdsnEn_C_St;/* Referenced by:
                                                    * '<S33>/V_Parameter28'
                                                    * '<S33>/V_Parameter29'
                                                    * '<S33>/V_Parameter30'
                                                    * '<S33>/V_Parameter31'
                                                    * '<S33>/V_Parameter41'
                                                    * '<S33>/V_Parameter42'
                                                    * '<S33>/V_Parameter43'
                                                    */

/* State switch of finish condition  */
extern const volatile real32_T LDPSC_FnsDuraMn_C_Sec;
                                      /* Referenced by: '<S33>/V_Parameter27' */

/* Minimum duration of finish state */
extern const volatile real32_T LDPSC_HdTiTrigLf_C_Sec;/* Referenced by:
                                                       * '<S79>/V_Parameter15'
                                                       * '<S80>/V_Parameter15'
                                                       */

/* Holding time of left warming trigger */
extern const volatile real32_T LDPSC_LaneWidth_BX_Mi[5];/* Referenced by:
                                                         * '<S78>/1-D Lookup Table3'
                                                         * '<S78>/1-D Lookup Table7'
                                                         */

/* breakpoint of lane width */
extern const volatile real32_T LDPSC_LnDectCrvLf_BX_ReMi[17];
                                  /* Referenced by: '<S78>/1-D Lookup Table5' */

/* Breakpoint of detected left lane curvature */
extern const volatile real32_T LDPSC_LnDectCrvRi_BX_ReMi[17];
                                  /* Referenced by: '<S78>/1-D Lookup Table4' */

/* Breakpoint of detected right lane curvature */
extern const volatile real32_T LDPSC_NoDgrCclOfst_C_Mi;/* Referenced by:
                                                        * '<S38>/V_Parameter51'
                                                        * '<S38>/V_Parameter54'
                                                        */

/* No danger offset distance of cancel state */
extern const volatile real32_T LDPSC_NoDgrFnsHeadAng_C_Rad;/* Referenced by:
                                                            * '<S33>/V_Parameter22'
                                                            * '<S33>/V_Parameter37'
                                                            */

/*  No danger of heading angle      */
extern const volatile real32_T LDPSC_NoDgrFnsOfst_C_Mi;/* Referenced by:
                                                        * '<S33>/V_Parameter21'
                                                        * '<S33>/V_Parameter36'
                                                        */

/* No danger offset distance of finish state */
extern const volatile real32_T LDPSC_NoDgrFnsSpdVelLat_C_Mps;/* Referenced by:
                                                              * '<S33>/V_Parameter24'
                                                              * '<S33>/V_Parameter39'
                                                              */

/* No danger of lateral speed */
extern const volatile uint8_T LDPSC_NoDgrSide_C_St;/* Referenced by:
                                                    * '<S77>/V_Parameter4'
                                                    * '<S77>/V_Parameter5'
                                                    */

/* Constant of no danger */
extern const volatile uint8_T LDPSC_PrjSpecQu_C_St;/* Referenced by:
                                                    * '<S79>/V_Parameter'
                                                    * '<S80>/V_Parameter'
                                                    */
extern const volatile real32_T LDPSC_SafetyFuncMaxTime_sec;/* Referenced by:
                                                            * '<S111>/V_Parameter1'
                                                            * '<S112>/V_Parameter1'
                                                            */

/* safety function active or error maximum */
extern const volatile uint8_T LDPSC_SidCdtnCclLf_C_St;
                                      /* Referenced by: '<S39>/V_Parameter57' */

/* Cancel constant of left side condition */
extern const volatile uint8_T LDPSC_SidCdtnCclRi_C_St;
                                      /* Referenced by: '<S39>/V_Parameter60' */

/* Cancel constant of right side condition */
extern const volatile uint8_T LDPSC_StrgRdyDrvActCtrl_C_St;
                                      /* Referenced by: '<S37>/V_Parameter16' */

/* Strong ready state of active control */
extern const volatile uint8_T LDPSC_StrgRdyDrvIVld_C_St;
                                      /* Referenced by: '<S37>/V_Parameter15' */

/* Strong ready state of invalid driver */
extern const volatile uint8_T LDPSC_StrgRdyErrSpcLDP_C_St;
                                      /* Referenced by: '<S37>/V_Parameter21' */

/* Strong ready state of error specific */
extern const volatile uint8_T LDPSC_StrgRdyFctCstm_C_St;
                                      /* Referenced by: '<S37>/V_Parameter20' */

/* Strong ready state of customer specific */
extern const volatile uint8_T LDPSC_StrgRdyNoAvlbVehSys_C_St;
                                      /* Referenced by: '<S37>/V_Parameter19' */

/* Strong ready state of no availible vehicle system signals */
extern const volatile uint16_T LDPSC_StrgRdyVehIvld_C_St;
                                      /* Referenced by: '<S37>/V_Parameter22' */

/* Strong ready state of invalid vehicle */
extern const volatile uint8_T LDPSC_StrgRdyVehSysErr_C_St;
                                      /* Referenced by: '<S37>/V_Parameter18' */

/* Strong ready state of vehicle system errors */
extern const volatile uint8_T LDPSC_SuppDrvActCtrl_C_St;
                                      /* Referenced by: '<S37>/V_Parameter24' */

/* Suppresion state of active control */
extern const volatile uint8_T LDPSC_SuppDrvIvld_C_St;
                                      /* Referenced by: '<S37>/V_Parameter23' */

/* Suppresion state of invalid driver */
extern const volatile uint8_T LDPSC_SuppErrSpcLDP_C_St;
                                      /* Referenced by: '<S37>/V_Parameter28' */

/* Suppresion state of error specific */
extern const volatile uint8_T LDPSC_SuppFctCstm_C_St;
                                      /* Referenced by: '<S37>/V_Parameter27' */

/* Suppresion state of customer specific */
extern const volatile uint8_T LDPSC_SuppNoAvlbVehSys_C_St;
                                      /* Referenced by: '<S37>/V_Parameter26' */

/* Suppresion state of no availible vehicle system signals */
extern const volatile uint16_T LDPSC_SuppVehIvld_C_St;
                                      /* Referenced by: '<S37>/V_Parameter29' */

/* Suppresion state of invalid vehicle */
extern const volatile uint8_T LDPSC_SuppVehSysErr_C_St;
                                      /* Referenced by: '<S37>/V_Parameter25' */

/* Suppresion state of vehicle system errors */
extern const volatile uint8_T LDPSC_SuppVehicleInvalid_C_St;
                                     /* Referenced by: '<S113>/V_Parameter28' */

/* Suppresion state of vehicle */
extern const volatile boolean_T LDPSC_Switch_C_B;
                                      /* Referenced by: '<S25>/V_Parameter11' */

/* Value of LDP disable switch */
extern const volatile real32_T LDPSC_TgtTrajPstnY_C_Mi;/* Referenced by:
                                                        * '<S33>/V_Parameter18'
                                                        * '<S33>/V_Parameter20'
                                                        * '<S33>/V_Parameter33'
                                                        * '<S33>/V_Parameter35'
                                                        * '<S38>/V_Parameter50'
                                                        * '<S38>/V_Parameter52'
                                                        */

/* Target trajectory lateral position  */
extern const volatile real32_T LDPSC_TiAbtDegr_C_Sec;
                                      /* Referenced by: '<S32>/V_Parameter63' */

/* Degradation time of abort state */
extern const volatile real32_T LDPSC_TiCclDegr_C_Sec;/* Referenced by: '<S32>/V_Parameter3' */

/* Degradation time of cancel state */
extern const volatile real32_T LDPSC_TiDgrFnsDegr_C_Sec;/* Referenced by: '<S32>/V_Parameter2' */

/* Degradation time of finish state */
extern const volatile real32_T LDPSC_TiStrgRdyDegr_C_Sec;/* Referenced by: '<S32>/V_Parameter1' */

/* Degradation time of no strong ready state */
extern const volatile real32_T LDPSC_TiToLnTrsdSpd_Cr_Sec[9];
                                  /* Referenced by: '<S78>/1-D Lookup Table6' */

/* Map table of time to lane crossing threshold */
extern const volatile real32_T LDPSC_TiToLnTrsdWdh_Cr_Sec[5];
                                  /* Referenced by: '<S78>/1-D Lookup Table7' */

/* Map table of time to lane crossing threshold */
extern const volatile uint8_T LDPSC_TrigCdtnEn_C_St;/* Referenced by:
                                                     * '<S79>/V_Parameter11'
                                                     * '<S79>/V_Parameter9'
                                                     * '<S80>/V_Parameter11'
                                                     * '<S80>/V_Parameter9'
                                                     */

/* Switch state of choose threshold  */
extern const volatile real32_T LDPSC_VehSpdXDTL_BX_Mps[9];/* Referenced by:
                                                           * '<S78>/1-D Lookup Table'
                                                           * '<S78>/1-D Lookup Table1'
                                                           * '<S78>/1-D Lookup Table2'
                                                           */

/* DTL breakpoint of vehicle speed */
extern const volatile real32_T LDPSC_VehSpdXTTL_BX_Mps[9];
                                  /* Referenced by: '<S78>/1-D Lookup Table6' */

/* TTL breakpoint of vehicle speed */
extern const volatile real32_T LDPSC_VehYawRateHyst_C_rps;/* Referenced by: '<S115>/Constant10' */

/* Vehicle yaw rate hysteresis */
extern const volatile real32_T LDPSC_VehYawRateMax_C_rps;/* Referenced by: '<S115>/Constant9' */

/* Vehicle yaw rate maximum */
extern const volatile real32_T LDPSC_WarmMxTi_C_Sec;/* Referenced by:
                                                     * '<S79>/V_Parameter2'
                                                     * '<S80>/V_Parameter1'
                                                     * '<S40>/V_Parameter1'
                                                     */

/* Maximum time of warming state */
extern const volatile uint8_T LDPSC_WkRdyDrvActCtrl_C_St;
                                      /* Referenced by: '<S37>/V_Parameter31' */

/* Weak ready state of active control */
extern const volatile uint8_T LDPSC_WkRdyDrvIVld_C_St;
                                      /* Referenced by: '<S37>/V_Parameter30' */

/* Weak ready state of invalid driver */
extern const volatile uint8_T LDPSC_WkRdyErrSpcLDP_C_St;
                                      /* Referenced by: '<S37>/V_Parameter35' */

/* Weak ready state of error specific */
extern const volatile uint8_T LDPSC_WkRdyFctCstm_C_St;
                                      /* Referenced by: '<S37>/V_Parameter34' */

/* Weak ready state of customer specific */
extern const volatile uint8_T LDPSC_WkRdyNoAvlbVehSys_C_St;
                                      /* Referenced by: '<S37>/V_Parameter33' */

/* Weak ready state of no availible vehicle system signals */
extern const volatile uint16_T LDPSC_WkRdyVehIvld_C_St;
                                      /* Referenced by: '<S37>/V_Parameter36' */

/* Weak ready state of invalid vehicle */
extern const volatile uint8_T LDPSC_WkRdyVehSysErr_C_St;
                                      /* Referenced by: '<S37>/V_Parameter32' */

/* Weak ready state of vehicle system errors */
extern const volatile uint8_T LDPTT_CurvInner_C_St;/* Referenced by:
                                                    * '<S121>/V_Parameter12'
                                                    * '<S121>/V_Parameter2'
                                                    */

/* Constant of inner curve */
extern const volatile uint8_T LDPTT_CurvOuter_C_St;/* Referenced by:
                                                    * '<S121>/V_Parameter13'
                                                    * '<S121>/V_Parameter3'
                                                    */

/* Constant of Outer curve */
extern const volatile uint8_T LDPTT_DgrSideLf_C_St;/* Referenced by:
                                                    * '<S121>/V_Parameter'
                                                    * '<S128>/V_Parameter'
                                                    */

/* Constant of left side danger */
extern const volatile uint8_T LDPTT_DgrSideRi_C_St;
                                      /* Referenced by: '<S121>/V_Parameter1' */

/* Constant of left side danger */
extern const volatile boolean_T LDPTT_EnLwFilt_C_B;
                                      /* Referenced by: '<S120>/V_Parameter2' */

/* Enable flag for Low-pass filter of target track lane parameters */
extern const volatile real32_T LDPTT_LnBdryCurvLf_BX_ReMi[6];/* Referenced by:
                                                              * '<S121>/Lookup Table'
                                                              * '<S121>/Lookup Table2'
                                                              * '<S121>/Lookup Table3'
                                                              */

/* X-axis for Filtered left lane clothoid curvature */
extern const volatile real32_T LDPTT_LnBdryCurvRi_BX_ReMi[6];
                                     /* Referenced by: '<S121>/Lookup Table5' */

/* X-axis for Filtered left lane clothoid curvature */
extern const volatile real32_T LDPTT_LnWidCalc_BX_Mi[6];/* Referenced by:
                                                         * '<S121>/Lookup Table1'
                                                         * '<S121>/Lookup Table4'
                                                         */

/* Calculation ego lane width */
extern const volatile real32_T LDPTT_MxTgtLatDev_C_Mi;/* Referenced by:
                                                       * '<S121>/V_Parameter10'
                                                       * '<S121>/V_Parameter11'
                                                       * '<S121>/V_Parameter17'
                                                       * '<S121>/V_Parameter7'
                                                       */

/* Maximal allowed distance between the middle of the vehicle and the planned target. */
extern const volatile real32_T LDPTT_MxTgtLatDstc_C_Mi;/* Referenced by:
                                                        * '<S121>/V_Parameter14'
                                                        * '<S121>/V_Parameter4'
                                                        */

/* Maximal allowed distance between the hazardous lane marking and the planned target. */
extern const volatile boolean_T LDPTT_TgtCntrLnEn_C_B;/* Referenced by:
                                                       * '<S121>/V_Parameter16'
                                                       * '<S121>/V_Parameter6'
                                                       */

/* Constant of the target in the center of the lane.  */
extern const volatile real32_T LDPTT_TgtLatDistcLf_Cr_Mi[6];
                                     /* Referenced by: '<S121>/Lookup Table1' */

/* Left Target Lateral distance by lane width */
extern const volatile real32_T LDPTT_TgtLatDistcRi_Cr_Mi[6];
                                     /* Referenced by: '<S121>/Lookup Table4' */

/* Left Target Lateral distance by lane width */
extern const volatile real32_T LDPTT_TgtOfstLfIn_Cr_Mi[6];/* Referenced by:
                                                           * '<S121>/Lookup Table'
                                                           * '<S121>/Lookup Table3'
                                                           */

/* Left Target Lateral distance Offset By inner curves */
extern const volatile real32_T LDPTT_TgtOfstLfOut_Cr_Mi[6];
                                     /* Referenced by: '<S121>/Lookup Table2' */

/* Left Target Lateral distance by outer curves */
extern const volatile real32_T LDPTT_TgtOfstRiOut_Cr_Mi[6];
                                     /* Referenced by: '<S121>/Lookup Table5' */

/* Left Target Lateral distance by outer curves */
extern const volatile real32_T LDPTT_TiTpLnLw_C_Sec;/* Referenced by:
                                                     * '<S126>/V_Parameter11'
                                                     * '<S127>/V_Parameter11'
                                                     * '<S128>/V_Parameter11'
                                                     */

/* Low-pass filter time of target track lane parameters */
extern const volatile real32_T LDPTV_AccXObst_C_Mps2;
                                     /* Referenced by: '<S151>/V_Parameter14' */

/* X-axis acceleration of obstacle */
extern const volatile uint8_T LDPTV_CurvInner_C_St;/* Referenced by:
                                                    * '<S157>/V_Parameter3'
                                                    * '<S157>/V_Parameter9'
                                                    */

/* Constant of inner curve */
extern const volatile uint8_T LDPTV_CurvOuter_C_St;/* Referenced by:
                                                    * '<S157>/V_Parameter10'
                                                    * '<S157>/V_Parameter5'
                                                    */

/* Constant of Outer curve */
extern const volatile real32_T LDPTV_CurvScalInner_Cr_Fct[6];
                                 /* Referenced by: '<S150>/1-D Lookup Table5' */

/* Planning Horizon Scaling Factor for the calculation in the TRJPLN Curvature dependant for an inner curve. */
extern const volatile real32_T LDPTV_CurvScalOuter_Cr_Fct[6];
                                 /* Referenced by: '<S150>/1-D Lookup Table4' */

/* Planning Horizon Scaling Factor for the calculation in the TRJPLN Curvature dependant for an outer curve. */
extern const volatile real32_T LDPTV_Curv_BX_ReMi[6];/* Referenced by:
                                                      * '<S150>/1-D Lookup Table4'
                                                      * '<S150>/1-D Lookup Table5'
                                                      */

/* Filtered center lane clothoid curvature for the LDP */
extern const volatile real32_T LDPTV_D2TPlanHorizonScal_Fct[6];
                                 /* Referenced by: '<S150>/1-D Lookup Table2' */

/* Filtered center lane clothoid Y0 position for the LDP */
extern const volatile uint8_T LDPTV_DgrSideLf_C_St;/* Referenced by:
                                                    * '<S152>/V_Parameter2'
                                                    * '<S157>/V_Parameter2'
                                                    * '<S157>/V_Parameter4'
                                                    */

/* Constant of left side danger */
extern const volatile uint8_T LDPTV_DgrSideRi_C_St;/* Referenced by:
                                                    * '<S152>/V_Parameter7'
                                                    * '<S157>/V_Parameter7'
                                                    * '<S157>/V_Parameter8'
                                                    */

/* Constant of left side danger */
extern const volatile real32_T LDPTV_DstcXObst_C_Mi;
                                     /* Referenced by: '<S151>/V_Parameter16' */

/* X-axis distance of obstacle */
extern const volatile real32_T LDPTV_DstcYObst_C_Mi;
                                     /* Referenced by: '<S151>/V_Parameter17' */

/* Y-axis distance of obstacle */
extern const volatile real32_T LDPTV_DstcYTgtAreaLf_C_Mi;
                                      /* Referenced by: '<S151>/V_Parameter2' */

/* Y-axis distance of host vehicle to left target area */
extern const volatile real32_T LDPTV_DstcYTgtAreaRi_C_Mi;
                                      /* Referenced by: '<S151>/V_Parameter3' */

/* Y-axis distance of host vehicle to right target area */
extern const volatile real32_T LDPTV_FTireAccMn_C_Mps2;
                                      /* Referenced by: '<S151>/V_Parameter1' */
extern const volatile real32_T LDPTV_FTireAccMx_C_Mps2;
                                     /* Referenced by: '<S151>/V_Parameter23' */
extern const volatile real32_T LDPTV_FctTgtDistY_C_Fct;
                                      /* Referenced by: '<S151>/V_Parameter4' */

/* Y-axis distance of obstacle */
extern const volatile boolean_T LDPTV_HiStatAcc_C_B;
                                      /* Referenced by: '<S148>/V_Parameter5' */

/* Switch for a high stationary accuracy in the LaDMC */
extern const volatile real32_T LDPTV_JerkLmtMx_C_Mps3;
                                     /* Referenced by: '<S151>/V_Parameter12' */

/* Maximum Jerk Allowed in the trajectory planning */
extern const volatile boolean_T LDPTV_LatCpstnEn_C_B;
                                      /* Referenced by: '<S151>/V_Parameter9' */

/* Switch for the latency compensation in trajectory plan */
extern const volatile real32_T LDPTV_LatVel_BX_Mps[6];/* Referenced by:
                                                       * '<S148>/1-D Lookup Table'
                                                       * '<S148>/1-D Lookup Table1'
                                                       * '<S148>/1-D Lookup Table2'
                                                       * '<S148>/1-D Lookup Table3'
                                                       * '<S150>/1-D Lookup Table1'
                                                       */

/* Lateral Velocity */
extern const volatile real32_T LDPTV_LmtCurvGradCtrlMx_C_ReMps;
                                     /* Referenced by: '<S151>/V_Parameter21' */

/* Maximum limiting gradient of curvature */
extern const volatile real32_T LDPTV_LmtCurvGradDecMx_C_ReMps;
                                     /* Referenced by: '<S151>/V_Parameter20' */

/* Maximum limiting curvature */
extern const volatile real32_T LDPTV_LmtCurvGradIncMx_C_ReMps;
                                     /* Referenced by: '<S151>/V_Parameter19' */

/* Maximum limiting gradient of curvature */
extern const volatile real32_T LDPTV_LmtCurvMx_C_ReMi;
                                      /* Referenced by: '<S151>/V_Parameter6' */

/* Maximum limiting curvature */
extern const volatile boolean_T LDPTV_LmtEn_C_B;
                                     /* Referenced by: '<S151>/V_Parameter10' */

/* Switch to limit the target curvature in trajectory controller */
extern const volatile real32_T LDPTV_LnBdryPstnYCent_Bx_Mi[6];
                                 /* Referenced by: '<S150>/1-D Lookup Table2' */

/* Filtered center lane clothoid Y0 position for the LDP */
extern const volatile real32_T LDPTV_MxTrqScalGradLmt_C_Fct;
                                      /* Referenced by: '<S148>/V_Parameter6' */

/* Maximum Torque Scaling ramp out gradien */
extern const volatile real32_T LDPTV_MxTrqScalInGrad_C_Res;
                                      /* Referenced by: '<S148>/V_Parameter7' */

/* Maximum Torque Scaling ramp in gradien */
extern const volatile real32_T LDPTV_MxTrqScalOutGrad_C_Res;
                                      /* Referenced by: '<S148>/V_Parameter2' */

/* Maximum Torque Scaling ramp out gradien */
extern const volatile real32_T LDPTV_PredTiAgl_C_Sec;
                                      /* Referenced by: '<S151>/V_Parameter8' */
extern const volatile real32_T LDPTV_PredTiCurv_C_Sec;
                                      /* Referenced by: '<S151>/V_Parameter7' */
extern const volatile uint8_T LDPTV_ReqFreeze_C_ST;
                                      /* Referenced by: '<S149>/V_Parameter3' */

/* Request freeze of LDP state */
extern const volatile uint8_T LDPTV_ReqOff_C_ST;
                                      /* Referenced by: '<S149>/V_Parameter5' */

/* Request off of LDP state */
extern const volatile uint8_T LDPTV_ReqOn_C_ST;
                                      /* Referenced by: '<S149>/V_Parameter4' */

/* Request on of LDP state */
extern const volatile real32_T LDPTV_SteWhlGradAbort_C_ReS;
                                      /* Referenced by: '<S148>/V_Parameter9' */

/* Steering Wheel Stiffness Abort Ramp Out Gradient */
extern const volatile real32_T LDPTV_SteWhlGradLmt_C_Fct;
                                      /* Referenced by: '<S148>/V_Parameter4' */

/* Steering Wheel Stiffness Limiter */
extern const volatile real32_T LDPTV_SteWhlGrad_C_ReS;
                                      /* Referenced by: '<S148>/V_Parameter8' */

/* Steering Wheel Stiffness Standard Ramp Out */
extern const volatile real32_T LDPTV_TiLmtEnDura_C_Sec;
                                     /* Referenced by: '<S151>/V_Parameter11' */

/* Switch to limit the target curvature in trajectory controller */
extern const volatile real32_T LDPTV_TrajPlanServQu_C_Fct;
                                      /* Referenced by: '<S151>/V_Parameter5' */
extern const volatile boolean_T LDPTV_TrigReplan_C_B;
                                     /* Referenced by: '<S151>/V_Parameter13' */

/* It has to be 1 for trajectory planning to calculate a trajectory */
extern const volatile real32_T LDPTV_TrqRampGradIn_C_ReS;
                                     /* Referenced by: '<S148>/V_Parameter12' */

/* Torque Standard Ramp In Gradient */
extern const volatile real32_T LDPTV_TrqRampOutGradAbort_C_ReS;
                                     /* Referenced by: '<S148>/V_Parameter11' */

/* Torque Ramp Out Abort Gradient */
extern const volatile real32_T LDPTV_TrqRampOutGrad_C_ReS;
                                     /* Referenced by: '<S148>/V_Parameter10' */

/* Torque Standard Ramp Out Gradient */
extern const volatile real32_T LDPTV_VXPlanHorizonScal_Cr_Fct[8];
                                  /* Referenced by: '<S150>/1-D Lookup Table' */

/* Planning Horizon Scaling Factor for the calculation in the TRJPLN.VX dependant. */
extern const volatile real32_T LDPTV_VYMD1DeratingLevel_Fct[6];
                                 /* Referenced by: '<S148>/1-D Lookup Table1' */

/* DMC Derating Level of the LDP function when Driving Mode is equal to 1. Lateral Velocity dependant. */
extern const volatile real32_T LDPTV_VYMD2DeratingLevel_Fct[6];
                                 /* Referenced by: '<S148>/1-D Lookup Table2' */

/* DMC Derating Level of the LDP function when Driving Mode is equal to 2. Lateral Velocity dependant. */
extern const volatile real32_T LDPTV_VYMD3DeratingLevel_Fct[6];
                                 /* Referenced by: '<S148>/1-D Lookup Table3' */

/* DMC Derating Level of the LDP function when Driving Mode is equal to 3. Lateral Velocity dependant. */
extern const volatile real32_T LDPTV_VYPlanningHorizon_Sec[6];
                                 /* Referenced by: '<S150>/1-D Lookup Table1' */

/* Filtered center lane clothoid Y0 position for the LDP */
extern const volatile real32_T LDPTV_VYStrWhStifRIGrad_Cr_Res[6];
                                  /* Referenced by: '<S148>/1-D Lookup Table' */

/* Standard Steering Wheel Stiffness Ramp In Gradient. Vy dependant. */
extern const volatile real32_T LDPTV_VehVelX_BX_Mps[8];/* Referenced by:
                                                        * '<S150>/1-D Lookup Table'
                                                        * '<S150>/1-D Lookup Table3'
                                                        */

/* Vehicle Speed which is computed based on the wheel speeds */
extern const volatile real32_T LDPTV_VeloXObst_C_Mps;
                                     /* Referenced by: '<S151>/V_Parameter18' */

/* X-axis velocity of obstacle */
extern const volatile real32_T LDPTV_WheightEndTi_Cr_Fct[8];
                                 /* Referenced by: '<S150>/1-D Lookup Table3' */

/* Weight of the end time for the calculation in the TRJPLN. Speed dependant */
extern const volatile real32_T LDPTV_WidObst_C_Mi;
                                     /* Referenced by: '<S151>/V_Parameter15' */

/* Width of obstacle */
extern const volatile real32_T LDPVSE_HodTiTrnSgl_C_Sec;/* Referenced by:
                                                         * '<S161>/V_Parameter3'
                                                         * '<S161>/V_Parameter7'
                                                         */

/* Value of turn signal holding time */
extern const volatile real32_T LDPVSE_LnWidTrsdMn_C_Mi;
                                      /* Referenced by: '<S160>/V_Parameter8' */

/* Minimum threshold of lane width */
extern const volatile real32_T LDPVSE_LnWidTrsdMx_C_Mi;/* Referenced by:
                                                        * '<S160>/V_Parameter20'
                                                        * '<S160>/V_Parameter7'
                                                        */

/* Maximum threshold of lane width */
extern const volatile real32_T LDPVSE_LnWidTrsdOfst_C_Mi;/* Referenced by:
                                                          * '<S160>/V_Parameter21'
                                                          * '<S160>/V_Parameter9'
                                                          */

/* Offset of lane width */
extern const volatile uint8_T LDPVSE_NoDgrSide_C_St;/* Referenced by: '<S162>/V_Parameter' */

/* State value of no danger of lane */
extern const volatile real32_T LDPVSE_SteAglSpdTrsdMx_C_Dgpm;
                                      /* Referenced by: '<S160>/V_Parameter5' */

/* Maximum threshold of steering wheel angle speed */
extern const volatile real32_T LDPVSE_SteAglSpdTrsdOfst_C_Dgpm;
                                      /* Referenced by: '<S160>/V_Parameter6' */

/* Offset of steering wheel angle speed */
extern const volatile real32_T LDPVSE_SteAglTrsdMx_C_Dgr;
                                      /* Referenced by: '<S160>/V_Parameter3' */

/* Maximum threshold of steering wheel angle */
extern const volatile real32_T LDPVSE_SteAglTrsdOfst_C_Dgr;
                                      /* Referenced by: '<S160>/V_Parameter4' */

/* Offset of steering wheel angle */
extern const volatile uint8_T LDPVSE_TrnSglLf_C_St;/* Referenced by:
                                                    * '<S161>/V_Parameter1'
                                                    * '<S161>/V_Parameter4'
                                                    */

/* State value of left turn signal  */
extern const volatile uint8_T LDPVSE_TrnSglRi_C_St;/* Referenced by:
                                                    * '<S161>/V_Parameter'
                                                    * '<S161>/V_Parameter5'
                                                    */

/* State value of right turn signal  */
extern const volatile boolean_T LDPVSE_TrnSglRstLfEn_C_B;
                                      /* Referenced by: '<S161>/V_Parameter2' */

/* Enable signal of left turn reset */
extern const volatile boolean_T LDPVSE_TrnSglRstRiEn_C_B;
                                      /* Referenced by: '<S161>/V_Parameter6' */

/* Enable signal of right turn reset */
extern const volatile real32_T LDPVSE_TrsdLnCltdCurvMx_Cr_Mps[8];
                                      /* Referenced by: '<S160>/Lookup Table' */

/* Curve of maximum threshold of clothiod curvature  */
extern const volatile real32_T LDPVSE_TrsdLnCltdCurvOfst_Cr_Mps[8];
                                     /* Referenced by: '<S160>/Lookup Table1' */

/* Curve of offset threshold of clothiod curvature  */
extern const volatile real32_T LDPVSE_VehAccSpdTrsdXMn_C_Npkg;
                                     /* Referenced by: '<S160>/V_Parameter11' */

/* Minimum threshold of longitudinal Acceleration */
extern const volatile real32_T LDPVSE_VehAccSpdTrsdXMx_C_Npkg;
                                     /* Referenced by: '<S160>/V_Parameter10' */

/* Maximum threshold of longitudinal Acceleration */
extern const volatile real32_T LDPVSE_VehAccSpdTrsdXOfst_C_Npkg;
                                     /* Referenced by: '<S160>/V_Parameter12' */

/* Offset of longitudinal Acceleration */
extern const volatile real32_T LDPVSE_VehAccSpdTrsdYMx_C_Npkg;
                                     /* Referenced by: '<S160>/V_Parameter13' */

/* Maximum threshold of lateral Acceleration */
extern const volatile real32_T LDPVSE_VehAccSpdTrsdYOfst_C_Npkg;
                                     /* Referenced by: '<S160>/V_Parameter14' */

/* Offset of lateral Acceleration */
extern const volatile real32_T LDPVSE_VehLatTrsdLDPMn_C_Msp;/* Referenced by:
                                                             * '<S177>/V_Parameter4'
                                                             * '<S180>/V_Parameter10'
                                                             */

/* Minimum threshold of Lateral vehicle speed */
extern const volatile real32_T LDPVSE_VehLatTrsdLDPMx_C_Msp;/* Referenced by:
                                                             * '<S177>/V_Parameter1'
                                                             * '<S180>/V_Parameter7'
                                                             */

/* Maximum threshold of Lateral vehicle speed */
extern const volatile real32_T LDPVSE_VehLatTrsdLDPOfst_C_Msp;/* Referenced by:
                                                               * '<S177>/V_Parameter2'
                                                               * '<S177>/V_Parameter5'
                                                               * '<S180>/V_Parameter11'
                                                               * '<S180>/V_Parameter8'
                                                               */

/* Offset of Lateral vehicle speed */
extern const volatile real32_T LDPVSE_VehLatTrsd_Cr_Msp[8];
                                      /* Referenced by: '<S162>/Lookup Table' */

/* Curve of maximum threshold of lateral velocity */
extern const volatile real32_T LDPVSE_VehSpdTrsdMn_C_Kmph;
                                      /* Referenced by: '<S171>/V_Parameter1' */

/* Minimum threshold of displayed longitudinal speed */
extern const volatile real32_T LDPVSE_VehSpdTrsdMx_C_Kmph;/* Referenced by: '<S160>/V_Parameter' */

/* Maximum threshold of displayed longitudinal speed */
extern const volatile real32_T LDPVSE_VehSpdTrsdOfst_C_Kmph;
                                      /* Referenced by: '<S160>/V_Parameter2' */

/* Offset of displayed longitudinal speed */
extern const volatile real32_T LDPVSE_VehSpdX_BX_Mps[8];/* Referenced by:
                                                         * '<S160>/Lookup Table'
                                                         * '<S160>/Lookup Table1'
                                                         * '<S162>/Lookup Table'
                                                         */

/* Breakpoint of vehicle speed  */

/* Declaration for custom storage class: Global */
extern boolean_T LDPDT_LnLengthLf_B;   /* '<S17>/Unit Delay' */
extern boolean_T LDPDT_LnLengthRi_B;   /* '<S21>/Unit Delay' */
extern real32_T LDPSC_ActiveStopWatch_sec;/* '<S54>/Unit Delay' */
extern boolean_T LDPSC_EdgeRisActive_B;/* '<S46>/Unit Delay' */
extern boolean_T LDPSC_PrevStandbyUnitDelay_bool;/* '<S28>/Unit Delay' */

/* Previous standby status */
extern boolean_T LDPSC_PrevSwitchUnitDelay_bool;/* '<S25>/Unit Delay' */

/* Previous standby status */
extern boolean_T LDPSC_RampTimeExpiredRSFF_bool;/* '<S29>/Unit Delay' */

/* Rampout time expired */
extern real32_T LDPSC_SafeFuncActiveTurnOnDelay_sec;/* '<S117>/Unit Delay' */
extern real32_T LDPSC_SafeFuncErrorTurnOnDelay_sec;/* '<S118>/Unit Delay' */
extern real32_T LDPSC_SusTimeExpiredTimerRetrigger_sec;/* '<S30>/Unit Delay' */
extern boolean_T LDPSC_VehYawRateHyst_bool;/* '<S119>/Unit Delay' */

/* Yawrate hysteresis--Used in LDPSC module */
extern real32_T LDPTT_LstLnBdryCurvRateRi_ReMi2;/* '<S127>/UnitDelay4' */

/* Last filtered right lane clothoid change of curvature from LDP */
extern real32_T LDPTT_LstLnBdryCurvRi_ReMi;/* '<S127>/UnitDelay3' */

/* Last filtered right lane clothoid curvature from LDP */
extern real32_T LDPTT_LstLnBdryHeadAglRi_Rad;/* '<S127>/UnitDelay2' */

/* Last filtered right lane clothoid heading angle from LDP */
extern real32_T LDPTT_LstLnBdryPstnXRi_Mi;/* '<S127>/UnitDelay' */

/* Last filtered right lane clothoid X0 position from LDP */
extern real32_T LDPTT_LstLnBdryPstnYRi_Mi;/* '<S127>/UnitDelay1' */

/* Last filtered right lane clothoid Y0 position from LDP */
extern real32_T LDPTT_LstLnBdryVldLengRi_Mi;/* '<S127>/UnitDelay5' */

/* Last filtered right lane clothoid length from LDP  */
extern uint8_T LDPVSE_PrevVehStartupSpd_Kmph;/* '<S171>/Unit Delay' */

/* Yawrate hysteresis--Used in LDPSC module */

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S4>/Signal Conversion4' : Eliminate redundant signal conversion block
 * Block '<S32>/Signal Conversion' : Eliminate redundant signal conversion block
 * Block '<S32>/Signal Conversion1' : Eliminate redundant signal conversion block
 * Block '<S79>/Signal Conversion1' : Eliminate redundant signal conversion block
 * Block '<S79>/Signal Conversion2' : Eliminate redundant signal conversion block
 * Block '<S79>/Signal Conversion3' : Eliminate redundant signal conversion block
 * Block '<S79>/Signal Conversion4' : Eliminate redundant signal conversion block
 * Block '<S79>/Signal Conversion5' : Eliminate redundant signal conversion block
 * Block '<S80>/Signal Conversion1' : Eliminate redundant signal conversion block
 * Block '<S80>/Signal Conversion2' : Eliminate redundant signal conversion block
 * Block '<S80>/Signal Conversion3' : Eliminate redundant signal conversion block
 * Block '<S80>/Signal Conversion4' : Eliminate redundant signal conversion block
 * Block '<S80>/Signal Conversion5' : Eliminate redundant signal conversion block
 * Block '<S161>/Signal Conversion1' : Eliminate redundant signal conversion block
 * Block '<S161>/Signal Conversion3' : Eliminate redundant signal conversion block
 * Block '<S28>/Constant1' : Unused code path elimination
 * Block '<S40>/Constant1' : Unused code path elimination
 * Block '<S37>/Constant29' : Unused code path elimination
 * Block '<S37>/Constant31' : Unused code path elimination
 * Block '<S32>/Constant' : Unused code path elimination
 * Block '<S59>/Constant3' : Unused code path elimination
 * Block '<S33>/Constant' : Unused code path elimination
 * Block '<S126>/V_Const10' : Unused code path elimination
 * Block '<S126>/V_Const12' : Unused code path elimination
 * Block '<S126>/V_Const6' : Unused code path elimination
 * Block '<S126>/V_Const7' : Unused code path elimination
 * Block '<S126>/V_Const8' : Unused code path elimination
 * Block '<S126>/V_Const9' : Unused code path elimination
 * Block '<S127>/V_Const10' : Unused code path elimination
 * Block '<S127>/V_Const12' : Unused code path elimination
 * Block '<S127>/V_Const6' : Unused code path elimination
 * Block '<S127>/V_Const7' : Unused code path elimination
 * Block '<S127>/V_Const8' : Unused code path elimination
 * Block '<S127>/V_Const9' : Unused code path elimination
 * Block '<S128>/V_Const10' : Unused code path elimination
 * Block '<S128>/V_Const12' : Unused code path elimination
 * Block '<S128>/V_Const6' : Unused code path elimination
 * Block '<S128>/V_Const7' : Unused code path elimination
 * Block '<S128>/V_Const8' : Unused code path elimination
 * Block '<S128>/V_Const9' : Unused code path elimination
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
 * hilite_system('LDPSA_model/LDPSA')    - opens subsystem LDPSA_model/LDPSA
 * hilite_system('LDPSA_model/LDPSA/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'LDPSA_model'
 * '<S1>'   : 'LDPSA_model/LDPSA'
 * '<S2>'   : 'LDPSA_model/LDPSA/LDP'
 * '<S3>'   : 'LDPSA_model/LDPSA/LDP/LDPDT'
 * '<S4>'   : 'LDPSA_model/LDPSA/LDP/LDPSC'
 * '<S5>'   : 'LDPSA_model/LDPSA/LDP/LDPTT'
 * '<S6>'   : 'LDPSA_model/LDPSA/LDP/LDPTV'
 * '<S7>'   : 'LDPSA_model/LDPSA/LDP/LDPVSE'
 * '<S8>'   : 'LDPSA_model/LDPSA/LDP/LDPDT/JudgeSignal'
 * '<S9>'   : 'LDPSA_model/LDPSA/LDP/LDPDT/LnMarkVld'
 * '<S10>'  : 'LDPSA_model/LDPSA/LDP/LDPDT/TLC_DLC_Calc'
 * '<S11>'  : 'LDPSA_model/LDPSA/LDP/LDPDT/JudgeSignal/LaneCurveType'
 * '<S12>'  : 'LDPSA_model/LDPSA/LDP/LDPDT/LnMarkVld/LnMarkLf'
 * '<S13>'  : 'LDPSA_model/LDPSA/LDP/LDPDT/LnMarkVld/LnMarkRi'
 * '<S14>'  : 'LDPSA_model/LDPSA/LDP/LDPDT/LnMarkVld/LnMarkLf/Bilateral hysteresis'
 * '<S15>'  : 'LDPSA_model/LDPSA/LDP/LDPDT/LnMarkVld/LnMarkLf/Unilateral hysteresis '
 * '<S16>'  : 'LDPSA_model/LDPSA/LDP/LDPDT/LnMarkVld/LnMarkLf/Unilateral hysteresis 1'
 * '<S17>'  : 'LDPSA_model/LDPSA/LDP/LDPDT/LnMarkVld/LnMarkLf/Unilateral hysteresis 2'
 * '<S18>'  : 'LDPSA_model/LDPSA/LDP/LDPDT/LnMarkVld/LnMarkRi/Bilateral hysteresis'
 * '<S19>'  : 'LDPSA_model/LDPSA/LDP/LDPDT/LnMarkVld/LnMarkRi/Unilateral hysteresis '
 * '<S20>'  : 'LDPSA_model/LDPSA/LDP/LDPDT/LnMarkVld/LnMarkRi/Unilateral hysteresis 1'
 * '<S21>'  : 'LDPSA_model/LDPSA/LDP/LDPDT/LnMarkVld/LnMarkRi/Unilateral hysteresis 2'
 * '<S22>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/LDP_State'
 * '<S23>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/RAMPOUTStChk'
 * '<S24>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput'
 * '<S25>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/SwitchCheck'
 * '<S26>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/RAMPOUTStChk/Enumerated Constant1'
 * '<S27>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/RAMPOUTStChk/Enumerated Constant2'
 * '<S28>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/RAMPOUTStChk/RampoutWatchDog'
 * '<S29>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/RAMPOUTStChk/RampoutWatchDog/RSFlipFlop'
 * '<S30>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/RAMPOUTStChk/RampoutWatchDog/TimerRetrigger'
 * '<S31>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Cancel&Ready'
 * '<S32>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Degradation'
 * '<S33>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrFns'
 * '<S34>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrTrig'
 * '<S35>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Suppression'
 * '<S36>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Cancel&Ready/Cancel'
 * '<S37>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Cancel&Ready/Ready'
 * '<S38>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Cancel&Ready/Cancel/LnDev'
 * '<S39>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Cancel&Ready/Cancel/SideCondition'
 * '<S40>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Cancel&Ready/Cancel/TiWarmMx'
 * '<S41>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Cancel&Ready/Cancel/TiWarmMx/ Edge1'
 * '<S42>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Cancel&Ready/Cancel/TiWarmMx/Enumerated Constant'
 * '<S43>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Cancel&Ready/Cancel/TiWarmMx/TimerRetrigger'
 * '<S44>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Cancel&Ready/Ready/ Edge'
 * '<S45>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Cancel&Ready/Ready/ Edge1'
 * '<S46>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Cancel&Ready/Ready/EdgeRising'
 * '<S47>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Cancel&Ready/Ready/EdgeRising1'
 * '<S48>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Cancel&Ready/Ready/Enumerated Constant'
 * '<S49>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Cancel&Ready/Ready/Enumerated Constant1'
 * '<S50>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Cancel&Ready/Ready/Enumerated Constant2'
 * '<S51>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Cancel&Ready/Ready/Enumerated Constant3'
 * '<S52>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Cancel&Ready/Ready/Enumerated Constant4'
 * '<S53>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Cancel&Ready/Ready/Enumerated Constant5'
 * '<S54>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Cancel&Ready/Ready/Stopwatch'
 * '<S55>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Cancel&Ready/Ready/Stopwatch1'
 * '<S56>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Cancel&Ready/Ready/TimerRetrigger'
 * '<S57>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Cancel&Ready/Ready/TimerRetrigger1'
 * '<S58>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Degradation/ Edge'
 * '<S59>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Degradation/FollowUpTimer'
 * '<S60>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrFns/ Edge1'
 * '<S61>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrFns/Bit Get3'
 * '<S62>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrFns/Bit Get4'
 * '<S63>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrFns/Bit Get5'
 * '<S64>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrFns/Bit Get6'
 * '<S65>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrFns/Bit Get7'
 * '<S66>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrFns/Bit Get8'
 * '<S67>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrFns/Bit Get9'
 * '<S68>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrFns/Enumerated Constant'
 * '<S69>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrFns/RiseDelay2'
 * '<S70>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrFns/Saturation'
 * '<S71>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrFns/Saturation1'
 * '<S72>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrFns/Saturation2'
 * '<S73>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrFns/Saturation3'
 * '<S74>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrFns/Saturation4'
 * '<S75>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrFns/Saturation5'
 * '<S76>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrFns/TimerRetrigger'
 * '<S77>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrTrig/DangerSide'
 * '<S78>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrTrig/Threshold'
 * '<S79>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrTrig/TrgrLf'
 * '<S80>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrTrig/TrgrRi'
 * '<S81>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrTrig/DangerSide/Enumerated Constant'
 * '<S82>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrTrig/DangerSide/Enumerated Constant1'
 * '<S83>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrTrig/TrgrLf/ Edge'
 * '<S84>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrTrig/TrgrLf/Bit Get'
 * '<S85>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrTrig/TrgrLf/Bit Get1'
 * '<S86>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrTrig/TrgrLf/EdgeRising'
 * '<S87>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrTrig/TrgrLf/EdgeRising1'
 * '<S88>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrTrig/TrgrLf/Enumerated Constant'
 * '<S89>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrTrig/TrgrLf/Enumerated Constant2'
 * '<S90>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrTrig/TrgrLf/FF'
 * '<S91>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrTrig/TrgrLf/FF1'
 * '<S92>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrTrig/TrgrLf/FollowUpTimer'
 * '<S93>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrTrig/TrgrLf/FollowUpTimer1'
 * '<S94>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrTrig/TrgrLf/RiseDelay'
 * '<S95>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrTrig/TrgrRi/ Edge'
 * '<S96>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrTrig/TrgrRi/Bit Get'
 * '<S97>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrTrig/TrgrRi/Bit Get1'
 * '<S98>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrTrig/TrgrRi/EdgeRising'
 * '<S99>'  : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrTrig/TrgrRi/EdgeRising1'
 * '<S100>' : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrTrig/TrgrRi/Enumerated Constant'
 * '<S101>' : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrTrig/TrgrRi/Enumerated Constant2'
 * '<S102>' : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrTrig/TrgrRi/FF'
 * '<S103>' : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrTrig/TrgrRi/FF2'
 * '<S104>' : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrTrig/TrgrRi/FollowUpTimer'
 * '<S105>' : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrTrig/TrgrRi/FollowUpTimer1'
 * '<S106>' : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/DgrTrig/TrgrRi/RiseDelay'
 * '<S107>' : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Suppression/DriverStInvalid'
 * '<S108>' : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Suppression/LaneCurveInvalid'
 * '<S109>' : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Suppression/LateralVelocityInvalid'
 * '<S110>' : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Suppression/MappingUint16'
 * '<S111>' : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Suppression/SafetyFunctionActive'
 * '<S112>' : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Suppression/SafetyFunctionError'
 * '<S113>' : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Suppression/VehicleInvalid'
 * '<S114>' : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Suppression/VehicleStInvalid'
 * '<S115>' : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Suppression/YawRateInvalid'
 * '<S116>' : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Suppression/MappingUint16/Set_bit'
 * '<S117>' : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Suppression/SafetyFunctionActive/RiseDelay2'
 * '<S118>' : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Suppression/SafetyFunctionError/RiseDelay2'
 * '<S119>' : 'LDPSA_model/LDPSA/LDP/LDPSC/StateInput/Suppression/YawRateInvalid/Hysteresis3'
 * '<S120>' : 'LDPSA_model/LDPSA/LDP/LDPTT/Filter_LDP_TargetTraj'
 * '<S121>' : 'LDPSA_model/LDPSA/LDP/LDPTT/TargetLateralDistanceCalculation'
 * '<S122>' : 'LDPSA_model/LDPSA/LDP/LDPTT/Filter_LDP_TargetTraj/ Edge'
 * '<S123>' : 'LDPSA_model/LDPSA/LDP/LDPTT/Filter_LDP_TargetTraj/Enumerated Constant'
 * '<S124>' : 'LDPSA_model/LDPSA/LDP/LDPTT/Filter_LDP_TargetTraj/Enumerated Constant1'
 * '<S125>' : 'LDPSA_model/LDPSA/LDP/LDPTT/Filter_LDP_TargetTraj/Enumerated Constant2'
 * '<S126>' : 'LDPSA_model/LDPSA/LDP/LDPTT/Filter_LDP_TargetTraj/LeftCorridorBoundary'
 * '<S127>' : 'LDPSA_model/LDPSA/LDP/LDPTT/Filter_LDP_TargetTraj/RightCorridorBoundary'
 * '<S128>' : 'LDPSA_model/LDPSA/LDP/LDPTT/Filter_LDP_TargetTraj/TargetTrajectory'
 * '<S129>' : 'LDPSA_model/LDPSA/LDP/LDPTT/Filter_LDP_TargetTraj/LeftCorridorBoundary/LowPassFilter'
 * '<S130>' : 'LDPSA_model/LDPSA/LDP/LDPTT/Filter_LDP_TargetTraj/LeftCorridorBoundary/LowPassFilter1'
 * '<S131>' : 'LDPSA_model/LDPSA/LDP/LDPTT/Filter_LDP_TargetTraj/LeftCorridorBoundary/LowPassFilter2'
 * '<S132>' : 'LDPSA_model/LDPSA/LDP/LDPTT/Filter_LDP_TargetTraj/LeftCorridorBoundary/LowPassFilter3'
 * '<S133>' : 'LDPSA_model/LDPSA/LDP/LDPTT/Filter_LDP_TargetTraj/LeftCorridorBoundary/LowPassFilter4'
 * '<S134>' : 'LDPSA_model/LDPSA/LDP/LDPTT/Filter_LDP_TargetTraj/LeftCorridorBoundary/LowPassFilter5'
 * '<S135>' : 'LDPSA_model/LDPSA/LDP/LDPTT/Filter_LDP_TargetTraj/RightCorridorBoundary/LowPassFilter'
 * '<S136>' : 'LDPSA_model/LDPSA/LDP/LDPTT/Filter_LDP_TargetTraj/RightCorridorBoundary/LowPassFilter1'
 * '<S137>' : 'LDPSA_model/LDPSA/LDP/LDPTT/Filter_LDP_TargetTraj/RightCorridorBoundary/LowPassFilter2'
 * '<S138>' : 'LDPSA_model/LDPSA/LDP/LDPTT/Filter_LDP_TargetTraj/RightCorridorBoundary/LowPassFilter3'
 * '<S139>' : 'LDPSA_model/LDPSA/LDP/LDPTT/Filter_LDP_TargetTraj/RightCorridorBoundary/LowPassFilter4'
 * '<S140>' : 'LDPSA_model/LDPSA/LDP/LDPTT/Filter_LDP_TargetTraj/RightCorridorBoundary/LowPassFilter5'
 * '<S141>' : 'LDPSA_model/LDPSA/LDP/LDPTT/Filter_LDP_TargetTraj/TargetTrajectory/LowPassFilter1'
 * '<S142>' : 'LDPSA_model/LDPSA/LDP/LDPTT/Filter_LDP_TargetTraj/TargetTrajectory/LowPassFilter2'
 * '<S143>' : 'LDPSA_model/LDPSA/LDP/LDPTT/Filter_LDP_TargetTraj/TargetTrajectory/LowPassFilter3'
 * '<S144>' : 'LDPSA_model/LDPSA/LDP/LDPTT/Filter_LDP_TargetTraj/TargetTrajectory/LowPassFilter4'
 * '<S145>' : 'LDPSA_model/LDPSA/LDP/LDPTT/Filter_LDP_TargetTraj/TargetTrajectory/LowPassFilter5'
 * '<S146>' : 'LDPSA_model/LDPSA/LDP/LDPTT/Filter_LDP_TargetTraj/TargetTrajectory/LowPassFilter6'
 * '<S147>' : 'LDPSA_model/LDPSA/LDP/LDPTT/TargetLateralDistanceCalculation/Enumerated Constant'
 * '<S148>' : 'LDPSA_model/LDPSA/LDP/LDPTV/DetermineStiffnessAndStatAccu'
 * '<S149>' : 'LDPSA_model/LDPSA/LDP/LDPTV/DetermineTrajGuiQualifier'
 * '<S150>' : 'LDPSA_model/LDPSA/LDP/LDPTV/SetLookupOutputs'
 * '<S151>' : 'LDPSA_model/LDPSA/LDP/LDPTV/SetParamOutputs'
 * '<S152>' : 'LDPSA_model/LDPSA/LDP/LDPTV/VelLatCalculation'
 * '<S153>' : 'LDPSA_model/LDPSA/LDP/LDPTV/DetermineStiffnessAndStatAccu/Enumerated Constant'
 * '<S154>' : 'LDPSA_model/LDPSA/LDP/LDPTV/DetermineStiffnessAndStatAccu/Enumerated Constant1'
 * '<S155>' : 'LDPSA_model/LDPSA/LDP/LDPTV/DetermineTrajGuiQualifier/Enumerated Constant'
 * '<S156>' : 'LDPSA_model/LDPSA/LDP/LDPTV/DetermineTrajGuiQualifier/Enumerated Constant1'
 * '<S157>' : 'LDPSA_model/LDPSA/LDP/LDPTV/SetLookupOutputs/CurveType'
 * '<S158>' : 'LDPSA_model/LDPSA/LDP/LDPTV/SetLookupOutputs/Enumerated Constant'
 * '<S159>' : 'LDPSA_model/LDPSA/LDP/LDPTV/SetLookupOutputs/Enumerated Constant1'
 * '<S160>' : 'LDPSA_model/LDPSA/LDP/LDPVSE/InVldLDP'
 * '<S161>' : 'LDPSA_model/LDPSA/LDP/LDPVSE/JudgeTrn'
 * '<S162>' : 'LDPSA_model/LDPSA/LDP/LDPVSE/SidCdtnLDP'
 * '<S163>' : 'LDPSA_model/LDPSA/LDP/LDPVSE/InVldLDP/Bilateral hysteresis'
 * '<S164>' : 'LDPSA_model/LDPSA/LDP/LDPVSE/InVldLDP/Bilateral hysteresis1'
 * '<S165>' : 'LDPSA_model/LDPSA/LDP/LDPVSE/InVldLDP/Bilateral hysteresis2'
 * '<S166>' : 'LDPSA_model/LDPSA/LDP/LDPVSE/InVldLDP/MappingUint2'
 * '<S167>' : 'LDPSA_model/LDPSA/LDP/LDPVSE/InVldLDP/Unilateral hysteresis '
 * '<S168>' : 'LDPSA_model/LDPSA/LDP/LDPVSE/InVldLDP/Unilateral hysteresis 1'
 * '<S169>' : 'LDPSA_model/LDPSA/LDP/LDPVSE/InVldLDP/Unilateral hysteresis 2'
 * '<S170>' : 'LDPSA_model/LDPSA/LDP/LDPVSE/InVldLDP/Unilateral hysteresis 3'
 * '<S171>' : 'LDPSA_model/LDPSA/LDP/LDPVSE/InVldLDP/VehStartupSpd'
 * '<S172>' : 'LDPSA_model/LDPSA/LDP/LDPVSE/InVldLDP/MappingUint2/Set_bit'
 * '<S173>' : 'LDPSA_model/LDPSA/LDP/LDPVSE/JudgeTrn/ Edge'
 * '<S174>' : 'LDPSA_model/LDPSA/LDP/LDPVSE/JudgeTrn/ Edge1'
 * '<S175>' : 'LDPSA_model/LDPSA/LDP/LDPVSE/JudgeTrn/FollowUpTimer'
 * '<S176>' : 'LDPSA_model/LDPSA/LDP/LDPVSE/JudgeTrn/FollowUpTimer1'
 * '<S177>' : 'LDPSA_model/LDPSA/LDP/LDPVSE/SidCdtnLDP/LfVehLatSpdVld'
 * '<S178>' : 'LDPSA_model/LDPSA/LDP/LDPVSE/SidCdtnLDP/MappingUint1'
 * '<S179>' : 'LDPSA_model/LDPSA/LDP/LDPVSE/SidCdtnLDP/MappingUint8'
 * '<S180>' : 'LDPSA_model/LDPSA/LDP/LDPVSE/SidCdtnLDP/RiVehLatSpdVld'
 * '<S181>' : 'LDPSA_model/LDPSA/LDP/LDPVSE/SidCdtnLDP/LfVehLatSpdVld/Bilateral hysteresis'
 * '<S182>' : 'LDPSA_model/LDPSA/LDP/LDPVSE/SidCdtnLDP/LfVehLatSpdVld/Unilateral hysteresis '
 * '<S183>' : 'LDPSA_model/LDPSA/LDP/LDPVSE/SidCdtnLDP/MappingUint1/Set_bit'
 * '<S184>' : 'LDPSA_model/LDPSA/LDP/LDPVSE/SidCdtnLDP/MappingUint8/Set_bit'
 * '<S185>' : 'LDPSA_model/LDPSA/LDP/LDPVSE/SidCdtnLDP/RiVehLatSpdVld/Bilateral hysteresis'
 * '<S186>' : 'LDPSA_model/LDPSA/LDP/LDPVSE/SidCdtnLDP/RiVehLatSpdVld/Unilateral hysteresis '
 */

/*-
 * Requirements for '<Root>': LDPSA
 */
#endif                                 /* RTW_HEADER_LDPSA_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
