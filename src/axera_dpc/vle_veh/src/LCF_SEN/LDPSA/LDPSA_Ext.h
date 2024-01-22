#ifndef LDP_EXT_TYPE_H
#define LDP_EXT_TYPE_H
#include "tue_common_libs.h"
#include "rtwtypes.h"

typedef struct {
    real32_T LDPSAI_VehSpdActu_Mps;
    real32_T LDPSAI_SpdVelShow_Kmph;
    real32_T LDPSAI_VehAccSpdX_Npkg;
    real32_T LDPSAI_VehAccSpdY_Npkg;
    real32_T LDPSAI_VehCurv_ReMi;
    uint8_T LDPSAI_TrnSgl_St;
    real32_T LDPSAI_WheSteAgl_Dgr;
    real32_T LDPSAI_SteAglSpd_Dgpm;
    boolean_T LDPSAI_LDPSwitchEn_B;
    uint8_T LDPSAI_LDPMod_St;
    boolean_T LDPSAI_LDPErrCdtn_B;
    boolean_T LDPSAI_DtctLnChag_B;
    real32_T LDPSAI_LnWidCalc_Mi;
    real32_T LDPSAI_PstnXLf_Mi;
    real32_T LDPSAI_PstnXRi_Mi;
    real32_T LDPSAI_PstnYLf_Mi;
    real32_T LDPSAI_PstnYSafeLf_Mi;
    real32_T LDPSAI_PstnYRi_Mi;
    real32_T LDPSAI_PstnYSafeRi_Mi;
    real32_T LDPSAI_HeadAglLf_Rad;
    real32_T LDPSAI_HeadAglSafeLf_Rad;
    real32_T LDPSAI_HeadAglRi_Rad;
    real32_T LDPSAI_HeadAglSafeRi_Rad;
    real32_T LDPSAI_CurvLf_ReMi;
    real32_T LDPSAI_CurvSafeLf_ReMi;
    real32_T LDPSAI_CurvRi_ReMi;
    real32_T LDPSAI_CurvSafeRi_ReMi;
    real32_T LDPSAI_CurvRateLf_ReMi2;
    real32_T LDPSAI_CurvRateRi_ReMi2;
    real32_T LDPSAI_VldLengLf_Mi;
    real32_T LDPSAI_VldLengRi_Mi;
    uint8_T LDPSAI_IvldLnSafeLf_St;
    uint16_T LDPSAI_LnIVldLf_St;
    uint8_T LDPSAI_IvldLnSafeRi_St;
    uint16_T LDPSAI_LnIVldRi_St;
    uint16_T LDPSAI_VehStIvld_St;
    uint8_T LDPSAI_IvldStDrv_St;
    uint8_T LDPSAI_CtrlStEn_St;
    uint8_T LDPSAI_StError_St;
    uint8_T LDPSAI_CtrlStNoAvlb_St;
    uint8_T LDPSAI_PrjSpecQu_St;
    boolean_T LDPSAI_DtctCstruSite_B;
    real32_T LDPSAI_CycleTime_Sec;
    real32_T LDPSAI_ABDTimeStamp_Sec;
    real32_T LDPSAI_PstnYCent_Mi;
    real32_T LDPSAI_VehYawRate_rps;
    boolean_T LDPSAI_AEBActive_B;
    boolean_T NVRAM_LDPSwitch_B;
    real32_T NVRAM_LDPStartupSpd_Kmph;
    real32_T LDPSAI_VehStartupSpdHMI_Kmph;
} sLDPSAInput_t;

typedef struct {
    real32_T LDPSAI_VehWid_Mi;
} sLDPSAParam_t;

typedef struct {
    UINT8_T LDPSC_DgrSide_St;         /* State of danger side */
    UINT8_T LDPSC_RdyToTrig_B;        /* Condition of Ready to trigger state */
    UINT8_T LDPSC_SysOut_St;          /* Actual state of LDP */
    REAL32_T LDPDT_LnPstnLf_Mi;       /* Position of  of left lane */
    REAL32_T LDPDT_LnPstnRi_Mi;       /* Position of  of right lane */
    REAL32_T LDPDT_LnHeadLf_Rad;      /* Heading angle of left lane */
    REAL32_T LDPDT_LnHeadRi_Rad;      /* Heading angle of rifht lane */
    REAL32_T LDPDT_LnCltdCurvLf_ReMi; /* Clothoid curvature of left lane */
    REAL32_T LDPDT_LnCltdCurvRi_ReMi; /* Clothoid curvature of right lane */
    real32_T LDPVSE_NVRAMVehStartupSpd_Kmph;/* '<S1>/LDP' */
    boolean_T LDPSC_NVRAMLDPSwitch_B;/* '<S1>/LDP' */
} sLDPSAOutput_t;

#ifndef Rte_TypeDef_sLDPSADebug_t
#define Rte_TypeDef_sLDPSADebug_t
typedef struct {
    real32_T LDPTT_LnBdryPstnXLf_Mi;/* '<S1>/LDP'
                                        * Filtered left lane clothoid X0 position from LDP
                                        */
    real32_T LDPTT_LnBdryPstnYLf_Mi;/* '<S1>/LDP'
                                     * Filtered left lane clothoid Y0 position from LDP
                                     */
    real32_T LDPTT_LnBdryHeadAglLf_Rad;/* '<S1>/LDP'
                                        * Filtered left lane clothoid heading angle from LDP
                                        */
    real32_T LDPTT_LnBdryCurvLf_ReMi;/* '<S1>/LDP'
                                      * Filtered left lane clothoid curvature from LDP
                                      */
    real32_T LDPTT_LnBdryCurvRateLf_ReMi2;/* '<S1>/LDP'
                                           * Filtered left lane clothoid change of curvature from LDP
                                           */
    real32_T LDPTT_LnBdryVldLengLf_Mi;/* '<S1>/LDP'
                                       * Filtered left lane clothoid length from LDP
                                       */
    real32_T LDPTT_LnBdryPstnXRi_Mi;/* '<S1>/LDP'
                                     * Filtered right lane clothoid X0 position from LDP
                                     */
    real32_T LDPTT_LnBdryPstnYRi_Mi;/* '<S1>/LDP'
                                     * Filtered right lane clothoid Y0 position from LDP
                                     */
    real32_T LDPTT_LnBdryHeadAglRi_Rad;/* '<S1>/LDP'
                                        * Filtered right lane clothoid heading angle from LDP
                                        */
    real32_T LDPTT_LnBdryCurvRi_ReMi;/* '<S1>/LDP'
                                      * Filtered right lane clothoid curvature from LDP
                                      */
    real32_T LDPTT_LnBdryCurvRateRi_ReMi2;/* '<S1>/LDP'
                                           * Filtered right lane clothoid change of curvature from LDP
                                           */
    real32_T LDPTT_LnBdryVldLengRi_Mi;/* '<S1>/LDP'
                                       * Filtered right lane clothoid length from LDP
                                       */
    real32_T LDPTT_LnBdryPstnXCent_Mi;/* '<S1>/LDP'
                                       * Filtered center lane clothoid X0 position from LDP
                                       */
    real32_T LDPTT_LnBdryPstnYCent_Mi;/* '<S1>/LDP'
                                       * Filtered center lane clothoid Y0 position from LDP
                                       */
    real32_T LDPTT_LnBdryHeadAglCent_Rad;/* '<S1>/LDP'
                                          * Filtered center lane clothoid heading angle from LDP
                                          */
    real32_T LDPTT_LnBdryCurvCent_ReMi;/* '<S1>/LDP'
                                        * Filtered center lane clothoid curvature from LDP
                                        */
    real32_T LDPTT_LnBdryCurvRateCent_ReMi2;/* '<S1>/LDP'
                                             * Filtered center lane clothoid change of curvature from LDP
                                             */
    real32_T LDPTT_LnBdryVldLengCent_Mi;/* '<S1>/LDP'
                                         * Filtered center lane clothoid length from LDP
                                         */
    real32_T LDPTT_TgtPstnYLf_Mi;   /* '<S1>/LDP'
                                     * Lateral distance to the target when a dangerous situation on the left side takes place
                                     */
    real32_T LDPTT_TgtPstnYRi_Mi;   /* '<S1>/LDP'
                                     * Lateral distance to the target when a dangerous situation on the right side takes place
                                     */
    real32_T LDPTV_FTireAccMx_Mps2; /* '<S1>/LDP' */
    real32_T LDPTV_FTireAccMn_Mps2; /* '<S1>/LDP' */
    real32_T LDPTV_DstcYTgtAreaLf_Mi;/* '<S1>/LDP'
                                      * Y-axis distance of host vehicle to left target area
                                      */
    real32_T LDPTV_DstcYTgtAreaRi_Mi;/* '<S1>/LDP'
                                      * Y-axis distance of host vehicle to right target area
                                      */
    real32_T LDPTV_FctTgtDistY_Fct; /* '<S1>/LDP' */
    real32_T LDPTV_TrajPlanServQu_Fct;/* '<S1>/LDP' */
    real32_T LDPTV_PredTiCurv_Sec;  /* '<S1>/LDP' */
    real32_T LDPTV_PredTiAgl_Sec;   /* '<S1>/LDP' */
    real32_T LDPTV_TiLmtEnDura_Sec; /* '<S1>/LDP' */
    real32_T LDPTV_JerkLmtMx_Mps3;  /* '<S1>/LDP'
                                     * Maximum Jerk Allowed in the trajectory planning
                                     */
    real32_T LDPTV_VeloXObst_Mps;   /* '<S1>/LDP'
                                     * X-axis velocity of obstacle
                                     */
    real32_T LDPTV_AccXObst_Mps2;   /* '<S1>/LDP'
                                     * X-axis acceleration of obstacle
                                     */
    real32_T LDPTV_DstcXObst_Mi;    /* '<S1>/LDP'
                                     * X-axis distance of obstacle
                                     */
    real32_T LDPTV_DstcYObst_Mi;    /* '<S1>/LDP'
                                     * Y-axis distance of obstacle
                                     */
    real32_T LDPTV_LmtCurvMx_ReMi;  /* '<S1>/LDP'
                                     * Maximal limiter curvature allowed.
                                     */
    real32_T LDPTV_LmtCurvGradIncMx_ReMps;/* '<S1>/LDP'
                                           * Maximal limiter curvature gradient allowed.
                                           */
    real32_T LDPTV_LmtCurvGradDecMx_ReMps;/* '<S1>/LDP'
                                           * Maximal limiter curvature gradient allowed.
                                           */
    real32_T LDPTV_SnsTiStamp_Sec;  /* '<S1>/LDP'
                                     * Sensor time stamp in seconds
                                     */
    real32_T LDPTV_SteWhlGradLmt_Fct;/* '<S1>/LDP'
                                      * Steering Wheel Stiffness Limiter
                                      */
    real32_T LDPTV_SteWhlGrad_ReS;  /* '<S1>/LDP'
                                     * Steering Wheel Stiffness Gradient
                                     */
    real32_T LDPTV_TrqRampGrad_ReS; /* '<S1>/LDP'
                                     * Torque Ramp Gradient
                                     */
    real32_T LDPTV_MxTrqScalGradLmt_Fct;/* '<S1>/LDP'
                                         * Maximum Torque Scaling Limiter (Torque saturation)
                                         */
    real32_T LDPTV_MxTrqScalGrad_ReS;/* '<S1>/LDP'
                                      * Maximum Torque Scaling Gradient
                                      */
    real32_T LDPTV_WeightEndTi_Fct; /* '<S1>/LDP'
                                     * Weight of the end time for the calculation in the TRJPLN
                                     */
    real32_T LDPTV_PlanningHorizon_Sec;/* '<S1>/LDP' */
    real32_T LDPTV_DMCDeraLvl_Fct;  /* '<S1>/LDP'
                                     * DMC Derating Level of the LDP function
                                     */
    real32_T LDPTV_WidObst_Mi;      /* '<S1>/LDP'
                                     * Width of obstacle
                                     */
    real32_T LDPTV_LmtCurvGradCtrlMx_ReMps;/* '<S1>/LDP'
                                            * Maximal limiter curvature gradient allowed.
                                            */
    real32_T LDPSC_CrvSensiDecayRi_Mi;/* '<S68>/Switch6' */
    real32_T LDPSC_DlcThdMode2_Mi;  /* '<S68>/1-D Lookup Table1'
                                     * DLC threshold at LDW mode 2
                                     */
    real32_T LDPSC_DlcThdMode3_Mi;  /* '<S68>/1-D Lookup Table2'
                                     * DLC threshold at LDW mode 3
                                     */
    real32_T LDPSC_DlcThdMode1_Mi;  /* '<S68>/1-D Lookup Table'
                                     * DLC threshold at LDW mode 1
                                     */
    real32_T LDPSC_DstcToLnTrsd_Mi; /* '<S68>/Product'
                                     *  Threshold distance to  lane crossing
                                     */
    real32_T LDPSC_DstcToLnTrsdCrvCpstnRi_Mi;/* '<S68>/1-D Lookup Table4'
                                              * Curvature compensation of threshold distance to right lane crossing
                                              */
    real32_T LDPSC_DstcToLnTrsdRi_Mi;/* '<S68>/Add3'
                                      *  Threshold distance to right lane crossing
                                      */
    real32_T LDPDT_CrvThdMaxRi_ReMi;/* '<S13>/1-D Lookup Table'
                                     * Curve of maximum threshold of right clothiod curvature
                                     */
    real32_T LDPDT_CrvThdHystRi_ReMi;/* '<S13>/1-D Lookup Table1'
                                      * Curve of offset threshold of right clothiod curvature
                                      */
    real32_T LDPDT_RawDstcToLnRi_Mi;/* '<S10>/Subtract1'
                                     * Raw Distance of between vehicel and right lane
                                     */
    real32_T LDPDT_DstcToLnRi_Mi;   /* '<S10>/Switch5'
                                     * Distance of between vehicel and right lane
                                     */
    real32_T LDPDT_RawLatVehSpdRi_Mps;/* '<S10>/Product1'
                                       * Raw right lateral vehicle speed
                                       */
    real32_T LDPDT_LatVehSpdRi_Mps; /* '<S10>/Switch2'
                                     * right lateral vehicle speed
                                     */
    real32_T LDPDT_TiToLnRi_Sec;    /* '<S10>/Switch4'
                                     * Time of vehicle to right lane
                                     */
    real32_T LDPSC_TiToLnTrsd_Sec;  /* '<S68>/Product1'
                                     * Threshold time of time to lane crossing
                                     */
    real32_T LDPVSE_MaxLatVel_Mps;  /* '<S142>/Lookup Table'
                                     * Maximum lateral velocity
                                     */
    real32_T LDPDT_CrvThdMaxLf_ReMi;/* '<S12>/1-D Lookup Table'
                                     * Curve of maximum threshold of left clothiod curvature
                                     */
    real32_T LDPDT_CrvThdHystLf_ReMi;/* '<S12>/1-D Lookup Table1'
                                      * Curve of offset threshold of left clothiod curvature
                                      */
    real32_T LDPDT_RawDstcToLnLf_Mi;/* '<S10>/Subtract'
                                     * Raw Distance of between vehicel and left lane
                                     */
    real32_T LDPDT_DstcToLnLf_Mi;   /* '<S10>/Switch'
                                     * Distance of between vehicel and left lane
                                     */
    real32_T LDPSC_DstcToLnTrsdCrvCpstnLf_Mi;/* '<S68>/1-D Lookup Table5'
                                              * Curvature compensation of threshold distance to left lane crossing
                                              */
    real32_T LDPSC_CrvSensiDecayLe_Mi;/* '<S68>/Switch5' */
    real32_T LDPSC_DstcToLnTrsdLf_Mi;/* '<S68>/Add2'
                                      *  Threshold distance to left lane crossing
                                      */
    real32_T LDPDT_RawLatVehSpdLf_Mps;/* '<S10>/Product'
                                       * Raw Left lateral vehicle speed
                                       */
    real32_T LDPDT_LatVehSpdLf_Mps; /* '<S10>/Switch1'
                                     * Left lateral vehicle speed
                                     */
    real32_T LDPDT_TiToLnLf_Sec;    /* '<S10>/Switch3'
                                     * Time of vehicle to left lane
                                     */
    real32_T LDPVSE_MaxCrvBySpd_ReMi;/* '<S140>/Lookup Table'
                                      * Maximum curvature for LDW invalid condition
                                      */
    real32_T LDPVSE_HystCrvBySpd_ReMi;/* '<S140>/Lookup Table1'
                                       * Curvature hysteresis for LDW invalid condition
                                       */
    real32_T LDPSC_WRBlockTime_Sec; /* '<S35>/Switch' */
    real32_T LDPSC_RampoutTime_Sec; /* '<S30>/Switch'
                                     * Rampout time
                                     */
    real32_T LDPTT_RawLnBdryPstnYLf_Mi;/* '<S101>/Switch'
                                        * Raw Filtered left lane clothoid Y0 position from LDP
                                        */
    real32_T LDPTT_RawLnBdryPstnYRi_Mi;/* '<S101>/Switch1'
                                        * Raw Filtered right lane clothoid Y0 position from LDP
                                        */
    real32_T LDPTT_TgtLatDstcRi_Mi; /* '<S101>/Switch9'
                                     * Distance between the hazardous lane marking and the planned target.
                                     */
    real32_T LDPTT_TgtLatDstcLf_Mi; /* '<S101>/Switch2'
                                     * Distance between the hazardous lane marking and the planned target.
                                     */
    real32_T LDPTT_RawBdryPstnYCent_Mi;/* '<S101>/Switch16'
                                        * Raw Filtered center lane clothoid Y0 position from LDP
                                        */
    real32_T LDPTV_LatVel_Mps;      /* '<S132>/Switch'
                                        * Lateral Velocity For LDPTV
                                        */
    uint16_T LDPSC_SuppValid_Debug; /* '<S33>/Signal Conversion8'
                                     * Suppression debug
                                     */
    uint8_T LDPTV_TrajCtrlSt_St;    /* '<S1>/LDP'
                                     * Trajectory Guidance Qualifier Output
                                     */
    uint8_T LDPDT_CurveTypeRi_St;   /* '<S11>/Switch2'
                                     * Curve Type of right Lane
                                     */
    uint8_T LDPVSE_SidCdtnLDPRi_St; /* '<S142>/Signal Conversion1'
                                     * State of right side at LDP
                                     */
    uint8_T LDPDT_CurveTypeLe_St;   /* '<S11>/Switch'
                                     * Curve Type of left Lane
                                     */
    uint8_T LDPVSE_SidCdtnLDPLf_St; /* '<S142>/Signal Conversion'
                                     * State of left side at LDP
                                     */
    uint8_T LDPVSE_IvldLDP_St;      /* '<S140>/Signal Conversion2'
                                     * Invalid state of LDP
                                     */
    boolean_T LDPTV_HighStatReq_B;  /* '<S1>/LDP'
                                     * High Stationary Accuracy required
                                     */
    boolean_T LDPTV_LatCpstnEn_B;   /* '<S1>/LDP'
                                     * Switch for the latency compensation in trajectory plan
                                     */
    boolean_T LDPTV_LmtEn_B;        /* '<S1>/LDP'
                                     * Switch to limit the target curvature in trajectory controller
                                     */
    boolean_T LDPTV_TrigReplan_B;   /* '<S1>/LDP'
                                     * It has to be 1 for trajectory planning to calculate a trajectory
                                     */    
    boolean_T LDPDT_RdyTrigLDP_B;   /* '<S8>/Equal'
                                     * Condition of ready to trigger LDP state
                                     */
    boolean_T LDPDT_EnaSafety_B;    /* '<S8>/AND'
                                     * Enable flag for data from the safety interface
                                     */
    boolean_T LDPDT_EnaByInVldQlfrRi_B;/* '<S13>/Relational Operator4'
                                           * Enable flag for right lane validity by left lane invalid qualifier
                                           */
    boolean_T LDPDT_EnaByInVldQlfrSfRi_B;/* '<S13>/Relational Operator1'
                                          * Enable flag for right lane validity by left lane invalid qualifier for safety interface
                                          */
    boolean_T LDPDT_LnTrigVldRi_B;  /* '<S13>/Logical Operator2'
                                     * Condition validity of right lane marker at LDP trigger
                                     */
    boolean_T LDPDT_CclByInVldQlfrRi_B;/* '<S13>/Relational Operator5'
                                        * Enable flag for right lane validity by left lane invalid qualifier for safety interface when cancel the function
                                        */
    boolean_T LDPDT_LnCclVldRi_B;   /* '<S13>/Logical Operator5'
                                     * Condition validity of right lane marker at LDP cancel
                                     */
    boolean_T LDPDT_LnMakVldRi_B;   /* '<S13>/Switch'
                                     * Condition validity of right lane marker
                                     */
    boolean_T LDPSC_RawTrigByDlcRi_B;/* '<S70>/Relational Operator3'
                                      * Raw trigger flag by DLC for right lane
                                      */
    boolean_T LDPSC_EnaTlcTrigRi_B; /* '<S70>/Relational Operator1'
                                     * Enable flag for Raw trigger flag by TLC for right lane
                                     */
    boolean_T LDPSC_RawTrigByTlcRi_B;/* '<S70>/Relational Operator2'
                                      * Raw trigger flag by TLC for right lane
                                      */
    boolean_T LDPSC_DlyTrigByTlcRi_B;/* '<S86>/AND'
                                      * Trigger flag by TLC for right lane
                                      */
    boolean_T LDPSC_EnaLdwTrigRi_B; /* '<S80>/AND'
                                     * Enable flag for LDW function trigger
                                     */
    boolean_T LDPSC_RstLdwTrigRi_B; /* '<S70>/Relational Operator4'
                                     * Reset flag for LDW function trigger
                                     */
    boolean_T LDPSC_HoldLdwTrigRi_B;/* '<S70>/Signal Conversion'
                                        * Enable flag for LDW function trigger after time holding
                                        */
    boolean_T LDPSC_ResetForSafeRi_B;/* '<S70>/Logical Operator4'
                                      * Reset flag for the safe situation condition of right lane
                                      */
    boolean_T LDPSC_SetForSafeRi_B; /* '<S70>/Relational Operator6'
                                     * Set flag for the safe situation condition of right lane
                                     */
    boolean_T LDPVSE_EdgeRiseTurnSglRi_B;/* '<S154>/AND' */
    boolean_T LDPVSE_TrnSglRi_B;    /* '<S141>/Signal Conversion2'
                                     * Condition of  right turn signal
                                     */
    boolean_T LDPVSE_RdyTrigLDW_B;  /* '<S142>/Equal'
                                     * Ready Trigger flag for LDW
                                     */
    boolean_T LDPVSE_VehLatSpdVldRi_B;/* '<S160>/Switch'
                                       * Validity of right lateral vehicle speed
                                       */
    boolean_T LDPSC_TrigBySideCondRi_B;/* '<S70>/Relational Operator7'
                                        * LDW function trigger flag by  side condition of right lane
                                        */
    boolean_T LDPSC_TrigByPrjSpecRi_B;/* '<S70>/Equal'
                                       * LDW function trigger flag by customer projects of right lane
                                       */
    boolean_T LDPSC_TrigRi_B;       /* '<S70>/Logical Operator3'
                                     * Condition of right trigger
                                     */
    boolean_T LDPDT_EnaByCstruSiteLf_B;/* '<S12>/AND'
                                        * Enable flag for left lane validity by construction site detected
                                        */
    boolean_T LDPDT_EnaByInVldQlfrLf_B;/* '<S12>/Relational Operator4'
                                        * Enable flag for left lane validity by left lane invalid qualifier
                                        */
    boolean_T LDPDT_EnaByInVldQlfrSfLf_B;/* '<S12>/Relational Operator1'
                                          * Enable flag for left lane validity by left lane invalid qualifier for safety interface
                                          */
    boolean_T LDPDT_LnTrigVldLf_B;  /* '<S12>/Logical Operator2'
                                     * Condition validity of left lane marker at LDP trigger
                                     */
    boolean_T LDPDT_CclByInVldQlfrLf_B;/* '<S12>/Relational Operator5'
                                        * Enable flag for left lane validity by left lane invalid qualifier for safety interface when cancel the function
                                        */
    boolean_T LDPDT_LnCclVldLf_B;   /* '<S12>/Logical Operator5'
                                     * Condition validity of left lane marker at LDP cancel
                                     */
    boolean_T LDPDT_LnMakVldLf_B;   /* '<S12>/Switch'
                                     * Condition validity of left lane marker
                                     */
    boolean_T LDPSC_RawTrigByDlcLf_B;/* '<S69>/Relational Operator3'
                                      * Raw trigger flag by DLC for left lane
                                      */
    boolean_T LDPSC_EnaTlcTrigLf_B; /* '<S69>/Relational Operator1'
                                     * Enable flag for Raw trigger flag by TLC for left lane
                                     */
    boolean_T LDPSC_RawTrigByTlcLf_B;/* '<S69>/Relational Operator2'
                                      * Raw trigger flag by TLC for left lane
                                      */
    boolean_T LDPSC_DlyTrigByTlcLf_B;/* '<S79>/AND'
                                         * Trigger flag by TLC for left lane
                                         */
    boolean_T LDPSC_EnaLdwTrigLf_B; /* '<S73>/AND'
                                     * Enable flag for LDW function trigger
                                     */
    boolean_T LDPSC_RstLdwTrigLf_B; /* '<S69>/Relational Operator4'
                                     * Reset flag for LDW function trigger
                                     */
    boolean_T LDPSC_HoldLdwTrigLf_B;/* '<S69>/Signal Conversion'
                                     * Enable flag for LDW function trigger after time holding
                                     */
    boolean_T LDPSC_ResetForSafeLf_B;/* '<S69>/Logical Operator4'
                                      * Reset flag for the safe situation condition of left lane
                                      */
    boolean_T LDPSC_SetForSafeLf_B; /* '<S69>/Relational Operator6'
                                     * Set flag for the safe situation condition of left lane
                                     */
    boolean_T LDPVSE_EdgeRiseTurnSglLf_B;/* '<S153>/AND' */
    boolean_T LDPVSE_TrnSglLf_B;    /* '<S141>/Signal Conversion'
                                     * Condition of  left turn signal
                                     */
    boolean_T LDPVSE_VehLatSpdVldLf_B;/* '<S157>/Switch'
                                       * Validity of left lateral vehicle speed
                                       */
    boolean_T LDPSC_TrigBySideCondLf_B;/* '<S69>/Relational Operator7'
                                        * LDW function trigger flag by  side condition of left lane
                                        */
    boolean_T LDPSC_TrigByPrjSpecLf_B;/* '<S69>/Equal'
                                       * LDW function trigger flag by customer projects of left lane
                                       */
    boolean_T LDPSC_TrigLf_B;       /* '<S69>/Logical Operator3'
                                     * Condition of left trigger
                                     */
    boolean_T LDPSC_Trig_B;         /* '<S67>/Logical Operator1'
                                     * Condition of trigger
                                     */
    boolean_T LDPSC_EnaDgrSide_B;   /* '<S67>/Logical Operator6'
                                     * Enable flag for Degerous side state
                                     */
    boolean_T LDPSC_FnsByDgrStLf_B; /* '<S31>/Relational Operator7' */
    boolean_T LDPSC_FnsByLatDistLf_B;/* '<S31>/Logical Operator8' */
    boolean_T LDPSC_FnsByHeadingLf_B;/* '<S31>/Logical Operator9' */
    boolean_T LDPSC_FnsByLatSpdLf_B;/* '<S31>/Logical Operator10' */
    boolean_T LDPSC_DgrFnsLf_B;     /* '<S31>/Logical Operator6' */
    boolean_T LDPSC_FnsByDgrStRi_B; /* '<S31>/Relational Operator9' */
    boolean_T LDPSC_FnsByLatDistRi_B;/* '<S31>/Logical Operator14' */
    boolean_T LDPSC_FnsByHeadingRi_B;/* '<S31>/Logical Operator15' */
    boolean_T LDPSC_FnsByLatSpdRi_B;/* '<S31>/Logical Operator13' */
    boolean_T LDPSC_DgrFnsRi_B;     /* '<S31>/Logical Operator12' */
    boolean_T LDPSC_MinLdwBySysSt_B;/* '<S31>/Relational Operator8' */
    boolean_T LDPSC_EdgeRiseForMinLdw_B;/* '<S50>/AND' */
    boolean_T LDPSC_HoldForMinLdw_B;/* '<S66>/GreaterThan1' */
    boolean_T LDPSC_FlagMinTimeLDW_B;/* '<S31>/Logical Operator11' */
    boolean_T LDPSC_DgrFns_B;       /* '<S59>/AND'
                                     * Condition of danger finish
                                     */
    boolean_T LDPSC_AbortBySpecific_B;/* '<S35>/Relational Operator2'
                                       * LDW abort conditions by LDW specific bitfield
                                       */
    boolean_T LDPSC_AbortByVehSt_B; /* '<S35>/Relational Operator1'
                                     * LDW abort conditions by vehicle state
                                     */
    boolean_T LDPSC_AbortByDrvSt_B; /* '<S35>/Relational Operator3'
                                     * LDW abort conditions by drive state
                                     */
    boolean_T LDPSC_AbortByCtrlSt_B;/* '<S35>/Relational Operator4'
                                     * LDW abort conditions by active control state
                                     */
    boolean_T LDPSC_AbortBySysSt_B; /* '<S35>/Relational Operator5'
                                     * LDW abort conditions by system state
                                     */
    boolean_T LDPSC_AbortByAvlSt_B; /* '<S35>/Relational Operator6'
                                     * LDW abort conditions by no available state
                                     */
    boolean_T LDPSC_AbortByPrjSpec_B;/* '<S35>/Relational Operator7'
                                      * LDW abort conditions by customer projects
                                      */
    boolean_T LDPSC_Abort_B;        /* '<S35>/Logical Operator6'
                                     * Condition of LDP abort state
                                     */
    boolean_T LDPSC_StrgRdy_B;      /* '<S35>/Logical Operator1'
                                     * Condition of LDP strong ready state
                                     */
    boolean_T LDPSC_SuppBySpecific_B;/* '<S35>/Relational Operator21'
                                      * LDW suppresion conditions by LDW specific bitfield
                                      */
    boolean_T LDPSC_SuppByVehSt_B;  /* '<S35>/Relational Operator20'
                                     * LDW suppresion conditions by vehicle state
                                     */
    boolean_T LDPSC_SuppByDrvSt_B;  /* '<S35>/Relational Operator15'
                                     * LDW suppresion conditions by drive state
                                     */
    boolean_T LDPSC_SuppByCtrlSt_B; /* '<S35>/Relational Operator16'
                                     * LDW suppresion conditions by active control state
                                     */
    boolean_T LDPSC_SuppBySysSt_B;  /* '<S35>/Relational Operator17'
                                     * LDW suppresion conditions by system state
                                     */
    boolean_T LDPSC_SuppyByAvlSt_B; /* '<S35>/Relational Operator18'
                                     * LDW suppresion conditions by no available state
                                     */
    boolean_T LDPSC_SuppPrjSpec_B;  /* '<S35>/Relational Operator19'
                                     * LDW suppresion conditions by customer projects
                                     */
    boolean_T LDPSC_Suppresion_B;   /* '<S35>/Logical Operator3' */
    boolean_T LDPSC_WeakRdyBySpecific_B;/* '<S35>/Relational Operator28'
                                         * LDW weak ready conditions by LDW specific bitfield
                                         */
    boolean_T LDPSC_WeakRdyByVehSt_B;/* '<S35>/Relational Operator27'
                                      * LDW weak ready conditions by vehicle state
                                      */
    boolean_T LDPSC_WeakRdyByDrvSt_B;/* '<S35>/Relational Operator22'
                                      * LDW weak ready conditions by drive state
                                      */
    boolean_T LDPSC_WeakRdyByCtrlSt_B;/* '<S35>/Relational Operator23'
                                       * LDW strong weak conditions by active control state
                                       */
    boolean_T LDPSC_WeakRdyBySysSt_B;/* '<S35>/Relational Operator24'
                                      * LDW weak ready conditions by system state
                                      */
    boolean_T LDPSC_WeakRdyByAvlSt_B;/* '<S35>/Relational Operator25'
                                      * LDW weak weak conditions by no available state
                                      */
    boolean_T LDPSC_WeakRdyByPrjSpec_B;/* '<S35>/Relational Operator26'
                                        * LDW weak weak conditions by customer projects
                                        */
    boolean_T LDPSC_BlockTimeBySysOut_B;/* '<S35>/Logical Operator9' */
    boolean_T LDPSC_RawBlockTimeByRampOut_B;/* '<S42>/AND' */
    boolean_T LDPSC_BlockTimeByRampOut_B;/* '<S47>/GreaterThan1' */
    boolean_T LDPSC_BlockTime_B;    /* '<S35>/Logical Operator7' */
    boolean_T LDPSC_WkRdy_B;        /* '<S35>/Logical Operator4'
                                     * Condition of LDP weak ready state
                                     */
    boolean_T LDPSC_CancelBySpecific_B;/* '<S34>/Relational Operator38'
                                        * LDW cancel conditions by LDW specific bitfield
                                        */
    boolean_T LDPSC_CancelByVehSt_B;/* '<S34>/Relational Operator37'
                                     * LDW cancel conditions by vehicle state
                                     */
    boolean_T LDPSC_CancelByDrvSt_B;/* '<S34>/Relational Operator32'
                                     * LDW cancel conditions by drive state
                                     */
    boolean_T LDPSC_CancelByCtrlSt_B;/* '<S34>/Relational Operator33'
                                      * LDW cancel conditions by active control state
                                      */
    boolean_T LDPSC_CancelBySysSt_B;/* '<S34>/Relational Operator34'
                                     * LDW cancel conditions by system state
                                     */
    boolean_T LDPSC_CancelByAvlSt_B;/* '<S34>/Relational Operator35'
                                     * LDW cancel conditions by no available state
                                     */
    boolean_T LDPSC_CancelByPrjSpec_B;/* '<S34>/Relational Operator36'
                                       * LDW cancel conditions by customer projects
                                       */
    boolean_T LDPSC_MaxDurationBySysSt_B;/* '<S38>/Relational Operator47' */
    boolean_T LDPSC_EdgRiseForSysSt_B;/* '<S39>/AND' */
    boolean_T LDPSC_MaxDurationByStDly_B;/* '<S38>/Logical Operator22' */
    boolean_T LDPSC_TiWarmMx_B;     /* '<S38>/Logical Operator23'
                                     * Condition of warming max time
                                     */
    boolean_T LDPSC_ErrSideByTrigLf_B;/* '<S37>/Logical Operator19' */
    boolean_T LDPSC_ErrSideBySideCondLf_B;/* '<S37>/Relational Operator42' */
    boolean_T LDPSC_ErrSidByPrjSpecLf_B;/* '<S37>/Relational Operator43' */
    boolean_T LDPSC_ErrSidCdtnLf_B; /* '<S37>/Logical Operator18'
                                     * Error condition of left side
                                     */
    boolean_T LDPSC_SideCondByDgrLf_B;/* '<S37>/Relational Operator41' */
    boolean_T LDPSC_CanelBySideLf_B;/* '<S37>/Logical Operator15' */
    boolean_T LDPSC_SideCondByDgrRi_B;/* '<S37>/Relational Operator44' */
    boolean_T LDPSC_ErrSideByTrigRi_B;/* '<S37>/Logical Operator21' */
    boolean_T LDPSC_ErrSideBySideCondRi_B;/* '<S37>/Relational Operator45' */
    boolean_T LDPSC_ErrSidByPrjSpecRi_B;/* '<S37>/Relational Operator46' */
    boolean_T LDPSC_ErrSidCdtnRi_B; /* '<S37>/Logical Operator2'
                                     * Error condition of right side
                                     */
    boolean_T LDPSC_CanelBySideRi_B;/* '<S37>/Logical Operator1' */
    boolean_T LDPSC_ErrSidCdtn_B;   /* '<S37>/Logical Operator16'
                                     * Error condition of side
                                     */
    boolean_T LDPSC_CLatDevByDlcLf_B;/* '<S36>/Logical Operator24' */
    boolean_T LDPSC_CLatDevByDgrLf_B;/* '<S36>/Relational Operator39' */
    boolean_T LDPSC_CclLatDevLf_B;  /* '<S36>/Logical Operator12'
                                     * Cancel condition of left lane deviation
                                     */
    boolean_T LDPSC_CLatDevByDlcRi_B;/* '<S36>/Relational Operator40' */
    boolean_T LDPSC_CLatDevByDgrRi_B;/* '<S36>/Logical Operator25' */
    boolean_T LDPSC_CclLatDevRi_B;  /* '<S36>/Logical Operator14'
                                     * Cancel condition of right lane deviation
                                     */
    boolean_T LDPSC_CclLatDev_B;    /* '<S36>/Logical Operator13'
                                     * Cancel condition of lane deviation
                                     */
    boolean_T LDPSC_Cancel_B;       /* '<S34>/Logical Operator11' */
    boolean_T LDPSC_Suppression_B;  /* '<S33>/OR'
                                     * Suppression condition
                                     */
    boolean_T LDPVSE_TgtCntrByLnWidth_B;/* '<S140>/Less Than' */
    boolean_T LDPVSE_TgtCntrLnEn_B; /* '<S140>/Logical Operator11'
                                     *  Enable the target in the center of the lane.
                                     */
    boolean_T LDPTV_CurvInner_B;    /* '<S137>/AND2'
                                     * The flag for the left or right lane marking is an inner curve
                                     */
    boolean_T LDPTV_CurvOuter_B;    /* '<S137>/AND1'
                                     * The flag for the left or right lane marking is an outer curve
                                     */
    boolean_T LDPSC_Degradation_B;  /* '<S30>/Logical Operator1' */
    boolean_T LDPSC_DegradationEdgeRise_B;/* '<S48>/AND' */
    boolean_T LDPSC_Degr_B;         /* '<S30>/Logical Operator2'
                                     * Condition of degradation
                                     */


    real32_T LDPSC_DlyTiOfTiToLnRiMn_Sec;/* '<S86>/Unit Delay'
                                          * Delay time of time to right lane crossing
                                          */
    real32_T LDPSC_HdTiTrigRi_Sec;  /* '<S85>/Unit Delay'
                                     * holding time right trigger
                                     */
    real32_T LDPVSE_HodTiTrnSglRi_Sec;/* '<S156>/Unit Delay'
                                       * Holding time of right turn signal
                                       */
    real32_T LDPSC_DlyTiOfTiToLnLfMn_Sec;/* '<S79>/Unit Delay'
                                          * Delay time of time to left lane crossing
                                          */
    real32_T LDPSC_HdTiTrigLf_Sec;  /* '<S78>/Unit Delay'
                                     *  holding time left trigger
                                     */
    real32_T LDPVSE_HodTiTrnSglLf_Sec;/* '<S155>/Unit Delay'
                                       * Holding time of left turn signal
                                       */
    real32_T LDPSC_HdTiWarming_Sec; /* '<S66>/Unit Delay'
                                     * Holding time of warming state start
                                     */
    real32_T LDPSC_DlyTiTgtFns_Sec; /* '<S59>/Unit Delay'
                                     * Delay time of LDP finish state
                                     */
    real32_T LDPSC_HdTiFns_Sec;     /* '<S47>/Unit Delay'
                                     * Holding time of finish state end
                                     */
    real32_T LDPSC_HdTiWarmMx_Sec;  /* '<S41>/Unit Delay'
                                     * Holding time of warming state start
                                     */
    real32_T LDPTT_LwLnBdryPstnXLf_Mi;/* '<S109>/Unit Delay'
                                       * Low filtered left lane clothoid X0 position from LDP
                                       */
    real32_T LDPTT_LstLnBdryPstnXLf_Mi;/* '<S106>/UnitDelay'
                                        * Last filtered left lane clothoid Y0 position from LDP
                                        */
    real32_T LDPTT_LstLnBdryVldLengLf_Mi;/* '<S106>/UnitDelay5'
                                          * Last filtered left lane clothoid length from LDP
                                          */
    real32_T LDPTT_LwLnBdryVldLengLf_Mi;/* '<S114>/Unit Delay'
                                         * Low filtered left lane clothoid length from LDP
                                         */
    real32_T LDPTT_LstLnBdryPstnYLf_Mi;/* '<S106>/UnitDelay1'
                                        * Last filtered left lane clothoid Y0 position from LDP
                                        */
    real32_T LDPTT_LstLnWidCalc_Mi; /* '<S101>/UnitDelay'
                                     * Calculation ego lane width
                                     */
    real32_T LDPTT_LwLnBdryPstnYLf_Mi;/* '<S110>/Unit Delay'
                                       * Low filtered left lane clothoid Y0 position from LDP
                                       */
    real32_T LDPTT_LwLnBdryPstnYRi_Mi;/* '<S116>/Unit Delay'
                                       * Low filtered right lane clothoid Y0 position from LDP
                                       */
    real32_T LDPTT_LwLnBdryPstnXRi_Mi;/* '<S115>/Unit Delay'
                                       * Low filtered right lane clothoid X0 position from LDP
                                       */
    real32_T LDPTT_LwLnBdryHeadAglRi_Rad;/* '<S117>/Unit Delay'
                                          * Low filtered right lane clothoid heading angle from LDP
                                          */
    real32_T LDPTT_LwLnBdryCurvRi_ReMi;/* '<S118>/Unit Delay'
                                        * Low filtered right lane clothoid curvature from LDP
                                        */
    real32_T LDPTT_LwLnBdryCurvRateRi_ReMi2;/* '<S119>/Unit Delay'
                                             * Low filtered right lane clothoid change of curvature from LDP
                                             */
    real32_T LDPTT_LwLnBdryVldLengRi_Mi;/* '<S120>/Unit Delay'
                                         * Low filtered right lane clothoid length from LDP
                                         */
    real32_T LDPTT_LstTgtLatDstcRi_Mi;/* '<S101>/UnitDelay4'
                                       * Last distance between the hazardous lane marking and the planned target.
                                       */
    real32_T LDPTT_LstMxTgtLatDevRi_Mi;/* '<S101>/UnitDelay6'
                                        * Last maximal distance between the middle of the vehicle and the planned target.
                                        */
    real32_T LDPTT_LstTgtLatDevRi_Mi;/* '<S101>/UnitDelay7'
                                      * Last Distance between the middle of the vehicle and the planned target.
                                      */
    real32_T LDPTT_LstTgtLatDevLf_Mi;/* '<S101>/UnitDelay3'
                                      * Last Distance between the middle of the vehicle and the planned target.
                                      */
    real32_T LDPTT_LstTgtLatDstcLf_Mi;/* '<S101>/UnitDelay1'
                                       * Last distance between the hazardous lane marking and the planned target.
                                       */
    real32_T LDPTT_LstMxTgtLatDevLf_Mi;/* '<S101>/UnitDelay2'
                                        * Last maximal distance between the middle of the vehicle and the planned target.
                                        */
    real32_T LDPTT_LwLnBdryPstnYCent_Mi;/* '<S121>/Unit Delay'
                                         * Low filtered center lane clothoid Y0 position from LDP
                                         */
    real32_T LDPTT_LstLnBdryPstnYCent_Mi;/* '<S108>/UnitDelay1'
                                          * Last filtered center lane clothoid Y0 position from LDP
                                          */
    real32_T LDPTT_LstLnBdryCurvCent_ReMi;/* '<S108>/UnitDelay3'
                                           * Last filtered center lane clothoid curvature from LDP
                                           */
    real32_T LDPTT_LwLnBdryCurvCent_ReMi;/* '<S123>/Unit Delay'
                                          * Low filtered center lane clothoid curvature from LDP
                                          */
    real32_T LDPTV_LstPlanningHorizon_Sec;/* '<S130>/UnitDelay1'
                                           * Planning Horizon Scaling Factor for the calculation in the TRJPLN
                                           */
    real32_T LDPTV_LstWeightEndTi_Fct;/* '<S130>/UnitDelay'
                                       * Last weight of the end time for the calculation in the TRJPLN
                                       */
    real32_T LDPTT_LwLnBdryHeadAglCent_Rad;/* '<S122>/Unit Delay'
                                            * Low filtered center lane clothoid heading angle from LDP
                                            */
    real32_T LDPTT_LstLnBdryHeadAglCent_Rad;/* '<S108>/UnitDelay2'
                                             * Last filtered center lane clothoid heading angle from LDP
                                             */
    real32_T LDPTT_LwLnBdryCurvRateCent_ReMi2;/* '<S124>/Unit Delay'
                                               * Low filtered center lane clothoid change of curvature from LDP
                                               */
    real32_T LDPTT_LstLnBdryCurvRateCent_ReMi2;/* '<S108>/UnitDelay4'
                                                * Last filtered center lane clothoid change of curvature from LDP
                                                */
    real32_T LDPTT_LwLnBdryPstnXCent_Mi;/* '<S125>/Unit Delay'
                                         * Low filtered center lane clothoid X0 position from LDP
                                         */
    real32_T LDPTT_LstLnBdryPstnXCent_Mi;/* '<S108>/UnitDelay'
                                          * Last filtered center lane clothoid X0 position from LDP
                                          */
    real32_T LDPTT_LwLnBdryVldLengCent_Mi;/* '<S126>/Unit Delay'
                                           * Low filtered center lane clothoid length from LDP
                                           */
    real32_T LDPTT_LstLnBdryVldLengCent_Mi;/* '<S108>/UnitDelay5'
                                            * Last filtered center lane clothoid length from LDP
                                            */
    real32_T LDPTV_LstSteWhlGrad_ReS;/* '<S128>/UnitDelay1'
                                      * Last Steering Wheel Stiffness Gradient by Lateral Velocity
                                      */
    real32_T LDPTV_LstDMCDeraLvl_Fct;/* '<S128>/UnitDelay'
                                      * Last DMC Derating Level of the LDP function
                                      */
    real32_T LDPTT_LstLnBdryHeadAglLf_Rad;/* '<S106>/UnitDelay2'
                                           * Last filtered left lane clothoid heading angle from LDP
                                           */
    real32_T LDPTT_LwLnBdryHeadAglLf_Rad;/* '<S111>/Unit Delay'
                                          * Low filtered left lane clothoid heading angle from LDP
                                          */
    real32_T LDPTT_LstLnBdryCurvLf_ReMi;/* '<S106>/UnitDelay3'
                                         * Last filtered left lane clothoid curvature from LDP
                                         */
    real32_T LDPTT_LwLnBdryCurvLf_ReMi;/* '<S112>/Unit Delay'
                                        * Low filtered left lane clothoid curvature from LDP
                                        */
    real32_T LDPTT_LstLnBdryCurvRateLf_ReMi2;/* '<S106>/UnitDelay4'
                                              * Last filtered left lane clothoid change of curvature from LDP
                                              */
    real32_T LDPTT_LwLnBdryCurvRateLf_ReMi2;/* '<S113>/Unit Delay'
                                             * Low filtered left lane clothoid change of curvature from LDP
                                             */
    real32_T LDPSC_HdTiDegr_Sec;    /* '<S49>/Unit Delay'
                                     * Holding time of degradation
                                     */
    uint8_T LDPSC_LstPrevDgrSide_St;/* '<S2>/Unit Delay' */
    uint8_T LDPSC_DgrSideOld_St;    /* '<S67>/UnitDelay1'
                                     * Old state of danger side
                                     */
    boolean_T LDPDT_UHysCltdCurvVldRi_B;/* '<S18>/Unit Delay'
                                         * Validity of right lane Clothoid curvature
                                         */
    boolean_T LDPDT_BHysHeadAglTrigVldRi_B;/* '<S17>/Unit Delay'
                                               * Valid trigger of right heading angle
                                               */
    boolean_T LDPDT_UHysHeadAglCclVldRi_B;/* '<S19>/Unit Delay'
                                           * Valid cancel of right heading angle
                                           */
    boolean_T LDPSC_HdTiTrigRiEn_B; /* '<S80>/Unit Delay'
                                     * Enable condition of holding time right trigger
                                     */
    boolean_T LDPSC_DisTrigRi_B;    /* '<S84>/Unit Delay'
                                     * Disable condition of right trigger
                                     */
    boolean_T LDPVSE_EdgeRisTrnSglLf_B;/* '<S154>/Unit Delay'
                                        * Edge rise of left turn signal
                                        */
    boolean_T LDPVSE_BHysLatVehSpdVldRi_B;/* '<S165>/Unit Delay'
                                           * Validity of right lateral vehicle speed before trigger state
                                           */
    boolean_T LDPVSE_UHysLatVehSpdVldRi_B;/* '<S166>/Unit Delay'
                                           * Validity of right lateral vehicle speed after trigger state
                                           */
    boolean_T LDPDT_UHysCltdCurvVldLf_B;/* '<S15>/Unit Delay'
                                         * Validity of left lane Clothoid curvature
                                         */
    boolean_T LDPDT_BHysHeadAglTrigVldLf_B;/* '<S14>/Unit Delay'
                                            * Valid trigger of left heading angle
                                            */
    boolean_T LDPDT_UHysHeadAglCclVldLf_B;/* '<S16>/Unit Delay'
                                           * Valid cancel of left heading angle
                                           */
    boolean_T LDPSC_HdTiTrigLfEn_B; /* '<S73>/Unit Delay'
                                     * Enable condition of holding time left trigger
                                     */
    boolean_T LDPSC_DisTrigLf_B;    /* '<S77>/Unit Delay'
                                     * Disable condition of left trigger
                                     */
    boolean_T LDPVSE_EdgeRisTrnSglRi_B;/* '<S153>/Unit Delay'
                                        * Edge rise of right turn signal
                                        */
    boolean_T LDPVSE_BHysLatVehSpdVldLf_B;/* '<S161>/Unit Delay'
                                           * Validity of left lateral vehicle speed before trigger state
                                           */
    boolean_T LDPVSE_UHysLatVehSpdVldLf_B;/* '<S162>/Unit Delay'
                                           * Validity of left lateral vehicle speed after trigger state
                                           */
    boolean_T LDPSC_EdgeRisWarming_B;/* '<S50>/Unit Delay'
                                      * Edge rise of warming state
                                      */
    boolean_T LDPVSE_BHysSpdVeh_B;  /* '<S143>/Unit Delay'
                                     * Validity of displayed longitudinal speed
                                     */
    boolean_T LDPVSE_UHysSteAgl_B;  /* '<S147>/Unit Delay'
                                     * Validity of steering wheel angle
                                     */
    boolean_T LDPVSE_UHysSteAglSpd_B;/* '<S148>/Unit Delay'
                                      * Validity of steering wheel angle speed
                                      */
    boolean_T LDPVSE_BHysAccVehX_B; /* '<S144>/Unit Delay'
                                     * Validity of  longitudinal Acceleration
                                     */
    boolean_T LDPVSE_BHysAccVehY_B; /* '<S149>/Unit Delay'
                                     * Validity of  lateral Acceleration
                                     */
    boolean_T LDPVSE_UHysVehCurv_B; /* '<S150>/Unit Delay'
                                     * Validity of  vehicle curvature
                                     */
    boolean_T LDPVSE_BHysLnWid_B;   /* '<S145>/Unit Delay'
                                     * Validity of lane width
                                     */
    boolean_T LDPSC_EdgeRisFns_B;   /* '<S42>/Unit Delay'
                                     * Edge rise of fginish and cancel state
                                     */
    boolean_T LDPSC_EdgeRisWarmMx_B;/* '<S39>/Unit Delay'
                                     * Edge rise of warming state
                                     */
    boolean_T LDPTT_CtrlIniEn_B;    /* '<S102>/Unit Delay'
                                     * Control initenable flag for LDP
                                     */
    boolean_T LDPTT_LstControl_B;   /* '<S101>/UnitDelay5'
                                     * Control Sate of LDP
                                     */
    boolean_T LDPTV_LstCtrl_St;     /* '<S128>/UnitDelay4'
                                     * Last Control State for DetermineStiffnessAndStatAccu
                                     */
    boolean_T LDPSC_EdgeRisDegr_B;  /* '<S48>/Unit Delay'
                                     * Edge rise of degradation
                                     */
    boolean_T LDPSC_DegrOld_B;      /* '<S30>/UnitDelay'
                                     * UnitDelay condition of degradation
                                     */
    uint8 LDPSC_SysOld_St;  /* '<S4>/UnitDelay'
                                        * Old state of LDP
                                        */
} sLDPSADebug_t;
#endif

extern void LCF_LDPSA_Reset(void);

extern void LCF_LDPSA_Exec(const sLDPSAInput_t* pLDPSAInput,
                           const sLDPSAParam_t* pLDPSAParam,
                           sLDPSAOutput_t* pLDPSAOutput,
                           sLDPSADebug_t* pLDPSADebug);
#endif
