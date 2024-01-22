#ifndef LDW_EXT_TYPE_H
#define LDW_EXT_TYPE_H
#include "tue_common_libs.h"
#include "rtwtypes.h"

typedef struct {
    real32_T LDWSAI_VehSpdActu_Mps; /* '<Root>/Inport1'
                                     * Vehicle speed
                                     */
    real32_T LDWSAI_SpdVelShow_Kmph;/* '<Root>/Inport2'
                                     * Tachometer Vehicle Speed in kilometers/hour
                                     */
    real32_T LDWSAI_VehAccSpdX_Npkg;/* '<Root>/Inport3'
                                     * Longitude acceleration spped
                                     */
    real32_T LDWSAI_VehAccSpdY_Npkg;/* '<Root>/Inport4'
                                     * Lateral acceleration spped
                                     */
    real32_T LDWSAI_VehCurv_ReMi;   /* '<Root>/Inport5'
                                     * Curvature of vehicle
                                     */
    uint8_T LDWSAI_TrnSgl_St;       /* '<Root>/Inport6'
                                     * State of turn signal
                                     */
    real32_T LDWSAI_WheSteAgl_Dgr;  /* '<Root>/Inport7'
                                     * Degrees of vehicle steer wheel angle
                                     */
    real32_T LDWSAI_SteAglSpd_Dgpm; /* '<Root>/Inport8'
                                     * vehicle steer wheel  angle speed
                                     */
    boolean_T LDWSAI_LDWSwitchEn_B; /* '<Root>/Inport9'
                                     * LDW switch of driver
                                     */
    uint8_T LDWSAI_LDWMod_St;       /* '<Root>/Inport10'
                                     * Driver control mode of LDW
                                     */
    boolean_T LDWSAI_LDWErrCdtn_B;  /* '<Root>/Inport11'
                                     * Error condition of LDW
                                     */
    boolean_T LDWSAI_DtctLnChag_B;  /* '<Root>/Inport12'
                                     * Condition of change detected lane
                                     */
    real32_T LDWSAI_LnWidCalc_Mi;   /* '<Root>/Inport13'
                                     * Lane width
                                     */
    real32_T LDWSAI_PstnYLf_Mi;     /* '<Root>/Inport16'
                                     * Distance between left lane and Y0
                                     */
    real32_T LDWSAI_PstnYSafeLf_Mi; /* '<Root>/Inport17'
                                     * Safe distance between left lane and Y0
                                     */
    real32_T LDWSAI_PstnYRi_Mi;     /* '<Root>/Inport18'
                                     * Distance between right lane and Y0
                                     */
    real32_T LDWSAI_PstnYSafeRi_Mi; /* '<Root>/Inport19'
                                     * Safe distance between right lane and Y0
                                     */
    real32_T LDWSAI_HeadAglLf_Rad;  /* '<Root>/Inport20'
                                     * Heading angle of left lane clothoid
                                     */
    real32_T LDWSAI_HeadAglSafeLf_Rad;/* '<Root>/Inport21'
                                       * Safe yaw angle of left lane
                                       */
    real32_T LDWSAI_HeadAglRi_Rad;  /* '<Root>/Inport22'
                                     * Heading angle of right lane clothoid
                                     */
    real32_T LDWSAI_HeadAglSafeRi_Rad;/* '<Root>/Inport23'
                                       * Safe yaw angle of right lane
                                       */
    real32_T LDWSAI_CurvLf_ReMi;    /* '<Root>/Inport24'
                                     * Curvature of left lane clothoid
                                     */
    real32_T LDWSAI_CurvSafeLf_ReMi;/* '<Root>/Inport25'
                                     * Safe curvature of left lane
                                     */
    real32_T LDWSAI_CurvRi_ReMi;    /* '<Root>/Inport26'
                                     * Curvature of right lane clothoid
                                     */
    real32_T LDWSAI_CurvSafeRi_ReMi;/* '<Root>/Inport27'
                                     * Safe curvature of rihgt lane
                                     */
    uint8_T LDWSAI_IvldLnSafeLf_St; /* '<Root>/Inport34'
                                     * Invalid safe state of left lane
                                     */
    uint16_T LDWSAI_LnIVldLf_St;    /* '<Root>/Inport35'
                                     * Invalid state of left lane
                                     */
    uint8_T LDWSAI_IvldLnSafeRi_St; /* '<Root>/Inport36'
                                     * Invalid safe state of right lane
                                     */
    uint16_T LDWSAI_LnIVldRi_St;    /* '<Root>/Inport37'
                                     * Invalid state of left lane
                                     */
    uint16_T LDWSAI_VehStIvld_St;   /* '<Root>/Inport38'
                                     * Invalid state of vehicle
                                     */
    uint8_T LDWSAI_IvldStDrv_St;    /* '<Root>/Inport39'
                                     * Invalid state of driver
                                     */
    uint8_T LDWSAI_CtrlStEn_St;     /* '<Root>/Inport40'
                                     * Active state of driver control
                                     */
    uint8_T LDWSAI_StError_St;      /* '<Root>/Inport41'
                                     * Error state of vehicle system
                                     */
    uint8_T LDWSAI_CtrlStNoAvlb_St; /* '<Root>/Inport42'
                                     * Not Avaible state of vehicle system
                                     */
    uint8_T LDWSAI_PrjSpecQu_St;    /* '<Root>/Inport43' */
    boolean_T LDWSAI_DtctCstruSite_B;/* '<Root>/Inport44'
                                      * Condition of Construction site
                                      */
    real32_T LDWSAI_CycleTime_Sec;  /* '<Root>/Inport45'
                                     * Vehicle speed
                                     */
    real32_T LDWSAI_VehYawRate_rps; /* '<Root>/Inport31'
                                     * Vehicle yawrate
                                     */
    boolean_T LDWSAI_AEBActive_B;   /* '<Root>/Inport47'
                                     * AEB active status
                                     */
    boolean_T NVRAM_LDWSwitch_B;    /* '<Root>/Inport48'
                                     * NVRAM LDW switch
                                     */
    real32_T NVRAM_LDWStartupSpd_Kmph;/* '<Root>/Inport49'
                                       * NVRAM vehicle startup speed
                                       */
    real32_T LDWSAI_VehStartupSpdHMI_Kmph;/* '<Root>/Inport50'
                                       * Vehicle startup speed HMI setting
                                       */
    boolean_T LDWSAI_LDPSwitchOn_B; /* '<Root>/Inport14'
                                        * Error condition of LDW
                                        */     
    real32_T LDWSAI_LnLengthLf_Mi;
    real32_T LDWSAI_LnLengthRi_Mi;                                                                    
} sLDWSAInReq_t;

typedef struct {
    real32_T LDWSAI_VehWid_Mi;      /* '<Root>/Inport'
                                 * vehicle width
                                 */
} sLDWSAParam_t;

typedef struct {
    UINT8_T LDWC_DgrSide_St;          /* State of danger side */
    UINT8_T LDWC_RdyToTrig_B;         /* Condition of Ready to trigger state */
    UINT8_T LDWC_SysOut_St;           /* Actual state of LDW */
    real32_T LDVSE_NVRAMVehStartupSpd_kmph;/* '<S1>/LDW' */
    boolean_T LDWC_NVRAMLDWSwitch_B;/* '<S1>/LDW' */
} sLDWSAOutPro_t;

#ifndef Rte_TypeDef_sLDWSADebug_t
#define Rte_TypeDef_sLDWSADebug_t
typedef struct {
    
    real32_T LDDT_LnHeadLf_Rad;     /* '<S6>/Switch'
                                     * Heading angle of left lane
                                     */
    real32_T LDDT_RawLatVehSpdLf_Mps;/* '<S8>/Product'
                                      * Raw Left lateral vehicle speed
                                      */
    real32_T LDDT_CrvThdMaxLf_ReMi; /* '<S9>/1-D Lookup Table'
                                     * Curve of maximum threshold of left clothiod curvature
                                     */
    real32_T LDDT_CrvThdHystLf_ReMi;/* '<S9>/1-D Lookup Table1'
                                     * Curve of offset threshold of left clothiod curvature
                                     */
    real32_T LDDT_LnCltdCurvLf_ReMi;/* '<S6>/Switch4'
                                     * Clothoid curvature of left lane
                                     */
    real32_T LDDT_LatVehSpdLf_Mps;  /* '<S8>/Switch1'
                                     * Left lateral vehicle speed
                                     */
    real32_T LDVSE_MaxLatVel_Mps;   /* '<S19>/Lookup Table'
                                     * Maximum lateral velocity
                                     */
    real32_T LDDT_CrvThdMaxRi_ReMi; /* '<S10>/1-D Lookup Table'
                                     * Curve of maximum threshold of right clothiod curvature
                                     */
    real32_T LDDT_CrvThdHystRi_ReMi;/* '<S10>/1-D Lookup Table1'
                                     * Curve of offset threshold of right clothiod curvature
                                     */
    real32_T LDDT_LnCltdCurvRi_ReMi;/* '<S6>/Switch5'
                                     * Clothoid curvature of right lane
                                     */
    real32_T LDDT_LnHeadRi_Rad;     /* '<S6>/Switch1'
                                     * Heading angle of rifht lane
                                     */
    real32_T LDDT_RawLatVehSpdRi_Mps;/* '<S8>/Product1'
                                      * Raw right lateral vehicle speed
                                      */
    real32_T LDDT_LatVehSpdRi_Mps;  /* '<S8>/Switch2'
                                     * right lateral vehicle speed
                                     */
    real32_T LDVSE_MaxCrvBySpd_ReMi;/* '<S17>/Lookup Table'
                                     * Maximum curvature for LDW invalid condition
                                     */
    real32_T LDVSE_HystCrvBySpd_ReMi;/* '<S17>/Lookup Table1'
                                      * Curvature hysteresis for LDW invalid condition
                                      */
    real32_T LDDT_LnPstnRi_Mi;      /* '<S6>/Switch3'
                                     * Position of  of right lane
                                     */
    real32_T LDDT_RawDstcToLnRi_Mi; /* '<S8>/Subtract1'
                                     * Raw Distance of between vehicel and right lane
                                     */
    real32_T LDDT_DstcToLnRi_Mi;    /* '<S8>/Switch4'
                                     * Distance of between vehicel and right lane
                                     */
    real32_T LDWC_DlcThdMode2_Mi;   /* '<S92>/1-D Lookup Table2'
                                     * DLC threshold at LDW mode 2
                                     */
    real32_T LDWC_DlcThdMode3_Mi;   /* '<S92>/1-D Lookup Table1'
                                     * DLC threshold at LDW mode 3
                                     */
    real32_T LDWC_DlcThdMode1_Mi;   /* '<S92>/1-D Lookup Table'
                                     * DLC threshold at LDW mode 1
                                     */
    real32_T LDWC_CrrctByLnWidth_Fct;/* '<S92>/1-D Lookup Table3'
                                      * DLC threshold corretction factor
                                      */
    real32_T LDWC_DstcToLnTrsd_Mi;  /* '<S92>/Product'
                                     *  Threshold distance to  lane crossing
                                     */
    real32_T LDWC_DstcToLnTrsdCrvCpstnRi_Mi;/* '<S92>/1-D Lookup Table4'
                                             * Curvature compensation of threshold distance to right lane crossing
                                             */
    real32_T LDWC_DstcToLnTrsdRi_Mi;/* '<S92>/Add1'
                                     *  Threshold distance to right lane crossing
                                     */
    real32_T LDDT_TiToLnRi_Sec;     /* '<S8>/Switch6'
                                     * Time of vehicle to right lane
                                     */
    real32_T LDWC_TiToLnTrsd_Sec;   /* '<S92>/Product1'
                                     * Threshold time of time to lane crossing
                                     */
    real32_T LDDT_LnPstnLf_Mi;      /* '<S6>/Switch2'
                                     * Position of  of left lane
                                     */
    real32_T LDDT_RawDstcToLnLf_Mi; /* '<S8>/Subtract'
                                     * Raw Distance of between vehicel and left lane
                                     */
    real32_T LDDT_DstcToLnLf_Mi;    /* '<S8>/Switch3'
                                     * Distance of between vehicel and left lane
                                     */
    real32_T LDWC_DstcToLnTrsdCrvCpstnLf_Mi;/* '<S92>/1-D Lookup Table7'
                                             * Curvature compensation of threshold distance to left lane crossing
                                             */
    real32_T LDWC_DstcToLnTrsdLf_Mi;/* '<S92>/Add'
                                     *  Threshold distance to left lane crossing
                                     */
    real32_T LDDT_TiToLnLf_Sec;     /* '<S8>/Switch5'
                                     * Time of vehicle to left lane
                                     */
    uint8_T LDVSE_SidCdtnLDWLf_St;  /* '<S19>/Signal Conversion3'
                                     * State of left side at LDW
                                     */
    uint8_T LDVSE_SidCdtnLDWRi_St;  /* '<S19>/Signal Conversion4'
                                     * State of right side at LDW
                                     */
    uint8_T LDVSE_IvldLDW_St;       /* '<S17>/Signal Conversion1'
                                     * Invalid state of LDW
                                     */
    uint16_T LDWC_SuppValid_Debug;   /* '<S59>/Signal Conversion8' */
    
    boolean_T LDDT_RdyTrigLDW_B;    /* '<S6>/Equal'
                                     * Condition of ready to trigger LDW state
                                     */
    boolean_T LDDT_EnaSafety_B;     /* '<S6>/AND'
                                     * Enable flag for data from the safety interface
                                     */
    boolean_T LDDT_EnaByCstruSiteLf_B;/* '<S9>/NOT'
                                       * Enable flag for left lane validity by construction site detected
                                       */
    boolean_T LDDT_EnaByInVldQlfrLf_B;/* '<S9>/Relational Operator4'
                                       * Enable flag for left lane validity by left lane invalid qualifier
                                       */
    boolean_T LDDT_EnaByInVldQlfrSfLf_B;/* '<S9>/Relational Operator1'
                                         * Enable flag for left lane validity by left lane invalid qualifier for safety interface
                                         */
    boolean_T LDDT_LnTrigVldLf_B;   /* '<S9>/Logical Operator2'
                                     * Condition validity of left lane marker at LDW trigger
                                     */
    boolean_T LDDT_CclByInVldQlfrLf_B;/* '<S9>/Relational Operator5'
                                       * Enable flag for left lane validity by left lane invalid qualifier for safety interface when cancel the function
                                       */
    boolean_T LDDT_LnCclVldLf_B;    /* '<S9>/Logical Operator5'
                                     * Condition validity of left lane marker at LDW cancel
                                     */
    boolean_T LDDT_LnMakVldLf_B;    /* '<S9>/Switch'
                                     * Condition validity of left lane marker
                                     */
    boolean_T LDVSE_RdyTrigLDW_B;   /* '<S19>/Equal'
                                     * Ready Trigger flag for LDW
                                     */
    boolean_T LDVSE_VehLatSpdVldLf_B;/* '<S34>/Switch1'
                                      * Validity of left lateral vehicle speed
                                      */
    boolean_T LDVSE_TrnSglLf_B;     /* '<S18>/Signal Conversion3'
                                     * Condition of  left turn signal
                                     */
    boolean_T LDDT_EnaByInVldQlfrRi_B;/* '<S10>/Relational Operator4'
                                       * Enable flag for right lane validity by left lane invalid qualifier
                                       */
    boolean_T LDDT_EnaByInVldQlfrSfRi_B;/* '<S10>/Relational Operator1'
                                         * Enable flag for right lane validity by left lane invalid qualifier for safety interface
                                         */
    boolean_T LDDT_LnTrigVldRi_B;   /* '<S10>/Logical Operator2'
                                     * Condition validity of right lane marker at LDW trigger
                                     */
    boolean_T LDDT_CclByInVldQlfrRi_B;/* '<S10>/Relational Operator5'
                                       * Enable flag for right lane validity by left lane invalid qualifier for safety interface when cancel the function
                                       */
    boolean_T LDDT_LnCclVldRi_B;    /* '<S10>/Logical Operator5'
                                     * Condition validity of right lane marker at LDW cancel
                                     */
    boolean_T LDDT_LnMakVldRi_B;    /* '<S10>/Switch2'
                                     * Condition validity of right lane marker
                                     */
    boolean_T LDVSE_VehLatSpdVldRi_B;/* '<S37>/Switch1'
                                      * Validity of right lateral vehicle speed
                                      */
    boolean_T LDVSE_TrnSglRi_B;     /* '<S18>/Signal Conversion4'
                                     * Condition of  right turn signal
                                     */
    boolean_T LDWC_FnsByLatSpdLf_B; /* '<S57>/Logical Operator10' */
    boolean_T LDWC_EnaDgrSide_B;    /* '<S91>/Logical Operator6'
                                     * Enable flag for Degerous side state
                                     */
    boolean_T LDWC_RawTrigByDlcRi_B;/* '<S94>/Relational Operator3'
                                     * Raw trigger flag by DLC for right lane
                                     */
    boolean_T LDWC_EnaTlcTrigRi_B;  /* '<S94>/Relational Operator1'
                                     * Enable flag for Raw trigger flag by TLC for right lane
                                     */
    boolean_T LDWC_RawTrigByTlcRi_B;/* '<S94>/Relational Operator2'
                                     * Raw trigger flag by TLC for right lane
                                     */
    boolean_T LDWC_DlyTrigByTlcRi_B;/* '<S110>/AND' */
    boolean_T LDWC_EnaLdwTrigRi_B;  /* '<S104>/AND'
                                     * Enable flag for LDW function trigger
                                     */
    boolean_T LDWC_RstTlcTrigRi_B;  /* '<S94>/Relational Operator4'
                                     * Reset flag for Raw trigger flag by TLC for right lane
                                     */
    boolean_T LDWC_ResetForSafeRi_B;/* '<S94>/Logical Operator4'
                                     * Reset flag for the safe situation condition of right lane
                                     */
    boolean_T LDWC_SetForSafeRi_B;  /* '<S94>/Relational Operator6'
                                     * Set flag for the safe situation condition of right lane
                                     */
    boolean_T LDWC_TrigBySideCondRi_B;/* '<S94>/Relational Operator7'
                                       * LDW function trigger flag by  side condition of right lane
                                       */
    boolean_T LDWC_TrigByPrjSpecRi_B;/* '<S94>/Equal'
                                      * LDW function trigger flag by customer projects of right lane
                                      */
    boolean_T LDWC_TrigRi_B;        /* '<S94>/Logical Operator3'
                                     * Condition of right trigger
                                     */
    boolean_T LDWC_RawTrigByDlcLf_B;/* '<S93>/Relational Operator3'
                                     * Raw trigger flag by DLC for left lane
                                     */
    boolean_T LDWC_EnaTlcTrigLf_B;  /* '<S93>/Relational Operator1'
                                     * Enable flag for Raw trigger flag by TLC for left lane
                                     */
    boolean_T LDWC_RawTrigByTlcLf_B;/* '<S93>/Relational Operator2'
                                     * Raw trigger flag by TLC for left lane
                                     */
    boolean_T LDWC_DlyTrigByTlcLf_B;/* '<S103>/AND' */
    boolean_T LDWC_EnaLdwTrigLf_B;  /* '<S97>/AND'
                                     * Enable flag for LDW function trigger
                                     */
    boolean_T LDWC_RstTlcTrigLf_B;  /* '<S93>/Relational Operator4'
                                     * Reset flag for Raw trigger flag by TLC for left lane
                                     */
    boolean_T LDWC_ResetForSafeLf_B;/* '<S93>/Logical Operator4'
                                     * Reset flag for the safe situation condition of left lane
                                     */
    boolean_T LDWC_SetForSafeLf_B;  /* '<S93>/Relational Operator6'
                                     * Set flag for the safe situation condition of left lane
                                     */
    boolean_T LDWC_TrigBySideCondLf_B;/* '<S93>/Relational Operator7'
                                       * LDW function trigger flag by  side condition of left lane
                                       */
    boolean_T LDWC_TrigByPrjSpecLf_B;/* '<S93>/Equal'
                                      * LDW function trigger flag by customer projects of left lane
                                      */
    boolean_T LDWC_TrigLf_B;        /* '<S93>/Logical Operator3'
                                     * Condition of left trigger
                                     */
    boolean_T LDWC_FnsByDgrStLf_B;  /* '<S57>/Relational Operator7' */
    boolean_T LDWC_FnsByLatDistLf_B;/* '<S57>/Logical Operator8' */
    boolean_T LDWC_FnsByHeadingLf_B;/* '<S57>/Logical Operator9' */
    boolean_T LDWC_DgrFnsLf_B;      /* '<S57>/Logical Operator6' */
    boolean_T LDWC_FnsByDgrStRi_B;  /* '<S57>/Relational Operator9' */
    boolean_T LDWC_FnsByLatDistRi_B;/* '<S57>/Logical Operator14' */
    boolean_T LDWC_FnsByHeadingRi_B;/* '<S57>/Logical Operator15' */
    boolean_T LDWC_FnsByLatSpdRi_B; /* '<S57>/Logical Operator13' */
    boolean_T LDWC_DgrFnsRi_B;      /* '<S57>/Logical Operator12' */
    boolean_T LDWC_MinLdwBySysSt_B; /* '<S57>/Relational Operator8' */
    boolean_T LDWC_EdgeRiseForMinLdw_B;/* '<S74>/AND' */
    boolean_T LDWC_HoldForMinLdw_B; /* '<S90>/GreaterThan1' */
    boolean_T LDWC_FlagMinTimeLDW_B;/* '<S57>/Logical Operator11' */
    boolean_T LDWC_DgrFns_B;        /* '<S83>/AND'
                                     * Condition of danger finish
                                     */
    boolean_T LDWC_CancelBySpecific_B;/* '<S60>/Relational Operator38'
                                       * LDW cancel conditions by LDW specific bitfield
                                       */
    boolean_T LDWC_CancelByVehSt_B; /* '<S60>/Relational Operator37'
                                     * LDW cancel conditions by vehicle state
                                     */
    boolean_T LDWC_CancelByDrvSt_B; /* '<S60>/Relational Operator32'
                                     * LDW cancel conditions by drive state
                                     */
    boolean_T LDWC_CancelByCtrlSt_B;/* '<S60>/Relational Operator33'
                                     * LDW cancel conditions by active control state
                                     */
    boolean_T LDWC_CancelBySysSt_B; /* '<S60>/Relational Operator34'
                                     * LDW cancel conditions by system state
                                     */
    boolean_T LDWC_CancelByAvlSt_B; /* '<S60>/Relational Operator35'
                                     * LDW cancel conditions by no available state
                                     */
    boolean_T LDWC_CancelByPrjSpec_B;/* '<S60>/Relational Operator36'
                                      * LDW cancel conditions by customer projects
                                      */
    boolean_T LDWC_MaxDurationBySysSt_B;/* '<S64>/Relational Operator47' */
    boolean_T LDWC_EdgRiseForSysSt_B;/* '<S65>/AND' */
    boolean_T LDWC_MaxDurationByStDly_B;/* '<S64>/Logical Operator22' */
    boolean_T LDWC_TiWarmMx_B;      /* '<S64>/Logical Operator23'
                                     * Condition of warming max time
                                     */
    boolean_T LDWC_ErrSideByTrigLf_B;/* '<S63>/Logical Operator19' */
    boolean_T LDWC_ErrSideBySideCondLf_B;/* '<S63>/Relational Operator42' */
    boolean_T LDWC_ErrSidByPrjSpecLf_B;/* '<S63>/Relational Operator43' */
    boolean_T LDWC_ErrSidCdtnLf_B;  /* '<S63>/Logical Operator18'
                                     * Error condition of left side
                                     */
    boolean_T LDWC_SideCondByDgrLf_B;/* '<S63>/Relational Operator41' */
    boolean_T LDWC_CanelBySideLf_B; /* '<S63>/Logical Operator15' */
    boolean_T LDWC_SideCondByDgrRi_B;/* '<S63>/Relational Operator44' */
    boolean_T LDWC_ErrSideByTrigRi_B;/* '<S63>/Logical Operator21' */
    boolean_T LDWC_ErrSideBySideCondRi_B;/* '<S63>/Relational Operator45' */
    boolean_T LDWC_ErrSidByPrjSpecRi_B;/* '<S63>/Relational Operator46' */
    boolean_T LDWC_ErrSidCdtnRi_B;  /* '<S63>/Logical Operator2'
                                     * Error condition of right side
                                     */
    boolean_T LDWC_CanelBySideRi_B; /* '<S63>/Logical Operator1' */
    boolean_T LDWC_ErrSidCdtn_B;    /* '<S63>/Logical Operator16'
                                     * Error condition of side
                                     */
    boolean_T LDWC_CLatDevByDlcLf_B;/* '<S62>/Logical Operator24' */
    boolean_T LDWC_CLatDevByDgrLf_B;/* '<S62>/Relational Operator39' */
    boolean_T LDWC_CclLatDevLf_B;   /* '<S62>/Logical Operator12'
                                     * Cancel condition of left lane deviation
                                     */
    boolean_T LDWC_CLatDevByDlcRi_B;/* '<S62>/Relational Operator40' */
    boolean_T LDWC_CLatDevByDgrRi_B;/* '<S62>/Logical Operator25' */
    boolean_T LDWC_CclLatDevRi_B;   /* '<S62>/Logical Operator14'
                                     * Cancel condition of right lane deviation
                                     */
    boolean_T LDWC_CclLatDev_B;     /* '<S62>/Logical Operator13'
                                     * Cancel condition of lane deviation
                                     */
    boolean_T LDWC_Cancel_B;        /* '<S60>/Logical Operator11' */
    boolean_T LDWC_AbortBySpecific_B;/* '<S61>/Relational Operator2'
                                      * LDW abort conditions by LDW specific bitfield
                                      */
    boolean_T LDWC_AbortByVehSt_B;  /* '<S61>/Relational Operator1'
                                     * LDW abort conditions by vehicle state
                                     */
    boolean_T LDWC_AbortByDrvSt_B;  /* '<S61>/Relational Operator3'
                                     * LDW abort conditions by drive state
                                     */
    boolean_T LDWC_AbortByCtrlSt_B; /* '<S61>/Relational Operator4'
                                     * LDW abort conditions by active control state
                                     */
    boolean_T LDWC_AbortBySysSt_B;  /* '<S61>/Relational Operator5'
                                     * LDW abort conditions by system state
                                     */
    boolean_T LDWC_AbortByAvlSt_B;  /* '<S61>/Relational Operator6'
                                     * LDW abort conditions by no available state
                                     */
    boolean_T LDWC_AbortByPrjSpec_B;/* '<S61>/Relational Operator7'
                                     * LDW abort conditions by customer projects
                                     */
    boolean_T LDWC_Abort_B;         /* '<S61>/Logical Operator6'
                                     * Condition of LDW abort state
                                     */
    boolean_T LDWC_StrgRdyBySpecific_B;/* '<S61>/Relational Operator9'
                                        * LDW strong ready conditions by LDW specific bitfield
                                        */
    boolean_T LDWC_StrgRdyByVehSt_B;/* '<S61>/Relational Operator8'
                                     * LDW strong ready conditions by vehicle state
                                     */
    boolean_T LDWC_StrgRdyByDrvSt_B;/* '<S61>/Relational Operator10'
                                     * LDW strong ready conditions by drive state
                                     */
    boolean_T LDWC_StrgRdyByCtrlSt_B;/* '<S61>/Relational Operator11'
                                      * LDW strong ready conditions by active control state
                                      */
    boolean_T LDWC_StrgRdyBySysSt_B;/* '<S61>/Relational Operator12'
                                     * LDW strong ready conditions by system state
                                     */
    boolean_T LDWC_StrgRdyByAvlSt_B;/* '<S61>/Relational Operator13'
                                     * LDW strong ready conditions by no available state
                                     */
    boolean_T LDWC_StrgRdyByPrjSpec_B;/* '<S61>/Relational Operator14'
                                       * LDW strong ready conditions by customer projects
                                       */
    boolean_T LDWC_StrgRdy_B;       /* '<S61>/Logical Operator1'
                                     * Condition of LDW strong ready state
                                     */
    boolean_T LDWC_Degradation_B;   /* '<S56>/Logical Operator1' */
    boolean_T LDWC_DegradationEdgeRise_B;/* '<S72>/AND' */
    boolean_T LDWC_Degr_B;          /* '<S56>/Logical Operator2'
                                     * Condition of degradation
                                     */
    boolean_T LDWC_Trig_B;          /* '<S91>/Logical Operator1'
                                     * Condition of trigger
                                     */
    boolean_T LDWC_SuppBySpecific_B;/* '<S61>/Relational Operator21'
                                     * LDW suppresion conditions by LDW specific bitfield
                                     */
    boolean_T LDWC_SuppByVehSt_B;   /* '<S61>/Relational Operator20'
                                     * LDW suppresion conditions by vehicle state
                                     */
    boolean_T LDWC_SuppByDrvSt_B;   /* '<S61>/Relational Operator15'
                                     * LDW suppresion conditions by drive state
                                     */
    boolean_T LDWC_SuppByCtrlSt_B;  /* '<S61>/Relational Operator16'
                                     * LDW suppresion conditions by active control state
                                     */
    boolean_T LDWC_SuppBySysSt_B;   /* '<S61>/Relational Operator17'
                                     * LDW suppresion conditions by system state
                                     */
    boolean_T LDWC_SuppyByAvlSt_B;  /* '<S61>/Relational Operator18'
                                     * LDW suppresion conditions by no available state
                                     */
    boolean_T LDWC_SuppPrjSpec_B;   /* '<S61>/Relational Operator19'
                                     * LDW suppresion conditions by customer projects
                                     */
    boolean_T LDWC_Suppresion_B;    /* '<S61>/Logical Operator3' */
    boolean_T LDWC_WeakRdyBySpecific_B;/* '<S61>/Relational Operator28'
                                        * LDW weak ready conditions by LDW specific bitfield
                                        */
    boolean_T LDWC_WeakRdyByVehSt_B;/* '<S61>/Relational Operator27'
                                     * LDW weak ready conditions by vehicle state
                                     */
    boolean_T LDWC_WeakRdyByDrvSt_B;/* '<S61>/Relational Operator22'
                                     * LDW weak ready conditions by drive state
                                     */
    boolean_T LDWC_WeakRdyByCtrlSt_B;/* '<S61>/Relational Operator23'
                                      * LDW strong weak conditions by active control state
                                      */
    boolean_T LDWC_WeakRdyBySysSt_B;/* '<S61>/Relational Operator24'
                                     * LDW weak ready conditions by system state
                                     */
    boolean_T LDWC_WeakRdyByAvlSt_B;/* '<S61>/Relational Operator25'
                                     * LDW weak weak conditions by no available state
                                     */
    boolean_T LDWC_WeakRdyByPrjSpec_B;/* '<S61>/Relational Operator26'
                                       * LDW weak weak conditions by customer projects
                                       */
    boolean_T LDWC_BlockTimeBySysOut_B;/* '<S61>/Logical Operator9' */
    boolean_T LDWC_RawBlockTimeByRampOut_B;/* '<S68>/AND' */
    boolean_T LDWC_BlockTimeByRampOut_B;/* '<S71>/GreaterThan1' */
    boolean_T LDWC_BlockTime_B;     /* '<S61>/Logical Operator10' */
    boolean_T LDWC_WkRdy_B;         /* '<S61>/Logical Operator4'
                                     * Condition of LDW weak ready state
                                     */
    boolean_T LDWC_Suppression_B;   /* '<S59>/OR'
                                     * Suppresion condition
                                     */
    
    real32_T LDVSE_HodTiTrnSglLf_Sec;/* '<S32>/Unit Delay'
                                      * Holding time of left turn signal
                                      */
    real32_T LDVSE_HodTiTrnSglRi_Sec;/* '<S33>/Unit Delay'
                                      * Holding time of right turn signal
                                      */
    real32_T LDWC_DlyTiOfTiToLnRiMn_Sec;/* '<S110>/Unit Delay'
                                         * Delay time of time to right lane crossing
                                         */
    real32_T LDWC_HdTiTrigRi_Sec;   /* '<S109>/Unit Delay'
                                     * holding time right trigger
                                     */
    real32_T LDWC_DlyTiOfTiToLnLfMn_Sec;/* '<S103>/Unit Delay'
                                         * Delay time of time to left lane crossing
                                         */
    real32_T LDWC_HdTiTrigLf_Sec;   /* '<S102>/Unit Delay'
                                     *  holding time left trigger
                                     */
    real32_T LDWC_HdTiWarming_Sec;  /* '<S90>/Unit Delay'
                                     * Holding time of warming state start
                                     */
    real32_T LDWC_DlyTiTgtFns_Sec;  /* '<S83>/Unit Delay'
                                     * Delay time of LDW finish state
                                     */
    real32_T LDWC_HdTiWarmMx_Sec;   /* '<S67>/Unit Delay'
                                     * Holding time of warming state start
                                     */
    real32_T LDWC_HdTiDegr_Sec;     /* '<S73>/Unit Delay'
                                     * Holding time of degradation
                                     */
    real32_T LDWC_HdTiFns_Sec;      /* '<S71>/Unit Delay'
                                     * Holding time of finish state end
                                     */
    uint8_T LDWC_DgrSideOld_St;     /* '<S91>/UnitDelay1'
                                     * Old state of danger side
                                     */
    boolean_T LDDT_UHysCltdCurvVldLf_B;/* '<S12>/Unit Delay'
                                        * Validity of left lane Clothoid curvature
                                        */
    boolean_T LDDT_BHysHeadAglTrigVldLf_B;/* '<S11>/Unit Delay'
                                           * Valid trigger of left heading angle
                                           */
    boolean_T LDDT_UHysHeadAglCclVldLf_B;/* '<S13>/Unit Delay'
                                          * Valid cancel of left heading angle
                                          */
    boolean_T LDVSE_BHysLatVehSpdVldLf_B;/* '<S38>/Unit Delay'
                                          * Validity of left lateral vehicle speed before trigger state
                                          */
    boolean_T LDVSE_UHysLatVehSpdVldLf_B;/* '<S39>/Unit Delay'
                                          * Validity of left lateral vehicle speed after trigger state
                                          */
    boolean_T LDVSE_EdgeRisTrnSglRi_B;/* '<S30>/Unit Delay'
                                       * Edge rise of right turn signal
                                       */
    boolean_T LDDT_UHysCltdCurvVldRi_B;/* '<S15>/Unit Delay'
                                        * Validity of right lane Clothoid curvature
                                        */
    boolean_T LDDT_BHysHeadAglTrigVldRi_B;/* '<S14>/Unit Delay'
                                           * Valid trigger of right heading angle
                                           */
    boolean_T LDDT_UHysHeadAglCclVldRi_B;/* '<S16>/Unit Delay'
                                          * Valid cancel of right heading angle
                                          */
    boolean_T LDVSE_BHysLatVehSpdVldRi_B;/* '<S42>/Unit Delay'
                                          * Validity of right lateral vehicle speed before trigger state
                                          */
    boolean_T LDVSE_UHysLatVehSpdVldRi_B;/* '<S43>/Unit Delay'
                                          * Validity of right lateral vehicle speed after trigger state
                                          */
    boolean_T LDVSE_EdgeRisTrnSglLf_B;/* '<S31>/Unit Delay'
                                       * Edge rise of left turn signal
                                       */
    boolean_T LDVSE_UHysSteAgl_B;   /* '<S24>/Unit Delay'
                                     * Validity of steering wheel angle
                                     */
    boolean_T LDVSE_BHysSpdVeh_B;   /* '<S20>/Unit Delay'
                                     * Validity of displayed longitudinal speed
                                     */
    boolean_T LDVSE_UHysSteAglSpd_B;/* '<S25>/Unit Delay'
                                     * Validity of steering wheel angle speed
                                     */
    boolean_T LDVSE_BHysAccVehX_B;  /* '<S21>/Unit Delay'
                                     * Validity of  longitudinal Acceleration
                                     */
    boolean_T LDVSE_BHysAccVehY_B;  /* '<S26>/Unit Delay'
                                     * Validity of  lateral Acceleration
                                     */
    boolean_T LDVSE_UHysVehCurv_B;  /* '<S27>/Unit Delay'
                                     * Validity of  vehicle curvature
                                     */
    boolean_T LDVSE_BHysLnWid_B;    /* '<S22>/Unit Delay'
                                     * Validity of lane width
                                     */
    boolean_T LDWC_HdTiTrigRiEn_B;  /* '<S104>/Unit Delay'
                                     * Enable condition of holding time right trigger
                                     */
    boolean_T LDWC_DisTrigRi_B;     /* '<S108>/Unit Delay'
                                     * Disable condition of right trigger
                                     */
    boolean_T LDWC_HdTiTrigLfEn_B;  /* '<S97>/Unit Delay'
                                     * Enable condition of holding time left trigger
                                     */
    boolean_T LDWC_DisTrigLf_B;     /* '<S101>/Unit Delay'
                                     * Disable condition of left trigger
                                     */
    boolean_T LDWC_EdgeRisWarming_B;/* '<S74>/Unit Delay'
                                     * Edge rise of warming state
                                     */
    boolean_T LDWC_EdgeRisWarmMx_B; /* '<S65>/Unit Delay'
                                     * Edge rise of warming state
                                     */
    boolean_T LDWC_EdgeRisDegr_B;   /* '<S72>/Unit Delay'
                                     * Edge rise of degradation
                                     */
    boolean_T LDWC_DegrOld_B;       /* '<S56>/UnitDelay'
                                     * UnitDelay condition of degradation
                                     */
    boolean_T LDWC_EdgeRisFns_B;    /* '<S68>/Unit Delay'
                                        * Edge rise of fginish and cancel state
                                        */                                 
} sLDWSADebug_t;
#endif
extern void LCF_LDWSA_Reset(void);

extern void LCF_LDWSA_Exec(const sLDWSAInReq_t* pLDWSAInput,
                           const sLDWSAParam_t* pLDWSAParam,
                           sLDWSAOutPro_t* pLDWSAOutput,
                           sLDWSADebug_t* pLDWSADebug);
#endif
