#ifndef VSDP_EXT_TYPE_H
#define VSDP_EXT_TYPE_H
#include "tue_common_libs.h"
#include "rtwtypes.h"

typedef struct {
    UINT8_T VSDPI_SysWarn_St;      /* State of lateral Adas system */
    REAL32_T VSDPI_AccPedPstn_Per; /* Acceleration pedal position */
    UINT8_T VSDPI_CtrlStAvlbABS_B; /* Condition of ABS is available */
    UINT8_T VSDPI_CtrlStAvlbACC_B; /* Condition of ACC is available */
    UINT8_T VSDPI_CtrlStAvlbEBA_B; /* Condition of EBA is available */
    UINT8_T VSDPI_CtrlStAvlbESC_B; /* Condition of ESC is available */
    UINT8_T VSDPI_CtrlStAvlbTCS_B; /* Condition of TSC is available */
    UINT8_T VSDPI_CtrlStAvlbVSM_B; /* Condition of VSM is available */
    UINT8_T VSDPI_CtrlStEnABS_B; /* Condition of ABS in actively controlling */
    UINT8_T VSDPI_CtrlStEnACC_B; /* Condition of ACC in actively controlling */
    UINT8_T VSDPI_CtrlStEnEBA_B; /* Condition of EBA in actively controlling  */
    UINT8_T VSDPI_CtrlStEnESC_B; /* Condition of ESC in actively controlling */
    UINT8_T VSDPI_CtrlStEnTCS_B; /* Condition of TCS in actively controlling */
    UINT8_T VSDPI_CtrlStEnVSM_B; /* Condition of VSM in actively controlling */
    UINT8_T VSDPI_DoorOpen_B;    /* Condition of the door is opened */
    UINT8_T VSDPI_DrvNoBuckledUp_B; /* Condition of the driver is not buckled */
    UINT8_T
    VSDPI_DtctOverSte_B; /* Condition of over steering has been detected */
    UINT8_T
    VSDPI_DtctRollerBench_B;      /* Condition of vehicle is on roller bench */
    UINT8_T VSDPI_DtctUnderSte_B; /* Condition of under steering has been
                                     detected */
    UINT8_T VSDPI_GrNeu_B;        /* Condition of the neutral gear is engaged */
    UINT8_T VSDPI_GrPark_B;       /* Condition of the parking gear is engaged */
    UINT8_T VSDPI_GrRvs_B;        /* Condition of the reverse gear is engaged */
    REAL32_T VSDPI_ManuActuTrqEPS_Nm; /* Actual manual torque of EPS */
    UINT8_T VSDPI_StBrightness_St;    /* Bright mess state */
    UINT8_T VSDPI_StErrABS_B;         /* Condition of ABS is in error state */
    UINT8_T VSDPI_StErrACC_B;         /* Condition of ACC is in error state */
    UINT8_T VSDPI_StErrEBA_B;         /* Condition of EBA is in error state */
    UINT8_T VSDPI_StErrESC_B;         /* Condition of ESC is in error state */
    UINT8_T VSDPI_StErrLatDMC_B;    /* Condition of LatDMC is in error state */
    UINT8_T VSDPI_StErrTSC_B;       /* Condition of TSC is in error state */
    UINT8_T VSDPI_StErrVDY_B;       /* Condition of VDY is in error state */
    UINT8_T VSDPI_StErrVSM_B;       /* Condition of VSM is in error state */
    UINT8_T VSDPI_StageWiper_St;    /* Wiper state */
    UINT8_T VSDPI_StateWiper_St;    /* Wiper State (From Conti) */
    REAL32_T VSDPI_SteAglFrt_Rad;   /* Effective steering angle at front axle */
    UINT8_T VSDPI_TrailerExs_B;     /* Condition of trailer is attached */
    UINT8_T VSDPI_TrnSglEnLf_B;     /* Condition of left turn signal is on */
    UINT8_T VSDPI_TrnSglEnRi_B;     /* Condition of right turn signal is on */
    UINT8_T VSDPI_TrnSglHarLigEn_B; /* Condition of turn hazard light is on */
    UINT8_T VSDPI_VehMoveBkwd_B;    /* Condition of vehicles moves backward */
    UINT8_T VSDPI_VehRdyToSta_B;    /* Condition of vehicle is ready to start */
    REAL32_T VSDPI_VehSpdX_Mps;     /* Vehicle Speed */
    REAL32_T VSDPI_CycleTime_Sec;   /* VSDP cycle time */
    boolean_T VSDPI_BrakePedalApplied_B; /*the brake pedal applied flag*/
    boolean_T VSDPI_CtrlStEnARP_B;
    boolean_T VSDPI_CtrlStEnHDC_B;
    boolean_T VSDPI_StErrARP_B;
    boolean_T VSDPI_StErrHDC_B;
    boolean_T VSDPI_BrakeDiscTempSts_B;
    boolean_T VSDPI_SignalInvalidLongi_B;
    boolean_T VSDPI_SignalInvalidLat_B;
} sVSDPInput_t;

typedef struct {
    REAL32_T Sys_VehWid_Mi; /* Not use in function, to make the parameter
                               structure exist */
} sVSDPParam_t;

typedef struct {
    UINT8_T VSDP_CtrlStEn_St; /* State of control feature is enable */
                              /* bit:0 ABS in actively controlling  */
                              /* bit:1 ACC in actively controlling  */
                              /* bit:2 ESC in actively controlling  */
                              /* bit:3 TCS in actively controlling  */
                              /* bit:4 VSM in actively controlling  */
                              /* bit:5 EBA in actively controlling  */
    UINT8_T
    VSDP_CtrlStNoAvlb_St;      /* State of control feature is not available */
                               /* bit:0 ABS is not available  */
                               /* bit:1 ACC is not  available */
                               /* bit:2 ESC is not  available */
                               /* bit:3 TSC is not  available */
                               /* bit:4 VSM is not  available */
                               /* bit:5 EBA is not  available */
    UINT8_T VSDP_IvldStDrv_St; /* Invalid state of driver */
                               /* bit:0 AccelPedal rate is invalid    */
                               /* bit:1 Driver is not buckled up      */
                               /* bit:2 Turn signal is harzard        */
                               /* bit:3 Turn signal is left or right  */
                               /* bit:4 Drive hand off                */
                               /* bit:5 Manual torque override        */
    UINT8_T VSDP_StError_St;   /* error state of system */
    /* bit:0 Condition of ABS is in error state      */
    /* bit:1 Condition of ACC is in error state      */
    /* bit:2 Condition of ESC is in error state      */
    /* bit:3 Condition of TSC is in error state      */
    /* bit:4 Condition of VSM is in error state      */
    /* bit:5 Condition of VDY is in error state      */
    /* bit:6 Condition of LatDMC is in error state   */
    /* bit:7 Condition of EBA is in error state      */
    UINT16_T VSDP_VehStIvld_St; /* Invalid state of vehicle system */
                                /* bit:0  Door open                  */
                                /* bit:1  Vehicle not ready to start */
                                /* bit:2  Trailer present            */
                                /* bit:3  Invalid gear engaged       */
                                /* bit:4  Vehicles moves backward    */
                                /* bit:5  Oversteering               */
                                /* bit:6  Understeering              */
                                /* bit:7  Vehicle is on roller bench */
                                /* bit:8  Out Limit Steer Angel      */
                                /* bit:9  Wiper Continuously Active  */
                                /* bit:10 No daytime                 */
} sVSDPOutput_t;

#ifndef Rte_TypeDef_sVSDPDebug_t
#define Rte_TypeDef_sVSDPDebug_t
typedef struct {
    UINT8_T VSDP_FFDtctSteAglStop_B; /* FF of steer angle is out range */
    UINT8_T VSDP_FFGrNoEnga_B;       /* FF of invalid engaged gear  */
    UINT8_T VSDP_FFStErrABS_B;       /* FF of ABS error condition */
    UINT8_T VSDP_FFStErrACC_B;       /* FF of ACC error condition */
    UINT8_T VSDP_FFStErrEBA_B;       /* FF of EBA error condition */
    UINT8_T VSDP_FFStErrESC_B;       /* FF of ESC error condition */
    UINT8_T VSDP_FFStErrLatDMC_B;    /* FF of LatDMC error condition */
    UINT8_T VSDP_FFStErrTSC_B;       /* FF of TSC error condition */
    UINT8_T VSDP_FFStErrVDY_B;       /* FF of VDY error condition */
    UINT8_T VSDP_FFStErrVSM_B;       /* FF of VSM error condition */
    REAL32_T
    VSDP_FalDlyTiAccPedPstnRate_Sec;       /* Fall delay time of acceleration
                                              pedal position rate is out range */
    REAL32_T VSDP_FalDlyTiCtrlStEnABS_Sec; /* Condition of ABS in actively
                                              controlling */
    REAL32_T VSDP_FalDlyTiCtrlStEnACC_Sec; /* Condition of ACC in actively
                                              controlling */
    REAL32_T VSDP_FalDlyTiCtrlStEnEBA_Sec; /* Condition of EBA in actively
                                              controlling  */
    REAL32_T VSDP_FalDlyTiCtrlStEnESC_Sec; /* Condition of ESC in actively
                                              controlling */
    REAL32_T VSDP_FalDlyTiCtrlStEnTCS_Sec; /* Condition of TSC in actively
                                              controlling */
    REAL32_T VSDP_FalDlyTiCtrlStEnVSM_Sec; /* Condition of VSM in actively
                                              controlling */
    REAL32_T VSDP_FalDlyTiNoDaytimeTrnOff_C_Sec; /* Fall edge of Rise  delay
                                                    time for no daytime is
                                                    active */
    REAL32_T VSDP_FalDlyTiStErrABS_C_Sec; /* Fall edge of rise delay time for
                                             ABS error condition  */
    REAL32_T VSDP_FalDlyTiStErrACC_C_Sec; /* Fall edge of rise delay time for
                                             ACC error condition  */
    REAL32_T VSDP_FalDlyTiStErrEBA_C_Sec; /* Fall edge of rise delay time for
                                             EBA error condition  */
    REAL32_T VSDP_FalDlyTiStErrESC_C_Sec; /* Fall edge of rise delay time for
                                             ESC error condition  */
    REAL32_T
    VSDP_FalDlyTiStErrLatDMC_C_Sec;       /* Fall edge of rise delay time for
                                             LatDMC error condition  */
    REAL32_T VSDP_FalDlyTiStErrTSC_C_Sec; /* Fall edge of rise delay time for
                                             TSC error condition  */
    REAL32_T VSDP_FalDlyTiStErrVDY_C_Sec; /* Fall edge of rise delay time for
                                             VDY error condition  */
    REAL32_T VSDP_FalDlyTiStErrVSM_C_Sec; /* Fall edge of rise delay time for
                                             VSM error condition  */
    REAL32_T
    VSDP_FalDlyTiTrnSglEn_Sec; /* Fall delay time of turn light is active */
    REAL32_T
    VSDP_FalDlyTiTrnSglHarLigEn_Sec; /* Fall delay time of hazard light is
                                        active */
    REAL32_T VSDP_FalDlyTiWiperEvtGapTiMx_C_Sec; /* Fall edge of rise  delay
                                                    time for wiper state is
                                                    active */
    REAL32_T VSDP_RisDlyTiManuActuTrq_Sec; /* Fall delay time of manual torque
                                              for EPS is out range */
    REAL32_T
    VSDP_RisDlyTiNoDaytimeMn_Sec; /* Rise  delay time for no daytime is
                                     active */
    REAL32_T
    VSDP_RisDlyTiStErrABS_Sec; /* Rise delay time for ABS error condition */
    REAL32_T
    VSDP_RisDlyTiStErrACC_Sec; /* Rise delay time for ACC error condition */
    REAL32_T
    VSDP_RisDlyTiStErrEBA_Sec; /* Rise delay time for EBA error condition */
    REAL32_T
    VSDP_RisDlyTiStErrESC_Sec; /* Rise delay time for ESC error condition */
    REAL32_T VSDP_RisDlyTiStErrLatDMC_Sec; /* Rise delay time for LatDMC error
                                              condition  */
    REAL32_T
    VSDP_RisDlyTiStErrTSC_Sec; /* Rise delay time for TSC error condition */
    REAL32_T
    VSDP_RisDlyTiStErrVDY_Sec; /* Rise delay time for VDY error condition */
    REAL32_T
    VSDP_RisDlyTiStErrVSM_Sec; /* Rise delay time for VSM error condition */
    REAL32_T VSDP_RisDlyTiWiperContiTiMn_Sec; /* continue time for wiper state
                                                 is active */
    REAL32_T VSDP_RisDlyTiWiperEnTiMn_Sec; /* Rise  delay time for wiper state
                                              is active */
    UINT8_T VSDP_RisEdgeStErrABS_B;    /* Rise edge of rise delay time for ABS
                                          error condition  */
    UINT8_T VSDP_RisEdgeStErrACC_B;    /* Rise edge of rise delay time for ACC
                                          error condition  */
    UINT8_T VSDP_RisEdgeStErrEBA_B;    /* Rise edge of rise delay time for EBA
                                          error condition  */
    UINT8_T VSDP_RisEdgeStErrESC_B;    /* Rise edge of rise delay time for ESC
                                          error condition  */
    UINT8_T VSDP_RisEdgeStErrLatDMC_B; /* Rise edge of rise delay time for
                                          LatDMC error condition  */
    UINT8_T VSDP_RisEdgeStErrTSC_B;    /* Rise edge of rise delay time for TSC
                                          error condition  */
    UINT8_T VSDP_RisEdgeStErrVDY_B;    /* Rise edge of rise delay time for VDY
                                          error condition  */
    UINT8_T VSDP_RisEdgeStErrVSM_B;    /* Rise edge of rise delay time for VSM
                                          error condition  */
    REAL32_T VSDP_TiTrigStErrABS_Sec; /* Time trigger of ABS error condition  */
    REAL32_T VSDP_TiTrigStErrACC_Sec; /* Time trigger of ACC error condition  */
    REAL32_T
    VSDP_TiTrigStErrLatDMC_Sec; /* Time trigger of LatDMC error condition */
    REAL32_T VSDP_TiTrigStErrTSC_Sec; /* Time trigger of TSC error condition  */
    REAL32_T VSDP_TiTrigStErrVDY_Sec; /* Time trigger of VDY error condition  */
    REAL32_T VSDP_TiTrigStErrVSM_Sec; /* Time trigger of VSM error condition  */
    REAL32_T
    VSDP_UstpAccPedPstn_Per; /* Unit step of acceleration pedal position */
} sVSDPDebug_t;
#endif

extern void LCF_VSDP_Reset(void);

extern void LCF_VSDP_Exec(const sVSDPInput_t* pVSDPInput,
                          const sVSDPParam_t* pVSDPParam,
                          sVSDPOutput_t* pVSDPOutput,
                          sVSDPDebug_t* pVSDPDebug);
#endif
