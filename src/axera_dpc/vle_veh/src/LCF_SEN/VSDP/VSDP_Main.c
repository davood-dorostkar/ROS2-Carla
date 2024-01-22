/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
#include "VSDP_Ext.h"
#include "VSDP.h"

#define ASW_QM_CORE2_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
UINT8_T VSDPI_SysWarn_St;       /* State of lateral Adas system */
REAL32_T VSDPI_AccPedPstn_Per;  /* Acceleration pedal position */
UINT8_T VSDPI_CtrlStAvlbABS_B;  /* Condition of ABS is available */
UINT8_T VSDPI_CtrlStAvlbACC_B;  /* Condition of ACC is available */
UINT8_T VSDPI_CtrlStAvlbEBA_B;  /* Condition of EBA is available */
UINT8_T VSDPI_CtrlStAvlbESC_B;  /* Condition of ESC is available */
UINT8_T VSDPI_CtrlStAvlbTCS_B;  /* Condition of TSC is available */
UINT8_T VSDPI_CtrlStAvlbVSM_B;  /* Condition of VSM is available */
UINT8_T VSDPI_CtrlStEnABS_B;    /* Condition of ABS in actively controlling */
UINT8_T VSDPI_CtrlStEnACC_B;    /* Condition of ACC in actively controlling */
UINT8_T VSDPI_CtrlStEnEBA_B;    /* Condition of EBA in actively controlling  */
UINT8_T VSDPI_CtrlStEnESC_B;    /* Condition of ESC in actively controlling */
UINT8_T VSDPI_CtrlStEnTCS_B;    /* Condition of TSC in actively controlling */
UINT8_T VSDPI_CtrlStEnVSM_B;    /* Condition of VSM in actively controlling */
UINT8_T VSDPI_DoorOpen_B;       /* Condition of the door is opened */
UINT8_T VSDPI_DrvNoBuckledUp_B; /* Condition of the driver is not buckled */
UINT8_T VSDPI_DtctOverSte_B; /* Condition of over steering has been detected */
UINT8_T VSDPI_DtctRollerBench_B; /* Condition of vehicle is on roller bench */
UINT8_T
VSDPI_DtctUnderSte_B;   /* Condition of under steering has been detected */
UINT8_T VSDPI_GrNeu_B;  /* Condition of the neutral gear is engaged */
UINT8_T VSDPI_GrPark_B; /* Condition of the parking gear is engaged */
UINT8_T VSDPI_GrRvs_B;  /* Condition of the reverse gear is engaged */
REAL32_T VSDPI_ManuActuTrqEPS_Nm; /* Actual manual torque of EPS */
UINT8_T VSDPI_StBrightness_St;    /* Bright mess state */
UINT8_T VSDPI_StErrABS_B;         /* Condition of ABS is in error state */
UINT8_T VSDPI_StErrACC_B;         /* Condition of ACC is in error state */
UINT8_T VSDPI_StErrEBA_B;         /* Condition of EBA is in error state */
UINT8_T VSDPI_StErrESC_B;         /* Condition of ESC is in error state */
UINT8_T VSDPI_StErrLatDMC_B;      /* Condition of LatDMC is in error state */
UINT8_T VSDPI_StErrTSC_B;         /* Condition of TSC is in error state */
UINT8_T VSDPI_StErrVDY_B;         /* Condition of VDY is in error state */
UINT8_T VSDPI_StErrVSM_B;         /* Condition of VSM is in error state */
UINT8_T VSDPI_StageWiper_St;      /* Wiper state */
UINT8_T VSDPI_StateWiper_St;      /* Wiper State(From Conti) */
REAL32_T VSDPI_SteAglFrt_Rad;     /* Effective steering angle at front axle */
UINT8_T VSDPI_TrailerExs_B;       /* Condition of trailer is attached */
UINT8_T VSDPI_TrnSglEnLf_B;       /* Condition of left turn signal is on */
UINT8_T VSDPI_TrnSglEnRi_B;       /* Condition of right turn signal is on */
UINT8_T VSDPI_TrnSglHarLigEn_B;   /* Condition of turn hazard light is on */
UINT8_T VSDPI_VehMoveBkwd_B;      /* Condition of vehicles moves backward */
UINT8_T VSDPI_VehRdyToSta_B;      /* Condition of vehicle is ready to start */
REAL32_T VSDPI_VehSpdX_Mps;       /* Vehicle Speed */
REAL32_T VSDPI_CycleTime_Sec;     /* VSDP cycle time */
boolean_T VSDPI_BrakePedalApplied_B; /*the brake pedal applied from IDB*/
boolean_T VSDPI_CtrlStEnARP_B;
boolean_T VSDPI_CtrlStEnHDC_B;
boolean_T VSDPI_StErrARP_B;
boolean_T VSDPI_StErrHDC_B;
boolean_T VSDPI_BrakeDiscTempSts_B;
boolean_T VSDPI_SignalInvalidLongi_B;
boolean_T VSDPI_SignalInvalidLat_B;
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"

extern void LCF_VSDP_Reset(void) { VSDP_initialize(); }

void LCF_VSDP_Exec(const sVSDPInput_t* pVSDPInput,
                   const sVSDPParam_t* pVSDPParam,
                   sVSDPOutput_t* pVSDPOutput,
                   sVSDPDebug_t* pVSDPDebug) {
    // printf("--------------------VSDP--------------------\n");
    /**************************************VSDP
     * input***********************************************************/
    VSDPI_SysWarn_St = pVSDPInput->VSDPI_SysWarn_St;
    VSDPI_AccPedPstn_Per = pVSDPInput->VSDPI_AccPedPstn_Per;
    VSDPI_CtrlStAvlbABS_B = pVSDPInput->VSDPI_CtrlStAvlbABS_B;
    VSDPI_CtrlStAvlbACC_B = pVSDPInput->VSDPI_CtrlStAvlbACC_B;
    VSDPI_CtrlStAvlbEBA_B = pVSDPInput->VSDPI_CtrlStAvlbEBA_B;
    VSDPI_CtrlStAvlbESC_B = pVSDPInput->VSDPI_CtrlStAvlbESC_B;
    VSDPI_CtrlStAvlbTCS_B = pVSDPInput->VSDPI_CtrlStAvlbTCS_B;
    VSDPI_CtrlStAvlbVSM_B = pVSDPInput->VSDPI_CtrlStAvlbVSM_B;
    VSDPI_CtrlStEnABS_B = pVSDPInput->VSDPI_CtrlStEnABS_B;
    VSDPI_CtrlStEnACC_B = pVSDPInput->VSDPI_CtrlStEnACC_B;
    VSDPI_CtrlStEnEBA_B = pVSDPInput->VSDPI_CtrlStEnEBA_B;
    VSDPI_CtrlStEnESC_B = pVSDPInput->VSDPI_CtrlStEnESC_B;
    VSDPI_CtrlStEnTCS_B = pVSDPInput->VSDPI_CtrlStEnTCS_B;
    VSDPI_CtrlStEnVSM_B = pVSDPInput->VSDPI_CtrlStEnVSM_B;
    VSDPI_DoorOpen_B = pVSDPInput->VSDPI_DoorOpen_B;
    VSDPI_DrvNoBuckledUp_B = pVSDPInput->VSDPI_DrvNoBuckledUp_B;
    VSDPI_DtctOverSte_B = pVSDPInput->VSDPI_DtctOverSte_B;
    VSDPI_DtctRollerBench_B = pVSDPInput->VSDPI_DtctRollerBench_B;
    VSDPI_DtctUnderSte_B = pVSDPInput->VSDPI_DtctUnderSte_B;
    VSDPI_GrNeu_B = pVSDPInput->VSDPI_GrNeu_B;
    VSDPI_GrPark_B = pVSDPInput->VSDPI_GrPark_B;
    VSDPI_GrRvs_B = pVSDPInput->VSDPI_GrRvs_B;
    VSDPI_ManuActuTrqEPS_Nm = pVSDPInput->VSDPI_ManuActuTrqEPS_Nm;
    VSDPI_StBrightness_St = pVSDPInput->VSDPI_StBrightness_St;
    VSDPI_StErrABS_B = pVSDPInput->VSDPI_StErrABS_B;
    VSDPI_StErrACC_B = pVSDPInput->VSDPI_StErrACC_B;
    VSDPI_StErrEBA_B = pVSDPInput->VSDPI_StErrEBA_B;
    VSDPI_StErrESC_B = pVSDPInput->VSDPI_StErrESC_B;
    VSDPI_StErrLatDMC_B = pVSDPInput->VSDPI_StErrLatDMC_B;
    VSDPI_StErrTSC_B = pVSDPInput->VSDPI_StErrTSC_B;
    VSDPI_StErrVDY_B = pVSDPInput->VSDPI_StErrVDY_B;
    VSDPI_StErrVSM_B = pVSDPInput->VSDPI_StErrVSM_B;
    VSDPI_StageWiper_St = pVSDPInput->VSDPI_StageWiper_St;
    VSDPI_StateWiper_St = pVSDPInput->VSDPI_StateWiper_St;
    VSDPI_SteAglFrt_Rad = pVSDPInput->VSDPI_SteAglFrt_Rad;
    VSDPI_TrailerExs_B = pVSDPInput->VSDPI_TrailerExs_B;
    VSDPI_TrnSglEnLf_B = pVSDPInput->VSDPI_TrnSglEnLf_B;
    VSDPI_TrnSglEnRi_B = pVSDPInput->VSDPI_TrnSglEnRi_B;
    VSDPI_TrnSglHarLigEn_B = pVSDPInput->VSDPI_TrnSglHarLigEn_B;
    VSDPI_VehMoveBkwd_B = pVSDPInput->VSDPI_VehMoveBkwd_B;
    VSDPI_VehRdyToSta_B = pVSDPInput->VSDPI_VehRdyToSta_B;
    VSDPI_VehSpdX_Mps = pVSDPInput->VSDPI_VehSpdX_Mps;
    VSDPI_CycleTime_Sec = pVSDPInput->VSDPI_CycleTime_Sec;
    VSDPI_BrakePedalApplied_B = pVSDPInput->VSDPI_BrakePedalApplied_B;
    VSDPI_CtrlStEnARP_B = pVSDPInput->VSDPI_CtrlStEnARP_B;
    VSDPI_CtrlStEnHDC_B = pVSDPInput->VSDPI_CtrlStEnHDC_B;
    VSDPI_StErrARP_B = pVSDPInput->VSDPI_StErrARP_B;
    VSDPI_StErrHDC_B = pVSDPInput->VSDPI_StErrHDC_B;
    VSDPI_BrakeDiscTempSts_B = pVSDPInput->VSDPI_BrakeDiscTempSts_B;
    VSDPI_SignalInvalidLongi_B = pVSDPInput->VSDPI_SignalInvalidLongi_B;
    VSDPI_SignalInvalidLat_B = pVSDPInput->VSDPI_SignalInvalidLat_B;
    /************************************VSDP
     * function*************************************************************/
    VSDP_step();

    /************************************LDPSA
     * output*************************************************************/
    pVSDPOutput->VSDP_CtrlStEn_St = VSDP_CtrlStEn_St;
    pVSDPOutput->VSDP_CtrlStNoAvlb_St = VSDP_CtrlStNoAvlb_St;
    pVSDPOutput->VSDP_IvldStDrv_St = VSDP_IvldStDrv_St;
    pVSDPOutput->VSDP_StError_St = VSDP_StError_St;
    pVSDPOutput->VSDP_VehStIvld_St = VSDP_VehStIvld_St;

    /************************************LDPSA
     * debug*************************************************************/
    pVSDPDebug->VSDP_FFDtctSteAglStop_B = VSDP_FFDtctSteAglStop_B;
    pVSDPDebug->VSDP_FFGrNoEnga_B = VSDP_FFGrNoEnga_B;
    pVSDPDebug->VSDP_FFStErrABS_B = VSDP_FFStErrABS_B;
    pVSDPDebug->VSDP_FFStErrACC_B = VSDP_FFStErrACC_B;
    pVSDPDebug->VSDP_FFStErrEBA_B = VSDP_FFStErrEBA_B;
    pVSDPDebug->VSDP_FFStErrESC_B = VSDP_FFStErrESC_B;
    pVSDPDebug->VSDP_FFStErrLatDMC_B = VSDP_FFStErrLatDMC_B;
    pVSDPDebug->VSDP_FFStErrTSC_B = VSDP_FFStErrTSC_B;
    pVSDPDebug->VSDP_FFStErrVDY_B = VSDP_FFStErrVDY_B;
    pVSDPDebug->VSDP_FFStErrVSM_B = VSDP_FFStErrVSM_B;
    pVSDPDebug->VSDP_FalDlyTiAccPedPstnRate_Sec =
        VSDP_FalDlyTiAccPedPstnRate_Sec;
    pVSDPDebug->VSDP_FalDlyTiCtrlStEnABS_Sec = VSDP_FalDlyTiCtrlStEnABS_Sec;
    pVSDPDebug->VSDP_FalDlyTiCtrlStEnACC_Sec = VSDP_FalDlyTiCtrlStEnACC_Sec;
    pVSDPDebug->VSDP_FalDlyTiCtrlStEnEBA_Sec = VSDP_FalDlyTiCtrlStEnEBA_Sec;
    pVSDPDebug->VSDP_FalDlyTiCtrlStEnESC_Sec = VSDP_FalDlyTiCtrlStEnESC_Sec;
    pVSDPDebug->VSDP_FalDlyTiCtrlStEnTCS_Sec = VSDP_FalDlyTiCtrlStEnTCS_Sec;
    pVSDPDebug->VSDP_FalDlyTiCtrlStEnVSM_Sec = VSDP_FalDlyTiCtrlStEnVSM_Sec;
    pVSDPDebug->VSDP_FalDlyTiNoDaytimeTrnOff_C_Sec =
        VSDP_FalDlyTiNoDaytimeTrnOff_C_Sec;
    pVSDPDebug->VSDP_FalDlyTiStErrABS_C_Sec = VSDP_FalDlyTiStErrABS_C_Sec;
    pVSDPDebug->VSDP_FalDlyTiStErrACC_C_Sec = VSDP_FalDlyTiStErrACC_C_Sec;
    pVSDPDebug->VSDP_FalDlyTiStErrEBA_C_Sec = VSDP_FalDlyTiStErrEBA_C_Sec;
    pVSDPDebug->VSDP_FalDlyTiStErrESC_C_Sec = VSDP_FalDlyTiStErrESC_C_Sec;
    pVSDPDebug->VSDP_FalDlyTiStErrLatDMC_C_Sec = VSDP_FalDlyTiStErrLatDMC_C_Sec;
    pVSDPDebug->VSDP_FalDlyTiStErrTSC_C_Sec = VSDP_FalDlyTiStErrTSC_C_Sec;
    pVSDPDebug->VSDP_FalDlyTiStErrVDY_C_Sec = VSDP_FalDlyTiStErrVDY_C_Sec;
    pVSDPDebug->VSDP_FalDlyTiStErrVSM_C_Sec = VSDP_FalDlyTiStErrVSM_C_Sec;
    pVSDPDebug->VSDP_FalDlyTiTrnSglEn_Sec = VSDP_FalDlyTiTrnSglEn_Sec;
    pVSDPDebug->VSDP_FalDlyTiTrnSglHarLigEn_Sec =
        VSDP_FalDlyTiTrnSglHarLigEn_Sec;
    pVSDPDebug->VSDP_FalDlyTiWiperEvtGapTiMx_C_Sec =
        VSDP_FalDlyTiWiperEvtGapTiMx_C_Sec;
    pVSDPDebug->VSDP_RisDlyTiManuActuTrq_Sec = VSDP_RisDlyTiManuActuTrq_Sec;
    pVSDPDebug->VSDP_RisDlyTiNoDaytimeMn_Sec = VSDP_RisDlyTiNoDaytimeMn_Sec;
    pVSDPDebug->VSDP_RisDlyTiStErrABS_Sec = VSDP_RisDlyTiStErrABS_Sec;
    pVSDPDebug->VSDP_RisDlyTiStErrACC_Sec = VSDP_RisDlyTiStErrACC_Sec;
    pVSDPDebug->VSDP_RisDlyTiStErrEBA_Sec = VSDP_RisDlyTiStErrEBA_Sec;
    pVSDPDebug->VSDP_RisDlyTiStErrESC_Sec = VSDP_RisDlyTiStErrESC_Sec;
    pVSDPDebug->VSDP_RisDlyTiStErrLatDMC_Sec = VSDP_RisDlyTiStErrLatDMC_Sec;
    pVSDPDebug->VSDP_RisDlyTiStErrTSC_Sec = VSDP_RisDlyTiStErrTSC_Sec;
    pVSDPDebug->VSDP_RisDlyTiStErrVDY_Sec = VSDP_RisDlyTiStErrVDY_Sec;
    pVSDPDebug->VSDP_RisDlyTiStErrVSM_Sec = VSDP_RisDlyTiStErrVSM_Sec;
    pVSDPDebug->VSDP_RisDlyTiWiperContiTiMn_Sec =
        VSDP_RisDlyTiWiperContiTiMn_Sec;
    pVSDPDebug->VSDP_RisDlyTiWiperEnTiMn_Sec = VSDP_RisDlyTiWiperEnTiMn_Sec;
    pVSDPDebug->VSDP_RisEdgeStErrABS_B = VSDP_RisEdgeStErrABS_B;
    pVSDPDebug->VSDP_RisEdgeStErrACC_B = VSDP_RisEdgeStErrACC_B;
    pVSDPDebug->VSDP_RisEdgeStErrEBA_B = VSDP_RisEdgeStErrEBA_B;
    pVSDPDebug->VSDP_RisEdgeStErrESC_B = VSDP_RisEdgeStErrESC_B;
    pVSDPDebug->VSDP_RisEdgeStErrLatDMC_B = VSDP_RisEdgeStErrLatDMC_B;
    pVSDPDebug->VSDP_RisEdgeStErrTSC_B = VSDP_RisEdgeStErrTSC_B;
    pVSDPDebug->VSDP_RisEdgeStErrVDY_B = VSDP_RisEdgeStErrVDY_B;
    pVSDPDebug->VSDP_RisEdgeStErrVSM_B = VSDP_RisEdgeStErrVSM_B;
    pVSDPDebug->VSDP_TiTrigStErrABS_Sec = VSDP_TiTrigStErrABS_Sec;
    pVSDPDebug->VSDP_TiTrigStErrACC_Sec = VSDP_TiTrigStErrACC_Sec;
    pVSDPDebug->VSDP_TiTrigStErrLatDMC_Sec = VSDP_TiTrigStErrLatDMC_Sec;
    pVSDPDebug->VSDP_TiTrigStErrTSC_Sec = VSDP_TiTrigStErrTSC_Sec;
    pVSDPDebug->VSDP_TiTrigStErrVDY_Sec = VSDP_TiTrigStErrVDY_Sec;
    pVSDPDebug->VSDP_TiTrigStErrVSM_Sec = VSDP_TiTrigStErrVSM_Sec;
    pVSDPDebug->VSDP_UstpAccPedPstn_Per = VSDP_UstpAccPedPstn_Per;
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */