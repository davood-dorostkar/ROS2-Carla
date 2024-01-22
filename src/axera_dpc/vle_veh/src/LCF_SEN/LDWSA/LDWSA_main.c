/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
#include "LDWSA_ext.h"
#include "LDWSA.h"

#define ASW_QM_CORE2_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
real32_T LDWSAI_VehWid_Mi;
real32_T LDWSAI_VehSpdActu_Mps;
real32_T LDWSAI_SpdVelShow_Kmph;
real32_T LDWSAI_VehAccSpdX_Npkg;
real32_T LDWSAI_VehAccSpdY_Npkg;
real32_T LDWSAI_VehCurv_ReMi;
uint8_T LDWSAI_TrnSgl_St;
real32_T LDWSAI_WheSteAgl_Dgr;
real32_T LDWSAI_SteAglSpd_Dgpm;
boolean_T LDWSAI_LDWSwitchEn_B;
uint8_T LDWSAI_LDWMod_St;
boolean_T LDWSAI_LDWErrCdtn_B;
boolean_T LDWSAI_DtctLnChag_B;
real32_T LDWSAI_LnWidCalc_Mi;
real32_T LDWSAI_PstnYLf_Mi;
real32_T LDWSAI_PstnYSafeLf_Mi;
real32_T LDWSAI_PstnYRi_Mi;
real32_T LDWSAI_PstnYSafeRi_Mi;
real32_T LDWSAI_HeadAglLf_Rad;
real32_T LDWSAI_HeadAglSafeLf_Rad;
real32_T LDWSAI_HeadAglRi_Rad;
real32_T LDWSAI_HeadAglSafeRi_Rad;
real32_T LDWSAI_CurvLf_ReMi;
real32_T LDWSAI_CurvSafeLf_ReMi;
real32_T LDWSAI_CurvRi_ReMi;
real32_T LDWSAI_CurvSafeRi_ReMi;
uint8_T LDWSAI_IvldLnSafeLf_St;
uint16_T LDWSAI_LnIVldLf_St;
uint8_T LDWSAI_IvldLnSafeRi_St;
uint16_T LDWSAI_LnIVldRi_St;
uint16_T LDWSAI_VehStIvld_St;
uint8_T LDWSAI_IvldStDrv_St;
uint8_T LDWSAI_CtrlStEn_St;
uint8_T LDWSAI_StError_St;
uint8_T LDWSAI_CtrlStNoAvlb_St;
uint8_T LDWSAI_PrjSpecQu_St;
boolean_T LDWSAI_DtctCstruSite_B;
real32_T LDWSAI_CycleTime_Sec;
real32_T LDWSAI_VehYawRate_rps;
boolean_T LDWSAI_AEBActive_B;
boolean_T NVRAM_LDWSwitch_B;
real32_T NVRAM_LDWStartupSpd_Kmph;
real32_T LDWSAI_VehStartupSpdHMI_Kmph;
boolean_T LDWSAI_LDPSwitchOn_B;
real32_T LDWSAI_LnLengthLf_Mi;
real32_T LDWSAI_LnLengthRi_Mi;
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"

void LCF_LDWSA_Reset(void) { LDWSA_initialize(); }

void LCF_LDWSA_Exec(const sLDWSAInReq_t* pLDWSAInput,
                    const sLDWSAParam_t* pLDWSAParam,
                    sLDWSAOutPro_t* pLDWSAOutput,
                    sLDWSADebug_t* pLDWSADebug) {
    // printf("--------------------LDWSA--------------------\n");
    /**************************************LDWSA
     * input***********************************************************/
    LDWSAI_VehWid_Mi = pLDWSAParam->LDWSAI_VehWid_Mi;

    LDWSAI_VehSpdActu_Mps = pLDWSAInput->LDWSAI_VehSpdActu_Mps;
    LDWSAI_SpdVelShow_Kmph = pLDWSAInput->LDWSAI_SpdVelShow_Kmph;
    LDWSAI_VehAccSpdX_Npkg = pLDWSAInput->LDWSAI_VehAccSpdX_Npkg;
    LDWSAI_VehAccSpdY_Npkg = pLDWSAInput->LDWSAI_VehAccSpdY_Npkg;
    LDWSAI_VehCurv_ReMi = pLDWSAInput->LDWSAI_VehCurv_ReMi;
    LDWSAI_TrnSgl_St = pLDWSAInput->LDWSAI_TrnSgl_St;
    LDWSAI_WheSteAgl_Dgr = pLDWSAInput->LDWSAI_WheSteAgl_Dgr;
    LDWSAI_SteAglSpd_Dgpm = pLDWSAInput->LDWSAI_SteAglSpd_Dgpm;
    LDWSAI_LDWSwitchEn_B = pLDWSAInput->LDWSAI_LDWSwitchEn_B;
    LDWSAI_LDWMod_St = pLDWSAInput->LDWSAI_LDWMod_St;
    LDWSAI_LDWErrCdtn_B = pLDWSAInput->LDWSAI_LDWErrCdtn_B;
    LDWSAI_DtctLnChag_B = pLDWSAInput->LDWSAI_DtctLnChag_B;
    LDWSAI_LnWidCalc_Mi = pLDWSAInput->LDWSAI_LnWidCalc_Mi;
    LDWSAI_PstnYLf_Mi = pLDWSAInput->LDWSAI_PstnYLf_Mi;
    LDWSAI_PstnYSafeLf_Mi = pLDWSAInput->LDWSAI_PstnYSafeLf_Mi;
    LDWSAI_PstnYRi_Mi = pLDWSAInput->LDWSAI_PstnYRi_Mi;
    LDWSAI_PstnYSafeRi_Mi = pLDWSAInput->LDWSAI_PstnYSafeRi_Mi;
    LDWSAI_HeadAglLf_Rad = pLDWSAInput->LDWSAI_HeadAglLf_Rad;
    LDWSAI_HeadAglSafeLf_Rad = pLDWSAInput->LDWSAI_HeadAglSafeLf_Rad;
    LDWSAI_HeadAglRi_Rad = pLDWSAInput->LDWSAI_HeadAglRi_Rad;
    LDWSAI_HeadAglSafeRi_Rad = pLDWSAInput->LDWSAI_HeadAglSafeRi_Rad;
    LDWSAI_CurvLf_ReMi = pLDWSAInput->LDWSAI_CurvLf_ReMi;
    LDWSAI_CurvSafeLf_ReMi = pLDWSAInput->LDWSAI_CurvSafeLf_ReMi;
    LDWSAI_CurvRi_ReMi = pLDWSAInput->LDWSAI_CurvRi_ReMi;
    LDWSAI_CurvSafeRi_ReMi = pLDWSAInput->LDWSAI_CurvSafeRi_ReMi;
    LDWSAI_IvldLnSafeLf_St = pLDWSAInput->LDWSAI_IvldLnSafeLf_St;
    LDWSAI_LnIVldLf_St = pLDWSAInput->LDWSAI_LnIVldLf_St;
    LDWSAI_IvldLnSafeRi_St = pLDWSAInput->LDWSAI_IvldLnSafeRi_St;
    LDWSAI_LnIVldRi_St = pLDWSAInput->LDWSAI_LnIVldRi_St;
    LDWSAI_VehStIvld_St = pLDWSAInput->LDWSAI_VehStIvld_St;
    LDWSAI_IvldStDrv_St = pLDWSAInput->LDWSAI_IvldStDrv_St;
    LDWSAI_CtrlStEn_St = pLDWSAInput->LDWSAI_CtrlStEn_St;
    LDWSAI_StError_St = pLDWSAInput->LDWSAI_StError_St;
    LDWSAI_CtrlStNoAvlb_St = pLDWSAInput->LDWSAI_CtrlStNoAvlb_St;
    LDWSAI_PrjSpecQu_St = pLDWSAInput->LDWSAI_PrjSpecQu_St;
    LDWSAI_DtctCstruSite_B = pLDWSAInput->LDWSAI_DtctCstruSite_B;
    LDWSAI_CycleTime_Sec = pLDWSAInput->LDWSAI_CycleTime_Sec;
    LDWSAI_VehYawRate_rps = pLDWSAInput->LDWSAI_VehYawRate_rps;
    LDWSAI_AEBActive_B = pLDWSAInput->LDWSAI_AEBActive_B;
    NVRAM_LDWSwitch_B = pLDWSAInput->NVRAM_LDWSwitch_B;
    NVRAM_LDWStartupSpd_Kmph = pLDWSAInput->NVRAM_LDWStartupSpd_Kmph;
    LDWSAI_VehStartupSpdHMI_Kmph = pLDWSAInput->LDWSAI_VehStartupSpdHMI_Kmph;
    LDWSAI_LDPSwitchOn_B = pLDWSAInput->LDWSAI_LDPSwitchOn_B;
    LDWSAI_LnLengthLf_Mi = pLDWSAInput->LDWSAI_LnLengthLf_Mi;
    LDWSAI_LnLengthRi_Mi = pLDWSAInput->LDWSAI_LnLengthRi_Mi;

    /************************************LDWSA
     * function*************************************************************/
    LDWSA_step();

    /************************************LDWSA
     * output*************************************************************/
    pLDWSAOutput->LDWC_DgrSide_St = ELDWTriggerDgrForHMI;
    pLDWSAOutput->LDWC_RdyToTrig_B = LDWC_RdyToTrig_B;
    pLDWSAOutput->LDWC_SysOut_St = LDWC_SysOut_St;
    pLDWSAOutput->LDVSE_NVRAMVehStartupSpd_kmph = LDVSE_NVRAMVehStartupSpd_kmph;
    pLDWSAOutput->LDWC_NVRAMLDWSwitch_B = LDWC_NVRAMLDWSwitch_B;

    /************************************LDWSA
     * debug*************************************************************/
    pLDWSADebug->LDDT_LnHeadLf_Rad = LDDT_LnHeadLf_Rad;
    pLDWSADebug->LDDT_RawLatVehSpdLf_Mps = LDDT_RawLatVehSpdLf_Mps;
    pLDWSADebug->LDDT_CrvThdMaxLf_ReMi = LDDT_CrvThdMaxLf_ReMi;
    pLDWSADebug->LDDT_CrvThdHystLf_ReMi = LDDT_CrvThdHystLf_ReMi;
    pLDWSADebug->LDDT_LnCltdCurvLf_ReMi = LDDT_LnCltdCurvLf_ReMi;
    pLDWSADebug->LDDT_LatVehSpdLf_Mps = LDDT_LatVehSpdLf_Mps;
    pLDWSADebug->LDVSE_MaxLatVel_Mps = LDVSE_MaxLatVel_Mps;
    pLDWSADebug->LDDT_CrvThdMaxRi_ReMi = LDDT_CrvThdMaxRi_ReMi;
    pLDWSADebug->LDDT_CrvThdHystRi_ReMi = LDDT_CrvThdHystRi_ReMi;
    pLDWSADebug->LDDT_LnCltdCurvRi_ReMi = LDDT_LnCltdCurvRi_ReMi;
    pLDWSADebug->LDDT_LnHeadRi_Rad = LDDT_LnHeadRi_Rad;
    pLDWSADebug->LDDT_RawLatVehSpdRi_Mps = LDDT_RawLatVehSpdRi_Mps;
    pLDWSADebug->LDDT_LatVehSpdRi_Mps = LDDT_LatVehSpdRi_Mps;
    pLDWSADebug->LDVSE_MaxCrvBySpd_ReMi = LDVSE_MaxCrvBySpd_ReMi;
    pLDWSADebug->LDVSE_HystCrvBySpd_ReMi = LDVSE_HystCrvBySpd_ReMi;
    pLDWSADebug->LDDT_LnPstnRi_Mi = LDDT_LnPstnRi_Mi;
    pLDWSADebug->LDDT_RawDstcToLnRi_Mi = LDDT_RawDstcToLnRi_Mi;
    pLDWSADebug->LDDT_DstcToLnRi_Mi = LDDT_DstcToLnRi_Mi;
    pLDWSADebug->LDWC_DlcThdMode2_Mi = LDWC_DlcThdMode2_Mi;
    pLDWSADebug->LDWC_DlcThdMode3_Mi = LDWC_DlcThdMode3_Mi;
    pLDWSADebug->LDWC_DlcThdMode1_Mi = LDWC_DlcThdMode1_Mi;
    pLDWSADebug->LDWC_CrrctByLnWidth_Fct = LDWC_CrrctByLnWidth_Fct;
    pLDWSADebug->LDWC_DstcToLnTrsd_Mi = LDWC_DstcToLnTrsd_Mi;
    pLDWSADebug->LDWC_DstcToLnTrsdCrvCpstnRi_Mi =
        LDWC_DstcToLnTrsdCrvCpstnRi_Mi;
    pLDWSADebug->LDWC_DstcToLnTrsdRi_Mi = LDWC_DstcToLnTrsdRi_Mi;
    pLDWSADebug->LDDT_TiToLnRi_Sec = LDDT_TiToLnRi_Sec;
    pLDWSADebug->LDWC_TiToLnTrsd_Sec = LDWC_TiToLnTrsd_Sec;
    pLDWSADebug->LDDT_LnPstnLf_Mi = LDDT_LnPstnLf_Mi;
    pLDWSADebug->LDDT_RawDstcToLnLf_Mi = LDDT_RawDstcToLnLf_Mi;
    pLDWSADebug->LDDT_DstcToLnLf_Mi = LDDT_DstcToLnLf_Mi;
    pLDWSADebug->LDWC_DstcToLnTrsdCrvCpstnLf_Mi =
        LDWC_DstcToLnTrsdCrvCpstnLf_Mi;
    pLDWSADebug->LDWC_DstcToLnTrsdLf_Mi = LDWC_DstcToLnTrsdLf_Mi;
    pLDWSADebug->LDDT_TiToLnLf_Sec = LDDT_TiToLnLf_Sec;
    pLDWSADebug->LDVSE_SidCdtnLDWLf_St = LDVSE_SidCdtnLDWLf_St;
    pLDWSADebug->LDVSE_SidCdtnLDWRi_St = LDVSE_SidCdtnLDWRi_St;
    pLDWSADebug->LDVSE_IvldLDW_St = LDVSE_IvldLDW_St;
    pLDWSADebug->LDWC_SuppValid_Debug = LDWC_SuppValid_Debug;
    pLDWSADebug->LDDT_RdyTrigLDW_B = LDDT_RdyTrigLDW_B;
    pLDWSADebug->LDDT_EnaSafety_B = LDDT_EnaSafety_B;
    pLDWSADebug->LDDT_EnaByCstruSiteLf_B = LDDT_EnaByCstruSiteLf_B;
    pLDWSADebug->LDDT_EnaByInVldQlfrLf_B = LDDT_EnaByInVldQlfrLf_B;
    pLDWSADebug->LDDT_EnaByInVldQlfrSfLf_B = LDDT_EnaByInVldQlfrSfLf_B;
    pLDWSADebug->LDDT_LnTrigVldLf_B = LDDT_LnTrigVldLf_B;
    pLDWSADebug->LDDT_CclByInVldQlfrLf_B = LDDT_CclByInVldQlfrLf_B;
    pLDWSADebug->LDDT_LnCclVldLf_B = LDDT_LnCclVldLf_B;
    pLDWSADebug->LDDT_LnMakVldLf_B = LDDT_LnMakVldLf_B;
    pLDWSADebug->LDVSE_RdyTrigLDW_B = LDVSE_RdyTrigLDW_B;
    pLDWSADebug->LDVSE_VehLatSpdVldLf_B = LDVSE_VehLatSpdVldLf_B;
    pLDWSADebug->LDVSE_TrnSglLf_B = LDVSE_TrnSglLf_B;
    pLDWSADebug->LDDT_EnaByInVldQlfrRi_B = LDDT_EnaByInVldQlfrRi_B;
    pLDWSADebug->LDDT_EnaByInVldQlfrSfRi_B = LDDT_EnaByInVldQlfrSfRi_B;
    pLDWSADebug->LDDT_LnTrigVldRi_B = LDDT_LnTrigVldRi_B;
    pLDWSADebug->LDDT_CclByInVldQlfrRi_B = LDDT_CclByInVldQlfrRi_B;
    pLDWSADebug->LDDT_LnCclVldRi_B = LDDT_LnCclVldRi_B;
    pLDWSADebug->LDDT_LnMakVldRi_B = LDDT_LnMakVldRi_B;
    pLDWSADebug->LDVSE_VehLatSpdVldRi_B = LDVSE_VehLatSpdVldRi_B;
    pLDWSADebug->LDVSE_TrnSglRi_B = LDVSE_TrnSglRi_B;
    pLDWSADebug->LDWC_FnsByLatSpdLf_B = LDWC_FnsByLatSpdLf_B;
    pLDWSADebug->LDWC_EnaDgrSide_B = LDWC_EnaDgrSide_B;
    pLDWSADebug->LDWC_RawTrigByDlcRi_B = LDWC_RawTrigByDlcRi_B;
    pLDWSADebug->LDWC_EnaTlcTrigRi_B = LDWC_EnaTlcTrigRi_B;
    pLDWSADebug->LDWC_RawTrigByTlcRi_B = LDWC_RawTrigByTlcRi_B;
    pLDWSADebug->LDWC_DlyTrigByTlcRi_B = LDWC_DlyTrigByTlcRi_B;
    pLDWSADebug->LDWC_EnaLdwTrigRi_B = LDWC_EnaLdwTrigRi_B;
    pLDWSADebug->LDWC_RstTlcTrigRi_B = LDWC_RstTlcTrigRi_B;
    pLDWSADebug->LDWC_ResetForSafeRi_B = LDWC_ResetForSafeRi_B;
    pLDWSADebug->LDWC_SetForSafeRi_B = LDWC_SetForSafeRi_B;
    pLDWSADebug->LDWC_TrigBySideCondRi_B = LDWC_TrigBySideCondRi_B;
    pLDWSADebug->LDWC_TrigByPrjSpecRi_B = LDWC_TrigByPrjSpecRi_B;
    pLDWSADebug->LDWC_TrigRi_B = LDWC_TrigRi_B;
    pLDWSADebug->LDWC_RawTrigByDlcLf_B = LDWC_RawTrigByDlcLf_B;
    pLDWSADebug->LDWC_EnaTlcTrigLf_B = LDWC_EnaTlcTrigLf_B;
    pLDWSADebug->LDWC_RawTrigByTlcLf_B = LDWC_RawTrigByTlcLf_B;
    pLDWSADebug->LDWC_DlyTrigByTlcLf_B = LDWC_DlyTrigByTlcLf_B;
    pLDWSADebug->LDWC_EnaLdwTrigLf_B = LDWC_EnaLdwTrigLf_B;
    pLDWSADebug->LDWC_RstTlcTrigLf_B = LDWC_RstTlcTrigLf_B;
    pLDWSADebug->LDWC_ResetForSafeLf_B = LDWC_ResetForSafeLf_B;
    pLDWSADebug->LDWC_SetForSafeLf_B = LDWC_SetForSafeLf_B;
    pLDWSADebug->LDWC_TrigBySideCondLf_B = LDWC_TrigBySideCondLf_B;
    pLDWSADebug->LDWC_TrigByPrjSpecLf_B = LDWC_TrigByPrjSpecLf_B;
    pLDWSADebug->LDWC_TrigLf_B = LDWC_TrigLf_B;
    pLDWSADebug->LDWC_FnsByDgrStLf_B = LDWC_FnsByDgrStLf_B;
    pLDWSADebug->LDWC_FnsByLatDistLf_B = LDWC_FnsByLatDistLf_B;
    pLDWSADebug->LDWC_FnsByHeadingLf_B = LDWC_FnsByHeadingLf_B;
    pLDWSADebug->LDWC_DgrFnsLf_B = LDWC_DgrFnsLf_B;
    pLDWSADebug->LDWC_FnsByDgrStRi_B = LDWC_FnsByDgrStRi_B;
    pLDWSADebug->LDWC_FnsByLatDistRi_B = LDWC_FnsByLatDistRi_B;
    pLDWSADebug->LDWC_FnsByHeadingRi_B = LDWC_FnsByHeadingRi_B;
    pLDWSADebug->LDWC_FnsByLatSpdRi_B = LDWC_FnsByLatSpdRi_B;
    pLDWSADebug->LDWC_DgrFnsRi_B = LDWC_DgrFnsRi_B;
    pLDWSADebug->LDWC_MinLdwBySysSt_B = LDWC_MinLdwBySysSt_B;
    pLDWSADebug->LDWC_EdgeRiseForMinLdw_B = LDWC_EdgeRiseForMinLdw_B;
    pLDWSADebug->LDWC_HoldForMinLdw_B = LDWC_HoldForMinLdw_B;
    pLDWSADebug->LDWC_FlagMinTimeLDW_B = LDWC_FlagMinTimeLDW_B;
    pLDWSADebug->LDWC_DgrFns_B = LDWC_DgrFns_B;
    pLDWSADebug->LDWC_CancelBySpecific_B = LDWC_CancelBySpecific_B;
    pLDWSADebug->LDWC_CancelByVehSt_B = LDWC_CancelByVehSt_B;
    pLDWSADebug->LDWC_CancelByDrvSt_B = LDWC_CancelByDrvSt_B;
    pLDWSADebug->LDWC_CancelByCtrlSt_B = LDWC_CancelByCtrlSt_B;
    pLDWSADebug->LDWC_CancelBySysSt_B = LDWC_CancelBySysSt_B;
    pLDWSADebug->LDWC_CancelByAvlSt_B = LDWC_CancelByAvlSt_B;
    pLDWSADebug->LDWC_CancelByPrjSpec_B = LDWC_CancelByPrjSpec_B;
    pLDWSADebug->LDWC_MaxDurationBySysSt_B = LDWC_MaxDurationBySysSt_B;
    pLDWSADebug->LDWC_EdgRiseForSysSt_B = LDWC_EdgRiseForSysSt_B;
    pLDWSADebug->LDWC_MaxDurationByStDly_B = LDWC_MaxDurationByStDly_B;
    pLDWSADebug->LDWC_TiWarmMx_B = LDWC_TiWarmMx_B;
    pLDWSADebug->LDWC_ErrSideByTrigLf_B = LDWC_ErrSideByTrigLf_B;
    pLDWSADebug->LDWC_ErrSideBySideCondLf_B = LDWC_ErrSideBySideCondLf_B;
    pLDWSADebug->LDWC_ErrSidByPrjSpecLf_B = LDWC_ErrSidByPrjSpecLf_B;
    pLDWSADebug->LDWC_ErrSidCdtnLf_B = LDWC_ErrSidCdtnLf_B;
    pLDWSADebug->LDWC_SideCondByDgrLf_B = LDWC_SideCondByDgrLf_B;
    pLDWSADebug->LDWC_CanelBySideLf_B = LDWC_CanelBySideLf_B;
    pLDWSADebug->LDWC_SideCondByDgrRi_B = LDWC_SideCondByDgrRi_B;
    pLDWSADebug->LDWC_ErrSideByTrigRi_B = LDWC_ErrSideByTrigRi_B;
    pLDWSADebug->LDWC_ErrSideBySideCondRi_B = LDWC_ErrSideBySideCondRi_B;
    pLDWSADebug->LDWC_ErrSidByPrjSpecRi_B = LDWC_ErrSidByPrjSpecRi_B;
    pLDWSADebug->LDWC_ErrSidCdtnRi_B = LDWC_ErrSidCdtnRi_B;
    pLDWSADebug->LDWC_CanelBySideRi_B = LDWC_CanelBySideRi_B;
    pLDWSADebug->LDWC_ErrSidCdtn_B = LDWC_ErrSidCdtn_B;
    pLDWSADebug->LDWC_CLatDevByDlcLf_B = LDWC_CLatDevByDlcLf_B;
    pLDWSADebug->LDWC_CLatDevByDgrLf_B = LDWC_CLatDevByDgrLf_B;
    pLDWSADebug->LDWC_CclLatDevLf_B = LDWC_CclLatDevLf_B;
    pLDWSADebug->LDWC_CLatDevByDlcRi_B = LDWC_CLatDevByDlcRi_B;
    pLDWSADebug->LDWC_CLatDevByDgrRi_B = LDWC_CLatDevByDgrRi_B;
    pLDWSADebug->LDWC_CclLatDevRi_B = LDWC_CclLatDevRi_B;
    pLDWSADebug->LDWC_CclLatDev_B = LDWC_CclLatDev_B;
    pLDWSADebug->LDWC_Cancel_B = LDWC_Cancel_B;
    pLDWSADebug->LDWC_AbortBySpecific_B = LDWC_AbortBySpecific_B;
    pLDWSADebug->LDWC_AbortByVehSt_B = LDWC_AbortByVehSt_B;
    pLDWSADebug->LDWC_AbortByDrvSt_B = LDWC_AbortByDrvSt_B;
    pLDWSADebug->LDWC_AbortByCtrlSt_B = LDWC_AbortByCtrlSt_B;
    pLDWSADebug->LDWC_AbortBySysSt_B = LDWC_AbortBySysSt_B;
    pLDWSADebug->LDWC_AbortByAvlSt_B = LDWC_AbortByAvlSt_B;
    pLDWSADebug->LDWC_AbortByPrjSpec_B = LDWC_AbortByPrjSpec_B;
    pLDWSADebug->LDWC_Abort_B = LDWC_Abort_B;
    pLDWSADebug->LDWC_StrgRdyBySpecific_B = LDWC_StrgRdyBySpecific_B;
    pLDWSADebug->LDWC_StrgRdyByVehSt_B = LDWC_StrgRdyByVehSt_B;
    pLDWSADebug->LDWC_StrgRdyByDrvSt_B = LDWC_StrgRdyByDrvSt_B;
    pLDWSADebug->LDWC_StrgRdyByCtrlSt_B = LDWC_StrgRdyByCtrlSt_B;
    pLDWSADebug->LDWC_StrgRdyBySysSt_B = LDWC_StrgRdyBySysSt_B;
    pLDWSADebug->LDWC_StrgRdyByAvlSt_B = LDWC_StrgRdyByAvlSt_B;
    pLDWSADebug->LDWC_StrgRdyByPrjSpec_B = LDWC_StrgRdyByPrjSpec_B;
    pLDWSADebug->LDWC_StrgRdy_B = LDWC_StrgRdy_B;
    pLDWSADebug->LDWC_Degradation_B = LDWC_Degradation_B;
    pLDWSADebug->LDWC_DegradationEdgeRise_B = LDWC_DegradationEdgeRise_B;
    pLDWSADebug->LDWC_Degr_B = LDWC_Degr_B;
    pLDWSADebug->LDWC_Trig_B = LDWC_Trig_B;
    pLDWSADebug->LDWC_SuppBySpecific_B = LDWC_SuppBySpecific_B;
    pLDWSADebug->LDWC_SuppByVehSt_B = LDWC_SuppByVehSt_B;
    pLDWSADebug->LDWC_SuppByDrvSt_B = LDWC_SuppByDrvSt_B;
    pLDWSADebug->LDWC_SuppByCtrlSt_B = LDWC_SuppByCtrlSt_B;
    pLDWSADebug->LDWC_SuppBySysSt_B = LDWC_SuppBySysSt_B;
    pLDWSADebug->LDWC_SuppyByAvlSt_B = LDWC_SuppyByAvlSt_B;
    pLDWSADebug->LDWC_SuppPrjSpec_B = LDWC_SuppPrjSpec_B;
    pLDWSADebug->LDWC_Suppresion_B = LDWC_Suppresion_B;
    pLDWSADebug->LDWC_WeakRdyBySpecific_B = LDWC_WeakRdyBySpecific_B;
    pLDWSADebug->LDWC_WeakRdyByVehSt_B = LDWC_WeakRdyByVehSt_B;
    pLDWSADebug->LDWC_WeakRdyByDrvSt_B = LDWC_WeakRdyByDrvSt_B;
    pLDWSADebug->LDWC_WeakRdyByCtrlSt_B = LDWC_WeakRdyByCtrlSt_B;
    pLDWSADebug->LDWC_WeakRdyBySysSt_B = LDWC_WeakRdyBySysSt_B;
    pLDWSADebug->LDWC_WeakRdyByAvlSt_B = LDWC_WeakRdyByAvlSt_B;
    pLDWSADebug->LDWC_WeakRdyByPrjSpec_B = LDWC_WeakRdyByPrjSpec_B;
    pLDWSADebug->LDWC_BlockTimeBySysOut_B = LDWC_BlockTimeBySysOut_B;
    pLDWSADebug->LDWC_RawBlockTimeByRampOut_B = LDWC_RawBlockTimeByRampOut_B;
    pLDWSADebug->LDWC_BlockTimeByRampOut_B = LDWC_BlockTimeByRampOut_B;
    pLDWSADebug->LDWC_BlockTime_B = LDWC_BlockTime_B;
    pLDWSADebug->LDWC_WkRdy_B = LDWC_WkRdy_B;
    pLDWSADebug->LDWC_Suppression_B = LDWC_Suppression_B;
    pLDWSADebug->LDVSE_HodTiTrnSglLf_Sec = LDVSE_HodTiTrnSglLf_Sec;
    pLDWSADebug->LDVSE_HodTiTrnSglRi_Sec = LDVSE_HodTiTrnSglRi_Sec;
    pLDWSADebug->LDWC_DlyTiOfTiToLnRiMn_Sec = LDWC_DlyTiOfTiToLnRiMn_Sec;
    pLDWSADebug->LDWC_HdTiTrigRi_Sec = LDWC_HdTiTrigRi_Sec;
    pLDWSADebug->LDWC_DlyTiOfTiToLnLfMn_Sec = LDWC_DlyTiOfTiToLnLfMn_Sec;
    pLDWSADebug->LDWC_HdTiTrigLf_Sec = LDWC_HdTiTrigLf_Sec;
    pLDWSADebug->LDWC_HdTiWarming_Sec = LDWC_HdTiWarming_Sec;
    pLDWSADebug->LDWC_DlyTiTgtFns_Sec = LDWC_DlyTiTgtFns_Sec;
    pLDWSADebug->LDWC_HdTiWarmMx_Sec = LDWC_HdTiWarmMx_Sec;
    pLDWSADebug->LDWC_HdTiDegr_Sec = LDWC_HdTiDegr_Sec;
    pLDWSADebug->LDWC_HdTiFns_Sec = LDWC_HdTiFns_Sec;
    pLDWSADebug->LDWC_DgrSideOld_St = LDWC_DgrSideOld_St;
    pLDWSADebug->LDDT_UHysCltdCurvVldLf_B = LDDT_UHysCltdCurvVldLf_B;
    pLDWSADebug->LDDT_BHysHeadAglTrigVldLf_B = LDDT_BHysHeadAglTrigVldLf_B;
    pLDWSADebug->LDDT_UHysHeadAglCclVldLf_B = LDDT_UHysHeadAglCclVldLf_B;
    pLDWSADebug->LDVSE_BHysLatVehSpdVldLf_B = LDVSE_BHysLatVehSpdVldLf_B;
    pLDWSADebug->LDVSE_UHysLatVehSpdVldLf_B = LDVSE_UHysLatVehSpdVldLf_B;
    pLDWSADebug->LDVSE_EdgeRisTrnSglRi_B = LDVSE_EdgeRisTrnSglRi_B;
    pLDWSADebug->LDDT_UHysCltdCurvVldRi_B = LDDT_UHysCltdCurvVldRi_B;
    pLDWSADebug->LDDT_BHysHeadAglTrigVldRi_B = LDDT_BHysHeadAglTrigVldRi_B;
    pLDWSADebug->LDDT_UHysHeadAglCclVldRi_B = LDDT_UHysHeadAglCclVldRi_B;
    pLDWSADebug->LDVSE_BHysLatVehSpdVldRi_B = LDVSE_BHysLatVehSpdVldRi_B;
    pLDWSADebug->LDVSE_UHysLatVehSpdVldRi_B = LDVSE_UHysLatVehSpdVldRi_B;
    pLDWSADebug->LDVSE_EdgeRisTrnSglLf_B = LDVSE_EdgeRisTrnSglLf_B;
    pLDWSADebug->LDVSE_UHysSteAgl_B = LDVSE_UHysSteAgl_B;
    pLDWSADebug->LDVSE_BHysSpdVeh_B = LDVSE_BHysSpdVeh_B;
    pLDWSADebug->LDVSE_UHysSteAglSpd_B = LDVSE_UHysSteAglSpd_B;
    pLDWSADebug->LDVSE_BHysAccVehX_B = LDVSE_BHysAccVehX_B;
    pLDWSADebug->LDVSE_BHysAccVehY_B = LDVSE_BHysAccVehY_B;
    pLDWSADebug->LDVSE_UHysVehCurv_B = LDVSE_UHysVehCurv_B;
    pLDWSADebug->LDVSE_BHysLnWid_B = LDVSE_BHysLnWid_B;
    pLDWSADebug->LDWC_HdTiTrigRiEn_B = LDWC_HdTiTrigRiEn_B;
    pLDWSADebug->LDWC_DisTrigRi_B = LDWC_DisTrigRi_B;
    pLDWSADebug->LDWC_HdTiTrigLfEn_B = LDWC_HdTiTrigLfEn_B;
    pLDWSADebug->LDWC_DisTrigLf_B = LDWC_DisTrigLf_B;
    pLDWSADebug->LDWC_EdgeRisWarming_B = LDWC_EdgeRisWarming_B;
    pLDWSADebug->LDWC_EdgeRisWarmMx_B = LDWC_EdgeRisWarmMx_B;
    pLDWSADebug->LDWC_EdgeRisDegr_B = LDWC_EdgeRisDegr_B;
    pLDWSADebug->LDWC_DegrOld_B = LDWC_DegrOld_B;
    pLDWSADebug->LDWC_EdgeRisFns_B = LDWC_EdgeRisFns_B;
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */