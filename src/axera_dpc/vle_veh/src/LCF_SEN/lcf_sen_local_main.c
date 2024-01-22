/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "lcf_sen_local_main.h"
#ifdef UBUNTU_SYSTEM
#include "BSW_DataLog_Server.h"
#endif
/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE2_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
#if 1
// LCFOPS module global values
sLCFOPSInReq_t g_LCF_OPSInput = {0};
sLCFOPSParam_t g_LCF_OPSParam = {0};
sLCFOPSOutPro_t g_LCF_OPSOutput = {0};

// LCCRA module global values
sLCCRAInReq_t g_LCF_LCCRAInput = {0};
sLCCRAParam_t g_LCF_LCCRAParam = {0};
sLCCRAOutPro_t g_LCF_LCCRAOutput = {0};

// ABPR module global values
sLBPInput_t g_LCF_LBPInput = {0};
sLBPParam_t g_LCF_LBPParam = {0};
sLBPOutput_t g_LCF_LBPOutput = {0};

// ALP module global values
sALPInReq_st g_LCF_ALPInput = {0};
sALPParam_st g_LCF_ALPParam = {0};
sALPOutput_st g_LCF_ALPOutput = {0};

// ODPR module global values
ODPRInReq_t g_LCF_ODPR_Input = {0};
ODPRParam_t g_LCF_ODPR_Param = {0};
ODPROutPro_t g_LCF_ODPR_Output = {0};

// VSDP module global values
sVSDPInput_t g_LCF_VSDP_Input = {0};
sVSDPParam_t g_LCF_VSDP_Param = {0};
sVSDPOutput_t g_LCF_VSDP_Output = {0};

// LDWSA module global values
sLDWSAInReq_t g_LCF_LDWSA_Input = {0};
sLDWSAParam_t g_LCF_LDWSA_Param = {0};
sLDWSAOutPro_t g_LCF_LDWSA_Output = {0};

// LDPSA module global values
sLDPSAInput_t g_LCF_LDPSA_Input = {0};
sLDPSAParam_t g_LCF_LDPSA_Param = {0};
sLDPSAOutput_t g_LCF_LDPSA_Output = {0};

// TJASA module global values
sTJASAInReq_t g_LCF_TJASA_Input = {0};
sTJASAParam_t g_LCF_TJASA_Param = {0};
sTJASAOutPro_t g_LCF_TJASA_Output = {0};

// MCTLFC module global values
sMCTLFCInReq_st g_LCF_MCTLFC_Input = {0};
sMCTLFCParam_st g_LCF_MCTLFC_Param = {0};
sMCTLFCOut_st g_LCF_MCTLFC_Output = {0};
#endif
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/* ****************************************************************************

  Functionname:     LcfSenExec

  @brief            LCF_SEN main function with input and output parameters

  @description      -

  @param[in]        reqPorts : pointer on the required ports structure
  @param[in,out]    proPorts : pointer on the provided ports structure

  @return           -
**************************************************************************** */
void LcfSenExec(const reqLcfSenPrtList_t* const reqPorts,
                const reqLcfSenParams* reqParams,
                proLcfSenPrtList_t* const proPorts,
                reqLcfSenDebug_t* proDebugs) {
    static uint16 uiCycleCnt = 0U;

    /* Cycle counter incremented every time Exec normal behaviour is executed
     * (no BASE_RETURN_ERROR) */
    uiCycleCnt++;

    /* Fill basic information in provided ports */
    LcfSenSetInfoDataProPorts(reqPorts, proPorts, uiCycleCnt);

    /*! At this place it is necessary to test lcfOpMode directly to get an
     * initialized component */
    switch (reqPorts->sBaseCtrlData.eOpMode) {
        case (uint8)BASE_OM_RESET:
            LcfSenReset(reqParams);
            break;  // no calculation should be done in RESET
        case (uint8)BASE_OM_IDLE:
            // COMP_STATE_NOT_RUNNING shall be set if the algo is called in the
            // idle op mode;
            proPorts->pAlgoCompState.eCompState = COMP_STATE_NOT_RUNNING;
            proPorts->pAlgoCompState.eGenAlgoQualifier = ALGO_QUAL_OK;
            // Except for the comp state and the NVM structure the signal state
            // shall be 'AL_SIG_STATE_INVALID'.
            proPorts->pLcfSenOutputData.sSigHeader.eSigStatus =
                AL_SIG_STATE_INVALID;
            proPorts->pLcfSenOutputToVehData.sSigHeader.eSigStatus =
                AL_SIG_STATE_INVALID;
            break;  // no calculation shall be done in IDLE
        case (uint8)BASE_OM_DEMO:
            proPorts->pAlgoCompState.eCompState = COMP_STATE_SUCCESS;
            proPorts->pAlgoCompState.eGenAlgoQualifier = ALGO_QUAL_OK;
            // TODO: fill demo values for all outputs
            break;                // no calculation shall be done in DEMO
        case (uint8)BASE_OM_RUN:  // check for other op modes
            /*if (LCFSEN_reqPorts_NullStatusCheck ==
            ALL_REQUIRED_PORTS_NOT_NULL_MASK)
            {*/
            // printf("LCF Sen Process is Running !!! \n");
            LcfSenProcess(reqPorts, reqParams, proPorts, proDebugs);
            //}
            // else {}
            break;
        default:
            break;
    }
#ifdef UBUNTU_SYSTEM
    // datalogger logic
    DATALOGInfo_t Record1, Record2, Record3, Record4, Record5, Record6, Record7,
        Record8, Record9, Record10, Record11, Record12, Record13, Record14,
        Record15, Record16, Record17, Record18, Record19, Record20, Record21;
    Record1.StructID = Data_sLBPInput_t_type;
    Record1.Length = sizeof(sLBPInput_t);
    Record1.SocBuf = (uint8*)&g_LCF_LBPInput;

    Record2.StructID = Data_sLBPParam_t_type;
    Record2.Length = sizeof(sLBPParam_t);
    Record2.SocBuf = (uint8*)&g_LCF_LBPParam;

    Record3.StructID = Data_sLBPOutput_t_type;
    Record3.Length = sizeof(sLBPOutput_t);
    Record3.SocBuf = (uint8*)&g_LCF_LBPOutput;

    Record4.StructID = Data_sALPInput_t_type;
    Record4.Length = sizeof(sALPInReq_st);
    Record4.SocBuf = (uint8*)&g_LCF_ALPInput;

    Record5.StructID = Data_sALPParam_t_type;
    Record5.Length = sizeof(sALPParam_st);
    Record5.SocBuf = (uint8*)&g_LCF_ALPParam;

    Record6.StructID = Data_sALPOutput_t_type;
    Record6.Length = sizeof(sALPOutput_st);
    Record6.SocBuf = (uint8*)&g_LCF_ALPOutput;

    Record7.StructID = Data_sVSDPInput_t_type;
    Record7.Length = sizeof(sVSDPInput_t);
    Record7.SocBuf = (uint8*)&g_LCF_VSDP_Input;

    Record8.StructID = Data_sVSDPParam_t_type;
    Record8.Length = sizeof(sVSDPParam_t);
    Record8.SocBuf = (uint8*)&g_LCF_VSDP_Param;

    Record9.StructID = Data_sVSDPOutput_t_type;
    Record9.Length = sizeof(sVSDPOutput_t);
    Record9.SocBuf = (uint8*)&g_LCF_VSDP_Output;

    Record10.StructID = Data_sLDPSAInput_t_type;
    Record10.Length = sizeof(sLDPSAInput_t);
    Record10.SocBuf = (uint8*)&g_LCF_LDPSA_Input;

    Record11.StructID = Data_sLDPSAParam_t_type;
    Record11.Length = sizeof(sLDPSAParam_t);
    Record11.SocBuf = (uint8*)&g_LCF_LDPSA_Param;

    Record12.StructID = Data_sLDPSAOutput_t_type;
    Record12.Length = sizeof(sLDPSAOutput_t);
    Record12.SocBuf = (uint8*)&g_LCF_LDPSA_Output;

    Record13.StructID = Data_ODPRInReq_t_type;
    Record13.Length = sizeof(ODPRInReq_t);
    Record13.SocBuf = (uint8*)&g_LCF_ODPR_Input;

    Record14.StructID = Data_ODPRParam_t_type;
    Record14.Length = sizeof(ODPRParam_t);
    Record14.SocBuf = (uint8*)&g_LCF_ODPR_Param;

    Record15.StructID = Data_ODPROutPro_t_type;
    Record15.Length = sizeof(ODPROutPro_t);
    Record15.SocBuf = (uint8*)&g_LCF_ODPR_Output;

    Record16.StructID = Data_TJASAInReq_t_type;
    Record16.Length = sizeof(sTJASAInReq_t);
    Record16.SocBuf = (uint8*)&g_LCF_TJASA_Input;

    Record17.StructID = Data_TJASAParam_t_type;
    Record17.Length = sizeof(sTJASAParam_t);
    Record17.SocBuf = (uint8*)&g_LCF_TJASA_Param;

    Record18.StructID = Data_TJASAOutPro_t_type;
    Record18.Length = sizeof(sTJASAOutPro_t);
    Record18.SocBuf = (uint8*)&g_LCF_TJASA_Output;

    Record19.StructID = Data_MCTLFCInReq_t_type;
    Record19.Length = sizeof(sMCTLFCInReq_st);
    Record19.SocBuf = (uint8*)&g_LCF_MCTLFC_Input;

    Record20.StructID = Data_MCTLFCParam_t_type;
    Record20.Length = sizeof(sMCTLFCParam_st);
    Record20.SocBuf = (uint8*)&g_LCF_MCTLFC_Param;

    Record21.StructID = Data_MCTLFCOutPro_t_type;
    Record21.Length = sizeof(sMCTLFCOut_st);
    Record21.SocBuf = (uint8*)&g_LCF_MCTLFC_Output;

    BSW_DataLog_FreezeData(Record1);
    BSW_DataLog_FreezeData(Record2);
    BSW_DataLog_FreezeData(Record3);
    BSW_DataLog_FreezeData(Record4);
    BSW_DataLog_FreezeData(Record5);
    BSW_DataLog_FreezeData(Record6);
    BSW_DataLog_FreezeData(Record7);
    BSW_DataLog_FreezeData(Record8);
    BSW_DataLog_FreezeData(Record9);
    BSW_DataLog_FreezeData(Record10);
    BSW_DataLog_FreezeData(Record11);
    BSW_DataLog_FreezeData(Record12);
    BSW_DataLog_FreezeData(Record13);
    BSW_DataLog_FreezeData(Record14);
    BSW_DataLog_FreezeData(Record15);
    BSW_DataLog_FreezeData(Record16);
    BSW_DataLog_FreezeData(Record17);
    BSW_DataLog_FreezeData(Record18);
    BSW_DataLog_FreezeData(Record19);
    BSW_DataLog_FreezeData(Record20);
    BSW_DataLog_FreezeData(Record21);
#endif
}

/* ****************************************************************************

  Functionname:     LcfSenReset

  @brief            Reset of the component

  @description      Initialization of all internal data storage.
                                        Shall be called once before processing
starts

  @param[in]
  @param[in,out]    proPorts : pointer on the provided ports structure

  @return           -

  @pre              -
  @post             All internal values and all interfaces are initialized
                                        to default values


**************************************************************************** */
void LcfSenReset(const reqLcfSenParams* reqParams) {
    // LCFOPS module global values
    memset(&g_LCF_OPSInput, 0, sizeof(sLCFOPSInReq_t));
    memset(&g_LCF_OPSParam, 0, sizeof(sLCFOPSParam_t));
    memset(&g_LCF_OPSOutput, 0, sizeof(sLCFOPSOutPro_t));

    // LCCRA module global values
    memset(&g_LCF_LCCRAInput, 0, sizeof(sLCCRAInReq_t));
    memset(&g_LCF_LCCRAParam, 0, sizeof(sLCCRAParam_t));
    memset(&g_LCF_LCCRAOutput, 0, sizeof(sLCCRAOutPro_t));

    // LBP module global values
    memset(&g_LCF_LBPInput, 0, sizeof(sLBPInput_t));
    memset(&g_LCF_LBPParam, 0, sizeof(sLBPParam_t));
    memset(&g_LCF_LBPOutput, 0, sizeof(sLBPOutput_t));

    // ALP module global values
    memset(&g_LCF_ALPInput, 0, sizeof(sALPInReq_st));
    memset(&g_LCF_ALPParam, 0, sizeof(sALPParam_st));
    memset(&g_LCF_ALPOutput, 0, sizeof(sALPOutput_st));

    // ODPR module global values
    memset(&g_LCF_ODPR_Input, 0, sizeof(ODPRInReq_t));
    memset(&g_LCF_ODPR_Param, 0, sizeof(ODPRParam_t));
    memset(&g_LCF_ODPR_Output, 0, sizeof(ODPROutPro_t));

    // VSDP module global values
    memset(&g_LCF_VSDP_Input, 0, sizeof(sVSDPInput_t));
    memset(&g_LCF_VSDP_Param, 0, sizeof(sVSDPParam_t));
    memset(&g_LCF_VSDP_Output, 0, sizeof(sVSDPOutput_t));

    // LDPSA module global values
    memset(&g_LCF_LDWSA_Input, 0, sizeof(sLDWSAInReq_t));
    memset(&g_LCF_LDWSA_Param, 0, sizeof(sLDWSAParam_t));
    memset(&g_LCF_LDWSA_Output, 0, sizeof(sLDWSAOutPro_t));

    // LDPSA module global values
    memset(&g_LCF_LDPSA_Input, 0, sizeof(sLDPSAInput_t));
    memset(&g_LCF_LDPSA_Param, 0, sizeof(sLDPSAParam_t));
    memset(&g_LCF_LDPSA_Output, 0, sizeof(sLDPSAOutput_t));

    // TJASA module global values
    memset(&g_LCF_TJASA_Input, 0, sizeof(sTJASAInReq_t));
    memset(&g_LCF_TJASA_Param, 0, sizeof(sTJASAParam_t));
    memset(&g_LCF_TJASA_Output, 0, sizeof(sTJASAOutPro_t));

    // MCTLFC module global values
    memset(&g_LCF_MCTLFC_Input, 0, sizeof(sMCTLFCInReq_st));
    memset(&g_LCF_MCTLFC_Param, 0, sizeof(sMCTLFCParam_st));
    memset(&g_LCF_MCTLFC_Output, 0, sizeof(sMCTLFCOut_st));

    // set parameter values for every modules
    LCF_SEN_ResetParams(reqParams, &g_LCF_LBPParam, &g_LCF_ALPParam,
                        &g_LCF_ODPR_Param, &g_LCF_VSDP_Param,
                        &g_LCF_LDWSA_Param, &g_LCF_LDPSA_Param,
                        &g_LCF_TJASA_Param, &g_LCF_MCTLFC_Param,
                        &g_LCF_LCCRAParam, &g_LCF_OPSParam);

    LCF_VSDP_Reset();
    LCF_LBP_Reset();
    LCF_ALP_Init_Reset();
    LCF_LDWSA_Reset();
    LCF_LDPSA_Reset();
    LCF_ODPR_Reset();
    LCF_TJASA_Reset();
    MCTLFC_Init();
    LCFOPS_Reset();
    LCCRA_Reset();
}

/* ****************************************************************************

 Functionname:     LcfSenSetInfoDataProPorts

 @brief            Fill provided ports

 @description      Fills version number and signal header for all provide ports
                                   Shall be called in normal behaviour Exec
function

 @param[in]        reqPorts : pointer on the required ports structure
 @param[in,out]    proPorts : pointer on the provided ports structure
 @param[in]        uiCycleCnt : cycle counter of Exec function

 @return           -

 @pre              -
 @post             Provided ports filled with version number and signal header

 @author

**************************************************************************** */
static void LcfSenSetInfoDataProPorts(const reqLcfSenPrtList_t* const reqPorts,
                                      proLcfSenPrtList_t* const proPorts,
                                      uint16 uiCycleCnt) {
    uint32 uiCtrlTimeStamp = 0U;
    uint16 uiCtrlMeasCounter = 0U;

    /* Fill version numbers of provided ports */
    proPorts->pAlgoCompState.uiVersionNumber = 20201202u;
    proPorts->pLcfSenOutputData.uiVersionNumber = 20201202u;
    proPorts->pLcfSenOutputToVehData.uiVersionNumber = 20201202u;

    /* Fill signal headers of provided ports */
    uiCtrlTimeStamp = reqPorts->sBaseCtrlData.sSigHeader.uiTimeStamp;
    uiCtrlMeasCounter = reqPorts->sBaseCtrlData.sSigHeader.uiMeasurementCounter;

    LcfSenSetSigHeader(&(proPorts->pAlgoCompState.sSigHeader), AL_SIG_STATE_OK,
                       uiCtrlTimeStamp, uiCtrlMeasCounter, uiCycleCnt);
    LcfSenSetSigHeader(&(proPorts->pLcfSenOutputData.sSigHeader),
                       AL_SIG_STATE_OK, uiCtrlTimeStamp, uiCtrlMeasCounter,
                       uiCycleCnt);
    LcfSenSetSigHeader(&(proPorts->pLcfSenOutputToVehData.sSigHeader),
                       AL_SIG_STATE_OK, uiCtrlTimeStamp, uiCtrlMeasCounter,
                       uiCycleCnt);

    /* Fill additional Algo comp state information */
    proPorts->pAlgoCompState.eShedulerSubModeRequest = BASE_SSM_NONE;
    proPorts->pAlgoCompState.uiAlgoVersionNumber = 20201202u;
}

/* ****************************************************************************

 Functionname:     LcfSenSetSigHeader

 @brief            Fill a signal header

 @description      Fill a signal header
                                   Shall be called in normal behaviour Exec
function

 @param[in,out]    psSigHeader : pointer on the signal header to be filled
 @param[in]        uiSigState : signal state
 @param[in]        uiCtrlTmp : timestamp
 @param[in]        uiCtrlMeasCnt : measurement counter
 @param[in]        uiCycleCnt : cycle counter of Exec function

 @return           -

 @pre              -
 @post             Signal header is filled

 @author

**************************************************************************** */
void LcfSenSetSigHeader(LCF_SenSignalHeader_t* const psSigHeader,
                        uint8 uiSigState,
                        uint32 uiCtrlTmp,
                        uint16 uiCtrlMeasCnt,
                        uint16 uiCycleCnt) {
    /* Fill signal header with the received arguments */
    psSigHeader->eSigStatus = uiSigState;
    psSigHeader->uiTimeStamp = uiCtrlTmp;
    psSigHeader->uiMeasurementCounter = uiCtrlMeasCnt;
    psSigHeader->uiCycleCounter = uiCycleCnt;
}

/* ****************************************************************************

 Functionname:     LcfSenProcess

 @brief            Template Main processing

 @description      Calls all processing functions of lcf sen and sub-components

 @param[in]        reqPorts : pointer on the required ports structure
 @param[in,out]    proPorts : pointer on the provided ports structure
 @param[in]        services : pointer on the services structure

 @return           -

 @pre              All states must be set
 @post             -

 @todo             review for unused code

 @author

**************************************************************************** */
static void LcfSenProcess(const reqLcfSenPrtList_t* const reqPorts,
                          const reqLcfSenParams* reqParams,
                          proLcfSenPrtList_t* const proPorts,
                          reqLcfSenDebug_t* proDebugs) {
    VSDPInputWrapper(reqPorts, reqParams, &g_LCF_VSDP_Input);
    LCF_VSDP_Exec(&g_LCF_VSDP_Input, &g_LCF_VSDP_Param, &g_LCF_VSDP_Output,
                  &(proDebugs->sVSDPDebug));

    // printf("VSDP : VSDP_CtrlStEn_St = %d\n",
    // g_LCF_VSDP_Output.VSDP_CtrlStEn_St);
    // printf("VSDP : VSDP_CtrlStNoAvlb_St = %d\n",
    // g_LCF_VSDP_Output.VSDP_CtrlStNoAvlb_St);
    // printf("VSDP : VSDP_IvldStDrv_St = %d\n",
    // g_LCF_VSDP_Output.VSDP_IvldStDrv_St);
    // printf("VSDP : VSDP_StError_St = %d\n",
    // g_LCF_VSDP_Output.VSDP_StError_St);
    // printf("VSDP : VSDP_VehStIvld_St = %d\n",
    // g_LCF_VSDP_Output.VSDP_VehStIvld_St);

    LBPInputWrapper(reqPorts, &g_LCF_LBPInput);

#if 1
    static float32 fPrevLfLanePosY0_met;
    static float32 fPrevRiLanePosY0_met;
    // static float32 fPrevLfLaneHeading_rad;
    // static float32 fPrevRiLaneHeading_rad;
    // static uint8 uLfNumIdx, uRiNumIdx;

#if 1
    g_LCF_LBPInput.uLaneChange = 0;
    if (g_LCF_LBPInput.fPosY0[0] > 2.0f && fPrevLfLanePosY0_met < 0.5f &&
        g_LCF_LBPInput.fPosY0[1] > -0.5f && fPrevRiLanePosY0_met < -2.0f) {
        g_LCF_LBPInput.uLaneChange = 1;
    } else if (g_LCF_LBPInput.fPosY0[0] < 0.5f && fPrevLfLanePosY0_met > 2.0f &&
               g_LCF_LBPInput.fPosY0[1] < -2.0f &&
               fPrevRiLanePosY0_met > -0.5f) {
        g_LCF_LBPInput.uLaneChange = 2;
    }
#endif

#if 0
    if (uLfNumIdx) {
        if (fABS(g_LCF_LBPInput.fPosY0[0] - fPrevLfLanePosY0_met) > 1.0f &&
            !g_LCF_LBPInput.uLaneChange && g_LCF_LBPInput.bAvailable[0]) {
            g_LCF_LBPInput.fPosY0[0] = fPrevLfLanePosY0_met;
            g_LCF_LBPInput.fPosY0[2] = fPrevLfLanePosY0_met;
            g_LCF_LBPInput.fHeadingAngle[0] = fPrevLfLaneHeading_rad;
            g_LCF_LBPInput.fHeadingAngle[2] = fPrevLfLaneHeading_rad;
        }
    }
    if (uRiNumIdx) {
        if (fABS(g_LCF_LBPInput.fPosY0[1] - fPrevRiLanePosY0_met) > 1.0f &&
            !g_LCF_LBPInput.uLaneChange && g_LCF_LBPInput.bAvailable[1]) {
            g_LCF_LBPInput.fPosY0[1] = fPrevRiLanePosY0_met;
            g_LCF_LBPInput.fPosY0[3] = fPrevRiLanePosY0_met;
            g_LCF_LBPInput.fHeadingAngle[1] = fPrevRiLaneHeading_rad;
            g_LCF_LBPInput.fHeadingAngle[3] = fPrevRiLaneHeading_rad;
        }
    }
#endif

    fPrevLfLanePosY0_met = g_LCF_LBPInput.fPosY0[0];
    fPrevRiLanePosY0_met = g_LCF_LBPInput.fPosY0[1];
// fPrevLfLaneHeading_rad = g_LCF_LBPInput.fHeadingAngle[0];
// fPrevRiLaneHeading_rad = g_LCF_LBPInput.fHeadingAngle[1];
// if (g_LCF_LBPInput.bAvailable[0]) {
//     uLfNumIdx = 1u;
// } else {
//     uLfNumIdx = 0u;
// }
// if (g_LCF_LBPInput.bAvailable[1]) {
//     uRiNumIdx = 1u;
// } else {
//     uRiNumIdx = 0u;
// }
#endif

    LCF_LBP_Exec(&g_LCF_LBPInput, &g_LCF_LBPParam, &g_LCF_LBPOutput,
                 &(proDebugs->sLBPDebug));
    LBPOutputWrapper(g_LCF_LBPOutput, proPorts);

#if 0
    static float32 fTempLaneWigth_met = 3.3f;

    g_LCF_LBPOutput.fValidLengthCtrlLf =
        reqPorts->sLCFLaneData.afValidLength_met[LCF_SEN_CAMERA_LANE_LE];
    g_LCF_LBPOutput.fValidLengthCtrlRi =
        reqPorts->sLCFLaneData.afValidLength_met[LCF_SEN_CAMERA_LANE_RI];

    g_LCF_LBPOutput.uLaneInvalidQualifierLf = 0u;
    g_LCF_LBPOutput.uLaneInvalidQualifierRi = 0u;

    g_LCF_LBPOutput.uInvalidQualifierSafeLf = 0u;
    g_LCF_LBPOutput.uInvalidQualifierSafeRi = 0u;

    if ((g_LCF_LBPOutput.fValidLengthCtrlLf > 5.f) &&
        (g_LCF_LBPOutput.fValidLengthCtrlRi > 5.f)) {
        g_LCF_LBPOutput.fPosX0CtrlLf = 0.f;
        g_LCF_LBPOutput.fPosY0CtrlLf = g_LCF_LBPOutput.fPosY0SafeLf;
        g_LCF_LBPOutput.fHeadingCtrlLf = g_LCF_LBPOutput.fHeadingSafeLf;
        g_LCF_LBPOutput.fCrvCtrlLf = g_LCF_LBPOutput.fCrvSafeLf;
        g_LCF_LBPOutput.fCrvRateCtrlLf =
            reqPorts->sLCFLaneData
                .afCurvatureChange_1pm2[LCF_SEN_CAMERA_LANE_LE];

        g_LCF_LBPOutput.fPosX0CtrlRi = 0.f;
        g_LCF_LBPOutput.fPosY0CtrlRi = g_LCF_LBPOutput.fPosY0SafeRi;
        g_LCF_LBPOutput.fHeadingCtrlRi = g_LCF_LBPOutput.fHeadingSafeRi;
        g_LCF_LBPOutput.fCrvCtrlRi = g_LCF_LBPOutput.fCrvSafeRi;
        g_LCF_LBPOutput.fCrvRateCtrlRi =
            reqPorts->sLCFLaneData
                .afCurvatureChange_1pm2[LCF_SEN_CAMERA_LANE_RI];

        // g_LCF_LBPOutput.fLaneWidth 				=
        // g_LCF_LBPOutput.fPosY0CtrlLf - g_LCF_LBPOutput.fPosY0CtrlRi;
    } else if (g_LCF_LBPOutput.fValidLengthCtrlLf > 5.f) {
        g_LCF_LBPOutput.fPosX0CtrlLf = 0.f;
        g_LCF_LBPOutput.fPosY0CtrlLf = g_LCF_LBPOutput.fPosY0SafeLf;
        g_LCF_LBPOutput.fHeadingCtrlLf = g_LCF_LBPOutput.fHeadingSafeLf;
        g_LCF_LBPOutput.fCrvCtrlLf = g_LCF_LBPOutput.fCrvSafeLf;
        g_LCF_LBPOutput.fCrvRateCtrlLf =
            reqPorts->sLCFLaneData
                .afCurvatureChange_1pm2[LCF_SEN_CAMERA_LANE_LE];

        g_LCF_LBPOutput.fPosX0CtrlRi = 0.f;
        g_LCF_LBPOutput.fPosY0CtrlRi =
            g_LCF_LBPOutput.fPosY0CtrlLf - fTempLaneWigth_met;
        g_LCF_LBPOutput.fHeadingCtrlRi = g_LCF_LBPOutput.fHeadingCtrlLf;
        g_LCF_LBPOutput.fCrvCtrlRi = g_LCF_LBPOutput.fCrvCtrlLf;
        g_LCF_LBPOutput.fCrvRateCtrlRi = g_LCF_LBPOutput.fCrvRateCtrlLf;
        g_LCF_LBPOutput.fValidLengthCtrlRi = g_LCF_LBPOutput.fValidLengthCtrlLf;

        // g_LCF_LBPOutput.fLaneWidth 				=
        // fTempLaneWigth_met;
    } else if (g_LCF_LBPOutput.fValidLengthCtrlRi > 5.f) {
        g_LCF_LBPOutput.fPosX0CtrlRi = 0.f;
        g_LCF_LBPOutput.fPosY0CtrlRi = g_LCF_LBPOutput.fPosY0SafeRi;
        g_LCF_LBPOutput.fHeadingCtrlRi = g_LCF_LBPOutput.fHeadingSafeRi;
        g_LCF_LBPOutput.fCrvCtrlRi = g_LCF_LBPOutput.fCrvSafeRi;
        g_LCF_LBPOutput.fCrvRateCtrlRi =
            reqPorts->sLCFLaneData
                .afCurvatureChange_1pm2[LCF_SEN_CAMERA_LANE_RI];

        g_LCF_LBPOutput.fPosX0CtrlLf = 0.f;
        g_LCF_LBPOutput.fPosY0CtrlLf =
            g_LCF_LBPOutput.fPosY0CtrlRi + fTempLaneWigth_met;
        g_LCF_LBPOutput.fHeadingCtrlLf = g_LCF_LBPOutput.fHeadingCtrlRi;
        g_LCF_LBPOutput.fCrvCtrlLf = g_LCF_LBPOutput.fCrvCtrlRi;
        g_LCF_LBPOutput.fCrvRateCtrlLf = g_LCF_LBPOutput.fCrvRateCtrlRi;
        g_LCF_LBPOutput.fValidLengthCtrlLf = g_LCF_LBPOutput.fValidLengthCtrlRi;

        // g_LCF_LBPOutput.fLaneWidth 				=
        // fTempLaneWigth_met;
    }
    g_LCF_LBPOutput.fLaneWidth =
        g_LCF_LBPOutput.fPosY0CtrlLf - g_LCF_LBPOutput.fPosY0CtrlRi;
    fTempLaneWigth_met = g_LCF_LBPOutput.fLaneWidth;

    g_LCF_LBPOutput.fPosX0CtrlCntr = 0.f;
    g_LCF_LBPOutput.fPosY0CtrlCntr =
        0.5f * (g_LCF_LBPOutput.fPosY0SafeLf + g_LCF_LBPOutput.fPosY0SafeRi);
    g_LCF_LBPOutput.fHeadingCtrlCntr = 0.5f * (g_LCF_LBPOutput.fHeadingSafeLf +
                                               g_LCF_LBPOutput.fHeadingSafeRi);
    g_LCF_LBPOutput.fCrvCtrlCntr =
        0.5f * (g_LCF_LBPOutput.fCrvSafeLf + g_LCF_LBPOutput.fCrvSafeRi);
    g_LCF_LBPOutput.fCrvRateCtrlCntr = 0.5f * (g_LCF_LBPOutput.fCrvRateCtrlLf +
                                               g_LCF_LBPOutput.fCrvRateCtrlRi);
    g_LCF_LBPOutput.fValidLengthCtrlCntr =
        0.5f * (g_LCF_LBPOutput.fValidLengthCtrlLf +
                g_LCF_LBPOutput.fValidLengthCtrlRi);
#endif

#if 0
	printf("LBP : g_LCF_LBPInput.bAvailable[0] = %u\n", g_LCF_LBPInput.bAvailable[0]);
	printf("LBP : g_LCF_LBPInput.bAvailable[1] = %u\n", g_LCF_LBPInput.bAvailable[1]);
	printf("LBP : g_LCF_LBPInput.fPosY0[0] = %f\n", g_LCF_LBPInput.fPosY0[0]);
	printf("LBP : g_LCF_LBPInput.fPosY0[1] = %f\n", g_LCF_LBPInput.fPosY0[1]);
	printf("LBP : g_LCF_LBPInput.fHeadingAngle[0] = %f\n", g_LCF_LBPInput.fHeadingAngle[0]);
	printf("LBP : g_LCF_LBPInput.fHeadingAngle[1] = %f\n", g_LCF_LBPInput.fHeadingAngle[1]);
	printf("LBP : g_LCF_LBPInput.fCurvature[0] = %f\n", g_LCF_LBPInput.fCurvature[0]);
	printf("LBP : g_LCF_LBPInput.fCurvature[1] = %f\n", g_LCF_LBPInput.fCurvature[1]);
	printf("LBP : g_LCF_LBPInput.fCurvatureRate[0] = %f\n", g_LCF_LBPInput.fCurvatureRate[0]);
	printf("LBP : g_LCF_LBPInput.fCurvatureRate[1] = %f\n", g_LCF_LBPInput.fCurvatureRate[1]);
	printf("LBP : g_LCF_LBPInput.fValidLength[0] = %f\n", g_LCF_LBPInput.fValidLength[0]);
	printf("LBP : g_LCF_LBPInput.fValidLength[1] = %f\n", g_LCF_LBPInput.fValidLength[1]);
	printf("LBP : g_LCF_LBPInput.uQuality[0] = %f\n", g_LCF_LBPInput.uQuality[0]);
	printf("LBP : g_LCF_LBPInput.uQuality[1] = %f\n", g_LCF_LBPInput.uQuality[1]);

    printf("LBP : g_LCF_LBPOutput.fPosY0SafeLf = %f\n", g_LCF_LBPOutput.fPosY0SafeLf);
	printf("LBP : g_LCF_LBPOutput.fPosY0SafeRi = %f\n", g_LCF_LBPOutput.fPosY0SafeRi);
    printf("LBP : g_LCF_LBPOutput.fPosY0CtrlCntr = %f\n", g_LCF_LBPOutput.fPosY0CtrlCntr);
    printf("LBP : g_LCF_LBPOutput.fPosY0CtrlLf = %f\n", g_LCF_LBPOutput.fPosY0CtrlLf);
    printf("LBP : g_LCF_LBPOutput.fPosY0CtrlRi = %f\n", g_LCF_LBPOutput.fPosY0CtrlRi);
    printf("LBP : g_LCF_LBPOutput.fLaneWidth = %f\n", g_LCF_LBPOutput.fLaneWidth);
#endif

    ALPInputWrapper(reqPorts, g_LCF_LBPOutput, reqParams, &g_LCF_ALPInput);
    LCF_ALP_Exec(&g_LCF_ALPInput, &g_LCF_ALPParam, &g_LCF_ALPOutput,
                 &(proDebugs->sALPDebug));
    ALPOutputWrapper(g_LCF_ALPOutput, proPorts);

#if 1
    static float32 fPrevLeftAdjLnPosY0 = 0.f;
    static float32 fPrevRightAdjLnPosY0 = 0.f;

    if ((g_LCF_ALPOutput.fLeftAdjLnDistance_met > 14.0f) &&
        (fPrevLeftAdjLnPosY0 < 10.0f)) {
        g_LCF_ALPOutput.fLeftAdjLnDistance_met = fPrevLeftAdjLnPosY0;
        fPrevLeftAdjLnPosY0 = 15.f;
    } else {
        fPrevLeftAdjLnPosY0 = g_LCF_ALPOutput.fLeftAdjLnDistance_met;
    }
    if ((g_LCF_ALPOutput.fRightAdjLnDistance_met < -14.0f) &&
        (fPrevRightAdjLnPosY0 > -10.0f)) {
        g_LCF_ALPOutput.fRightAdjLnDistance_met = fPrevRightAdjLnPosY0;
        fPrevRightAdjLnPosY0 = -15.f;
    } else {
        fPrevRightAdjLnPosY0 = g_LCF_ALPOutput.fRightAdjLnDistance_met;
    }
#endif

    // printf("ALP : g_LCF_ALPOutput.fLeftAdjLnDistance_met = %f\n",
    //        g_LCF_ALPOutput.fLeftAdjLnDistance_met);
    // printf("ALP : g_LCF_ALPOutput.fRightAdjLnDistance_met = %f\n",
    //        g_LCF_ALPOutput.fRightAdjLnDistance_met);

    LCFOPSInputWrapper(reqPorts, &g_LCF_OPSInput);
    LCFOPS_Exec(&g_LCF_OPSInput, &g_LCF_OPSParam, &g_LCF_OPSOutput,
                &(proDebugs->sLCFOPSDebug));

    LCCRAInputWrapper(reqPorts, g_LCF_OPSOutput, g_LCF_LBPOutput,
                      g_LCF_ALPOutput, &g_LCF_LCCRAInput);
    // printf("LCCRA input obj num:%d\n",
    //        g_LCF_LCCRAInput.Fusion_TargetObject_str->NumTgtObj);
    // printf("LCCRA obj 0:%d,%f,%f\n",
    //        g_LCF_LCCRAInput.Fusion_TargetObject_str->Objects[0].ID,
    //        g_LCF_LCCRAInput.Fusion_TargetObject_str->Objects[0].Position[0],
    //        g_LCF_LCCRAInput.Fusion_TargetObject_str->Objects[0].Position[1]);

    LCCRA_Exec(&g_LCF_LCCRAInput, &g_LCF_LCCRAParam, &g_LCF_LCCRAOutput,
               &(proDebugs->sLCCRADebug));
    // printf("g_LCF_LCCRAOutput:%d,%d,%d,%d,%d,%d\n",
    //        g_LCF_LCCRAOutput.LCCRA_LeftFrontSafeFlag_bool,
    //        g_LCF_LCCRAOutput.LCCRA_LeftRearSafeFlag_bool,
    //        g_LCF_LCCRAOutput.LCCRA_LeftSafeFlag_bool,
    //        g_LCF_LCCRAOutput.LCCRA_RightFrontSafeFlag_bool,
    //        g_LCF_LCCRAOutput.LCCRA_RightRearSafeFlag_bool,
    //        g_LCF_LCCRAOutput.LCCRA_RightSafeFlag_bool);
    LDWSAInputWrapper(reqPorts, g_LCF_VSDP_Output, g_LCF_LBPOutput, reqParams,
                      g_LCF_TJASA_Output, proDebugs->sTJASADebug,
                      &g_LCF_LDWSA_Input);
    LCF_LDWSA_Exec(&g_LCF_LDWSA_Input, &g_LCF_LDWSA_Param, &g_LCF_LDWSA_Output,
                   &(proDebugs->sLDWSADebug));
    LDWSAOutputWrapper(g_LCF_LDWSA_Output, proPorts);

    LDPSAInputWrapper(reqPorts, g_LCF_VSDP_Output, g_LCF_LBPOutput, reqParams,
                      g_LCF_TJASA_Output, proDebugs->sTJASADebug,
                      &g_LCF_LDPSA_Input);
    LCF_LDPSA_Exec(&g_LCF_LDPSA_Input, &g_LCF_LDPSA_Param, &g_LCF_LDPSA_Output,
                   &(proDebugs->sLDPSADebug));
    LDPSAOutputWrapper(g_LCF_LDPSA_Output, proPorts);

#if 0
	// printf("LDP : g_LCF_LDPSA_Input.LDPSAI_PstnXLf_Mi = %f\n", g_LCF_LDPSA_Input.LDPSAI_PstnXLf_Mi);
	printf("LDP : g_LCF_LDPSA_Input.LDPSAI_PstnYLf_Mi = %f\n", g_LCF_LDPSA_Input.LDPSAI_PstnYLf_Mi);
	// printf("LDP : g_LCF_LDPSA_Input.LDPSAI_HeadAglLf_Rad = %f\n", g_LCF_LDPSA_Input.LDPSAI_HeadAglLf_Rad);
	// printf("LDP : g_LCF_LDPSA_Input.LDPSAI_CurvLf_ReMi = %f\n", g_LCF_LDPSA_Input.LDPSAI_CurvLf_ReMi);
	printf("LDP : g_LCF_LDPSA_Input.LDPSAI_VldLengLf_Mi = %f\n", g_LCF_LDPSA_Input.LDPSAI_VldLengLf_Mi);
	// printf("LDP : g_LCF_LDPSA_Input.LDPSAI_PstnXRi_Mi = %f\n", g_LCF_LDPSA_Input.LDPSAI_PstnXRi_Mi);
	printf("LDP : g_LCF_LDPSA_Input.LDPSAI_PstnYRi_Mi = %f\n", g_LCF_LDPSA_Input.LDPSAI_PstnYRi_Mi);
	// printf("LDP : g_LCF_LDPSA_Input.LDPSAI_HeadAglRi_Rad = %f\n", g_LCF_LDPSA_Input.LDPSAI_HeadAglRi_Rad);
	// printf("LDP : g_LCF_LDPSA_Input.LDPSAI_CurvRi_ReMi = %f\n", g_LCF_LDPSA_Input.LDPSAI_CurvRi_ReMi);
	printf("LDP : g_LCF_LDPSA_Input.LDPSAI_VldLengRi_Mi = %f\n", g_LCF_LDPSA_Input.LDPSAI_VldLengRi_Mi);
 	// printf("LDP : g_LCF_LDPSA_Input.LDPSAI_VehWid_Mi = %f\n", g_LCF_LDPSA_Input.LDPSAI_VehWid_Mi);

	printf("LDP : g_LCF_LDPSA_Output.LDWC_SysOut_St = %d\n", g_LCF_LDPSA_Output.LDWC_SysOut_St);
	printf("LDP : g_LCF_LDPSA_Output.LDWC_DgrSide_St = %d\n", g_LCF_LDPSA_Output.LDWC_DgrSide_St);
	printf("LDP : g_LCF_LDPSA_Output.LDPSC_SysOut_St = %d\n", g_LCF_LDPSA_Output.LDPSC_SysOut_St);
	printf("LDP : g_LCF_LDPSA_Output.LDPSC_DgrSide_St = %d\n", g_LCF_LDPSA_Output.LDPSC_DgrSide_St);
	printf("LDP : sLDPSADebug.LDWC_TrigLf_B = %d\n", proDebugs->sLDPSADebug.LDWC_TrigLf_B);
	printf("LDP : sLDPSADebug.LDWC_TrigRi_B = %d\n", proDebugs->sLDPSADebug.LDWC_TrigRi_B);
	printf("LDP : sLDPSADebug.LDWC_Trig_B = %d\n", proDebugs->sLDPSADebug.LDWC_Trig_B);
	printf("LDP : sLDPSADebug.LDWC_StrgRdy_B = %d\n", proDebugs->sLDPSADebug.LDWC_StrgRdy_B);
	printf("LDP : sLDPSADebug.LDWC_WkRdy_B = %d\n", proDebugs->sLDPSADebug.LDWC_WkRdy_B);
	printf("LDP : sLDPSADebug.LDDT_DstcToLnLf_Mi = %f\n", proDebugs->sLDPSADebug.LDDT_DstcToLnLf_Mi);
	printf("LDP : sLDPSADebug.LDDT_LnPstnLf_Mi = %f\n", proDebugs->sLDPSADebug.LDDT_LnPstnLf_Mi);
	printf("LDP : sLDPSADebug.LDDT_RawDstcToLnLf_Mi = %f\n", proDebugs->sLDPSADebug.LDDT_RawDstcToLnLf_Mi);
	// printf("LDP : sLDPSADebug.LDWC_DstcToLnTrsdLf_Mi = %f\n", proDebugs->sLDPSADebug.LDWC_DstcToLnTrsdLf_Mi);
	printf("LDP : sLDPSADebug.LDDT_DstcToLnRi_Mi = %f\n", proDebugs->sLDPSADebug.LDDT_DstcToLnRi_Mi);
	printf("LDP : sLDPSADebug.LDDT_LnPstnRi_Mi = %f\n", proDebugs->sLDPSADebug.LDDT_LnPstnRi_Mi);
	printf("LDP : sLDPSADebug.LDDT_RawDstcToLnRi_Mi = %f\n", proDebugs->sLDPSADebug.LDDT_RawDstcToLnRi_Mi);
	// printf("LDP : sLDPSADebug.LDWC_DstcToLnTrsdRi_Mi = %f\n", proDebugs->sLDPSADebug.LDWC_DstcToLnTrsdRi_Mi);
	printf("LDP : sLDPSADebug.LDWC_RawTrigByDlcRi_B = %d\n", proDebugs->sLDPSADebug.LDWC_RawTrigByDlcRi_B);
	printf("LDP : sLDPSADebug.LDWC_EnaLdwTrigRi_B = %d\n", proDebugs->sLDPSADebug.LDWC_EnaLdwTrigRi_B);
	printf("LDPSC : sLDPSADebug.LDPSC_DgrFns_B = %d\n", proDebugs->sLDPSADebug.LDPSC_DgrFns_B);
	printf("LDPSC : sLDPSADebug.LDPSC_Cancel_B = %d\n", proDebugs->sLDPSADebug.LDPSC_Cancel_B);

	printf("LDP : LDPTT_RawLnBdryPstnYLf_Mi = %f\n", proDebugs->sLDPSADebug.LDPTT_RawLnBdryPstnYLf_Mi);
 	printf("LDP : LDPTT_RawLnBdryPstnYRi_Mi = %f\n", proDebugs->sLDPSADebug.LDPTT_RawLnBdryPstnYRi_Mi);	
	printf("LDP : LDPTT_LnBdryPstnYLf_Mi = %f\n", proDebugs->sLDPSADebug.LDPTT_LnBdryPstnYLf_Mi);
 	printf("LDP : LDPTT_LnBdryPstnYRi_Mi = %f\n", proDebugs->sLDPSADebug.LDPTT_LnBdryPstnYRi_Mi);
	printf("LDP : LDPTT_TgtPstnYLf_Mi = %f\n", proDebugs->sLDPSADebug.LDPTT_TgtPstnYLf_Mi);
 	printf("LDP : LDPTT_TgtPstnYLf_Mi = %f\n", proDebugs->sLDPSADebug.LDPTT_TgtPstnYRi_Mi);	
	printf("LDP : LDPTT_LnBdryPstnYCent_Mi = %f\n", proDebugs->sLDPSADebug.LDPTT_LnBdryPstnYCent_Mi);
	printf("LDP : LDPTT_LnBdryPstnYCent_Mi = %f\n", proDebugs->sLDPSADebug.LDPTT_LnBdryPstnYCent_Mi);
	printf("LDP : LDPTT_LnBdryPstnYCent_Mi = %f\n", proDebugs->sLDPSADebug.LDPTT_LnBdryPstnYCent_Mi);
	printf("LDP : LDPTT_LnBdryPstnYCent_Mi = %f\n", proDebugs->sLDPSADebug.LDPTT_LnBdryPstnYCent_Mi);
	printf("LDP : LDPTV_TrajCtrlSt_St = %d\n", proDebugs->sLDPSADebug.LDPTV_TrajCtrlSt_St);
	
 	// printf("LDP : LDPTT_LnBdryHeadAglLf_Rad = %f\n", proDebugs->sLDPSADebug.LDPTT_LnBdryHeadAglLf_Rad);
 	// printf("LDP : LDPTT_LnBdryHeadAglRi_Rad = %f\n", proDebugs->sLDPSADebug.LDPTT_LnBdryHeadAglRi_Rad);
 	// printf("LDP : LDPTT_LnBdryCurvLf_ReMi = %f\n", proDebugs->sLDPSADebug.LDPTT_LnBdryCurvLf_ReMi);
 	// printf("LDP : LDPTT_LnBdryCurvRi_ReMi = %f\n", proDebugs->sLDPSADebug.LDPTT_LnBdryCurvRi_ReMi);
 	// printf("LDP : LDPTT_LnBdryVldLengLf_Mi = %f\n", proDebugs->sLDPSADebug.LDPTT_LnBdryVldLengLf_Mi);
 	// printf("LDP : LDPTT_LnBdryVldLengRi_Mi = %f\n", proDebugs->sLDPSADebug.LDPTT_LnBdryVldLengRi_Mi);

#endif

    ODPRInputWrapper(reqPorts, g_LCF_LBPOutput, reqParams, &g_LCF_ODPR_Input);
    LCF_ODPR_Exec(&g_LCF_ODPR_Input, &g_LCF_ODPR_Param, &g_LCF_ODPR_Output,
                  &(proDebugs->sODPRDebug));

    TJASAInputWrapper(reqPorts, reqParams, g_LCF_LBPOutput, g_LCF_ALPOutput,
                      g_LCF_ODPR_Output, g_LCF_VSDP_Output, g_LCF_LCCRAOutput,
                      &g_LCF_TJASA_Input);

#if 0
	static float32 ftemp;
	if (g_LCF_TJASA_Input.LCFRCV_TurnSignalLeft_bool)
	{

	}
	g_LCF_TJASA_Input.LCFRCV_TurnSignalLeft_bool = BASICTurnOffDelay(g_LCF_TJASA_Input.LCFRCV_TurnSignalLeft_bool,8.0,0.1f,&ftemp);

#endif

    LCF_TJASA_Exec(&g_LCF_TJASA_Input, &g_LCF_TJASA_Param, &g_LCF_TJASA_Output,
                   &(proDebugs->sTJASADebug));
    TJASAOutputWrapper(g_LCF_TJASA_Output, proPorts);

#if 0
	printf("TJA : LCFRCV_TJASwitch_nu = %d\n", g_LCF_TJASA_Input.LCFRCV_TJASwitch_nu);	
	printf("TJA : VDy_VehVelocity_kph = %f\n", g_LCF_TJASA_Input.VDy_VehVelocity_kph);	
	printf("TJA : ABPR_LeCrvQuality_perc = %d\n", g_LCF_TJASA_Input.ABPR_LeCrvQuality_perc);	
	printf("TJA : ABPR_LeLnQuality_perc = %d\n", g_LCF_TJASA_Input.ABPR_LeLnQuality_perc);	
	printf("TJA : ABPR_RiCrvQuality_perc = %d\n", g_LCF_TJASA_Input.ABPR_RiCrvQuality_perc);	
	printf("TJA : ABPR_RiLnQuality_perc = %d\n", g_LCF_TJASA_Input.ABPR_RiLnQuality_perc);
	printf("TJA : ABPR_LeLnInvalidQu_btf = %d\n", g_LCF_TJASA_Input.ABPR_LeLnInvalidQu_btf);
	printf("TJA : ABPR_RiLnInvalidQu_btf = %d\n", g_LCF_TJASA_Input.ABPR_RiLnInvalidQu_btf);
	printf("TJA : LCFRCV_TurnSignalLeft_bool = %d\n", g_LCF_TJASA_Input.LCFRCV_TurnSignalLeft_bool);
	printf("TJA : LCFRCV_TurnSignalRight_bool = %d\n", g_LCF_TJASA_Input.LCFRCV_TurnSignalRight_bool);

	printf("TJASTM : TJASTM_SysStateTJA_nu = %d\n", g_LCF_TJASA_Output.TJASTM_SysStateTJA_nu);	
	printf("TJASTM : TJASTM_LatCtrlMode_nu = %d\n", g_LCF_TJASA_Output.TJASTM_LatCtrlMode_nu);

	printf("TJAGEN : TJAGEN_StrongReady_bool = %d\n", proDebugs->sTJASADebug.TJAGEN_StrongReady_bool);
	printf("TJAGEN : TJAGEN_WeakReady_bool = %d\n", proDebugs->sTJASADebug.TJAGEN_WeakReady_bool);
	printf("TJAGEN : TJAGEN_Clearance_bool = %d\n", proDebugs->sTJASADebug.TJAGEN_Clearance_bool);

	printf("TJALKA : TJALKA_StrongReady_bool = %d\n", proDebugs->sTJASADebug.TJALKA_StrongReady_bool);	
	printf("TJALKA : TJALKA_WeakReady_bool = %d\n", proDebugs->sTJASADebug.TJALKA_WeakReady_bool);	
	printf("TJALKA : TJALKA_Cancel_bool = %d\n", proDebugs->sTJASADebug.TJALKA_Cancel_bool);	
	printf("TJALKA : TJALKA_LaneCenterInvalid_btf = %d\n", proDebugs->sTJASADebug.TJALKA_LaneCenterInvalid_btf);	
	printf("TJALKA : TJALKA_LnQualityInv_btf = %d\n", proDebugs->sTJASADebug.TJALKA_LnQualityInv_btf);	
	printf("TJALKA : TJALKA_LnBndValid_nu = %d\n", proDebugs->sTJASADebug.TJALKA_LnBndValid_nu);		
	printf("TJALKA : LKA_LeLnCrvQualityValid_bool = %d\n", proDebugs->sTJASADebug.LKA_LeLnCrvQualityValid_bool);		
	printf("TJALKA : LKA_LeLnQualityValid_bool = %d\n", proDebugs->sTJASADebug.LKA_LeLnQualityValid_bool);	

	printf("TJASLC : TJASLC_StrongReady_bool = %d\n", proDebugs->sTJASADebug.TJASLC_StrongReady_bool);	
	printf("TJASLC : TJASLC_WeakReady_bool = %d\n", proDebugs->sTJASADebug.TJASLC_WeakReady_bool);	
	printf("TJASLC : TJASLC_Cancel_bool = %d\n", proDebugs->sTJASADebug.TJASLC_Cancel_bool);
	printf("TJASLC : TJASLC_ManeuverState_nu = %d\n", proDebugs->sTJASADebug.TJASLC_ManeuverState_nu);
	printf("TJASLC : TJASLC_LeLaneChangeInvalid_btf = %d\n", proDebugs->sTJASADebug.TJASLC_LeLaneChangeInvalid_btf);	
	printf("TJASLC : TJASLC_RiLaneChangeInvalid_btf = %d\n", proDebugs->sTJASADebug.TJASLC_RiLaneChangeInvalid_btf);	
	printf("TJASLC : TJASLC_TriggerInvalid_btf = %d\n", proDebugs->sTJASADebug.TJASLC_TriggerInvalid_btf);	

    printf("TJATTG : TJATTG_LeCridrBndPosY0_met = %f\n", g_LCF_TJASA_Output.TJATTG_LeCridrBndPosY0_met);
    printf("TJATTG : TJATTG_RiCridrBndPosY0_met = %f\n", g_LCF_TJASA_Output.TJATTG_RiCridrBndPosY0_met);
    printf("TJATTG : TJATTG_TgtTrajPosY0_met = %f\n", g_LCF_TJASA_Output.TJATTG_TgtTrajPosY0_met);

	printf("TJATTG : TJATTG_TransTriggerReplan_bool = %d\n", g_LCF_TJASA_Output.TJATTG_TransTriggerReplan_bool);
#endif

    MCTLFCInputWrapper(reqPorts, reqParams, &g_LCF_TJASA_Output,
                       g_LCF_LDPSA_Output, proDebugs->sLDPSADebug,
                       &g_LCF_MCTLFC_Input);
    MCTLFC_Exec(&g_LCF_MCTLFC_Input, &g_LCF_MCTLFC_Param, &g_LCF_MCTLFC_Output,
                &(proDebugs->sMCTLFCDebug));
    MCTLFCOutputWrapper(g_LCF_MCTLFC_Output, proPorts);
    // printf("g_LCF_MCTLFC_Output.CSCLTA_TrajGuiQualifier_nu = %d\n",
    //        g_LCF_MCTLFC_Output.CSCLTA_TrajGuiQualifier_nu);
    // printf("g_LCF_MCTLFC_Output.CSCLTA_ControllingFunction_nu = %d\n",
    //        g_LCF_MCTLFC_Output.CSCLTA_ControllingFunction_nu);
}

/* ****************************************************************************
 Functionname:     LBPInputWrapper
 @brief: the input wrapper function for the LBP process

 @description: the input wrapper function for the LBP process

 @param[in]reqLcfSenPrtList_t:the input of the LCF whole module

 @param[out]pLBPInput: the input of the LBP sub-module

 @return           -
 @pre
 @post             -
 @author
**************************************************************************** */
static void LBPInputWrapper(const reqLcfSenPrtList_t* pLCFInput,
                            sLBPInput_t* pLBPInput) {
    pLBPInput->bAvailable[UN_CPL_LF] =
        pLCFInput->sLCFLaneData.abAvailable[LCF_SEN_CAMERA_LANE_LE];
    pLBPInput->bAvailable[CPL_LF] =
        pLCFInput->sLCFLaneData.abAvailable[LCF_SEN_CAMERA_LANE_LE];
    pLBPInput->bAvailable[UN_CPL_RI] =
        pLCFInput->sLCFLaneData.abAvailable[LCF_SEN_CAMERA_LANE_RI];
    pLBPInput->bAvailable[CPL_RI] =
        pLCFInput->sLCFLaneData.abAvailable[LCF_SEN_CAMERA_LANE_RI];

    pLBPInput->bVitural[UN_CPL_LF] =
        pLCFInput->sLCFLaneData.abVitural[LCF_SEN_CAMERA_LANE_LE];
    pLBPInput->bVitural[CPL_LF] =
        pLCFInput->sLCFLaneData.abVitural[LCF_SEN_CAMERA_LANE_LE];
    pLBPInput->bVitural[UN_CPL_RI] =
        pLCFInput->sLCFLaneData.abVitural[LCF_SEN_CAMERA_LANE_RI];
    pLBPInput->bVitural[CPL_RI] =
        pLCFInput->sLCFLaneData.abVitural[LCF_SEN_CAMERA_LANE_RI];

    pLBPInput->bCompState = pLCFInput->sLCFLaneData.uiCompState_nu;
    pLBPInput->bParallelModelActiv =
        pLCFInput->sLCFLaneData.bParallelModelActiv;
    pLBPInput->fAgeMeter = 0.f;
    pLBPInput->fCurvature[UN_CPL_LF] =
        pLCFInput->sLCFLaneData.afCurvature_1pm[LCF_SEN_CAMERA_LANE_LE];
    pLBPInput->fCurvature[CPL_LF] =
        pLCFInput->sLCFLaneData.afCurvature_1pm[LCF_SEN_CAMERA_LANE_LE];
    pLBPInput->fCurvature[UN_CPL_RI] =
        pLCFInput->sLCFLaneData.afCurvature_1pm[LCF_SEN_CAMERA_LANE_RI];
    pLBPInput->fCurvature[CPL_RI] =
        pLCFInput->sLCFLaneData.afCurvature_1pm[LCF_SEN_CAMERA_LANE_RI];

    pLBPInput->fCurvatureRate[UN_CPL_LF] =
        pLCFInput->sLCFLaneData.afCurvatureChange_1pm2[LCF_SEN_CAMERA_LANE_LE];
    pLBPInput->fCurvatureRate[CPL_LF] =
        pLCFInput->sLCFLaneData.afCurvatureChange_1pm2[LCF_SEN_CAMERA_LANE_LE];
    pLBPInput->fCurvatureRate[UN_CPL_RI] =
        pLCFInput->sLCFLaneData.afCurvatureChange_1pm2[LCF_SEN_CAMERA_LANE_RI];
    pLBPInput->fCurvatureRate[CPL_RI] =
        pLCFInput->sLCFLaneData.afCurvatureChange_1pm2[LCF_SEN_CAMERA_LANE_RI];

    pLBPInput->fEventDistance[UN_CPL_LF] =
        pLCFInput->sLCFLaneData.afEventDistance_met[LCF_SEN_CAMERA_LANE_LE];
    pLBPInput->fEventDistance[CPL_LF] =
        pLCFInput->sLCFLaneData.afEventDistance_met[LCF_SEN_CAMERA_LANE_LE];
    pLBPInput->fEventDistance[UN_CPL_RI] =
        pLCFInput->sLCFLaneData.afEventDistance_met[LCF_SEN_CAMERA_LANE_RI];
    pLBPInput->fEventDistance[CPL_RI] =
        pLCFInput->sLCFLaneData.afEventDistance_met[LCF_SEN_CAMERA_LANE_RI];
    pLBPInput->fFeatureCoordsXYLe = 0.f;
    pLBPInput->fFeatureCoordsXYRi = 0.f;

    pLBPInput->fHeadingAngle[UN_CPL_LF] =
        pLCFInput->sLCFLaneData.afHeadingAngle_rad[LCF_SEN_CAMERA_LANE_LE];
    pLBPInput->fHeadingAngle[CPL_LF] =
        pLCFInput->sLCFLaneData.afHeadingAngle_rad[LCF_SEN_CAMERA_LANE_LE];
    pLBPInput->fHeadingAngle[UN_CPL_RI] =
        pLCFInput->sLCFLaneData.afHeadingAngle_rad[LCF_SEN_CAMERA_LANE_RI];
    pLBPInput->fHeadingAngle[CPL_RI] =
        pLCFInput->sLCFLaneData.afHeadingAngle_rad[LCF_SEN_CAMERA_LANE_RI];

    pLBPInput->fPosY0[UN_CPL_LF] =
        pLCFInput->sLCFLaneData.afPosY0_met[LCF_SEN_CAMERA_LANE_LE];
    pLBPInput->fPosY0[CPL_LF] =
        pLCFInput->sLCFLaneData.afPosY0_met[LCF_SEN_CAMERA_LANE_LE];
    pLBPInput->fPosY0[UN_CPL_RI] =
        pLCFInput->sLCFLaneData.afPosY0_met[LCF_SEN_CAMERA_LANE_RI];
    pLBPInput->fPosY0[CPL_RI] =
        pLCFInput->sLCFLaneData.afPosY0_met[LCF_SEN_CAMERA_LANE_RI];

    pLBPInput->fSineWaveDtct = pLCFInput->sLCFLaneData.fSineWaveDtct_nu;
    pLBPInput->fStdDevCurvature[UN_CPL_LF] =
        pLCFInput->sLCFLaneData.afStdDevCurvature_1pm[LCF_SEN_CAMERA_LANE_LE];
    pLBPInput->fStdDevCurvature[CPL_LF] =
        pLCFInput->sLCFLaneData.afStdDevCurvature_1pm[LCF_SEN_CAMERA_LANE_LE];
    pLBPInput->fStdDevCurvature[UN_CPL_RI] =
        pLCFInput->sLCFLaneData.afStdDevCurvature_1pm[LCF_SEN_CAMERA_LANE_RI];
    pLBPInput->fStdDevCurvature[CPL_RI] =
        pLCFInput->sLCFLaneData.afStdDevCurvature_1pm[LCF_SEN_CAMERA_LANE_RI];

    pLBPInput->fStdDevCurvatureRate[UN_CPL_LF] =
        pLCFInput->sLCFLaneData
            .afStdDevCurvatureChange_1pm2[LCF_SEN_CAMERA_LANE_LE];
    pLBPInput->fStdDevCurvatureRate[CPL_LF] =
        pLCFInput->sLCFLaneData
            .afStdDevCurvatureChange_1pm2[LCF_SEN_CAMERA_LANE_LE];
    pLBPInput->fStdDevCurvatureRate[UN_CPL_RI] =
        pLCFInput->sLCFLaneData
            .afStdDevCurvatureChange_1pm2[LCF_SEN_CAMERA_LANE_RI];
    pLBPInput->fStdDevCurvatureRate[CPL_RI] =
        pLCFInput->sLCFLaneData
            .afStdDevCurvatureChange_1pm2[LCF_SEN_CAMERA_LANE_RI];

    pLBPInput->fStdDevHeadingAngle[UN_CPL_LF] =
        pLCFInput->sLCFLaneData
            .afStdDevHeadingAngle_rad[LCF_SEN_CAMERA_LANE_LE];
    pLBPInput->fStdDevHeadingAngle[CPL_LF] =
        pLCFInput->sLCFLaneData
            .afStdDevHeadingAngle_rad[LCF_SEN_CAMERA_LANE_LE];
    pLBPInput->fStdDevHeadingAngle[UN_CPL_RI] =
        pLCFInput->sLCFLaneData
            .afStdDevHeadingAngle_rad[LCF_SEN_CAMERA_LANE_RI];
    pLBPInput->fStdDevHeadingAngle[CPL_RI] =
        pLCFInput->sLCFLaneData
            .afStdDevHeadingAngle_rad[LCF_SEN_CAMERA_LANE_RI];

    pLBPInput->fStdDevPosY0[UN_CPL_LF] =
        pLCFInput->sLCFLaneData.afStdDevPosY0_met[LCF_SEN_CAMERA_LANE_LE];
    pLBPInput->fStdDevPosY0[CPL_LF] =
        pLCFInput->sLCFLaneData.afStdDevPosY0_met[LCF_SEN_CAMERA_LANE_LE];
    pLBPInput->fStdDevPosY0[UN_CPL_RI] =
        pLCFInput->sLCFLaneData.afStdDevPosY0_met[LCF_SEN_CAMERA_LANE_RI];
    pLBPInput->fStdDevPosY0[CPL_RI] =
        pLCFInput->sLCFLaneData.afStdDevPosY0_met[LCF_SEN_CAMERA_LANE_RI];

    pLBPInput->fSysCycleTime = pLCFInput->sBaseCtrlData.fSystemSenCylceTime_sec;
    pLBPInput->fTstamp =
        (float32)(pLCFInput->sLCFLaneData.sSigHeader.uiTimeStamp) / 1000000.f;

    pLBPInput->fValidLength[UN_CPL_LF] =
        pLCFInput->sLCFLaneData.afValidLength_met[LCF_SEN_CAMERA_LANE_LE];
    pLBPInput->fValidLength[CPL_LF] =
        pLCFInput->sLCFLaneData.afValidLength_met[LCF_SEN_CAMERA_LANE_LE];
    pLBPInput->fValidLength[UN_CPL_RI] =
        pLCFInput->sLCFLaneData.afValidLength_met[LCF_SEN_CAMERA_LANE_RI];
    pLBPInput->fValidLength[CPL_RI] =
        pLCFInput->sLCFLaneData.afValidLength_met[LCF_SEN_CAMERA_LANE_RI];

    pLBPInput->fVehVelX = pLCFInput->sLCFVehDyn.fEgoVelX_mps;
    pLBPInput->fVehYawRate = pLCFInput->sLCFVehDyn.fEgoYawRate_rps;
    pLBPInput->fVehYawRateStd = pLCFInput->sLCFVehDyn.fEgoVehYawRateStd_rpss;
    pLBPInput->fVertAvailable = 0.f;
    pLBPInput->fVertCurvature = 0.f;
    pLBPInput->fVertCurvatureRate = 0.f;
    pLBPInput->fVertSlopeChange = pLCFInput->sLCFLaneData.fVertSlopeChange_nu;
    pLBPInput->fVertValidLength = 0.f;
    pLBPInput->uCamStatusQualifier =
        pLCFInput->sLCFLaneData.uiCamStuatuQualifier_nu;

    pLBPInput->uColor[UN_CPL_LF] =
        pLCFInput->sLCFLaneData.auColor_nu[LCF_SEN_CAMERA_LANE_LE];
    pLBPInput->uColor[CPL_LF] =
        pLCFInput->sLCFLaneData.auColor_nu[LCF_SEN_CAMERA_LANE_LE];
    pLBPInput->uColor[UN_CPL_RI] =
        pLCFInput->sLCFLaneData.auColor_nu[LCF_SEN_CAMERA_LANE_RI];
    pLBPInput->uColor[CPL_RI] =
        pLCFInput->sLCFLaneData.auColor_nu[LCF_SEN_CAMERA_LANE_RI];

    pLBPInput->uEventQuality[UN_CPL_LF] =
        pLCFInput->sLCFLaneData.auEventQuality_nu[LCF_SEN_CAMERA_LANE_LE];
    pLBPInput->uEventQuality[CPL_LF] =
        pLCFInput->sLCFLaneData.auEventQuality_nu[LCF_SEN_CAMERA_LANE_LE];
    pLBPInput->uEventQuality[UN_CPL_RI] =
        pLCFInput->sLCFLaneData.auEventQuality_nu[LCF_SEN_CAMERA_LANE_RI];
    pLBPInput->uEventQuality[CPL_RI] =
        pLCFInput->sLCFLaneData.auEventQuality_nu[LCF_SEN_CAMERA_LANE_RI];

    pLBPInput->uEventType[UN_CPL_LF] =
        pLCFInput->sLCFLaneData.auEventType_nu[LCF_SEN_CAMERA_LANE_LE];
    pLBPInput->uEventType[CPL_LF] =
        pLCFInput->sLCFLaneData.auEventType_nu[LCF_SEN_CAMERA_LANE_LE];
    pLBPInput->uEventType[UN_CPL_RI] =
        pLCFInput->sLCFLaneData.auEventType_nu[LCF_SEN_CAMERA_LANE_RI];
    pLBPInput->uEventType[CPL_RI] =
        pLCFInput->sLCFLaneData.auEventType_nu[LCF_SEN_CAMERA_LANE_RI];

    pLBPInput->uLaneChange = pLCFInput->sLCFLaneData.uLaneChange_nu;
    pLBPInput->uLeftIndex = pLCFInput->sLCFLaneData.uLeftIndex_nu;

    pLBPInput->uMarkerType[UN_CPL_LF] =
        pLCFInput->sLCFLaneData.auMarkerType_nu[LCF_SEN_CAMERA_LANE_LE];
    pLBPInput->uMarkerType[CPL_LF] =
        pLCFInput->sLCFLaneData.auMarkerType_nu[LCF_SEN_CAMERA_LANE_LE];
    pLBPInput->uMarkerType[UN_CPL_RI] =
        pLCFInput->sLCFLaneData.auMarkerType_nu[LCF_SEN_CAMERA_LANE_RI];
    pLBPInput->uMarkerType[CPL_RI] =
        pLCFInput->sLCFLaneData.auMarkerType_nu[LCF_SEN_CAMERA_LANE_RI];

    pLBPInput->uQuality[UN_CPL_LF] =
        pLCFInput->sLCFLaneData.auQuality_nu[LCF_SEN_CAMERA_LANE_LE];
    pLBPInput->uQuality[CPL_LF] =
        pLCFInput->sLCFLaneData.auQuality_nu[LCF_SEN_CAMERA_LANE_LE];
    pLBPInput->uQuality[UN_CPL_RI] =
        pLCFInput->sLCFLaneData.auQuality_nu[LCF_SEN_CAMERA_LANE_RI];
    pLBPInput->uQuality[CPL_RI] =
        pLCFInput->sLCFLaneData.auQuality_nu[LCF_SEN_CAMERA_LANE_RI];

    pLBPInput->uRightIndex = pLCFInput->sLCFLaneData.uRightIndex_nu;
    pLBPInput->uRoadWorks = pLCFInput->sLCFLaneData.uRoadWorks_btf;
    pLBPInput->uWeatherCond = pLCFInput->sLCFLaneData.uWeatherCond_nu;
}

/* ****************************************************************************
 Functionname:     LBPOutputWrapper
 @brief: the output wrapper function for the LBP process

 @description: convert LBP sub-module's result to LCF Sen output structure

 @param[in]
                        const sLBPOutput_t sLBPOutput: LBP sub-module's output
 @param[out]
                        proLcfSenPrtList_t* LCFSenOutput: LCF Sen output pointer
 @return           -
 @pre
 @post             -
 @author
**************************************************************************** */
static void LBPOutputWrapper(const sLBPOutput_t sLBPOutput,
                             proLcfSenPrtList_t* LCFSenOutput) {
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.uLaneInvalidQualifierLf =
        sLBPOutput.uLaneInvalidQualifierLf; /* Qualifier for left lane invalid,
                                               (0, 0~65535, -) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.uLaneInvalidQualifierRi =
        sLBPOutput.uLaneInvalidQualifierRi; /* Qualifier for right lane invalid,
                                               (0, 0~65535, -)*/
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.fLaneWidth =
        sLBPOutput.fLaneWidth; /* Lane width by uncoupled lane processing (0,
                                  0~10, m) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.bLaneChangeDtct =
        sLBPOutput.bLaneChangeDtct; /* Flag that indicates a detected lane
                                       change, (0~1, -, ) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.fPosX0CtrlLf =
        sLBPOutput.fPosX0CtrlLf; /* Left lane clothoid X0 position, (0,
                                    -300~300, m) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.fPosY0CtrlLf =
        sLBPOutput.fPosY0CtrlLf; /* Left lane clothoid Y0
                                    position  (init +10m), (0,
                                    -15~15, m) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.fHeadingCtrlLf =
        sLBPOutput.fHeadingCtrlLf; /* Left lane clothoid heading angle, (0,
                                      -0.7854~0.7854, rad) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.fCrvCtrlLf =
        sLBPOutput
            .fCrvCtrlLf; /* Left lane clothoid curvature, (0, -0.1~0.1, 1/m) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.fCrvRateCtrlLf =
        sLBPOutput.fCrvRateCtrlLf; /* Left lane clothoid change of curvature,
                                      (0, -0.001~0.001, 1/m^2) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.fValidLengthCtrlLf =
        sLBPOutput
            .fValidLengthCtrlLf; /* Left lane clothoid length, (0, 0~300, m) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.fPosX0CtrlCntr =
        sLBPOutput.fPosX0CtrlCntr; /* Center lane clothoid X0 position, (0,
                                      -300~300, m) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.fPosY0CtrlCntr =
        sLBPOutput.fPosY0CtrlCntr; /* Center lane clothoid Y0 position (init
                                      +10m), (0, -15~15, m) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.fHeadingCtrlCntr =
        sLBPOutput.fHeadingCtrlCntr; /* Center lane clothoid heading angle, (0,
                                        -0.7854~0.7854, rad) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.fCrvCtrlCntr =
        sLBPOutput.fCrvCtrlCntr; /* Center lane clothoid curvature, (0,
                                    -0.1~0.1, 1/m*/
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.fCrvRateCtrlCntr =
        sLBPOutput.fCrvRateCtrlCntr; /* Center lane clothoid change of
                                        curvature, (0, -0.001~0.001, 1/m^2) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.fValidLengthCtrlCntr =
        sLBPOutput.fValidLengthCtrlCntr; /* Center lane clothoid length, (0,
                                            0~300, m) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.fPosX0CtrlRi =
        sLBPOutput.fPosX0CtrlRi; /* Right lane clothoid X0 position, (0,
                                    -300~300, m) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.fPosY0CtrlRi =
        sLBPOutput.fPosY0CtrlRi; /* Right lane clothoid Y0
                                    position (init +10m), (0,
                                    -15~15, m) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.fHeadingCtrlRi =
        sLBPOutput.fHeadingCtrlRi; /* Right lane clothoid heading angle, (0,
                                      -0.7854~0.7854, rad) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.fCrvCtrlRi =
        sLBPOutput
            .fCrvCtrlRi; /* Right lane clothoid curvature, (0, -0.1~0.1, 1/m) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.fCrvRateCtrlRi =
        sLBPOutput.fCrvRateCtrlRi; /* Right lane clothoid change of curvature,
                                      (0, -0.001~0.001, 1/m^2) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.fValidLengthCtrlRi =
        sLBPOutput
            .fValidLengthCtrlRi; /* Right lane clothoid length, (0, 0~300, m) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.fPosY0SafeLf =
        sLBPOutput.fPosY0SafeLf; /* Left lane clothoid Y0 position (safety
                                    interface), (-15~15, m)*/
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.fHeadingSafeLf =
        sLBPOutput.fHeadingSafeLf; /* Left lane clothoid heading angle (safety
                                      interface), (-0.7854~0.7854, rad) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.fCrvSafeLf =
        sLBPOutput.fCrvSafeLf; /* Left lane clothoid curvature (safety
                                  interface), (-0.1~0.1, 1/m) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.fPosY0SafeRi =
        sLBPOutput.fPosY0SafeRi; /* Right lane clothoid Y0 position (safety
                                    interface), (-15~15, m) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.fHeadingSafeRi =
        sLBPOutput.fHeadingSafeRi; /* Right lane clothoid heading angle (safety
                                      interface), (-0.7854~0.7854, rad) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.fCrvSafeRi =
        sLBPOutput.fCrvSafeRi; /* Right lane clothoid curvature (safety
                                  interface), (-0.1~0.1, 1/m) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.uInvalidQualifierSafeLf =
        sLBPOutput
            .uInvalidQualifierSafeLf; /* Left lane invalid qualifier for the
                                         safety interface, (0~255, -) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.uInvalidQualifierSafeRi =
        sLBPOutput
            .uInvalidQualifierSafeRi; /* Right lane invalid qualifier for the
                                         safety interface, (0~255, -) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.uLaneValidQualDMC =
        sLBPOutput.uLaneValidQualDMC; /* Lane valid qualifier for LatDMC, (0,
                                         0~255, -) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.uVisualValidQualifier =
        sLBPOutput.uVisualValidQualifier; /* Qualifier for the visualization,
                                             (0, 0~255, -) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.uLaneTypeLf =
        sLBPOutput.uLaneTypeLf; /* Left lane type(eg,9 means road edge) ,
                                   (0~255, -) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.uLaneTypeRi =
        sLBPOutput.uLaneTypeRi; /* Right lane type(eg,9 means road edge) ,
                                   (0~255, -) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.bConstructionSiteDtct =
        sLBPOutput.bConstructionSiteDtct; /* 1: Construction site detected =
                                             sLBPOutput.; 0: No construction
                                             site (0~1, -) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.uOverallQualityLf =
        sLBPOutput.uOverallQualityLf; /* Quality of the left  lane information
                                         based on all properties, (0~255, %)*/
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.uOverallQualityRi =
        sLBPOutput.uOverallQualityRi; /* Quality of the right lane information
                                         based on all properties, (0~255, %) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.uCrvQualityLf =
        sLBPOutput.uCrvQualityLf; /* Quality of the left
                                     curvature information,
                                     (0~255, %) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.uCrvQualityRi =
        sLBPOutput.uCrvQualityRi; /* Quality of the right
                                     curvature information,
                                     (0~255, %) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.fABDTimeStamp =
        sLBPOutput
            .fABDTimeStamp; /* ABD data timestamp in seconds , (0~4295, s) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.uRangeCheckQualifier =
        sLBPOutput.uRangeCheckQualifier; /* Input range check qualifier bitfield
                                            , (0~65535, s) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.fFltQualityCntr =
        sLBPOutput
            .fFltQualityCntr; /* Center lane Kalman quality, (0, 0~100, %) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.uFltStatusCntr =
        sLBPOutput
            .uFltStatusCntr; /* Center lane Kalman filter status, (0, 0~5, -) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.uOutRangeCheckQualifier =
        sLBPOutput.uOutRangeCheckQualifier; /* Output data range qualifier, (0,
                                               0~65535, -) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.uPercStraightDtct =
        sLBPOutput.uPercStraightDtct; /* Confidence for straight detection -
                                         value from 0 to 100, (0, 0~100, %) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.uPercExitLf =
        sLBPOutput.uPercExitLf; /* Exit percent for left side, (0, 0~100, %) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.uPercExitRi =
        sLBPOutput.uPercExitRi; /* Exit percent for right side, (0, 0~100, %) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.bBridgePossibleUnCplLf =
        sLBPOutput.bBridgePossibleUnCplLf; /* Left lane uncoupled lane bridging
                                              possible, (0, 0~1, -) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.bBridgePossibleUnCplRi =
        sLBPOutput.bBridgePossibleUnCplRi; /* Right lane uncoupled lane bridging
                                              possible, (0, 0~1, -) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.uPercUpDownHillDtct =
        sLBPOutput
            .uPercUpDownHillDtct; /* Indicates downhill / uphill scenario
                                     detection confidence, (0, 0~100, %) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.fLaneWidthUnCpl =
        sLBPOutput.fLaneWidthUnCpl; /* Raw lane width of ABD
                                       interface uncoupled lane
                                       data, (0, 0~10, m) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.fLaneWidthCpl =
        sLBPOutput.fLaneWidthCpl; /* Raw lane width of ABD interface coupled
                                     lane data, (0, 0~10, m) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.uOverallQualityUnCplLf =
        sLBPOutput.uOverallQualityUnCplLf; /* Left uncoupled lane quality, (0,
                                              0~100, %) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.uOverallQualityUnCplRi =
        sLBPOutput.uOverallQualityUnCplRi; /* Right uncoupled lane quality, (0,
                                              0~100, %) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.uOverallQualityCplLf =
        sLBPOutput.uOverallQualityCplLf; /* Left coupled lane quality, (0,
                                            0~100, %) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.uOverallQualityCplRi =
        sLBPOutput.uOverallQualityCplRi; /* Right coupled lane quality, (0,
                                            0~100, %) */
    LCFSenOutput->pLcfSenOutputData.sLBPOutput.uBtfBridgeUnCpl =
        sLBPOutput.uBtfBridgeUnCpl; /* Uncoupled lane bridge bitfield, (0,
                                       0~100, %) */
}

/* ****************************************************************************
 Functionname:   ALPInputWrapper
 @brief:		 the input wrapper function for the ALP process

 @description:	 the input wrapper function for the ALP process

 @param[in]		 reqLcfSenPrtList_t* pLCFInput:	the input of the
LCF whole module
 @param[in]		 sLBPOutput_t sLBPOutput:		the output of
the LBP sub-module
 @param[in]		 reqLcfSenParams* pParams:		the parameter
of the LCF whole module

 @param[out]	 sALPInReq_st* pALPInput:		the input of the ALP
sub-module

 @return         -
 @pre
 @post           -
 @author
**************************************************************************** */
static void ALPInputWrapper(const reqLcfSenPrtList_t* pLCFInput,
                            const sLBPOutput_t sLBPOutput,
                            const reqLcfSenParams* pParams,
                            sALPInReq_st* pALPInput) {
    pALPInput->aALPAdjLanes[eLeftLaneIndex].bAvailable_bool =
        pLCFInput->sLCFLaneData.abAvailable[LCF_SEN_CAMERA_LANE_AJLE];
    pALPInput->aALPAdjLanes[eLeftLaneIndex].fCurvatureRate_1pm2 =
        pLCFInput->sLCFLaneData
            .afCurvatureChange_1pm2[LCF_SEN_CAMERA_LANE_AJLE];
    pALPInput->aALPAdjLanes[eLeftLaneIndex].fCurvature_1pm =
        pLCFInput->sLCFLaneData.afCurvature_1pm[LCF_SEN_CAMERA_LANE_AJLE];
    pALPInput->aALPAdjLanes[eLeftLaneIndex].fHeadingAngle_rad =
        pLCFInput->sLCFLaneData.afHeadingAngle_rad[LCF_SEN_CAMERA_LANE_AJLE];
    pALPInput->aALPAdjLanes[eLeftLaneIndex].fPosY0_met =
        pLCFInput->sLCFLaneData.afPosY0_met[LCF_SEN_CAMERA_LANE_AJLE];
    pALPInput->aALPAdjLanes[eLeftLaneIndex].fValidLength_met =
        pLCFInput->sLCFLaneData.afValidLength_met[LCF_SEN_CAMERA_LANE_AJLE];
    pALPInput->aALPAdjLanes[eLeftLaneIndex].uMarkerType_nu =
        pLCFInput->sLCFLaneData.auMarkerType_nu[LCF_SEN_CAMERA_LANE_AJLE];
    pALPInput->aALPAdjLanes[eLeftLaneIndex].uQuality_nu =
        pLCFInput->sLCFLaneData.auQuality_nu[LCF_SEN_CAMERA_LANE_AJLE];

    pALPInput->aALPAdjLanes[eRightLaneIndex].bAvailable_bool =
        pLCFInput->sLCFLaneData.abAvailable[LCF_SEN_CAMERA_LANE_AJRI];
    pALPInput->aALPAdjLanes[eRightLaneIndex].fCurvatureRate_1pm2 =
        pLCFInput->sLCFLaneData
            .afCurvatureChange_1pm2[LCF_SEN_CAMERA_LANE_AJRI];
    pALPInput->aALPAdjLanes[eRightLaneIndex].fCurvature_1pm =
        pLCFInput->sLCFLaneData.afCurvature_1pm[LCF_SEN_CAMERA_LANE_AJRI];
    pALPInput->aALPAdjLanes[eRightLaneIndex].fHeadingAngle_rad =
        pLCFInput->sLCFLaneData.afHeadingAngle_rad[LCF_SEN_CAMERA_LANE_AJRI];
    pALPInput->aALPAdjLanes[eRightLaneIndex].fPosY0_met =
        pLCFInput->sLCFLaneData.afPosY0_met[LCF_SEN_CAMERA_LANE_AJRI];
    pALPInput->aALPAdjLanes[eRightLaneIndex].fValidLength_met =
        pLCFInput->sLCFLaneData.afValidLength_met[LCF_SEN_CAMERA_LANE_AJRI];
    pALPInput->aALPAdjLanes[eRightLaneIndex].uMarkerType_nu =
        pLCFInput->sLCFLaneData.auMarkerType_nu[LCF_SEN_CAMERA_LANE_AJRI];
    pALPInput->aALPAdjLanes[eRightLaneIndex].uQuality_nu =
        pLCFInput->sLCFLaneData.auQuality_nu[LCF_SEN_CAMERA_LANE_AJRI];

    pALPInput->aALPEgoLanes[eLeftLaneIndex].fCurvatureRate_1pm2 =
        sLBPOutput.fCrvRateCtrlLf;
    pALPInput->aALPEgoLanes[eLeftLaneIndex].fCurvature_1pm =
        sLBPOutput.fCrvCtrlLf;
    pALPInput->aALPEgoLanes[eLeftLaneIndex].fHeadingAngle_rad =
        sLBPOutput.fHeadingCtrlLf;
    pALPInput->aALPEgoLanes[eLeftLaneIndex].fPosY0_met =
        sLBPOutput.fPosY0CtrlLf;
    pALPInput->aALPEgoLanes[eLeftLaneIndex].fValidLength_met =
        sLBPOutput.fValidLengthCtrlLf;
    pALPInput->aALPEgoLanes[eLeftLaneIndex].uEgoQuality_btf =
        sLBPOutput.uLaneInvalidQualifierLf;

    pALPInput->aALPEgoLanes[eRightLaneIndex].fCurvatureRate_1pm2 =
        sLBPOutput.fCrvRateCtrlRi;
    pALPInput->aALPEgoLanes[eRightLaneIndex].fCurvature_1pm =
        sLBPOutput.fCrvCtrlRi;
    pALPInput->aALPEgoLanes[eRightLaneIndex].fHeadingAngle_rad =
        sLBPOutput.fHeadingCtrlRi;
    pALPInput->aALPEgoLanes[eRightLaneIndex].fPosY0_met =
        sLBPOutput.fPosY0CtrlRi;
    pALPInput->aALPEgoLanes[eRightLaneIndex].fValidLength_met =
        sLBPOutput.fValidLengthCtrlRi;
    pALPInput->aALPEgoLanes[eRightLaneIndex].uEgoQuality_btf =
        sLBPOutput.uLaneInvalidQualifierRi;

    pALPInput->bLaneChangeDetected_nu = sLBPOutput.bLaneChangeDtct;
    pALPInput->fCycleTime_sec = pParams->LCFSen_Kf_SysCycleTime_sec;
    pALPInput->fEgoVelX_mps = pLCFInput->sLCFVehDyn.fEgoVelX_mps;
    pALPInput->fLaneWidth_met = sLBPOutput.fLaneWidth;
}

/* ****************************************************************************
 Functionname:      ALPOutputWrapper
 @brief:			the output wrapper function for the ALP process

 @description:		convert ALP sub-module's result to LCF Sen output
structure

 @param[in]			const sALPOutput_t sALPOutput: ALP sub-module's
output
 @param[out]		proLcfSenPrtList_t* LCFSenOutput: LCF Sen output pointer
 @return           -
 @pre
 @post             -
 @author
**************************************************************************** */
static void ALPOutputWrapper(const sALPOutput_st sALPOutput,
                             proLcfSenPrtList_t* LCFSenOutput) {
    LCFSenOutput->pLcfSenOutputData.sALPOutput.bLeftAdjLnBridging_bool =
        sALPOutput.bLeftAdjLnBridging_bool;
    LCFSenOutput->pLcfSenOutputData.sALPOutput.bLeftAdjLnStepDeounced_bool =
        sALPOutput.bLeftAdjLnStepDeounced_bool;
    LCFSenOutput->pLcfSenOutputData.sALPOutput.bLeftAdjLnStepOnBridging_bool =
        sALPOutput.bLeftAdjLnStepOnBridging_bool;
    LCFSenOutput->pLcfSenOutputData.sALPOutput.uiLeftAdjLnAliveCount_per =
        sALPOutput.uiLeftAdjLnAliveCount_per;
    LCFSenOutput->pLcfSenOutputData.sALPOutput.uiLeftAdjLnInvalidQu_btf =
        sALPOutput.uiLeftAdjLnInvalidQu_btf;
    LCFSenOutput->pLcfSenOutputData.sALPOutput.fLeftAdjLnCrvRate_1pm2 =
        sALPOutput.fLeftAdjLnCrvRate_1pm2;
    LCFSenOutput->pLcfSenOutputData.sALPOutput.fLeftAdjLnCurvature_1pm =
        sALPOutput.fLeftAdjLnCurvature_1pm;
    LCFSenOutput->pLcfSenOutputData.sALPOutput.fLeftAdjLnDistance_met =
        sALPOutput.fLeftAdjLnDistance_met;
    LCFSenOutput->pLcfSenOutputData.sALPOutput.fLeftAdjLnValidLength_met =
        sALPOutput.fLeftAdjLnValidLength_met;
    LCFSenOutput->pLcfSenOutputData.sALPOutput.fLeftAdjLnYawAngle_rad =
        sALPOutput.fLeftAdjLnYawAngle_rad;

    LCFSenOutput->pLcfSenOutputData.sALPOutput.bRightAdjLnBridging_bool =
        sALPOutput.bRightAdjLnBridging_bool;
    LCFSenOutput->pLcfSenOutputData.sALPOutput.bRightAdjLnStepDeounced_bool =
        sALPOutput.bRightAdjLnStepDeounced_bool;
    LCFSenOutput->pLcfSenOutputData.sALPOutput.bRightAdjLnStepOnBridging_bool =
        sALPOutput.bRightAdjLnStepOnBridging_bool;
    LCFSenOutput->pLcfSenOutputData.sALPOutput.uiRightAdjLnAliveCount_per =
        sALPOutput.uiRightAdjLnAliveCount_per;
    LCFSenOutput->pLcfSenOutputData.sALPOutput.uiRightAdjLnInvalidQu_btf =
        sALPOutput.uiRightAdjLnInvalidQu_btf;
    LCFSenOutput->pLcfSenOutputData.sALPOutput.fRightAdjLnCrvRate_1pm2 =
        sALPOutput.fRightAdjLnCrvRate_1pm2;
    LCFSenOutput->pLcfSenOutputData.sALPOutput.fRightAdjLnCurvature_1pm =
        sALPOutput.fRightAdjLnCurvature_1pm;
    LCFSenOutput->pLcfSenOutputData.sALPOutput.fRightAdjLnDistance_met =
        sALPOutput.fRightAdjLnDistance_met;
    LCFSenOutput->pLcfSenOutputData.sALPOutput.fRightAdjLnValidLength_met =
        sALPOutput.fRightAdjLnValidLength_met;
    LCFSenOutput->pLcfSenOutputData.sALPOutput.fRightAdjLnYawAngle_rad =
        sALPOutput.fRightAdjLnYawAngle_rad;
}
/* ****************************************************************************
 Functionname:     VSDPInputWrapper
 @brief: the input wrapper function for the VSDP process

 @description: the input wrapper function for the VSDP process

 @param[in]reqLcfSenPrtList_t:the input of the LCF whole module

 @param[out]pVSDPInput: the input of the VSDP sub-module

 @return           -
 @pre
 @post             -
 @author
**************************************************************************** */
static void VSDPInputWrapper(const reqLcfSenPrtList_t* pLCFInput,
                             const reqLcfSenParams* pParams,
                             sVSDPInput_t* pVSDPInput) {
    pVSDPInput->VSDPI_SysWarn_St =
        pLCFInput->sLcfSenInputFromVehData.sHandsoffWarn.HOD_StSysWarning;
    pVSDPInput->VSDPI_AccPedPstn_Per = pLCFInput->sLCFVehSig.fAccPedPstn_Per;
    pVSDPInput->VSDPI_CtrlStAvlbABS_B = pLCFInput->sLCFVehSig.bCtrlStAvlbABS;
    pVSDPInput->VSDPI_CtrlStAvlbACC_B = pLCFInput->sLCFVehSig.bCtrlStAvlbACC;
    pVSDPInput->VSDPI_CtrlStAvlbEBA_B = pLCFInput->sLCFVehSig.bCtrlStAvlbEBA;
    pVSDPInput->VSDPI_CtrlStAvlbESC_B = pLCFInput->sLCFVehSig.bCtrlStAvlbESC;
    pVSDPInput->VSDPI_CtrlStAvlbTCS_B = pLCFInput->sLCFVehSig.bCtrlStAvlbTCS;
    pVSDPInput->VSDPI_CtrlStAvlbVSM_B = pLCFInput->sLCFVehSig.bCtrlStAvlbVSM;
    pVSDPInput->VSDPI_CtrlStEnABS_B = pLCFInput->sLCFVehSig.bCtrlStEnABS;
    pVSDPInput->VSDPI_CtrlStEnACC_B = pLCFInput->sLCFVehSig.bCtrlStEnACC;
    pVSDPInput->VSDPI_CtrlStEnEBA_B = pLCFInput->sLCFVehSig.bCtrlStEnEBA;
    pVSDPInput->VSDPI_CtrlStEnESC_B = pLCFInput->sLCFVehSig.bCtrlStEnESC;
    pVSDPInput->VSDPI_CtrlStEnTCS_B = pLCFInput->sLCFVehSig.bCtrlStEnTCS;
    pVSDPInput->VSDPI_CtrlStEnVSM_B = pLCFInput->sLCFVehSig.bCtrlStEnVSM;
    pVSDPInput->VSDPI_DoorOpen_B = pLCFInput->sLCFVehSig.bDoorOpen;
    pVSDPInput->VSDPI_DrvNoBuckledUp_B = pLCFInput->sLCFVehSig.bDrvNoBuckledUp;
    pVSDPInput->VSDPI_DtctOverSte_B = pLCFInput->sLCFVehSig.bDtctOverSte;
    pVSDPInput->VSDPI_DtctRollerBench_B =
        pLCFInput->sLCFVehSig.bDtctRollerBench;
    pVSDPInput->VSDPI_DtctUnderSte_B = pLCFInput->sLCFVehSig.bDtctUnderSte;
    pVSDPInput->VSDPI_GrNeu_B = pLCFInput->sLCFVehSig.bGrNeu;
    pVSDPInput->VSDPI_GrPark_B = pLCFInput->sLCFVehSig.bGrPark;
    pVSDPInput->VSDPI_GrRvs_B = pLCFInput->sLCFVehSig.bGrRvs;
    pVSDPInput->VSDPI_ManuActuTrqEPS_Nm =
        pLCFInput->sLCFVehDyn.fManuActuTrqEPS_nm;
    pVSDPInput->VSDPI_StageWiper_St = pLCFInput->sLCFVehSig.uStageWiper_St;
    pVSDPInput->VSDPI_StateWiper_St = pLCFInput->sLCFVehSig.uStateWiper_St;
    pVSDPInput->VSDPI_StBrightness_St = pLCFInput->sLCFVehSig.uStBrightness_nu;
    pVSDPInput->VSDPI_SteAglFrt_Rad =
        pLCFInput->sLCFVehDyn.fFrontAxleSteerAgl_rad;
    pVSDPInput->VSDPI_StErrABS_B = pLCFInput->sLCFVehSig.bStErrABS;
    pVSDPInput->VSDPI_StErrACC_B = pLCFInput->sLCFVehSig.bStErrACC;
    pVSDPInput->VSDPI_StErrEBA_B = pLCFInput->sLCFVehSig.bStErrEBA;
    pVSDPInput->VSDPI_StErrESC_B = pLCFInput->sLCFVehSig.bStErrESC;
    pVSDPInput->VSDPI_StErrLatDMC_B = pLCFInput->sLCFVehSig.bStErrLatDMC;
    pVSDPInput->VSDPI_StErrTSC_B = pLCFInput->sLCFVehSig.bStErrTSC;
    pVSDPInput->VSDPI_StErrVDY_B = pLCFInput->sLCFVehSig.bStErrVDY;
    pVSDPInput->VSDPI_StErrVSM_B = pLCFInput->sLCFVehSig.bStErrVSM;
    pVSDPInput->VSDPI_TrailerExs_B = pLCFInput->sLCFVehSig.bTrailerExs;
    pVSDPInput->VSDPI_TrnSglEnLf_B = pLCFInput->sLCFVehSig.bTrnSglEnLf;
    pVSDPInput->VSDPI_TrnSglEnRi_B = pLCFInput->sLCFVehSig.bTrnSglEnRi;
    pVSDPInput->VSDPI_TrnSglHarLigEn_B = pLCFInput->sLCFVehSig.bTrnSglHarLigEn;
    pVSDPInput->VSDPI_VehMoveBkwd_B = pLCFInput->sLCFVehSig.bVehMoveBkwd_bool;
    // (pLCFInput->sLCFVehDyn.uEgoMotionState_nu ==
    //  VED_LONG_MOT_STATE_MOVE_RWD);
    pVSDPInput->VSDPI_VehRdyToSta_B = pLCFInput->sLCFVehSig.bVehRdyToSta;
    pVSDPInput->VSDPI_VehSpdX_Mps = pLCFInput->sLCFVehDyn.fEgoVelX_mps;
    pVSDPInput->VSDPI_CycleTime_Sec = pParams->LCFSen_Kf_SysCycleTime_sec;
    pVSDPInput->VSDPI_BrakePedalApplied_B =
        pLCFInput->sLCFVehSig.bBrakePedalApplied;
    pVSDPInput->VSDPI_CtrlStEnARP_B = pLCFInput->sLCFVehSig.bCtrlStEnARP;
    pVSDPInput->VSDPI_CtrlStEnHDC_B = pLCFInput->sLCFVehSig.bCtrlStEnHDC;
    pVSDPInput->VSDPI_StErrARP_B = pLCFInput->sLCFVehSig.bStErrARP;
    pVSDPInput->VSDPI_StErrHDC_B = pLCFInput->sLCFVehSig.bStErrHDC;
    pVSDPInput->VSDPI_BrakeDiscTempSts_B =
        pLCFInput->sLCFVehSig.VSDPI_BrakeDiscTempSts_B;
    pVSDPInput->VSDPI_SignalInvalidLongi_B =
        pLCFInput->sLCFVehSig.VSDPI_SignalInvalidLongi_B;
    pVSDPInput->VSDPI_SignalInvalidLat_B =
        pLCFInput->sLCFVehSig.VSDPI_SignalInvalidLat_B;
}

/* ****************************************************************************
 Functionname:     LDWSAInputWrapper
 @brief: the input wrapper function for the LDWSA process

 @description: the input wrapper function for the LDWSA process

 @param[in]
                        reqLcfSenPrtList_t:the input of the LCF whole module
                        const sVSDPOutput_t sVSDPOutput: the calculated output
data of VSDP sub-module
                        const sLBPOutput_t sLBPOutput: the calculated output
data of LBP sub-module
                        const reqLcfSenParams* pParams: the input parameters of
the LCF whole module
 @param[out]
                        sLDPSAInput_t* pLDPSAInput: the input of the LDPSA
sub-module

 @return           -
 @pre
 @post             -
 @author
**************************************************************************** */
static void LDWSAInputWrapper(const reqLcfSenPrtList_t* pLCFInput,
                              const sVSDPOutput_t sVSDPOutput,
                              const sLBPOutput_t sLBPOutput,
                              const reqLcfSenParams* pParams,
                              const sTJASAOutPro_t sTJASAOutput,
                              const sTJASADebug_t debug,
                              sLDWSAInReq_t* sLDWSAInput) {
    sLDWSAInput->LDWSAI_VehSpdActu_Mps = pLCFInput->sLCFVehDyn.fEgoVelX_mps;
    sLDWSAInput->LDWSAI_SpdVelShow_Kmph =
        pLCFInput->sLCFVehSig.fSpeedometerVelocity_kmh;
    sLDWSAInput->LDWSAI_VehAccSpdX_Npkg = pLCFInput->sLCFVehDyn.fEgoAccelX_mpss;
    sLDWSAInput->LDWSAI_VehAccSpdY_Npkg = pLCFInput->sLCFVehDyn.fEgoAccelY_mpss;
    sLDWSAInput->LDWSAI_VehCurv_ReMi = pLCFInput->sLCFVehDyn.fEgoCurve_1pm;
    sLDWSAInput->LDWSAI_TrnSgl_St =
        pLCFInput->sLCFVehSig.bTrnSglEnLf
            ? 1u
            : (pLCFInput->sLCFVehSig.bTrnSglEnRi
                   ? 2u
                   : 0u);  // TrnSglLf_C_St : 1,TrnSglRi_C_St : 2;
    sLDWSAInput->LDWSAI_WheSteAgl_Dgr =
        TUE_RAD2DEG(pLCFInput->sLCFVehDyn.fSteerWheelAgl_rad);
    sLDWSAInput->LDWSAI_SteAglSpd_Dgpm =
        TUE_RAD2DEG(pLCFInput->sLCFVehDyn.fSteerWheelAglChng_rps);
    sLDWSAInput->LDWSAI_LDWSwitchEn_B = pLCFInput->sBaseCtrlData.bLDWSwitchEn;
    sLDWSAInput->LDWSAI_LDWMod_St = pLCFInput->sBaseCtrlData.uLDWMode_nu;
    // sLDWSAInput->LDWSAI_LDWErrCdtn_B =
    // pLCFInput->sLCFVehSig.bLDWErrCondition;
    if (pLCFInput->sLCFVehSig.bFCA_LDW) {
        sLDWSAInput->LDWSAI_LDWErrCdtn_B = 1;
    } else {
        sLDWSAInput->LDWSAI_LDWErrCdtn_B = 0;  // No Error
    }
    sLDWSAInput->LDWSAI_DtctLnChag_B = sLBPOutput.bLaneChangeDtct;
    sLDWSAInput->LDWSAI_LnWidCalc_Mi = sLBPOutput.fLaneWidth;
    sLDWSAInput->LDWSAI_PstnYLf_Mi = sLBPOutput.fPosY0CtrlLf;
    sLDWSAInput->LDWSAI_PstnYSafeLf_Mi = sLBPOutput.fPosY0SafeLf;
    sLDWSAInput->LDWSAI_PstnYRi_Mi = sLBPOutput.fPosY0CtrlRi;
    sLDWSAInput->LDWSAI_PstnYSafeRi_Mi = sLBPOutput.fPosY0SafeRi;
    sLDWSAInput->LDWSAI_HeadAglLf_Rad = sLBPOutput.fHeadingCtrlLf;
    sLDWSAInput->LDWSAI_HeadAglSafeLf_Rad = sLBPOutput.fHeadingSafeLf;
    sLDWSAInput->LDWSAI_HeadAglRi_Rad = sLBPOutput.fHeadingCtrlRi;
    sLDWSAInput->LDWSAI_HeadAglSafeRi_Rad = sLBPOutput.fHeadingSafeRi;
    sLDWSAInput->LDWSAI_CurvLf_ReMi = sLBPOutput.fCrvCtrlLf;
    sLDWSAInput->LDWSAI_CurvSafeLf_ReMi = sLBPOutput.fCrvSafeLf;
    sLDWSAInput->LDWSAI_CurvRi_ReMi = sLBPOutput.fCrvCtrlRi;
    sLDWSAInput->LDWSAI_CurvSafeRi_ReMi = sLBPOutput.fCrvSafeRi;
    sLDWSAInput->LDWSAI_IvldLnSafeLf_St = sLBPOutput.uInvalidQualifierSafeLf;
    sLDWSAInput->LDWSAI_LnIVldLf_St = sLBPOutput.uLaneInvalidQualifierLf;
    sLDWSAInput->LDWSAI_IvldLnSafeRi_St = sLBPOutput.uInvalidQualifierSafeRi;
    sLDWSAInput->LDWSAI_LnIVldRi_St = sLBPOutput.uLaneInvalidQualifierRi;
    sLDWSAInput->LDWSAI_VehStIvld_St = sVSDPOutput.VSDP_VehStIvld_St;
    sLDWSAInput->LDWSAI_IvldStDrv_St = sVSDPOutput.VSDP_IvldStDrv_St;
    sLDWSAInput->LDWSAI_CtrlStEn_St = sVSDPOutput.VSDP_CtrlStEn_St;
    // Passive condition from FID
    if (pLCFInput->sLCFVehSig.bRCA_LDW) {
        sLDWSAInput->LDWSAI_StError_St = 1;
    } else {
        sLDWSAInput->LDWSAI_StError_St = sVSDPOutput.VSDP_StError_St;
    }
    if (sTJASAOutput.TJATVG_TrajGuiQu_nu == 5 ||
        sTJASAOutput.TJATVG_TrajGuiQu_nu == 6 ||
        sTJASAOutput.TJATVG_TrajGuiQu_nu == 7 ||
        debug.SLC_AbortState_enum == 1 || debug.SLC_AbortState_enum == 2) {
        sLDWSAInput->LDWSAI_StError_St = 1;
    }
    sLDWSAInput->LDWSAI_CtrlStNoAvlb_St = sVSDPOutput.VSDP_CtrlStNoAvlb_St;
    sLDWSAInput->LDWSAI_PrjSpecQu_St = 0u;
    sLDWSAInput->LDWSAI_DtctCstruSite_B = sLBPOutput.bConstructionSiteDtct;
    sLDWSAInput->LDWSAI_CycleTime_Sec = pParams->LCFSen_Kf_SysCycleTime_sec;
    sLDWSAInput->LDWSAI_VehYawRate_rps = pLCFInput->sLCFVehDyn.fEgoYawRate_rps;
    sLDWSAInput->LDWSAI_AEBActive_B = false;
    sLDWSAInput->NVRAM_LDWSwitch_B = false;
    sLDWSAInput->NVRAM_LDWStartupSpd_Kmph = 20.f;
    sLDWSAInput->LDWSAI_VehStartupSpdHMI_Kmph =
        pLCFInput->sBaseCtrlData.fLDWStartupSpdHMI_kph;
    sLDWSAInput->LDWSAI_LDPSwitchOn_B = false;
    // Only use safety
    sLDWSAInput->LDWSAI_PstnYLf_Mi = sLDWSAInput->LDWSAI_PstnYSafeLf_Mi;
    sLDWSAInput->LDWSAI_PstnYRi_Mi = sLDWSAInput->LDWSAI_PstnYSafeRi_Mi;
    sLDWSAInput->LDWSAI_HeadAglLf_Rad = sLDWSAInput->LDWSAI_HeadAglSafeLf_Rad;
    sLDWSAInput->LDWSAI_HeadAglRi_Rad = sLDWSAInput->LDWSAI_HeadAglSafeRi_Rad;
    sLDWSAInput->LDWSAI_CurvLf_ReMi = sLDWSAInput->LDWSAI_CurvSafeLf_ReMi;
    sLDWSAInput->LDWSAI_CurvRi_ReMi = sLDWSAInput->LDWSAI_CurvSafeRi_ReMi;
    sLDWSAInput->LDWSAI_LnLengthLf_Mi = sLBPOutput.fValidLengthCtrlLf;
    sLDWSAInput->LDWSAI_LnLengthRi_Mi = sLBPOutput.fValidLengthCtrlRi;
    sLDWSAInput->LDWSAI_LnIVldLf_St = sLDWSAInput->LDWSAI_IvldLnSafeLf_St;
    sLDWSAInput->LDWSAI_LnIVldRi_St = sLDWSAInput->LDWSAI_IvldLnSafeRi_St;
}

/* ****************************************************************************
 Functionname:     LDWSAOutputWrapper
 @brief: the output wrapper function for the LDWSA process

 @description: convert LDWSA sub-module's result to LCF Sen output structure

 @param[in]
                        const sLDPSAOutput_t sLDPSAOutput: LDWSA sub-module's
output
 @param[out]
                        proLcfSenPrtList_t* LCFSenOutput: LCF Sen output pointer
 @return           -
 @pre
 @post             -
 @author
**************************************************************************** */
static void LDWSAOutputWrapper(const sLDWSAOutPro_t sLDWSAOutput,
                               proLcfSenPrtList_t* pLCFSenOutput) {
    pLCFSenOutput->pLcfSenOutputData.sLDWSAOutput.LDWC_DgrSide_St =
        sLDWSAOutput.LDWC_DgrSide_St;
    pLCFSenOutput->pLcfSenOutputData.sLDWSAOutput.LDWC_RdyToTrig_B =
        sLDWSAOutput.LDWC_RdyToTrig_B;
    pLCFSenOutput->pLcfSenOutputData.sLDWSAOutput.LDWC_SysOut_St =
        sLDWSAOutput.LDWC_SysOut_St;
    pLCFSenOutput->pLcfSenOutputData.sLDWSAOutput
        .LDVSE_NVRAMVehStartupSpd_kmph =
        sLDWSAOutput.LDVSE_NVRAMVehStartupSpd_kmph;
    pLCFSenOutput->pLcfSenOutputData.sLDWSAOutput.LDWC_NVRAMLDWSwitch_B =
        sLDWSAOutput.LDWC_NVRAMLDWSwitch_B;
    pLCFSenOutput->sNVRAMData.LCFSen_Nb_LDWPowerOffSwitch_nu =
        sLDWSAOutput.LDWC_NVRAMLDWSwitch_B;
    pLCFSenOutput->sNVRAMData.LCFSen_Nf_LDWStartupSpd_kph =
        sLDWSAOutput.LDVSE_NVRAMVehStartupSpd_kmph;
}

/* ****************************************************************************
 Functionname:     LDPSAInputWrapper
 @brief: the input wrapper function for the LDPSA process

 @description: the input wrapper function for the LDPSA process

 @param[in]
                        reqLcfSenPrtList_t:the input of the LCF whole module
                        const sVSDPOutput_t sVSDPOutput: the calculated output
data of VSDP sub-module
                        const sLBPOutput_t sLBPOutput: the calculated output
data of LBP sub-module
                        const reqLcfSenParams* pParams: the input parameters of
the LCF whole module
 @param[out]
                        sLDPSAInput_t* pLDPSAInput: the input of the LDPSA
sub-module

 @return           -
 @pre
 @post             -
 @author
**************************************************************************** */
static void LDPSAInputWrapper(const reqLcfSenPrtList_t* pLCFInput,
                              const sVSDPOutput_t sVSDPOutput,
                              const sLBPOutput_t sLBPOutput,
                              const reqLcfSenParams* pParams,
                              const sTJASAOutPro_t sTJASAOutput,
                              const sTJASADebug_t debug,
                              sLDPSAInput_t* pLDPSAInput) {
    pLDPSAInput->LDPSAI_VehSpdActu_Mps = pLCFInput->sLCFVehDyn.fEgoVelX_mps;
    pLDPSAInput->LDPSAI_SpdVelShow_Kmph =
        pLCFInput->sLCFVehSig.fSpeedometerVelocity_kmh;
    pLDPSAInput->LDPSAI_VehAccSpdX_Npkg = pLCFInput->sLCFVehDyn.fEgoAccelX_mpss;
    pLDPSAInput->LDPSAI_VehAccSpdY_Npkg = pLCFInput->sLCFVehDyn.fEgoAccelY_mpss;
    pLDPSAInput->LDPSAI_VehCurv_ReMi = pLCFInput->sLCFVehDyn.fEgoCurve_1pm;
    pLDPSAInput->LDPSAI_TrnSgl_St =
        pLCFInput->sLCFVehSig.bTrnSglEnLf
            ? 1u
            : (pLCFInput->sLCFVehSig.bTrnSglEnRi
                   ? 2u
                   : 0u);  // LDPVSE_TrnSglLf_C_St : 1,LDPVSE_TrnSglRi_C_St : 2;
    pLDPSAInput->LDPSAI_WheSteAgl_Dgr =
        TUE_RAD2DEG(pLCFInput->sLCFVehDyn.fSteerWheelAgl_rad);
    pLDPSAInput->LDPSAI_SteAglSpd_Dgpm =
        TUE_RAD2DEG(pLCFInput->sLCFVehDyn.fSteerWheelAglChng_rps);
    pLDPSAInput->LDPSAI_LDPSwitchEn_B = pLCFInput->sBaseCtrlData.bLDPSwitchEn;
    pLDPSAInput->LDPSAI_LDPMod_St = pLCFInput->sBaseCtrlData.uLDPMode_nu;
    // pLDPSAInput->LDPSAI_LDPErrCdtn_B =
    // pLCFInput->sLCFVehSig.bLDPErrCondition;
    if (pLCFInput->sLCFVehSig.bFCA_LDP || pLCFInput->sLCFVehSig.bFCA_LDW) {
        pLDPSAInput->LDPSAI_LDPErrCdtn_B = 1;
    } else {
        pLDPSAInput->LDPSAI_LDPErrCdtn_B = 0;  // No Error
    }
    pLDPSAInput->LDPSAI_DtctLnChag_B = sLBPOutput.bLaneChangeDtct;
    pLDPSAInput->LDPSAI_LnWidCalc_Mi = sLBPOutput.fLaneWidth;
    pLDPSAInput->LDPSAI_PstnXLf_Mi = sLBPOutput.fPosX0CtrlLf;
    pLDPSAInput->LDPSAI_PstnXRi_Mi = sLBPOutput.fPosX0CtrlRi;
    pLDPSAInput->LDPSAI_PstnYLf_Mi = sLBPOutput.fPosY0CtrlLf;
    pLDPSAInput->LDPSAI_PstnYSafeLf_Mi = sLBPOutput.fPosY0SafeLf;
    pLDPSAInput->LDPSAI_PstnYRi_Mi = sLBPOutput.fPosY0CtrlRi;
    pLDPSAInput->LDPSAI_PstnYSafeRi_Mi = sLBPOutput.fPosY0SafeRi;
    pLDPSAInput->LDPSAI_HeadAglLf_Rad = sLBPOutput.fHeadingCtrlLf;
    pLDPSAInput->LDPSAI_HeadAglSafeLf_Rad = sLBPOutput.fHeadingSafeLf;
    pLDPSAInput->LDPSAI_HeadAglRi_Rad = sLBPOutput.fHeadingCtrlRi;
    pLDPSAInput->LDPSAI_HeadAglSafeRi_Rad = sLBPOutput.fHeadingSafeRi;
    pLDPSAInput->LDPSAI_CurvLf_ReMi = sLBPOutput.fCrvCtrlLf;
    pLDPSAInput->LDPSAI_CurvSafeLf_ReMi = sLBPOutput.fCrvSafeLf;
    pLDPSAInput->LDPSAI_CurvRi_ReMi = sLBPOutput.fCrvCtrlRi;
    pLDPSAInput->LDPSAI_CurvSafeRi_ReMi = sLBPOutput.fCrvSafeRi;
    pLDPSAInput->LDPSAI_CurvRateLf_ReMi2 = sLBPOutput.fCrvRateCtrlLf;
    pLDPSAInput->LDPSAI_CurvRateRi_ReMi2 = sLBPOutput.fCrvRateCtrlRi;
    pLDPSAInput->LDPSAI_VldLengLf_Mi = sLBPOutput.fValidLengthCtrlLf;
    pLDPSAInput->LDPSAI_VldLengRi_Mi = sLBPOutput.fValidLengthCtrlRi;
    pLDPSAInput->LDPSAI_IvldLnSafeLf_St = sLBPOutput.uInvalidQualifierSafeLf;
    pLDPSAInput->LDPSAI_LnIVldLf_St = sLBPOutput.uLaneInvalidQualifierLf;
    pLDPSAInput->LDPSAI_IvldLnSafeRi_St = sLBPOutput.uInvalidQualifierSafeRi;
    pLDPSAInput->LDPSAI_LnIVldRi_St = sLBPOutput.uLaneInvalidQualifierRi;
    pLDPSAInput->LDPSAI_VehStIvld_St = sVSDPOutput.VSDP_VehStIvld_St;
    pLDPSAInput->LDPSAI_IvldStDrv_St = sVSDPOutput.VSDP_IvldStDrv_St;
    pLDPSAInput->LDPSAI_CtrlStEn_St = sVSDPOutput.VSDP_CtrlStEn_St;
    // Passive condition from FID
    if (pLCFInput->sLCFVehSig.bRCA_LDP || pLCFInput->sLCFVehSig.bRCA_LDW) {
        pLDPSAInput->LDPSAI_StError_St = 1;
    } else {
        pLDPSAInput->LDPSAI_StError_St = sVSDPOutput.VSDP_StError_St;
    }
    if (sTJASAOutput.TJATVG_TrajGuiQu_nu == 5 ||
        sTJASAOutput.TJATVG_TrajGuiQu_nu == 6 ||
        sTJASAOutput.TJATVG_TrajGuiQu_nu == 7 ||
        debug.SLC_AbortState_enum == 1 || debug.SLC_AbortState_enum == 2) {
        pLDPSAInput->LDPSAI_StError_St = 1;
    }
    pLDPSAInput->LDPSAI_CtrlStNoAvlb_St = sVSDPOutput.VSDP_CtrlStNoAvlb_St;
    pLDPSAInput->LDPSAI_PrjSpecQu_St = 0u;
    pLDPSAInput->LDPSAI_DtctCstruSite_B = sLBPOutput.bConstructionSiteDtct;
    pLDPSAInput->LDPSAI_CycleTime_Sec = pParams->LCFSen_Kf_SysCycleTime_sec;
    pLDPSAInput->LDPSAI_ABDTimeStamp_Sec =
        (float32)(pLCFInput->sLCFLaneData.sSigHeader.uiTimeStamp) / 1000000.0f;
    pLDPSAInput->LDPSAI_PstnYCent_Mi = sLBPOutput.fPosY0CtrlCntr;
    pLDPSAInput->LDPSAI_VehYawRate_rps = pLCFInput->sLCFVehDyn.fEgoYawRate_rps;
    pLDPSAInput->LDPSAI_AEBActive_B = false;
    pLDPSAInput->NVRAM_LDPSwitch_B = false;
    pLDPSAInput->NVRAM_LDPStartupSpd_Kmph = 20.f;
    pLDPSAInput->LDPSAI_VehStartupSpdHMI_Kmph =
        pLCFInput->sBaseCtrlData.fLDPStartupSpdHMI_kph;
    // Only use safety
    pLDPSAInput->LDPSAI_PstnYLf_Mi = pLDPSAInput->LDPSAI_PstnYSafeLf_Mi;
    pLDPSAInput->LDPSAI_PstnYRi_Mi = pLDPSAInput->LDPSAI_PstnYSafeRi_Mi;
    pLDPSAInput->LDPSAI_HeadAglLf_Rad = pLDPSAInput->LDPSAI_HeadAglSafeLf_Rad;
    pLDPSAInput->LDPSAI_HeadAglRi_Rad = pLDPSAInput->LDPSAI_HeadAglSafeRi_Rad;
    pLDPSAInput->LDPSAI_CurvLf_ReMi = pLDPSAInput->LDPSAI_CurvSafeLf_ReMi;
    pLDPSAInput->LDPSAI_CurvRi_ReMi = pLDPSAInput->LDPSAI_CurvSafeRi_ReMi;
    pLDPSAInput->LDPSAI_LnIVldLf_St = pLDPSAInput->LDPSAI_IvldLnSafeLf_St;
    pLDPSAInput->LDPSAI_LnIVldRi_St = pLDPSAInput->LDPSAI_IvldLnSafeRi_St;
}

/* ****************************************************************************
 Functionname:     LDPSAOutputWrapper
 @brief: the output wrapper function for the LDP/LDW process

 @description: convert LDP/LDW sub-module's result to LCF Sen output structure

 @param[in]
                        const sLDPSAOutput_t sLDPSAOutput: LDP/LDW sub-module's
output
 @param[out]
                        proLcfSenPrtList_t* LCFSenOutput: LCF Sen output pointer
 @return           -
 @pre
 @post             -
 @author
**************************************************************************** */
static void LDPSAOutputWrapper(const sLDPSAOutput_t sLDPSAOutput,
                               proLcfSenPrtList_t* pLCFSenOutput) {
    pLCFSenOutput->pLcfSenOutputData.sLDPSAOutput.LDPDT_LnCltdCurvLf_ReMi =
        sLDPSAOutput.LDPDT_LnCltdCurvLf_ReMi;
    pLCFSenOutput->pLcfSenOutputData.sLDPSAOutput.LDPDT_LnHeadLf_Rad =
        sLDPSAOutput.LDPDT_LnHeadLf_Rad;
    pLCFSenOutput->pLcfSenOutputData.sLDPSAOutput.LDPDT_LnPstnLf_Mi =
        sLDPSAOutput.LDPDT_LnPstnLf_Mi;
    pLCFSenOutput->pLcfSenOutputData.sLDPSAOutput.LDPDT_LnPstnRi_Mi =
        sLDPSAOutput.LDPDT_LnPstnRi_Mi;
    pLCFSenOutput->pLcfSenOutputData.sLDPSAOutput.LDPSC_DgrSide_St =
        sLDPSAOutput.LDPSC_DgrSide_St;
    pLCFSenOutput->pLcfSenOutputData.sLDPSAOutput.LDPSC_RdyToTrig_B =
        sLDPSAOutput.LDPSC_RdyToTrig_B;
    pLCFSenOutput->pLcfSenOutputData.sLDPSAOutput.LDPSC_SysOut_St =
        sLDPSAOutput.LDPSC_SysOut_St;
    pLCFSenOutput->pLcfSenOutputData.sLDPSAOutput.LDPDT_LnHeadRi_Rad =
        sLDPSAOutput.LDPDT_LnHeadRi_Rad;
    pLCFSenOutput->pLcfSenOutputData.sLDPSAOutput.LDPDT_LnCltdCurvRi_ReMi =
        sLDPSAOutput.LDPDT_LnCltdCurvRi_ReMi;
    pLCFSenOutput->pLcfSenOutputData.sLDPSAOutput
        .LDPVSE_NVRAMVehStartupSpd_Kmph =
        sLDPSAOutput.LDPVSE_NVRAMVehStartupSpd_Kmph;
    pLCFSenOutput->pLcfSenOutputData.sLDPSAOutput.LDPSC_NVRAMLDPSwitch_B =
        sLDPSAOutput.LDPSC_NVRAMLDPSwitch_B;
    pLCFSenOutput->sNVRAMData.LCFSen_Nb_LDPPowerOffSwitch_nu =
        sLDPSAOutput.LDPSC_NVRAMLDPSwitch_B;
    pLCFSenOutput->sNVRAMData.LCFSen_Nf_LDPStartupSpd_kph =
        sLDPSAOutput.LDPVSE_NVRAMVehStartupSpd_Kmph;
}

/* ****************************************************************************
 Functionname:     ODPRInputWrapper
 @brief: the input wrapper function for the ODPR process

 @description: the input wrapper function for the ODPR process

 @param[in]
                        const reqLcfSenPrtList_t* pLCFInput:the input of the LCF
whole module
                        const sLBPOutput_t sLBPOutput: the calculated output
data of LBP sub-module
 @param[out]
                         ODPRInReq_t* pODPRInput: the input of the ODPR
sub-module

 @return           -
 @pre
 @post             -
 @author
**************************************************************************** */
static void ODPRInputWrapper(const reqLcfSenPrtList_t* pLCFInput,
                             const sLBPOutput_t sLBPOutput,
                             const reqLcfSenParams* pParams,
                             ODPRInReq_t* pODPRInput) {
    memcpy(&(pODPRInput->sAccObject), &(pLCFInput->sLCFACCOOIData.sACCOOIData),
           sizeof(ODPRInAccFRObj_t));

    static float32 fPrevPosX = 0.f;
    static float32 fPrevPosY = 0.f;
    static float32 fPrevHeading = 0.f;

    float32 lowpassFac = 0.2f;

    pODPRInput->sAccObject.fObjPosX_met =
        pODPRInput->sAccObject.fObjPosX_met * lowpassFac +
        fPrevPosX * (1.f - lowpassFac);
    pODPRInput->sAccObject.fObjPosY_met =
        pODPRInput->sAccObject.fObjPosY_met * lowpassFac +
        fPrevPosY * (1.f - lowpassFac);
    pODPRInput->sAccObject.fObjRelHeadingAngle_rad =
        pODPRInput->sAccObject.fObjRelHeadingAngle_rad * lowpassFac +
        fPrevHeading * (1.f - lowpassFac);

    fPrevPosX = pODPRInput->sAccObject.fObjPosX_met;
    fPrevPosY = pODPRInput->sAccObject.fObjPosY_met;
    fPrevHeading = pODPRInput->sAccObject.fObjRelHeadingAngle_rad;

    pODPRInput->sAccObject.uiObjSensorSource_btf = 0x11;
    pODPRInput->sAccObject.uiObjMeasState_nu = 2u;

    pODPRInput->sEgoVehSig.fEgoCurve_1pm = pLCFInput->sLCFVehDyn.fEgoCurve_1pm;
    pODPRInput->sEgoVehSig.fEgoVelX_mps = pLCFInput->sLCFVehDyn.fEgoVelX_mps;
    pODPRInput->sEgoVehSig.fEgoYawRate_rps =
        pLCFInput->sLCFVehDyn.fEgoYawRate_rps;

    pODPRInput->sSystemPara.fSystemCylceTime_sec =
        pParams->LCFSen_Kf_SysCycleTime_sec;

    pODPRInput->sLaneData.bLaneChangeDetected_bool = sLBPOutput.bLaneChangeDtct;

    pODPRInput->sLaneData.fLeftLaneClothoidCurveDer_1pm2 =
        sLBPOutput.fCrvRateCtrlLf;
    pODPRInput->sLaneData.fLeftLaneClothoidCurve_1pm = sLBPOutput.fCrvCtrlLf;
    pODPRInput->sLaneData.fLeftLaneClothoidHeading_rad =
        sLBPOutput.fHeadingCtrlLf;
    pODPRInput->sLaneData.fLeftLaneClothoidLength_met =
        sLBPOutput.fValidLengthCtrlLf;
    pODPRInput->sLaneData.fLeftLaneClothoidPosY0_met = sLBPOutput.fPosY0CtrlLf;

    pODPRInput->sLaneData.fRightLaneClothoidCurveDer_1pm2 =
        sLBPOutput.fCrvRateCtrlRi;
    pODPRInput->sLaneData.fRightLaneClothoidCurve_1pm = sLBPOutput.fCrvCtrlRi;
    pODPRInput->sLaneData.fRightLaneClothoidHeading_rad =
        sLBPOutput.fHeadingCtrlRi;
    pODPRInput->sLaneData.fRightLaneClothoidLength_met =
        sLBPOutput.fValidLengthCtrlRi;
    pODPRInput->sLaneData.fRightLaneClothoidPosY0_met = sLBPOutput.fPosY0CtrlRi;

    pODPRInput->sLaneData.uiLeftLaneInvalidCheck_btf =
        sLBPOutput.uLaneInvalidQualifierLf;
    pODPRInput->sLaneData.uiLeftLaneQuality_perc = sLBPOutput.uOverallQualityLf;
    pODPRInput->sLaneData.uiRightLaneInvalidCheck_btf =
        sLBPOutput.uLaneInvalidQualifierRi;
    pODPRInput->sLaneData.uiRightLaneQuality_perc =
        sLBPOutput.uOverallQualityRi;

    // memcpy(&(pODPRInput->sObjectList),
    // &pLCFInput->sSSRObjectList.sSSRObjList,
    //        sizeof(ODPRInSSRObjList_t));
}

/* ****************************************************************************
 Functionname:   TJASAInputWrapper
 @brief:		 the input wrapper function for the TJASA process

 @description:	 the input wrapper function for the TJASA process

 @param[in]		 reqLcfSenPrtList_t* pLCFInput:	the input of the
LCF whole module
 @param[in]		 sLBPOutput_t sLBPOutput:		the output of
the LBP sub-module
 @param[in]		 reqLcfSenParams* pLCFParam:	the parameter of the
LCF whole module
 @param[in]	     sALPInReq_st sALPOutput:		the output of the
ALP sub-module
 @param[in]		 ODPROutPro_t sODPROutput:		the output of
the ODPR sub-module
 @param[in]      sVSDPOutput_t sVSDPOutput:		the output of the VSDP
sub-module
 @param[out]     sTJASAInReq_t* pTJASAInput:	the input of the TJASA
sub-module

 @return         -
 @pre
 @post           -
 @author
**************************************************************************** */
static void TJASAInputWrapper(const reqLcfSenPrtList_t* pLCFInput,
                              const reqLcfSenParams* pLCFParam,
                              const sLBPOutput_t sLBPOutput,
                              const sALPOutput_st sALPOutput,
                              const ODPROutPro_t sODPROutput,
                              const sVSDPOutput_t sVSDPOutput,
                              const sLCCRAOutPro_t sLCCRAOUtput,
                              sTJASAInReq_t* pTJASAInput) {
    pTJASAInput->ABPR_LaneChangeDetected_bool = sLBPOutput.bLaneChangeDtct;
    pTJASAInput->ABPR_LaneWidth_met = sLBPOutput.fLaneWidth;
    pTJASAInput->ABPR_ConstructionSite_bool = sLBPOutput.bConstructionSiteDtct;
    pTJASAInput->ABPR_UncoupledLaneWidth_met = sLBPOutput.fLaneWidthUnCpl;
    pTJASAInput->ABPR_ABDTimeStamp_sec = sLBPOutput.fABDTimeStamp;

    pTJASAInput->ABPR_LeftLaneType_enum = sLBPOutput.uLaneTypeLf;
    pTJASAInput->ABPR_LeLnClthPosX0_met = sLBPOutput.fPosX0CtrlLf;
    pTJASAInput->ABPR_LeLnClthPosY0_met = sLBPOutput.fPosY0CtrlLf;
    pTJASAInput->ABPR_LeLnInvalidQu_btf = sLBPOutput.uLaneInvalidQualifierLf;
    pTJASAInput->ABPR_LeLnQuality_perc = sLBPOutput.uOverallQualityLf;
    pTJASAInput->ABPR_LeCrvQuality_perc = sLBPOutput.uCrvQualityLf;
    pTJASAInput->ABPR_LeLnClthHeading_rad = sLBPOutput.fHeadingCtrlLf;
    pTJASAInput->ABPR_LeLnClthCrv_1pm = sLBPOutput.fCrvCtrlLf;
    pTJASAInput->ABPR_LeLnClthCrvChng_1pm2 = sLBPOutput.fCrvRateCtrlLf;
    pTJASAInput->ABPR_LeLnClthLength_met = sLBPOutput.fValidLengthCtrlLf;

    pTJASAInput->ABPR_LeAdjLnClthPosX0_met = 0.f;
    pTJASAInput->ABPR_LeAdjLnClthPosY0_met = sALPOutput.fLeftAdjLnDistance_met;
    pTJASAInput->ABPR_LeAdjLnInvalidQu_btf =
        sALPOutput.uiLeftAdjLnInvalidQu_btf;
    pTJASAInput->ABPR_LeAdjLnClthHeading_rad =
        sALPOutput.fLeftAdjLnYawAngle_rad;
    pTJASAInput->ABPR_LeAdjLnClthCrv_1pm = sALPOutput.fLeftAdjLnCurvature_1pm;
    pTJASAInput->ABPR_LeAdjLnClthCrvChng_1pm2 =
        sALPOutput.fLeftAdjLnCrvRate_1pm2;
    pTJASAInput->ABPR_LeAdjLnClthLength_met =
        sALPOutput.fLeftAdjLnValidLength_met;

    pTJASAInput->ABPR_RightLaneType_enum = sLBPOutput.uLaneTypeRi;
    pTJASAInput->ABPR_RiLnClthPosX0_met = sLBPOutput.fPosX0CtrlRi;
    pTJASAInput->ABPR_RiLnClthPosY0_met = sLBPOutput.fPosY0CtrlRi;
    pTJASAInput->ABPR_RiLnInvalidQu_btf = sLBPOutput.uLaneInvalidQualifierRi;
    pTJASAInput->ABPR_RiLnQuality_perc = sLBPOutput.uOverallQualityRi;
    pTJASAInput->ABPR_RiCrvQuality_perc = sLBPOutput.uCrvQualityRi;
    pTJASAInput->ABPR_RiLnClthHeading_rad = sLBPOutput.fHeadingCtrlRi;
    pTJASAInput->ABPR_RiLnClthCrv_1pm = sLBPOutput.fCrvCtrlRi;
    pTJASAInput->ABPR_RiLnClthCrvChng_1pm2 = sLBPOutput.fCrvRateCtrlRi;
    pTJASAInput->ABPR_RiLnClthLength_met = sLBPOutput.fValidLengthCtrlLf;

    pTJASAInput->ABPR_RiAdjLnClthPosX0_met = 0.f;
    pTJASAInput->ABPR_RiAdjLnClthPosY0_met = sALPOutput.fRightAdjLnDistance_met;
    pTJASAInput->ABPR_RiAdjLnInvalidQu_btf =
        sALPOutput.uiRightAdjLnInvalidQu_btf;
    pTJASAInput->ABPR_RiAdjLnClthHeading_rad =
        sALPOutput.fRightAdjLnYawAngle_rad;
    pTJASAInput->ABPR_RiAdjLnClthCrv_1pm = sALPOutput.fRightAdjLnCurvature_1pm;
    pTJASAInput->ABPR_RiAdjLnClthCrvChng_1pm2 =
        sALPOutput.fRightAdjLnCrvRate_1pm2;
    pTJASAInput->ABPR_RiAdjLnClthLength_met =
        sALPOutput.fRightAdjLnValidLength_met;

    pTJASAInput->ABPR_CntrLnClthPosX0_met = sLBPOutput.fPosX0CtrlCntr;
    pTJASAInput->ABPR_CntrLnClthPosY0_met = sLBPOutput.fPosY0CtrlCntr;
    pTJASAInput->ABPR_CntrLnClthHeading_rad = sLBPOutput.fHeadingCtrlCntr;
    pTJASAInput->ABPR_CntrLnClthCrv_1pm = sLBPOutput.fCrvCtrlCntr;
    pTJASAInput->ABPR_CntrLnClthCrvChng_1pm2 = sLBPOutput.fCrvRateCtrlCntr;
    pTJASAInput->ABPR_CntrLnClthLength_met = sLBPOutput.fValidLengthCtrlCntr;

    pTJASAInput->ODPFOH_TgtObjClothoidInv_btf =
        sODPROutput.sFOHOutData.fTgtObjClothoidInvalidCheck_btf;
    pTJASAInput->ODPFOH_TgtObjPosX0_met =
        sODPROutput.sFOHOutData.fTgtObjPosX0_met;
    pTJASAInput->ODPFOH_TgtObjPosY0_met =
        sODPROutput.sFOHOutData.fTgtObjPosY0_met;
    pTJASAInput->ODPFOH_TgtObjHeadAng_rad =
        sODPROutput.sFOHOutData.fTgtObjHeading_rad;
    pTJASAInput->ODPFOH_TgtObjCrv_1pm =
        sODPROutput.sFOHOutData.fTgtObjCurve_1pm;
    pTJASAInput->ODPFOH_TgtObjCrvChng_1pm2 =
        sODPROutput.sFOHOutData.fTgtObjCurveDer_1pm2;
    pTJASAInput->ODPFOH_TgtObjLength_met =
        sODPROutput.sFOHOutData.fTgtObjLength_met;
    pTJASAInput->ODPFOP_AccFRObjTStamp_sec =
        sODPROutput.sFOPOutData.fAccObjTimeStamp_sec;
    pTJASAInput->ODPFOP_AccObjInvBitfield_btf =
        sODPROutput.sFOPOutData.uiAccObjInvalidCheck_btf;
    pTJASAInput->ODPFOP_AccObjPosX_met =
        sODPROutput.sFOPOutData.fAccObjPosX_met;
    pTJASAInput->ODPFOP_AccObjPosY_met =
        sODPROutput.sFOPOutData.fAccObjPosY_met;

    pTJASAInput->LCA_ActiveLeft_bool =
        pLCFInput->sLCFLBSCheckState.LCA_ActiveLeft_bool;
    pTJASAInput->LCA_ActiveRight_bool =
        pLCFInput->sLCFLBSCheckState.LCA_ActiveRight_bool;
    pTJASAInput->LCA_WarningLeft_bool =
        pLCFInput->sLCFLBSCheckState.LCA_WarningLeft_bool;
    pTJASAInput->LCA_WarningRight_bool =
        pLCFInput->sLCFLBSCheckState.LCA_WarningRight_bool;

    pTJASAInput->BSD_ActiveLeft_bool =
        pLCFInput->sLCFLBSCheckState.BSD_ActiveLeft_bool;
    pTJASAInput->BSD_ActiveRight_bool =
        pLCFInput->sLCFLBSCheckState.BSD_ActiveRight_bool;
    pTJASAInput->BSD_WarningLeft_bool =
        pLCFInput->sLCFLBSCheckState.BSD_WarningLeft_bool;
    pTJASAInput->BSD_WarningRight_bool =
        pLCFInput->sLCFLBSCheckState.BSD_WarningRight_bool;

    pTJASAInput->CUSTOM_PrjSpecQu_btf = pLCFInput->sCUSTOMState.uPrjSpecQu_btf;

    pTJASAInput->S_ODPSOP_MSFlag_RearLeft_nu = FALSE;
    pTJASAInput->S_ODPSOP_MSFlag_RearRight_nu = FALSE;

    pTJASAInput->TRJPLN_QuStatusTrajPlan_nu =
        pLCFInput->sLcfSenInputFromVehData.sTrajPlanState.uQuStatusTrajPlan_nu;
    pTJASAInput->TRJCTR_QuServTrajCtr_nu =
        pLCFInput->sLcfSenInputFromVehData.sTrajCtrlState.uQuServTrajCtr_nu;
    pTJASAInput->MDCTR_ControllingFunction_nu =
        (E_MCTLCF_ControllingFunction_nu)pLCFInput->sLcfSenInputFromSenData
            .sMCTLFCState.uControllingFunction_nu;

    pTJASAInput->LCFRCV_SysCycleTimeSen_sec =
        pLCFParam->LCFSen_Kf_SysCycleTime_sec;
    pTJASAInput->LCFRCV_TurnSignalLeverHold_bool =
        pLCFInput->sLCFVehSig.bTurnSignalLeverHold;
    pTJASAInput->LCFRCV_TurnSignalLeft_bool = pLCFInput->sLCFVehSig.bTrnSglEnLf;
    pTJASAInput->LCFRCV_TurnSignalRight_bool =
        pLCFInput->sLCFVehSig.bTrnSglEnRi;
    pTJASAInput->LCFRCV_DrivingMode_nu = pLCFInput->sLCFVehSig.uDrivingMode_nu;
    pTJASAInput->LCFRCV_LKASwitch_nu = pLCFInput->sLCFVehSig.bLKASwitch;
    pTJASAInput->LCFRCV_TJASwitch_nu = pLCFInput->sLCFVehSig.bTJASwitch;
    pTJASAInput->LCFRCV_ErrorStateTJA_bool =
        pLCFInput->sLCFVehSig.bErrorStateTJA;
    pTJASAInput->LCFRCV_ErrorStateLKA_bool =
        pLCFInput->sLCFVehSig.bErrorStateLKA;
    pTJASAInput->LCFRCV_VehStopped_nu = pLCFInput->sLCFVehSig.bVehStopped;

    pTJASAInput->LCFRCV_SysStOnLatDMC_bool =
        pLCFInput->sLcfSenInputFromDMCData.bSysStOnLatDMC;

    pTJASAInput->LCFRCV_SteerWAngle_deg =
        pLCFInput->sLCFVehDyn.fSteerWheelAgl_rad * 180.f / TUE_CML_Pi;
    pTJASAInput->VDy_VehAclX_mps2 = pLCFInput->sLCFVehDyn.fEgoAccelX_mpss;
    pTJASAInput->VDy_VehAclY_mps2 = pLCFInput->sLCFVehDyn.fEgoAccelY_mpss;
    pTJASAInput->VDy_VehVelocity_kph =
        pLCFInput->sLCFVehDyn.fEgoVelX_mps * 3.6f;
    pTJASAInput->VDy_VehCrv_1pm = pLCFInput->sLCFVehDyn.fEgoCurve_1pm;
    pTJASAInput->VDy_VehYawRate_rps = pLCFInput->sLCFVehDyn.fEgoYawRate_rps;
    pTJASAInput->VDy_VehVelX_mps = pLCFInput->sLCFVehDyn.fEgoVelX_mps;

    pTJASAInput->VDPDRV_ActiveStCtrl_btf = sVSDPOutput.VSDP_CtrlStEn_St;
    pTJASAInput->VDPDRV_SysStError_btf = sVSDPOutput.VSDP_StError_St;
    pTJASAInput->VDPDRV_VehStInvalid_btf = sVSDPOutput.VSDP_VehStIvld_St;
    pTJASAInput->VDPDRV_DrvStInvalid_btf = sVSDPOutput.VSDP_IvldStDrv_St;
    pTJASAInput->VDPDRV_SysStNotAvailable_btf =
        sVSDPOutput.VSDP_CtrlStNoAvlb_St;

    pTJASAInput->VDy_DashboardVelocity_kph =
        pLCFInput->sLCFVehSig.fSpeedometerVelocity_kmh;
    pTJASAInput->LCFRCV_ManualTorque_nm =
        pLCFInput->sLCFVehDyn.fManuActuTrqEPS_nm;
    pTJASAInput->LCFRCV_SteerWAngleGrad_degps =
        pLCFInput->sLCFVehDyn.fSteerWheelAglChng_rps * 180.f / TUE_CML_Pi;
    pTJASAInput->LCFRCV_SLCHMISwitch_bool =
        pLCFInput->sLCFVehSig.bSLCHMISwitch_bool;
    pTJASAInput->LCFRCV_TJAAudioSwitch_bool = FALSE;

    pTJASAInput->LCCRA_LeftSafeFlag_bool = sLCCRAOUtput.LCCRA_LeftSafeFlag_bool;
    pTJASAInput->LCCRA_RightSafeFlag_bool =
        sLCCRAOUtput.LCCRA_RightSafeFlag_bool;
    pTJASAInput->LCCRA_LeftFrontSafeFlag_bool =
        sLCCRAOUtput.LCCRA_LeftFrontSafeFlag_bool;
    pTJASAInput->LCCRA_LeftRearSafeFlag_bool =
        sLCCRAOUtput.LCCRA_LeftRearSafeFlag_bool;
    pTJASAInput->LCCRA_RightFrontSafeFlag_bool =
        sLCCRAOUtput.LCCRA_RightFrontSafeFlag_bool;
    pTJASAInput->LCCRA_RightRearSafeFlag_bool =
        sLCCRAOUtput.LCCRA_RightRearSafeFlag_bool;

    pTJASAInput->LCFSEN_Nb_DCLCSwitchNVRAM_nu =
        pLCFInput->sNVRAMData.LCFSEN_Nb_DCLCSwitchNVRAM_nu;
    pTJASAInput->LCCRA_LeftHighLightID_nu =
        sLCCRAOUtput.LCCRA_LeftHighLightID_nu;
    pTJASAInput->LCCRA_RightHighLightID_nu =
        sLCCRAOUtput.LCCRA_RightHighLightID_nu;

    pTJASAInput->VLCVEH_ACCStatus_nu = pLCFInput->sVLCData.VLCVEH_ACCStatus_nu;

    pTJASAInput->LCFRCV_PilotOnLeverSwitch_bool =
        pLCFInput->sLCFVehSig.LCFRCV_PilotOnLeverSwitch_bool;
    pTJASAInput->LCFRCV_PilotOffLeverSwitch_bool =
        pLCFInput->sLCFVehSig.LCFRCV_PilotOffLeverSwitch_bool;
    pTJASAInput->LCFRCV_TurnLightReqSt_nu =
        pLCFInput->sLCFVehSig.LCFRCV_TurnLightReqSt_nu;

    pTJASAInput->LCFRCV_DoorOpen_bool =
        pLCFInput->sLCFVehSig.LCFRCV_DoorOpen_bool;
    pTJASAInput->LCFRCV_HoodOpen_bool =
        pLCFInput->sLCFVehSig.LCFRCV_HoodOpen_bool;
    pTJASAInput->LCFRCV_TrunkUnLock_bool =
        pLCFInput->sLCFVehSig.LCFRCV_TrunkUnLock_bool;
    pTJASAInput->LCFRCV_bPGear_bool = pLCFInput->sLCFVehSig.LCFRCV_bPGear_bool;
    pTJASAInput->LCCRA_inFrontSafeFlag_bool =
        sLCCRAOUtput.LCCRA_FrontSafeFlag_bool;
    pTJASAInput->LCFRCV_inFrontDangerObjID_nu =
        sLCCRAOUtput.LCCRA_FrontDangerObjID_nu;
    pTJASAInput->EMFID_bFCA_bool = pLCFInput->sLCFVehSig.bFCA;
    pTJASAInput->EMFID_bFCB_bool = pLCFInput->sLCFVehSig.bFCB;
    pTJASAInput->EMFID_bRCA_bool = pLCFInput->sLCFVehSig.bRCA;
    pTJASAInput->EMFID_bRCC_bool = pLCFInput->sLCFVehSig.bRCC;
    pTJASAInput->LCFRCV_bPilotOnOff_bool = pLCFInput->sLCFVehSig.bPilotOnOff;

    pTJASAInput->ADCS11_Parking_WorkSts =
        pLCFInput->sLCFVehSig.ADCS11_Parking_WorkSts;
    pTJASAInput->ADCS4_AVM_Sts = pLCFInput->sLCFVehSig.ADCS4_AVM_Sts;
}

/* ****************************************************************************
 Functionname:     TJASAOutputWrapper
 @brief: the output wrapper function for the TJASA process

 @description: convert TJASA sub-module's result to LCF Sen output structure

 @param[in]
                        const sTJASAOutPro_t sTJASAOutput: TJASA sub-module's
output
 @param[out]
                        proLcfSenPrtList_t* LCFSenOutput: LCF Sen output pointer
 @return           -
 @pre
 @post             -
 @author
**************************************************************************** */
static void TJASAOutputWrapper(const sTJASAOutPro_t sTJASAOutput,
                               proLcfSenPrtList_t* pLCFSenOutput) {
    pLCFSenOutput->pLcfSenOutputToVehData.sTJASAOutput.TJASTM_LatCtrlMode_nu =
        sTJASAOutput.TJASTM_LatCtrlMode_nu;
    pLCFSenOutput->pLcfSenOutputData.sTJAOutput.TJASLC_LaneChangWarning_nu =
        sTJASAOutput.TJASLC_LaneChangWarning_nu;
    pLCFSenOutput->pLcfSenOutputData.sTJAOutput.TJASTM_SysStateHWA_nu =
        sTJASAOutput.TJASTM_SysStateHWA_nu;
    pLCFSenOutput->pLcfSenOutputData.sTJAOutput.TJASTM_NpilotSysInfo =
        sTJASAOutput.TJASTM_NpilotSysInfo;
    pLCFSenOutput->pLcfSenOutputData.sTJAOutput.TJASTM_PilotAudioPlay =
        sTJASAOutput.TJASTM_PilotAudioPlay;
    pLCFSenOutput->pLcfSenOutputData.sTJAOutput
        .TJASTM_LatCtrlHandsOffReleaseWarn_nu =
        sTJASAOutput.TJASTM_LatCtrlHandsOffReleaseWarn_nu;
    pLCFSenOutput->pLcfSenOutputData.sTJAOutput.TJASLC_SLCAudioPlay_nu =
        sTJASAOutput.TJASLC_SLCAudioPlay_nu;
    pLCFSenOutput->pLcfSenOutputData.sTJAOutput.TJASTM_PilotEnableACCSwitch_nu =
        sTJASAOutput.TJASTM_PilotEnableACCSwitch_nu;
    pLCFSenOutput->pLcfSenOutputData.sTJAOutput
        .TJASTM_PilotDisableACCSwitch_nu =
        sTJASAOutput.TJASTM_PilotDisableACCSwitch_nu;
    pLCFSenOutput->sNVRAMData.LCFSEN_Nb_DCLCSwitchNVRAM_nu =
        sTJASAOutput.TJASLC_Nb_DCLCSwitchNVRAM_nu;
}

/* ****************************************************************************
 Functionname:   MCTLFCInputWrapper
 @brief:		 the input wrapper function for the MCTLFC process

 @description:	 the input wrapper function for the MCTLFC process

 @param[in]		 reqLcfSenPrtList_t* pLCFInput:	the input of the
LCF whole module
 @param[in]		 sLBPOutput_t sLBPOutput:		the output of
the LBP sub-module
 @param[in]		 reqLcfSenParams* pLCFParam:	the parameter of the
LCF whole module
 @param[in]	     sALPInReq_st sALPOutput:		the output of the
ALP sub-module
 @param[in]		 ODPROutPro_t sODPROutput:		the output of
the ODPR sub-module
 @param[in]      sVSDPOutput_t sVSDPOutput:		the output of the VSDP
sub-module
 @param[out]     sTJASAInReq_t* pTJASAInput:	the input of the TJASA
sub-module

 @return         -
 @pre
 @post           -
 @author
**************************************************************************** */
static void MCTLFCInputWrapper(const reqLcfSenPrtList_t* pLCFInput,
                               const reqLcfSenParams* pLCFParam,
                               const sTJASAOutPro_t* sTJASAOutput,
                               const sLDPSAOutput_t sLDPSAOutput,
                               const sLDPSADebug_t sLDPSADebug,
                               sMCTLFCInReq_st* pMCTLFCInput) {
    pMCTLFCInput->MCTLFC_LCFRCV_SysCycleTimeSen_sec =
        pLCFParam->LCFSen_Kf_SysCycleTime_sec;
    pMCTLFCInput->MCTLFC_LCFRCV_TJASTM_SysStateTJA_nu =
        sTJASAOutput->TJASTM_SysStateTJA_nu;
    pMCTLFCInput->MCTLFC_DPLSMI_SysStateLDP_nu = sLDPSAOutput.LDPSC_SysOut_St;
    pMCTLFCInput->MCTLFC_DPOSTM_SysStateLDPOC_nu = 0u;
    pMCTLFCInput->MCTLFC_DPRSMI_SysStateRDP_nu = 0u;
    pMCTLFCInput->MCTLFC_LCRSMI_SysStateALCA_nu = 0u;
    pMCTLFCInput->MCTLFC_LCSSTM_SysStateAOLC_nu = 0u;
    pMCTLFCInput->MCTLFC_ESASTM_SysStateESA_nu = 0u;

    /* LDP */
    pMCTLFCInput->MCTLFC_SysStateLDP_nu = sLDPSAOutput.LDPSC_SysOut_St;

    pMCTLFCInput->MCTLFC_DPLTTG_LeCridrBndCrvChng_1pm2 =
        sLDPSADebug.LDPTT_LnBdryCurvRateLf_ReMi2;
    pMCTLFCInput->MCTLFC_DPLTTG_LeCridrBndCrv_1pm =
        sLDPSADebug.LDPTT_LnBdryCurvLf_ReMi;
    pMCTLFCInput->MCTLFC_DPLTTG_LeCridrBndHeadAng_rad =
        sLDPSADebug.LDPTT_LnBdryHeadAglLf_Rad;
    pMCTLFCInput->MCTLFC_DPLTTG_LeCridrBndLength_met =
        sLDPSADebug.LDPTT_LnBdryVldLengLf_Mi;
    pMCTLFCInput->MCTLFC_DPLTTG_LeCridrBndPosX0_met =
        sLDPSADebug.LDPTT_LnBdryPstnXLf_Mi;
    pMCTLFCInput->MCTLFC_DPLTTG_LeCridrBndPosY0_met =
        sLDPSADebug.LDPTT_LnBdryPstnYLf_Mi;
    pMCTLFCInput->MCTLFC_DPLTTG_RiCridrBndCrvChng_1pm2 =
        sLDPSADebug.LDPTT_LnBdryCurvRateRi_ReMi2;
    pMCTLFCInput->MCTLFC_DPLTTG_RiCridrBndCrv_1pm =
        sLDPSADebug.LDPTT_LnBdryCurvRi_ReMi;
    pMCTLFCInput->MCTLFC_DPLTTG_RiCridrBndHeadAng_rad =
        sLDPSADebug.LDPTT_LnBdryHeadAglRi_Rad;
    pMCTLFCInput->MCTLFC_DPLTTG_RiCridrBndLength_met =
        sLDPSADebug.LDPTT_LnBdryVldLengRi_Mi;
    pMCTLFCInput->MCTLFC_DPLTTG_RiCridrBndPosX0_met =
        sLDPSADebug.LDPTT_LnBdryPstnXRi_Mi;
    pMCTLFCInput->MCTLFC_DPLTTG_RiCridrBndPosY0_met =
        sLDPSADebug.LDPTT_LnBdryPstnYRi_Mi;
    pMCTLFCInput->MCTLFC_DPLTTG_TgtTrajCrvChng_1pm2 =
        sLDPSADebug.LDPTT_LnBdryCurvRateCent_ReMi2;
    pMCTLFCInput->MCTLFC_DPLTTG_TgtTrajCrv_1pm =
        sLDPSADebug.LDPTT_LnBdryCurvCent_ReMi;
    pMCTLFCInput->MCTLFC_DPLTTG_TgtTrajHeadAng_rad =
        sLDPSADebug.LDPTT_LnBdryHeadAglCent_Rad;
    pMCTLFCInput->MCTLFC_DPLTTG_TgtTrajLength_met =
        sLDPSADebug.LDPTT_LnBdryVldLengCent_Mi;
    pMCTLFCInput->MCTLFC_DPLTTG_TgtTrajPosX0_met =
        sLDPSADebug.LDPTT_LnBdryPstnXCent_Mi;
    pMCTLFCInput->MCTLFC_DPLTTG_TgtTrajPosY0_met =
        sLDPSADebug.LDPTT_LnBdryPstnYCent_Mi;

    pMCTLFCInput->MCTLFC_DPLTVG_DeratingLevel_nu =
        sLDPSADebug.LDPTV_LstDMCDeraLvl_Fct;  // The data type of this signal
    // needs to be confirmed.（float32
    // -> uint8）
    pMCTLFCInput->MCTLFC_DPLTVG_DistYToLeTgtArea_met =
        sLDPSADebug.LDPTV_DstcYTgtAreaLf_Mi;
    pMCTLFCInput->MCTLFC_DPLTVG_DistYToRiTgtArea_met =
        sLDPSADebug.LDPTV_DstcYTgtAreaRi_Mi;
    pMCTLFCInput->MCTLFC_DPLTVG_FTireAclMax_mps2 =
        sLDPSADebug.LDPTV_FTireAccMx_Mps2;
    pMCTLFCInput->MCTLFC_DPLTVG_FTireAclMin_mps2 =
        sLDPSADebug.LDPTV_FTireAccMn_Mps2;
    pMCTLFCInput->MCTLFC_DPLTVG_GrdLimitTgtCrvTGC_1pms =
        sLDPSADebug.LDPTV_LmtCurvGradIncMx_ReMps;
    pMCTLFCInput->MCTLFC_DPLTVG_HighStatAccu_bool =
        sLDPSADebug.LDPTV_HighStatReq_B;
    pMCTLFCInput->MCTLFC_DPLTVG_LimiterActivated_bool =
        sLDPSADebug.LDPTV_LmtEn_B;
    pMCTLFCInput->MCTLFC_DPLTVG_LimiterTimeDuration_sec =
        sLDPSADebug.LDPTV_TiLmtEnDura_Sec;
    pMCTLFCInput->MCTLFC_DPLTVG_LtcyCompActivated_bool =
        sLDPSADebug.LDPTV_LatCpstnEn_B;
    pMCTLFCInput->MCTLFC_DPLTVG_MaxCrvGrdBuildup_1pms =
        sLDPSADebug.LDPTV_LmtCurvGradDecMx_ReMps;
    pMCTLFCInput->MCTLFC_DPLTVG_MaxCrvGrdRed_1pms =
        sLDPSADebug.LDPTV_LmtCurvGradCtrlMx_ReMps;
    pMCTLFCInput->MCTLFC_DPLTVG_MaxCrvTrajGuiCtrl_1pm =
        sLDPSADebug.LDPTV_LmtCurvMx_ReMi;
    pMCTLFCInput->MCTLFC_DPLTVG_MaxJerkAllowed_mps3 =
        sLDPSADebug.LDPTV_JerkLmtMx_Mps3;
    pMCTLFCInput->MCTLFC_DPLTVG_MaxTrqScaILimit_nu =
        sLDPSADebug.LDPTV_MxTrqScalGradLmt_Fct;
    pMCTLFCInput->MCTLFC_DPLTVG_MaxTrqScalGrad_1ps =
        sLDPSADebug.LDPTV_MxTrqScalGrad_ReS;
    pMCTLFCInput->MCTLFC_DPLTVG_ObstacleAclX_mps2 =
        sLDPSADebug.LDPTV_AccXObst_Mps2;
    pMCTLFCInput->MCTLFC_DPLTVG_ObstacleDistX_met =
        sLDPSADebug.LDPTV_DstcXObst_Mi;
    pMCTLFCInput->MCTLFC_DPLTVG_ObstacleDistY_met =
        sLDPSADebug.LDPTV_DstcYObst_Mi;
    pMCTLFCInput->MCTLFC_DPLTVG_ObstacleVelX_mps =
        sLDPSADebug.LDPTV_VeloXObst_Mps;
    pMCTLFCInput->MCTLFC_DPLTVG_ObstacleWidth_met =
        sLDPSADebug.LDPTV_WidObst_Mi;
    pMCTLFCInput->MCTLFC_DPLTVG_PlanningHorzion_sec =
        sLDPSADebug.LDPTV_LstPlanningHorizon_Sec;
    pMCTLFCInput->MCTLFC_DPLTVG_PredTimeCrv_sec =
        sLDPSADebug.LDPTV_PredTiCurv_Sec;
    pMCTLFCInput->MCTLFC_DPLTVG_PredTimeHeadAng_sec =
        sLDPSADebug.LDPTV_PredTiAgl_Sec;
    pMCTLFCInput->MCTLFC_DPLTVG_SensorTStamp_sec =
        sLDPSADebug.LDPTV_SnsTiStamp_Sec;
    pMCTLFCInput->MCTLFC_DPLTVG_StrWhStifGrad_1ps =
        sLDPSADebug.LDPTV_SteWhlGrad_ReS;
    pMCTLFCInput->MCTLFC_DPLTVG_StrWhStifLimit_nu =
        sLDPSADebug.LDPTV_SteWhlGradLmt_Fct;
    pMCTLFCInput->MCTLFC_DPLTVG_TrajGuiQu_nu = sLDPSADebug.LDPTV_TrajCtrlSt_St;
    pMCTLFCInput->MCTLFC_DPLTVG_TrajPlanServQu_nu =
        sLDPSADebug.LDPTV_TrajPlanServQu_Fct;  // The data type of this signal
                                               // needs to be
    // confirmed.（float32 -> uint8）
    pMCTLFCInput->MCTLFC_DPLTVG_TriggerReplan_bool =
        sLDPSADebug.LDPTV_TrigReplan_B;
    pMCTLFCInput->MCTLFC_DPLTVG_WeightEndTime_nu =
        sLDPSADebug.LDPTV_LstWeightEndTi_Fct;
    pMCTLFCInput->MCTLFC_DPLTVG_WeightTgtDistY_nu =
        sLDPSADebug.LDPTV_FctTgtDistY_Fct;

    /* TJASA */
    pMCTLFCInput->MCTLFC_SysStateTJA_nu = sTJASAOutput->TJASTM_SysStateTJA_nu;

    pMCTLFCInput->MCTLFC_TJATTG_LeCridrBndCrvChng_1pm2 =
        sTJASAOutput->TJATTG_LeCridrBndCrvChng_1pm2;
    pMCTLFCInput->MCTLFC_TJATTG_LeCridrBndCrv_1pm =
        sTJASAOutput->TJATTG_LeCridrBndCrv_1pm;
    pMCTLFCInput->MCTLFC_TJATTG_LeCridrBndHeadAng_rad =
        sTJASAOutput->TJATTG_LeCridrBndHeadAng_rad;
    pMCTLFCInput->MCTLFC_TJATTG_LeCridrBndLength_met =
        sTJASAOutput->TJATTG_LeCridrBndLength_met;
    pMCTLFCInput->MCTLFC_TJATTG_LeCridrBndPosX0_met =
        sTJASAOutput->TJATTG_LeCridrBndPosX0_met;
    pMCTLFCInput->MCTLFC_TJATTG_LeCridrBndPosY0_met =
        sTJASAOutput->TJATTG_LeCridrBndPosY0_met;
    pMCTLFCInput->MCTLFC_TJATTG_RiCridrBndCrvChng_1pm2 =
        sTJASAOutput->TJATTG_RiCridrBndCrvChng_1pm2;
    pMCTLFCInput->MCTLFC_TJATTG_RiCridrBndCrv_1pm =
        sTJASAOutput->TJATTG_RiCridrBndCrv_1pm;
    pMCTLFCInput->MCTLFC_TJATTG_RiCridrBndHeadAng_rad =
        sTJASAOutput->TJATTG_RiCridrBndHeadAng_rad;
    pMCTLFCInput->MCTLFC_TJATTG_RiCridrBndLength_met =
        sTJASAOutput->TJATTG_RiCridrBndLength_met;
    pMCTLFCInput->MCTLFC_TJATTG_RiCridrBndPosX0_met =
        sTJASAOutput->TJATTG_RiCridrBndPosX0_met;
    pMCTLFCInput->MCTLFC_TJATTG_RiCridrBndPosY0_met =
        sTJASAOutput->TJATTG_RiCridrBndPosY0_met;
    pMCTLFCInput->MCTLFC_TJATTG_TgtTrajCrvChng_1pm2 =
        sTJASAOutput->TJATTG_TgtTrajCrvChng_1pm2;
    pMCTLFCInput->MCTLFC_TJATTG_TgtTrajCrv_1pm =
        sTJASAOutput->TJATTG_TgtTrajCrv_1pm;
    pMCTLFCInput->MCTLFC_TJATTG_TgtTrajHeadAng_rad =
        sTJASAOutput->TJATTG_TgtTrajHeadAng_rad;
    pMCTLFCInput->MCTLFC_TJATTG_TgtTrajLength_met =
        sTJASAOutput->TJATTG_TgtTrajLength_met;
    pMCTLFCInput->MCTLFC_TJATTG_TgtTrajPosX0_met =
        sTJASAOutput->TJATTG_TgtTrajPosX0_met;
    pMCTLFCInput->MCTLFC_TJATTG_TgtTrajPosY0_met =
        sTJASAOutput->TJATTG_TgtTrajPosY0_met;

    pMCTLFCInput->MCTLFC_TJATVG_DeratingLevel_nu =
        sTJASAOutput->TJATVG_DeratingLevel_nu;
    pMCTLFCInput->MCTLFC_TJATVG_DistYToLeTgtArea_met =
        sTJASAOutput->TJATVG_DistYTolLeTgtArea_met;
    pMCTLFCInput->MCTLFC_TJATVG_DistYToRiTgtArea_met =
        sTJASAOutput->TJATVG_DistYTolRiTgtArea_met;
    pMCTLFCInput->MCTLFC_TJATVG_FTireAclMax_mps2 =
        sTJASAOutput->TJATVG_FTireAclMax_mps2;
    pMCTLFCInput->MCTLFC_TJATVG_FTireAclMin_mps2 =
        sTJASAOutput->TJATVG_FTireAclMin_mps2;
    pMCTLFCInput->MCTLFC_TJATVG_GrdLimitTgtCrvTGC_1pms =
        sTJASAOutput
            ->TJATVG_MaxCrvGrdTGC_1pms;  // TJASA模型中的信号输出也应该为GrdLimitTgtCrvTGC_1pms，待确认TJATVG模块输出信号是否有误
    pMCTLFCInput->MCTLFC_TJATVG_HighStatAccu_bool =
        sTJASAOutput->TJATVG_HighStatAccu_bool;
    pMCTLFCInput->MCTLFC_TJATVG_LimiterActivated_bool =
        sTJASAOutput->TJATVG_LimiterActivated_nu;
    pMCTLFCInput->MCTLFC_TJATVG_LimiterTimeDuration_sec =
        sTJASAOutput->TJATVG_LimiterTimeDuration_sec;
    pMCTLFCInput->MCTLFC_TJATVG_LtcyCompActivated_bool =
        sTJASAOutput->TJATVG_LtcyCompActivated_nu;
    pMCTLFCInput->MCTLFC_TJATVG_MaxCrvGrdBuildup_1pms =
        sTJASAOutput->TJATVG_MaxCrvGrdBuildup_1pms;
    pMCTLFCInput->MCTLFC_TJATVG_MaxCrvGrdRed_1pms =
        sTJASAOutput->TJATVG_MaxCrvGrdRed_1pms;
    pMCTLFCInput->MCTLFC_TJATVG_MaxCrvTrajGuiCtrl_1pm =
        sTJASAOutput->TJATVG_MaxCrvTrajGuiCtl_1pm;
    pMCTLFCInput->MCTLFC_TJATVG_MaxJerkAllowed_mps3 =
        sTJASAOutput->TJATVG_MaxJerkAllowed_mps3;
    pMCTLFCInput->MCTLFC_TJATVG_MaxTrqScaILimit_nu =
        sTJASAOutput->TJATVG_MaxTrqScalLimit_nu;
    pMCTLFCInput->MCTLFC_TJATVG_MaxTrqScalGrad_1ps =
        sTJASAOutput->TJATVG_MaxTrqScalGrad_1ps;
    pMCTLFCInput->MCTLFC_TJATVG_ObstacleAclX_mps2 =
        sTJASAOutput->TJATVG_ObstacleAclX_mps2;
    pMCTLFCInput->MCTLFC_TJATVG_ObstacleDistX_met =
        sTJASAOutput->TJATVG_ObstacleDistX_met;
    pMCTLFCInput->MCTLFC_TJATVG_ObstacleDistY_met =
        sTJASAOutput->TJATVG_ObstacleDistY_met;
    pMCTLFCInput->MCTLFC_TJATVG_ObstacleVelX_mps =
        sTJASAOutput->TJATVG_ObstacleVelX_mps;
    pMCTLFCInput->MCTLFC_TJATVG_ObstacleWidth_met =
        sTJASAOutput->TJATVG_ObstacleWidth_met;
    pMCTLFCInput->MCTLFC_TJATVG_PlanningHorzion_sec =
        sTJASAOutput->TJATVG_PlanningHorizon_sec;
    pMCTLFCInput->MCTLFC_TJATVG_PredTimeCrv_sec =
        sTJASAOutput->TJATVG_PredTimeCrv_sec;
    pMCTLFCInput->MCTLFC_TJATVG_PredTimeHeadAng_sec =
        sTJASAOutput->TJATVG_PredTimeHeadAng_sec;
    pMCTLFCInput->MCTLFC_TJATVG_SensorTStamp_sec =
        sTJASAOutput->TJATVG_SensorTStamp_sec;
    pMCTLFCInput->MCTLFC_TJATVG_StrWhStifGrad_1ps =
        sTJASAOutput->TJATVG_StrWhStifGrad_1ps;
    pMCTLFCInput->MCTLFC_TJATVG_StrWhStifLimit_nu =
        sTJASAOutput->TJATVG_StrWhStifLimit_nu;
    pMCTLFCInput->MCTLFC_TJATVG_TrajGuiQu_nu =
        sTJASAOutput->TJATVG_TrajGuiQu_nu;
    pMCTLFCInput->MCTLFC_TJATVG_TrajPlanServQu_nu =
        sTJASAOutput->TJATVG_TrajPlanServQu_nu;
    pMCTLFCInput->MCTLFC_TJATVG_TriggerReplan_bool =
        sTJASAOutput->TJATVG_TriggerReplan_nu;
    pMCTLFCInput->MCTLFC_TJATVG_TrqRampGrad_1ps =
        sTJASAOutput->TJATVG_TrqRampGrad_1ps;
    pMCTLFCInput->MCTLFC_TJATVG_WeightEndTime_nu =
        sTJASAOutput->TJATVG_WeightEndTime_nu;
    pMCTLFCInput->MCTLFC_TJATVG_WeightTgtDistY_nu =
        sTJASAOutput->TJATVG_WeightTgtDistY_nu;

    /* RDP */
    pMCTLFCInput->MCTLFC_SysStateRDP_nu = 0u;

    pMCTLFCInput->MCTLFC_DPRTTG_LeCridrBndCrvChng_1pm2 = 0.f;
    pMCTLFCInput->MCTLFC_DPRTTG_LeCridrBndCrv_1pm = 0.f;
    pMCTLFCInput->MCTLFC_DPRTTG_LeCridrBndHeadAng_rad = 0.f;
    pMCTLFCInput->MCTLFC_DPRTTG_LeCridrBndLength_met = 0.f;
    pMCTLFCInput->MCTLFC_DPRTTG_LeCridrBndPosX0_met = 0.f;
    pMCTLFCInput->MCTLFC_DPRTTG_LeCridrBndPosY0_met = 0.f;
    pMCTLFCInput->MCTLFC_DPRTTG_RiCridrBndCrvChng_1pm2 = 0.f;
    pMCTLFCInput->MCTLFC_DPRTTG_RiCridrBndCrv_1pm = 0.f;
    pMCTLFCInput->MCTLFC_DPRTTG_RiCridrBndHeadAng_rad = 0.f;
    pMCTLFCInput->MCTLFC_DPRTTG_RiCridrBndLength_met = 0.f;
    pMCTLFCInput->MCTLFC_DPRTTG_RiCridrBndPosX0_met = 0.f;
    pMCTLFCInput->MCTLFC_DPRTTG_RiCridrBndPosY0_met = 0.f;
    pMCTLFCInput->MCTLFC_DPRTTG_TgtTrajCrvChng_1pm2 = 0.f;
    pMCTLFCInput->MCTLFC_DPRTTG_TgtTrajCrv_1pm = 0.f;
    pMCTLFCInput->MCTLFC_DPRTTG_TgtTrajHeadAng_rad = 0.f;
    pMCTLFCInput->MCTLFC_DPRTTG_TgtTrajLength_met = 0.f;
    pMCTLFCInput->MCTLFC_DPRTTG_TgtTrajPosX0_met = 0.f;
    pMCTLFCInput->MCTLFC_DPRTTG_TgtTrajPosY0_met = 0.f;

    pMCTLFCInput->MCTLFC_DPRTVG_DeratingLevel_nu = 0u;
    pMCTLFCInput->MCTLFC_DPRTVG_DistYToLeTgtArea_met = 0.f;
    pMCTLFCInput->MCTLFC_DPRTVG_DistYToRiTgtArea_met = 0.f;
    pMCTLFCInput->MCTLFC_DPRTVG_FTireAclMax_mps2 = 0.f;
    pMCTLFCInput->MCTLFC_DPRTVG_FTireAclMin_mps2 = 0.f;
    pMCTLFCInput->MCTLFC_DPRTVG_GrdLimitTgtCrvTGC_1pms = 0.f;
    pMCTLFCInput->MCTLFC_DPRTVG_HighStatAccu_bool = FALSE;
    pMCTLFCInput->MCTLFC_DPRTVG_LimiterActivated_bool = FALSE;
    pMCTLFCInput->MCTLFC_DPRTVG_LimiterTimeDuration_sec = 0.f;
    pMCTLFCInput->MCTLFC_DPRTVG_LtcyCompActivated_bool = FALSE;
    pMCTLFCInput->MCTLFC_DPRTVG_MaxCrvGrdBuildup_1pms = 0.f;
    pMCTLFCInput->MCTLFC_DPRTVG_MaxCrvGrdRed_1pms = 0.f;
    pMCTLFCInput->MCTLFC_DPRTVG_MaxCrvTrajGuiCtrl_1pm = 0.f;
    pMCTLFCInput->MCTLFC_DPRTVG_MaxJerkAllowed_mps3 = 0.f;
    pMCTLFCInput->MCTLFC_DPRTVG_MaxTrqScaILimit_nu = 0.f;
    pMCTLFCInput->MCTLFC_DPRTVG_MaxTrqScalGrad_1ps = 0.f;
    pMCTLFCInput->MCTLFC_DPRTVG_ObstacleAclX_mps2 = 0.f;
    pMCTLFCInput->MCTLFC_DPRTVG_ObstacleDistX_met = 0.f;
    pMCTLFCInput->MCTLFC_DPRTVG_ObstacleDistY_met = 0.f;
    pMCTLFCInput->MCTLFC_DPRTVG_ObstacleVelX_mps = 0.f;
    pMCTLFCInput->MCTLFC_DPRTVG_ObstacleWidth_met = 0.f;
    pMCTLFCInput->MCTLFC_DPRTVG_PlanningHorzion_sec = 0.f;
    pMCTLFCInput->MCTLFC_DPRTVG_PredTimeCrv_sec = 0.f;
    pMCTLFCInput->MCTLFC_DPRTVG_PredTimeHeadAng_sec = 0.f;
    pMCTLFCInput->MCTLFC_DPRTVG_SensorTStamp_sec = 0.f;
    pMCTLFCInput->MCTLFC_DPRTVG_StrWhStifGrad_1ps = 0.f;
    pMCTLFCInput->MCTLFC_DPRTVG_StrWhStifLimit_nu = 0.f;
    pMCTLFCInput->MCTLFC_DPRTVG_TrajGuiQu_nu = 0u;
    pMCTLFCInput->MCTLFC_DPRTVG_TrajPlanServQu_nu = 0u;
    pMCTLFCInput->MCTLFC_DPRTVG_TriggerReplan_bool = FALSE;
    pMCTLFCInput->MCTLFC_DPRTVG_TrqRampGrad_1ps = 0.f;
    pMCTLFCInput->MCTLFC_DPRTVG_WeightEndTime_nu = 0.f;
    pMCTLFCInput->MCTLFC_DPRTVG_WeightTgtDistY_nu = 0.f;

    /* LDPOC */
    pMCTLFCInput->MCTLFC_SysStateLDPOC_nu = 0u;

    pMCTLFCInput->MCTLFC_DPOTTG_LeCridrBndCrvChng_1pm2 = 0.f;
    pMCTLFCInput->MCTLFC_DPOTTG_LeCridrBndCrv_1pm = 0.f;
    pMCTLFCInput->MCTLFC_DPOTTG_LeCridrBndHeadAng_rad = 0.f;
    pMCTLFCInput->MCTLFC_DPOTTG_LeCridrBndLength_met = 0.f;
    pMCTLFCInput->MCTLFC_DPOTTG_LeCridrBndPosX0_met = 0.f;
    pMCTLFCInput->MCTLFC_DPOTTG_LeCridrBndPosY0_met = 0.f;
    pMCTLFCInput->MCTLFC_DPOTTG_RiCridrBndCrvChng_1pm2 = 0.f;
    pMCTLFCInput->MCTLFC_DPOTTG_RiCridrBndCrv_1pm = 0.f;
    pMCTLFCInput->MCTLFC_DPOTTG_RiCridrBndHeadAng_rad = 0.f;
    pMCTLFCInput->MCTLFC_DPOTTG_RiCridrBndLength_met = 0.f;
    pMCTLFCInput->MCTLFC_DPOTTG_RiCridrBndPosX0_met = 0.f;
    pMCTLFCInput->MCTLFC_DPOTTG_RiCridrBndPosY0_met = 0.f;
    pMCTLFCInput->MCTLFC_DPOTTG_TgtTrajCrvChng_1pm2 = 0.f;
    pMCTLFCInput->MCTLFC_DPOTTG_TgtTrajCrv_1pm = 0.f;
    pMCTLFCInput->MCTLFC_DPOTTG_TgtTrajHeadAng_rad = 0.f;
    pMCTLFCInput->MCTLFC_DPOTTG_TgtTrajLength_met = 0.f;
    pMCTLFCInput->MCTLFC_DPOTTG_TgtTrajPosX0_met = 0.f;
    pMCTLFCInput->MCTLFC_DPOTTG_TgtTrajPosY0_met = 0.f;

    pMCTLFCInput->MCTLFC_DPOTVG_DeratingLevel_nu = 0u;
    pMCTLFCInput->MCTLFC_DPOTVG_DistYToLeTgtArea_met = 0.f;
    pMCTLFCInput->MCTLFC_DPOTVG_DistYToRiTgtArea_met = 0.f;
    pMCTLFCInput->MCTLFC_DPOTVG_FTireAclMax_mps2 = 0.f;
    pMCTLFCInput->MCTLFC_DPOTVG_FTireAclMin_mps2 = 0.f;
    pMCTLFCInput->MCTLFC_DPOTVG_GrdLimitTgtCrvTGC_1pms = 0.f;
    pMCTLFCInput->MCTLFC_DPOTVG_HighStatAccu_bool = FALSE;
    pMCTLFCInput->MCTLFC_DPOTVG_LimiterActivated_bool = FALSE;
    pMCTLFCInput->MCTLFC_DPOTVG_LimiterTimeDuration_sec = 0.f;
    pMCTLFCInput->MCTLFC_DPOTVG_LtcyCompActivated_bool = FALSE;
    pMCTLFCInput->MCTLFC_DPOTVG_MaxCrvGrdBuildup_1pms = 0.f;
    pMCTLFCInput->MCTLFC_DPOTVG_MaxCrvGrdRed_1pms = 0.f;
    pMCTLFCInput->MCTLFC_DPOTVG_MaxCrvTrajGuiCtrl_1pm = 0.f;
    pMCTLFCInput->MCTLFC_DPOTVG_MaxJerkAllowed_mps3 = 0.f;
    pMCTLFCInput->MCTLFC_DPOTVG_MaxTrqScaILimit_nu = 0.f;
    pMCTLFCInput->MCTLFC_DPOTVG_MaxTrqScalGrad_1ps = 0.f;
    pMCTLFCInput->MCTLFC_DPOTVG_ObstacleAclX_mps2 = 0.f;
    pMCTLFCInput->MCTLFC_DPOTVG_ObstacleDistX_met = 0.f;
    pMCTLFCInput->MCTLFC_DPOTVG_ObstacleDistY_met = 0.f;
    pMCTLFCInput->MCTLFC_DPOTVG_ObstacleVelX_mps = 0.f;
    pMCTLFCInput->MCTLFC_DPOTVG_ObstacleWidth_met = 0.f;
    pMCTLFCInput->MCTLFC_DPOTVG_PlanningHorzion_sec = 0.f;
    pMCTLFCInput->MCTLFC_DPOTVG_PredTimeCrv_sec = 0.f;
    pMCTLFCInput->MCTLFC_DPOTVG_PredTimeHeadAng_sec = 0.f;
    pMCTLFCInput->MCTLFC_DPOTVG_SensorTStamp_sec = 0.f;
    pMCTLFCInput->MCTLFC_DPOTVG_StrWhStifGrad_1ps = 0.f;
    pMCTLFCInput->MCTLFC_DPOTVG_StrWhStifLimit_nu = 0.f;
    pMCTLFCInput->MCTLFC_DPOTVG_TrajGuiQu_nu = 0u;
    pMCTLFCInput->MCTLFC_DPOTVG_TrajPlanServQu_nu = 0u;
    pMCTLFCInput->MCTLFC_DPOTVG_TriggerReplan_bool = FALSE;
    pMCTLFCInput->MCTLFC_DPOTVG_TrqRampGrad_1ps = 0.f;
    pMCTLFCInput->MCTLFC_DPOTVG_WeightEndTime_nu = 0.f;
    pMCTLFCInput->MCTLFC_DPOTVG_WeightTgtDistY_nu = 0.f;

    /* ALCA */
    pMCTLFCInput->MCTLFC_SysStateALCA_nu = 0u;

    pMCTLFCInput->MCTLFC_LCRTTG_LeCridrBndCrvChng_1pm2 = 0.f;
    pMCTLFCInput->MCTLFC_LCRTTG_LeCridrBndCrv_1pm = 0.f;
    pMCTLFCInput->MCTLFC_LCRTTG_LeCridrBndHeadAng_rad = 0.f;
    pMCTLFCInput->MCTLFC_LCRTTG_LeCridrBndLength_met = 0.f;
    pMCTLFCInput->MCTLFC_LCRTTG_LeCridrBndPosX0_met = 0.f;
    pMCTLFCInput->MCTLFC_LCRTTG_LeCridrBndPosY0_met = 0.f;
    pMCTLFCInput->MCTLFC_LCRTTG_RiCridrBndCrvChng_1pm2 = 0.f;
    pMCTLFCInput->MCTLFC_LCRTTG_RiCridrBndCrv_1pm = 0.f;
    pMCTLFCInput->MCTLFC_LCRTTG_RiCridrBndHeadAng_rad = 0.f;
    pMCTLFCInput->MCTLFC_LCRTTG_RiCridrBndLength_met = 0.f;
    pMCTLFCInput->MCTLFC_LCRTTG_RiCridrBndPosX0_met = 0.f;
    pMCTLFCInput->MCTLFC_LCRTTG_RiCridrBndPosY0_met = 0.f;
    pMCTLFCInput->MCTLFC_LCRTTG_TgtTrajCrvChng_1pm2 = 0.f;
    pMCTLFCInput->MCTLFC_LCRTTG_TgtTrajCrv_1pm = 0.f;
    pMCTLFCInput->MCTLFC_LCRTTG_TgtTrajHeadAng_rad = 0.f;
    pMCTLFCInput->MCTLFC_LCRTTG_TgtTrajLength_met = 0.f;
    pMCTLFCInput->MCTLFC_LCRTTG_TgtTrajPosX0_met = 0.f;
    pMCTLFCInput->MCTLFC_LCRTTG_TgtTrajPosY0_met = 0.f;

    pMCTLFCInput->MCTLFC_LCRTVG_DeratingLevel_nu = 0u;
    pMCTLFCInput->MCTLFC_LCRTVG_DistYToLeTgtArea_met = 0.f;
    pMCTLFCInput->MCTLFC_LCRTVG_DistYToRiTgtArea_met = 0.f;
    pMCTLFCInput->MCTLFC_LCRTVG_FTireAclMax_mps2 = 0.f;
    pMCTLFCInput->MCTLFC_LCRTVG_FTireAclMin_mps2 = 0.f;
    pMCTLFCInput->MCTLFC_LCRTVG_GrdLimitTgtCrvTGC_1pms = 0.f;
    pMCTLFCInput->MCTLFC_LCRTVG_HighStatAccu_bool = FALSE;
    pMCTLFCInput->MCTLFC_LCRTVG_LimiterActivated_bool = FALSE;
    pMCTLFCInput->MCTLFC_LCRTVG_LimiterTimeDuration_sec = 0.f;
    pMCTLFCInput->MCTLFC_LCRTVG_LtcyCompActivated_bool = FALSE;
    pMCTLFCInput->MCTLFC_LCRTVG_MaxCrvGrdBuildup_1pms = 0.f;
    pMCTLFCInput->MCTLFC_LCRTVG_MaxCrvGrdRed_1pms = 0.f;
    pMCTLFCInput->MCTLFC_LCRTVG_MaxCrvTrajGuiCtrl_1pm = 0.f;
    pMCTLFCInput->MCTLFC_LCRTVG_MaxJerkAllowed_mps3 = 0.f;
    pMCTLFCInput->MCTLFC_LCRTVG_MaxTrqScaILimit_nu = 0.f;
    pMCTLFCInput->MCTLFC_LCRTVG_MaxTrqScalGrad_1ps = 0.f;
    pMCTLFCInput->MCTLFC_LCRTVG_ObstacleAclX_mps2 = 0.f;
    pMCTLFCInput->MCTLFC_LCRTVG_ObstacleDistX_met = 0.f;
    pMCTLFCInput->MCTLFC_LCRTVG_ObstacleDistY_met = 0.f;
    pMCTLFCInput->MCTLFC_LCRTVG_ObstacleVelX_mps = 0.f;
    pMCTLFCInput->MCTLFC_LCRTVG_ObstacleWidth_met = 0.f;
    pMCTLFCInput->MCTLFC_LCRTVG_PlanningHorzion_sec = 0.f;
    pMCTLFCInput->MCTLFC_LCRTVG_PredTimeCrv_sec = 0.f;
    pMCTLFCInput->MCTLFC_LCRTVG_PredTimeHeadAng_sec = 0.f;
    pMCTLFCInput->MCTLFC_LCRTVG_SensorTStamp_sec = 0.f;
    pMCTLFCInput->MCTLFC_LCRTVG_StrWhStifGrad_1ps = 0.f;
    pMCTLFCInput->MCTLFC_LCRTVG_StrWhStifLimit_nu = 0.f;
    pMCTLFCInput->MCTLFC_LCRTVG_TrajGuiQu_nu = 0u;
    pMCTLFCInput->MCTLFC_LCRTVG_TrajPlanServQu_nu = 0u;
    pMCTLFCInput->MCTLFC_LCRTVG_TriggerReplan_bool = FALSE;
    pMCTLFCInput->MCTLFC_LCRTVG_TrqRampGrad_1ps = 0.f;
    pMCTLFCInput->MCTLFC_LCRTVG_WeightEndTime_nu = 0.f;
    pMCTLFCInput->MCTLFC_LCRTVG_WeightTgtDistY_nu = 0.f;
}

/* ****************************************************************************
 Functionname:     MCTLFCOutputWrapper
 @brief:		   the output wrapper function for the MCTLFC process

 @description:	   convert MCTLFC sub-module's result to LCF Sen output
structure

 @param[in]		   const sMCTLFCOut_st sMCTLFCOutput: MCTLFC
sub-module's output
 @param[out]       proLcfSenPrtList_t* LCFSenOutput: LCF Sen output pointer
 @return           -
 @pre
 @post             -
 @author
**************************************************************************** */
static void MCTLFCOutputWrapper(const sMCTLFCOut_st sMCTLFCOutput,
                                proLcfSenPrtList_t* pLCFSenOutput) {
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut
        .CSCLTA_LeCridrBndPosX0_met = sMCTLFCOutput.CSCLTA_LeCridrBndPosX0_met;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut
        .CSCLTA_LeCridrBndPosY0_met = sMCTLFCOutput.CSCLTA_LeCridrBndPosY0_met;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_LeCridrHeadAng_rad =
        sMCTLFCOutput.CSCLTA_LeCridrHeadAng_rad;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_LeCridrBndCrv_1pm =
        sMCTLFCOutput.CSCLTA_LeCridrBndCrv_1pm;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut
        .CSCLTA_LeCridrBndCrvChng_1pm2 =
        sMCTLFCOutput.CSCLTA_LeCridrBndCrvChng_1pm2;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut
        .CSCLTA_LeCridrBndLength_met =
        sMCTLFCOutput.CSCLTA_LeCridrBndLength_met;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut
        .CSCLTA_RiCridrBndPosX0_met = sMCTLFCOutput.CSCLTA_RiCridrBndPosX0_met;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut
        .CSCLTA_RiCridrBndPosY0_met = sMCTLFCOutput.CSCLTA_RiCridrBndPosY0_met;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_RiCridrHeadAng_rad =
        sMCTLFCOutput.CSCLTA_RiCridrHeadAng_rad;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_RiCridrBndCrv_1pm =
        sMCTLFCOutput.CSCLTA_RiCridrBndCrv_1pm;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut
        .CSCLTA_RiCridrBndCrvChng_1pm2 =
        sMCTLFCOutput.CSCLTA_RiCridrBndCrvChng_1pm2;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut
        .CSCLTA_RiCridrBndLength_met =
        sMCTLFCOutput.CSCLTA_RiCridrBndLength_met;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_TgtTrajPosX0_met =
        sMCTLFCOutput.CSCLTA_TgtTrajPosX0_met;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_TgtTrajPosY0_met =
        sMCTLFCOutput.CSCLTA_TgtTrajPosY0_met;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_TgtTrajHeadAng_rad =
        sMCTLFCOutput.CSCLTA_TgtTrajHeadAng_rad;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_TgtTrajCrv_1pm =
        sMCTLFCOutput.CSCLTA_TgtTrajCrv_1pm;

    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut
        .CSCLTA_TgtTrajCrvChng_1pm2 = sMCTLFCOutput.CSCLTA_TgtTrajCrvChng_1pm2;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_TgtTrajLength_met =
        sMCTLFCOutput.CSCLTA_TgtTrajLength_met;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_WeightTgtDistY_nu =
        sMCTLFCOutput.CSCLTA_WeightTgtDistY_nu;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_WeightEndTime_nu =
        sMCTLFCOutput.CSCLTA_WeightEndTime_nu;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut
        .CSCLTA_DistYToILeTgtArea_met =
        sMCTLFCOutput.CSCLTA_DistYToILeTgtArea_met;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut
        .CSCLTA_DistYToIRiTgtArea_met =
        sMCTLFCOutput.CSCLTA_DistYToIRiTgtArea_met;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_FTireAclMax_mps2 =
        sMCTLFCOutput.CSCLTA_FTireAclMax_mps2;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_FTireAclMin_mps2 =
        sMCTLFCOutput.CSCLTA_FTireAclMin_mps2;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut
        .CSCLTA_PredTimeHeadAng_sec = sMCTLFCOutput.CSCLTA_PredTimeHeadAng_sec;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_PredTimeCrv_sec =
        sMCTLFCOutput.CSCLTA_PredTimeCrv_sec;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut
        .CSCLTA_PlanningHorzion_sec = sMCTLFCOutput.CSCLTA_PlanningHorzion_sec;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_ObstacleVelX_mps =
        sMCTLFCOutput.CSCLTA_ObstacleVelX_mps;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_ObstacleAclX_mps2 =
        sMCTLFCOutput.CSCLTA_ObstacleAclX_mps2;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_ObstacleWidth_met =
        sMCTLFCOutput.CSCLTA_ObstacleWidth_met;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_ObstacleDistX_met =
        sMCTLFCOutput.CSCLTA_ObstacleDistX_met;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_ObstacleDistY_met =
        sMCTLFCOutput.CSCLTA_ObstacleDistY_met;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_SensorTStamp_sec =
        sMCTLFCOutput.CSCLTA_SensorTStamp_sec;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut
        .CSCLTA_MaxCrvTrajGuiCtrl_1pm =
        sMCTLFCOutput.CSCLTA_MaxCrvTrajGuiCtrl_1pm;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut
        .CSCLTA_MaxCrvGrdBuildup_1pms =
        sMCTLFCOutput.CSCLTA_MaxCrvGrdBuildup_1pms;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_MaxCrvGrdRed_1pms =
        sMCTLFCOutput.CSCLTA_MaxCrvGrdRed_1pms;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut
        .CSCLTA_GrdLimitTgtCrvGC_1pms =
        sMCTLFCOutput.CSCLTA_GrdLimitTgtCrvGC_1pms;

    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_StrWhStifLimit_nu =
        sMCTLFCOutput.CSCLTA_StrWhStifLimit_nu;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_StrWhStifGrad_1ps =
        sMCTLFCOutput.CSCLTA_StrWhStifGrad_1ps;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_TrqRampGrad_1ps =
        sMCTLFCOutput.CSCLTA_TrqRampGrad_1ps;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_MaxTrqScalLimit_nu =
        sMCTLFCOutput.CSCLTA_MaxTrqScalLimit_nu;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_MaxTrqScalGrad_nu =
        sMCTLFCOutput.CSCLTA_MaxTrqScalGrad_nu;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut
        .CSCLTA_LimiterDuration_sec = sMCTLFCOutput.CSCLTA_LimiterDuration_sec;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut
        .CSCLTA_MaxJerkAllowed_mps3 = sMCTLFCOutput.CSCLTA_MaxJerkAllowed_mps3;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_DeratingLevel_nu =
        sMCTLFCOutput.CSCLTA_DeratingLevel_nu;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut
        .CSCLTA_ControllingFunction_nu =
        sMCTLFCOutput.CSCLTA_ControllingFunction_nu;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_SysStateLCF =
        sMCTLFCOutput.CSCLTA_SysStateLCF;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_TgtPlanServQu_nu =
        sMCTLFCOutput.CSCLTA_TgtPlanServQu_nu;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut
        .CSCLTA_TrajGuiQualifier_nu = sMCTLFCOutput.CSCLTA_TrajGuiQualifier_nu;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_TriggerReplan_nu =
        sMCTLFCOutput.CSCLTA_TriggerReplan_nu;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut
        .CSCLTA_LatencyCompActivated_bool =
        sMCTLFCOutput.CSCLTA_LatencyCompActivated_bool;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_HighStatAccu_bool =
        sMCTLFCOutput.CSCLTA_HighStatAccu_bool;
    pLCFSenOutput->pLcfSenOutputToVehData.sMCTLFCOut
        .CSCLTA_LimiterActivated_nu = sMCTLFCOutput.CSCLTA_LimiterActivated_nu;
}

/* ****************************************************************************
 Functionname:     LCF_SEN_ResetParams
 @brief: set parameters input data only while reset process triggered

 @description: set parameters input data only while reset process triggered

 @param[in]
                        const reqLcfSenParams* pLCFInputParam: system paramters
from RTE data
 @param[out]
                         sLBPParam_t* pLBPParam: algo parameters of LBP
sub-module
                         sALPParam_t* pALPParam: algo parameters of ALP
sub-module
                         ODPRParam_t* pODPRParam: algo parameters of ODPR
sub-module
                         sVSDPParam_t* pVSDPParam: algo parameters of VSDP
sub-module
                         sLDPSAParam_t* pLDPSAParam: algo parameters of LDPSA
sub-module

 @return           -
 @pre
 @post             -
 @author
**************************************************************************** */
static void LCF_SEN_ResetParams(const reqLcfSenParams* pLCFInputParam,
                                sLBPParam_t* pLBPParam,
                                sALPParam_st* pALPParam,
                                ODPRParam_t* pODPRParam,
                                sVSDPParam_t* pVSDPParam,
                                sLDWSAParam_t* pLDWSAParam,
                                sLDPSAParam_t* pLDPSAParam,
                                sTJASAParam_t* pTJASAParam,
                                sMCTLFCParam_st* pMCTLFCParam,
                                sLCCRAParam_t* pLCCRAParam,
                                sLCFOPSParam_t* pLCFOPSParam) {
    pLCCRAParam->LCFRCV_SysCycleTimeSen_sec =
        pLCFInputParam->LCFSen_Kf_SysCycleTime_sec;
    pLCFOPSParam->LCFRCV_SysCycleTimeSen_sec =
        pLCFInputParam->LCFSen_Kf_SysCycleTime_sec;
    pLBPParam->fSysCycleTime = pLCFInputParam->LCFSen_Kf_SysCycleTime_sec;

    pALPParam->bAllowBridging_bool =
        pLCFInputParam->LCFSen_Kb_bAllowBridging_bool;
    pALPParam->fAliveCounterTime_sec =
        pLCFInputParam->LCFSen_Kf_fAliveCounterTime_sec;
    pALPParam->fDebounceTLatPosStep_sec =
        pLCFInputParam->LCFSen_Kf_fDebounceTLatPosStep_sec;
    pALPParam->fEgoVehLength_met = pLCFInputParam->LCFSen_Kf_fEgoLength_met;
    pALPParam->fLatPosStepDtcLimit_met =
        pLCFInputParam->LCFSen_Kf_fLatPosStepDtcLimit_met;
    pALPParam->LatPosPT1TConst_sec =
        pLCFInputParam->LCFSen_Kf_LatPosPT1TConst_sec;
    pALPParam->uiMinAliveCounter_perc =
        pLCFInputParam->LCFSen_Ku_uiMinAliveCounter_perc;

    pODPRParam->fVehOverhangFront_met =
        pLCFInputParam->LCFSen_Kf_fVehOverhangFront_met;
    pODPRParam->fVehWheelBase_met = pLCFInputParam->LCFSen_Kf_fVehWheelBase_met;

    pVSDPParam->Sys_VehWid_Mi = pLCFInputParam->LCFSen_Kf_fEgoWidth_met;

    pLDWSAParam->LDWSAI_VehWid_Mi = pLCFInputParam->LCFSen_Kf_fEgoWidth_met;

    pLDPSAParam->LDPSAI_VehWid_Mi = pLCFInputParam->LCFSen_Kf_fEgoWidth_met;

    pTJASAParam->uTemp_nu = 0u; /* Not used, just for the unified interface */

    pMCTLFCParam->tmp = 0u; /* Not used, just for the unified interface */
}

/* ****************************************************************************
 Functionname:     LCFOPSInputWrapper
 @brief:		   the input wrapper function for the LCF Object pre
selection process

 @description:	   convert LCF Sen input to LCF Object pre selection module 's
input structure

 @param[in]		   const reqLcfSenPrtList_t LCFInput:  LCF Sen input
 @param[out]        sLCFOPSInReq_t* LCFOPSIn: LCF object pre-selection module
input
 @return           -
 @pre
 @post             -
 @author
**************************************************************************** */
static void LCFOPSInputWrapper(const reqLcfSenPrtList_t* LCFInput,
                               sLCFOPSInReq_t* LCFOPSIn) {
    LCFOPSIn->fEgoCurve_1pm = LCFInput->sLCFVehDyn.fEgoCurve_1pm;
    LCFOPSIn->inObjects = &LCFInput->sFusionObjectsArray;
}

/* ****************************************************************************
 Functionname:     LCCRAInputWrapper
 @brief:		   the input wrapper function for the LCF lane change
collision assist module

 @description:	   convert LCF Sen input and lane process module output to lane
change collission assist module 's input structure

 @param[in]		    const reqLcfSenPrtList_t LCFInput:  LCF Sen input
                    const sLCFOPSOutPro_t LCFOPSOutput: object list after object
preselection
                    const sLBPOutput_t LBPOutput: ego lane process output
                    const sALPOutput_st ALPOutput: adjacent lane output
 @param[out]        sLCCRAInReq_t* LCCRAIn:lane change collision assist module
input input
 @return           -
 @pre
 @post             -
 @author
**************************************************************************** */
static void LCCRAInputWrapper(const reqLcfSenPrtList_t* LCFInput,
                              sLCFOPSOutPro_t LCFOPSOutput,
                              sLBPOutput_t LBPOutput,
                              sALPOutput_st ALPOutput,
                              sLCCRAInReq_t* LCCRAIn) {
    LCCRAIn->Fusion_TargetObject_str = &LCFOPSOutput.outObjects;

    LCCRAIn->ABPR_LnWidth_met = LBPOutput.fLaneWidth;
    if (LCCRAIn->ABPR_LnWidth_met > 5.0f || LCCRAIn->ABPR_LnWidth_met < 2.2f) {
        LCCRAIn->ABPR_LnWidthValid_bool = FALSE;
    } else {
        LCCRAIn->ABPR_LnWidthValid_bool = TRUE;
    }
    // LCCRAIn->ABPR_LnWidthValid_bool = TRUE;
    LCCRAIn->ALP_LeLnClthCrv_1pm = ALPOutput.fLeftAdjLnCurvature_1pm;
    LCCRAIn->ALP_LeLnClthCrvChng_1pm2 = ALPOutput.fLeftAdjLnCrvRate_1pm2;
    LCCRAIn->ALP_LeLnClthHeading_rad = ALPOutput.fLeftAdjLnYawAngle_rad;
    LCCRAIn->ALP_LeLnClthPosY0_met = ALPOutput.fLeftAdjLnDistance_met;
    if (ALPOutput.uiLeftAdjLnInvalidQu_btf & 0x09 != 0) {
        LCCRAIn->ALP_LeLnValid_bool = FALSE;
    } else {
        LCCRAIn->ALP_LeLnValid_bool = TRUE;
    }

    LCCRAIn->ALP_RiLnClthCrv_1pm = ALPOutput.fRightAdjLnCurvature_1pm;
    LCCRAIn->ALP_RiLnClthCrvChng_1pm2 = ALPOutput.fRightAdjLnCrvRate_1pm2;
    LCCRAIn->ALP_RiLnClthHeading_rad = ALPOutput.fRightAdjLnYawAngle_rad;
    LCCRAIn->ALP_RiLnClthPosY0_met = ALPOutput.fRightAdjLnDistance_met;
    LCCRAIn->ALP_RiLnValid_bool = TRUE;
    if (ALPOutput.uiRightAdjLnInvalidQu_btf & 0x09 != 0) {
        LCCRAIn->ALP_RiLnValid_bool = FALSE;
    } else {
        LCCRAIn->ALP_RiLnValid_bool = TRUE;
    }

    LCCRAIn->LBP_LeLnClthCrv_1pm = LBPOutput.fCrvCtrlLf;
    LCCRAIn->LBP_LeLnClthCrvChng_1pm2 = LBPOutput.fCrvRateCtrlLf;
    LCCRAIn->LBP_LeLnClthHeading_rad = LBPOutput.fHeadingCtrlLf;
    LCCRAIn->LBP_LeLnClthPosY0_met = LBPOutput.fPosY0CtrlLf;
    LCCRAIn->LBP_LeLnValid_bool = TRUE;
    if (LBPOutput.uLaneInvalidQualifierLf & 0x9fff != 0) {
        LCCRAIn->LBP_LeLnValid_bool = FALSE;
    } else {
        LCCRAIn->LBP_LeLnValid_bool = TRUE;
    }

    LCCRAIn->LBP_RiLnClthCrv_1pm = LBPOutput.fCrvCtrlRi;
    LCCRAIn->LBP_RiLnClthCrvChng_1pm2 = LBPOutput.fCrvRateCtrlRi;
    LCCRAIn->LBP_RiLnClthHeading_rad = LBPOutput.fHeadingCtrlRi;
    LCCRAIn->LBP_RiLnClthPosY0_met = LBPOutput.fPosY0CtrlRi;
    LCCRAIn->LBP_RiLnValid_bool = TRUE;
    if (LBPOutput.uLaneInvalidQualifierRi & 0x9fff != 0) {
        LCCRAIn->LBP_RiLnValid_bool = FALSE;
    } else {
        LCCRAIn->LBP_RiLnValid_bool = TRUE;
    }

    LCCRAIn->VED_EgoClthCrv_1pm = LCFInput->sLCFVehDyn.fEgoCurve_1pm;
    LCCRAIn->VED_EgoClthCrvChng_1pm2 = 0.f;
    LCCRAIn->VED_EgoVelocity_mps = LCFInput->sLCFVehDyn.fEgoVelX_mps;
    LCCRAIn->VED_EgoYawRate_rps = LCFInput->sLCFVehDyn.fEgoYawRate_rps;
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
