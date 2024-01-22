/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
// Copyright [2021] <Copyright Owner>
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "lcf_veh_local_main.h"
#ifdef UBUNTU_SYSTEM
#include "Rte_CtApLCFVEH.h"
#endif
/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE2_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
// TP module global values
static TRJPLN_TrajectoryPlanInReq_t g_LCF_TP_Input = {0};
static TRJPLN_TrajectoryPlanParam_t g_LCF_TP_Param = {0};
static TRJPLN_TrajectoryPlanOutPro_t g_LCF_TP_Output = {0};

// TC module global values
static sTJATCTInReq_st g_LCF_TC_Input = {0};
static sTJATCTParam_st g_LCF_TC_Param = {0};
static sTJATCTOut_st g_LCF_TC_Output = {0};

// LCK module global values
static sLCKInput_t g_LCF_LCK_Input = {0};
static sLCKParam_t g_LCF_LCK_Param = {0};
static sLCKOutput_t g_LCF_LCK_Output = {0};

// LCD module global values
static sLCDInput_t g_LCF_LCD_Input = {0};
static sLCDParams_t g_LCF_LCD_Param = {0};
static sLCDOutput_t g_LCF_LCD_Output = {0};

// HOD module global values
static sHODInput_t g_LCF_HOD_Input = {0};
static sHODParam_t g_LCF_HOD_Param = {0};
static sHODOutput_t g_LCF_HOD_Output = {0};
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/* ****************************************************************************

  Functionname:     LcfVehExec

  @brief            LCF_Veh main function with input and output parameters

  @description      -

  @param[in]        reqPorts : pointer on the required ports structure
  @param[in,out]    proPorts : pointer on the provided ports structure

  @return           -
**************************************************************************** */
void LcfVehExec(const reqLcfVehPrtList_t* const reqPorts,
                const reqLcfVehParams* reqParams,
                proLcfVehPrtList_t* const proPorts,
                reqLcfVehDebug_t* proDebugs) {
    static uint16 uiCycleCnt = 0U;

    /* Cycle counter incremented every time Exec normal behaviour is executed
     * (no BASE_RETURN_ERROR) */
    uiCycleCnt++;

    /* Fill basic information in provided ports */
    LcfVehSetInfoDataProPorts(reqPorts, proPorts, uiCycleCnt);
    // printf("TP is In \n");
    // printf("TP : reqPorts->sBaseCtrlData.eOpMode = %d\n",
    // (uint8)reqPorts->sBaseCtrlData.eOpMode);

    /*! At this place it is necessary to test lcfOpMode directly to get an
     * initialized component */
    switch (reqPorts->sBaseCtrlData.eOpMode) {
        case (uint8)BASE_OM_RESET:
            LcfVehReset(reqParams);
            break;  // no calculation should be done in RESET
        case (uint8)BASE_OM_IDLE:
            // COMP_STATE_NOT_RUNNING shall be set if the algo is called in the
            // idle op mode;
            proPorts->pAlgoCompState.eCompState = COMP_STATE_NOT_RUNNING;
            proPorts->pAlgoCompState.eGenAlgoQualifier = ALGO_QUAL_OK;
            // Except for the comp state and the NVM structure the signal state
            // shall be 'AL_SIG_STATE_INVALID'.
            proPorts->pLcfVehOutputData.sSigHeader.eSigStatus =
                AL_SIG_STATE_INVALID;
            // proPorts->pLcfSenOutputToVehData.sSigHeader.eSigStatus =
            // AL_SIG_STATE_INVALID;
            break;  // no calculation shall be done in IDLE
        case (uint8)BASE_OM_DEMO:
            proPorts->pAlgoCompState.eCompState = COMP_STATE_SUCCESS;
            proPorts->pAlgoCompState.eGenAlgoQualifier = ALGO_QUAL_OK;
            // (username): Stuff." [readability/todo]fill demo values for
            // all outputs
            break;                // no calculation shall be done in DEMO
        case (uint8)BASE_OM_RUN:  // check for other op modes
            /*if (LCFSEN_reqPorts_NullStatusCheck ==
            ALL_REQUIRED_PORTS_NOT_NULL_MASK)
            {*/
            LcfVehProcess(reqPorts, reqParams, proPorts, proDebugs);
            // printf("TP is Running \n");
            //}
            // else {}
            break;
        default:
            break;
    }
#ifdef UBUNTU_SYSTEM
        // datalogger logic
        // DATALOGInfo_t Record1, Record2, Record3;
        // Record1.StructID = Data_sTPInput_t_type;
        // Record1.Length = sizeof(TRJPLN_TrajectoryPlanInReq_t);
        // Record1.SocBuf = (uint8*)&g_LCF_TP_Input;

        // Record2.StructID = Data_sTPParam_t_type;
        // Record2.Length = sizeof(TRJPLN_TrajectoryPlanParam_t);
        // Record2.SocBuf = (uint8*)&g_LCF_TP_Param;

        // Record3.StructID = Data_sTPOutput_t_type;
        // Record3.Length = sizeof(TRJPLN_TrajectoryPlanOutPro_t);
        // Record3.SocBuf = (uint8*)&g_LCF_TP_Output;

        // BSW_DataLog_FreezeData(Record1);
        // BSW_DataLog_FreezeData(Record2);
        // BSW_DataLog_FreezeData(Record3);

#endif
}

/* ****************************************************************************

  Functionname:     LcfVehReset

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
void LcfVehReset(const reqLcfVehParams* reqParams) {
    // TP module global values
    memset(&g_LCF_TP_Input, 0, sizeof(TRJPLN_TrajectoryPlanInReq_t));
    memset(&g_LCF_TP_Param, 0, sizeof(TRJPLN_TrajectoryPlanParam_t));
    memset(&g_LCF_TP_Output, 0, sizeof(TRJPLN_TrajectoryPlanOutPro_t));

    // TC module global values
    memset(&g_LCF_TC_Input, 0, sizeof(sTJATCTInReq_st));
    memset(&g_LCF_TC_Param, 0, sizeof(sTJATCTParam_st));
    memset(&g_LCF_TC_Output, 0, sizeof(sTJATCTOut_st));

    // LCK module global values
    memset(&g_LCF_LCK_Input, 0, sizeof(sLCKInput_t));
    memset(&g_LCF_LCK_Param, 0, sizeof(sLCKParam_t));
    memset(&g_LCF_LCK_Output, 0, sizeof(sLCKOutput_t));

    // LCD module global values
    memset(&g_LCF_LCD_Input, 0, sizeof(sLCDInput_t));
    memset(&g_LCF_LCD_Param, 0, sizeof(sLCDParams_t));
    memset(&g_LCF_LCD_Output, 0, sizeof(sLCDOutput_t));

    // set parameter values for every modules
    LCF_VEH_ResetParams(reqParams, &g_LCF_TP_Param, &g_LCF_TC_Param,
                        &g_LCF_LCK_Param, &g_LCF_LCD_Param);

    LCF_TrajectoryPlan_Reset();
    TJATCT_Init();
    LCKReset();
    LCDReset();
    LCF_HOD_Reset();
}

/* ****************************************************************************

 Functionname:     LcfVehSetInfoDataProPorts

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
static void LcfVehSetInfoDataProPorts(const reqLcfVehPrtList_t* const reqPorts,
                                      proLcfVehPrtList_t* const proPorts,
                                      uint16 uiCycleCnt) {
    uint32 uiCtrlTimeStamp = 0U;
    uint16 uiCtrlMeasCounter = 0U;

    /* Fill version numbers of provided ports */
    proPorts->pAlgoCompState.uiVersionNumber = 20201202u;
    proPorts->pLcfVehOutputData.uiVersionNumber = 20201202u;
    // proPorts->pLcfSenOutputToVehData.uiVersionNumber = 20201202u;

    /* Fill signal headers of provided ports */
    uiCtrlTimeStamp = reqPorts->sBaseCtrlData.sSigHeader.uiTimeStamp;
    uiCtrlMeasCounter = reqPorts->sBaseCtrlData.sSigHeader.uiMeasurementCounter;

    LcfVehSetSigHeader(&(proPorts->pAlgoCompState.sSigHeader), AL_SIG_STATE_OK,
                       uiCtrlTimeStamp, uiCtrlMeasCounter, uiCycleCnt);
    LcfVehSetSigHeader(&(proPorts->pLcfVehOutputData.sSigHeader),
                       AL_SIG_STATE_OK, uiCtrlTimeStamp, uiCtrlMeasCounter,
                       uiCycleCnt);
    // LcfVehSetSigHeader(&(proPorts->pLcfSenOutputToVehData.sSigHeader),
    // AL_SIG_STATE_OK, uiCtrlTimeStamp, uiCtrlMeasCounter, uiCycleCnt);

    /* Fill additional Algo comp state information */
    proPorts->pAlgoCompState.eShedulerSubModeRequest = BASE_SSM_NONE;
    proPorts->pAlgoCompState.uiAlgoVersionNumber = 20201202u;
}

/* ****************************************************************************

 Functionname:     LcfVehSetSigHeader

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
void LcfVehSetSigHeader(LCF_VehSignalHeader_t* const psSigHeader,
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

 Functionname:     LcfVehProcess

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
static void LcfVehProcess(const reqLcfVehPrtList_t* const reqPorts,
                          const reqLcfVehParams* reqParams,
                          proLcfVehPrtList_t* const proPorts,
                          reqLcfVehDebug_t* proDebugs) {
    TPInputWrapper(reqPorts, reqParams, &g_LCF_TP_Input);

    // printf("--------------------TP Input--------------------\n");
    // printf("TP : g_LCF_TP_Input.bLatencyCompActivated = %d\n",
    // g_LCF_TP_Input.bLatencyCompActivated);
    // printf("TP : g_LCF_TP_Input.bSysStOffLatDMC = %d\n",
    // g_LCF_TP_Input.bSysStOffLatDMC);
    // printf("TP : g_LCF_TP_Input.bTriggerReplan = %d\n",
    //       g_LCF_TP_Input.bTriggerReplan);
    // printf("TP : g_LCF_TP_Input.fCycleTimeVeh_sec = %f\n",
    //       g_LCF_TP_Input.fCycleTimeVeh_sec);
    // printf("TP : g_LCF_TP_Input.fDistYToLeTgtArea_met = %f\n",
    // g_LCF_TP_Input.fDistYToLeTgtArea_met);
    // printf("TP : g_LCF_TP_Input.fDistYToRiTgtArea_met = %f\n",
    // g_LCF_TP_Input.fDistYToRiTgtArea_met);
    // printf("TP : g_LCF_TP_Input.fEgoAccelX_mps2 = %f\n",
    // g_LCF_TP_Input.fEgoAccelX_mps2);
    // printf("TP : g_LCF_TP_Input.fEgoCurve_1pm = %f\n",
    // g_LCF_TP_Input.fEgoCurve_1pm);
    // printf("TP : g_LCF_TP_Input.fEgoVelX_mps = %f\n",
    //       g_LCF_TP_Input.fEgoVelX_mps);
    // printf("TP : g_LCF_TP_Input.fEgoYawRate_rps = %f\n",
    //       g_LCF_TP_Input.fEgoYawRate_rps);
    // printf("TP : g_LCF_TP_Input.fEPSManualTrqActVal_Nm = %f\n",
    // g_LCF_TP_Input.fEPSManualTrqActVal_Nm);
    // printf("TP : g_LCF_TP_Input.fFTireAclMax_mps2 = %f\n",
    // g_LCF_TP_Input.fFTireAclMax_mps2);
    // printf("TP : g_LCF_TP_Input.fFTireAclMin_mps2 = %f\n",
    // g_LCF_TP_Input.fFTireAclMin_mps2);
    // printf("TP : g_LCF_TP_Input.fKappaSumCommand_1pm = %f\n",
    // g_LCF_TP_Input.fKappaSumCommand_1pm);
    // printf("TP : g_LCF_TP_Input.fLeCridBndCrv_1pm = %f\n",
    // g_LCF_TP_Input.fLeCridBndCrv_1pm);
    // printf("TP : g_LCF_TP_Input.fLeCridBndCrvChng_1pm2 = %f\n",
    // g_LCF_TP_Input.fLeCridBndCrvChng_1pm2);
    // printf("TP : g_LCF_TP_Input.fLeCridBndHeadAng_rad = %f\n",
    // g_LCF_TP_Input.fLeCridBndHeadAng_rad);
    // printf("TP : g_LCF_TP_Input.fLeCridBndLength_met = %f\n",
    // g_LCF_TP_Input.fLeCridBndLength_met);
    // printf("TP : g_LCF_TP_Input.fLeCridBndPosX0_met = %f\n",
    // g_LCF_TP_Input.fLeCridBndPosX0_met);
    // printf("TP : g_LCF_TP_Input.fLeCridBndPosY0_met = %f\n",
    // g_LCF_TP_Input.fLeCridBndPosY0_met);
    // printf("TP : g_LCF_TP_Input.fMaxJerkAllowed_mps3 = %f\n",
    // g_LCF_TP_Input.fMaxJerkAllowed_mps3);
    // printf("TP : g_LCF_TP_Input.fPlanningHorizon_sec = %f\n",
    // g_LCF_TP_Input.fPlanningHorizon_sec);
    // printf("TP : g_LCF_TP_Input.fPredTimeCurve_sec = %f\n",
    // g_LCF_TP_Input.fPredTimeCurve_sec);
    // printf("TP : g_LCF_TP_Input.fPredTimeHeadAng_sec = %f\n",
    // g_LCF_TP_Input.fPredTimeHeadAng_sec);
    // printf("TP : g_LCF_TP_Input.fRiCridBndCrv_1pm = %f\n",
    // g_LCF_TP_Input.fRiCridBndCrv_1pm);
    // printf("TP : g_LCF_TP_Input.fRiCridBndCrvChng_1pm2 = %f\n",
    // g_LCF_TP_Input.fRiCridBndCrvChng_1pm2);
    // printf("TP : g_LCF_TP_Input.fRiCridBndHeadAng_rad = %f\n",
    // g_LCF_TP_Input.fRiCridBndHeadAng_rad);
    // printf("TP : g_LCF_TP_Input.fRiCridBndLength_met = %f\n",
    // g_LCF_TP_Input.fRiCridBndLength_met);
    // printf("TP : g_LCF_TP_Input.fRiCridBndPosX0_met = %f\n",
    // g_LCF_TP_Input.fRiCridBndPosX0_met);
    // printf("TP : g_LCF_TP_Input.fRiCridBndPosY0_met = %f\n",
    // g_LCF_TP_Input.fRiCridBndPosY0_met);
    // printf("TP : g_LCF_TP_Input.fSensorTimeStamp_sec = %f\n",
    // g_LCF_TP_Input.fSensorTimeStamp_sec);
    // printf("TP : g_LCF_TP_Input.fTgtTrajCrvChng_1pm2 = %f\n",
    // g_LCF_TP_Input.fTgtTrajCrvChng_1pm2);
    // printf("TP : g_LCF_TP_Input.fTgtTrajCurve_1pm = %f\n",
    // g_LCF_TP_Input.fTgtTrajCurve_1pm);
    // printf("TP : g_LCF_TP_Input.fTgtTrajHeadingAng_rad = %f\n",
    // g_LCF_TP_Input.fTgtTrajHeadingAng_rad);
    // printf("TP : g_LCF_TP_Input.fTgtTrajLength_met = %f\n",
    // g_LCF_TP_Input.fTgtTrajLength_met);
    // printf("TP : g_LCF_TP_Input.fTgtTrajPosX0_met = %f\n",
    // g_LCF_TP_Input.fTgtTrajPosX0_met);
    // printf("TP : g_LCF_TP_Input.fTgtTrajPosY0_met = %f\n",
    // g_LCF_TP_Input.fTgtTrajPosY0_met);
    // printf("TP : g_LCF_TP_Input.fWeightEndTime_nu = %f\n",
    // g_LCF_TP_Input.fWeightEndTime_nu);
    // printf("TP : g_LCF_TP_Input.fWeightTgtDistY_nu = %f\n",
    // g_LCF_TP_Input.fWeightTgtDistY_nu);
    // printf("TP : g_LCF_TP_Input.uiSenToVehTStamp_us =
    // %f\n",g_LCF_TP_Input.uiSenToVehTStamp_us * 1e-6f);
    // printf("TP : g_LCF_TP_Input.uiControllingFunction_nu = %d\n",
    //       g_LCF_TP_Input.uiControllingFunction_nu);
    // printf("TP : g_LCF_TP_Input.uiLatCtrlMode_nu = %d\n",
    //       g_LCF_TP_Input.uiLatCtrlMode_nu);
    // printf("TP : g_LCF_TP_Input.uiLeCrvQuality_per = %d\n",
    // g_LCF_TP_Input.uiLeCrvQuality_per);
    // printf("TP : g_LCF_TP_Input.uiLeLnQuality_per = %d\n",
    // g_LCF_TP_Input.uiLeLnQuality_per);
    // printf("TP : g_LCF_TP_Input.uiOdometerState_nu = %d\n",
    // g_LCF_TP_Input.uiOdometerState_nu);
    // printf("TP : g_LCF_TP_Input.uiRiCrvQuality_per = %d\n",
    // g_LCF_TP_Input.uiRiCrvQuality_per);
    // printf("TP : g_LCF_TP_Input.uiRiLnQuality_per = %d\n",
    // g_LCF_TP_Input.uiRiLnQuality_per);
    // printf("TP : g_LCF_TP_Input.uiSysStateLCF_nu = %d\n",
    //       g_LCF_TP_Input.uiSysStateLCF_nu);
    // printf("TP : g_LCF_TP_Input.uiTrajGuiQualifier_nu = %d\n",
    //       g_LCF_TP_Input.uiTrajGuiQualifier_nu);
    // printf("TP : g_LCF_TP_Input.uiTrajPlanServQu_nu = %d\n",
    // g_LCF_TP_Input.uiTrajPlanServQu_nu);
    // printf("TP : g_LCF_TP_Input.uiVehSync4LCO_us = %d\n",
    // g_LCF_TP_Input.uiVehSync4LCO_us);

    LCF_TrajectoryPlan_Exec(&g_LCF_TP_Input, &g_LCF_TP_Param, &g_LCF_TP_Output,
                            &(proDebugs->sTPDebug));
    TPOutputWrapper(g_LCF_TP_Input, g_LCF_TP_Output, proDebugs->sTPDebug,
                    proPorts);

    // printf("--------------------TP Output--------------------\n");

    // printf("TP : g_LCF_TP_Output.uiTrajGuiQualifier_nu = %d\n",
    //       g_LCF_TP_Output.uiTrajGuiQualifier_nu);
    // printf("TP : g_LCF_TP_Output.fTrajDistY_met = %f\n",
    //       g_LCF_TP_Output.fTrajDistY_met);
    // printf("TP : g_LCF_TP_Output.fCurDistY_met = %f\n",
    //       g_LCF_TP_Output.fCurDistY_met);
    // printf("TP : g_LCF_TP_Output.fCtrlErrDistY_met =
    // %f\n",g_LCF_TP_Output.fCtrlErrDistY_met);
    // printf("TP : g_LCF_TP_Output.fTrajHeading_rad = %f\n",
    //       g_LCF_TP_Output.fTrajHeading_rad);
    // printf("TP : g_LCF_TP_Output.fCurHeading_rad = %f\n",
    //       g_LCF_TP_Output.fCurHeading_rad);
    // printf("TP : g_LCF_TP_Output.fCtrlErrHeadingAngle_rad =
    // %f\n",g_LCF_TP_Output.fCtrlErrHeadingAngle_rad);
    // printf("TP : g_LCF_TP_Output.bTrigTrajReplan = %d\n",
    //       proDebugs->sTPDebug.pCalcEnableDebug.bTrigTrajReplan);
    // printf("TP : g_LCF_TP_Output.bReplanCurValues = %d\n",
    //       proDebugs->sTPDebug.pCalcEnableDebug.bReplanCurValues);
    // printf("TP : g_LCF_TP_Output.bReplanTgtValues = %d\n",
    //       proDebugs->sTPDebug.pCalcEnableDebug.bReplanTgtValues);

    TCInputWrapper(reqPorts, reqParams, &g_LCF_TP_Output, &g_LCF_TC_Input);
    TJATCT_Exec(&g_LCF_TC_Input, &g_LCF_TC_Param, &g_LCF_TC_Output,
                &(proDebugs->sTCDebug));
    TCOutputWrapper(proDebugs->sTCDebug, g_LCF_TC_Output, proPorts);
    TCInputOutWrapper(g_LCF_TC_Input, proPorts);

    LCKInputWrapper(reqPorts, reqParams, &g_LCF_LCK_Input);
    LCKProcess(&g_LCF_LCK_Input, &g_LCF_LCK_Param, &g_LCF_LCK_Output,
               &(proDebugs->sLCKDebug));

    LCDInputWrapper(reqPorts, reqParams, &g_LCF_LCK_Input, &g_LCF_LCK_Output,
                    &g_LCF_LCD_Input);
    LCDProcess(&g_LCF_LCD_Input, &g_LCF_LCD_Param, &g_LCF_LCD_Output,
               &(proDebugs->sLCDDebug));
    LCDOutputWrapper(g_LCF_LCD_Output, proPorts);

    HODInputWrapper(reqPorts, reqParams, &g_LCF_HOD_Input);
    LCF_HOD_Exec(&g_LCF_HOD_Input, &g_LCF_HOD_Param, &g_LCF_HOD_Output,
                 &(proDebugs->sHODDebug));
    HODOutputWrapper(proDebugs->sHODDebug, g_LCF_HOD_Output, proPorts);
}

/* ****************************************************************************
 Functionname:     TPInputWrapper
 @brief: the input wrapper function for the TP process

 @description: the input wrapper function for the TP process

 @param[in]
                        reqLcfVehPrtList_t* pLCFVehInput:the input of the LCF
whole module
 @param[in]
                        reqLcfVehParams* pLcfVehParams:the parameter of the LCF
whole module
 @param[out]
                        TRJPLN_TrajectoryPlanInReq_t* pTPInput: the input of the
TP sub-module

 @return           -
 @pre
 @post             -
 @author
**************************************************************************** */
static void TPInputWrapper(const reqLcfVehPrtList_t* pLCFVehInput,
                           const reqLcfVehParams* pLcfVehParams,
                           TRJPLN_TrajectoryPlanInReq_t* pTPInput) {
    pTPInput->fCycleTimeVeh_sec = pLcfVehParams->LCFVeh_Kf_fSysCycleTime_sec;

    pTPInput->fEgoVelX_mps = pLCFVehInput->sLCFVehDyn.fEgoVelX_mps;
    pTPInput->fEgoAccelX_mps2 = pLCFVehInput->sLCFVehDyn.fEgoAccelX_mps2;
    pTPInput->fEgoCurve_1pm = pLCFVehInput->sLCFVehDyn.fEgoCurve_1pm;
    pTPInput->fEgoYawRate_rps = pLCFVehInput->sLCFVehDyn.fEgoYawRate_rps;

    pTPInput->fKappaSumCommand_1pm =
        pLCFVehInput->sTPRCVInReq.fKappaSumCommand_1pm;
    pTPInput->fEPSManualTrqActVal_Nm =
        pLCFVehInput->sTPRCVInReq.fEPSManualTrqActVal_Nm;
    pTPInput->uiOdometerState_nu = pLCFVehInput->sTPRCVInReq.uiOdometerState_nu;
    pTPInput->uiVehSync4LCO_us = pLCFVehInput->sTPRCVInReq.uiVehSync4LCO_us;
    pTPInput->uiSenToVehTStamp_us =
        pLCFVehInput->sTPRCVInReq.uiSenToVehTStamp_us;

    pTPInput->bSysStOffLatDMC =
        pLCFVehInput->sLcfVehInputFromDMCData.bSysStOffLatDMC;
    pTPInput->bSysStReqLatDMC =
        pLCFVehInput->sLcfVehInputFromDMCData.bSysStReqLatDMC;

    pTPInput->uiLeLnQuality_per =
        pLCFVehInput->sLcfSVehInputFromSenData.sLBPData_t.uiLeLnQuality_per;
    pTPInput->uiRiLnQuality_per =
        pLCFVehInput->sLcfSVehInputFromSenData.sLBPData_t.uiRiLnQuality_per;
    pTPInput->uiLeCrvQuality_per =
        pLCFVehInput->sLcfSVehInputFromSenData.sLBPData_t.uiLeCrvQuality_per;
    pTPInput->uiRiCrvQuality_per =
        pLCFVehInput->sLcfSVehInputFromSenData.sLBPData_t.uiRiCrvQuality_per;

    pTPInput->uiLatCtrlMode_nu =
        pLCFVehInput->sLcfSVehInputFromSenData.sTJASAState.uiLatCtrlMode_nu;

    pTPInput->uiControllingFunction_nu =
        pLCFVehInput->sLcfSVehInputFromSenData.sCSCLTAData
            .uiControllingFunction_nu;
    pTPInput->fPlanningHorizon_sec =
        pLCFVehInput->sLcfSVehInputFromSenData.sCSCLTAData.fPlanningHorizon_sec;
    pTPInput->uiSysStateLCF_nu =
        pLCFVehInput->sLcfSVehInputFromSenData.sCSCLTAData.uiSysStateLCF_nu;
    pTPInput->fPredTimeCurve_sec =
        pLCFVehInput->sLcfSVehInputFromSenData.sCSCLTAData.fPredTimeCurve_sec;
    pTPInput->fPredTimeHeadAng_sec =
        pLCFVehInput->sLcfSVehInputFromSenData.sCSCLTAData.fPredTimeHeadAng_sec;
    pTPInput->bTriggerReplan =
        pLCFVehInput->sLcfSVehInputFromSenData.sCSCLTAData.bTriggerReplan;
    pTPInput->fLeCridBndPosX0_met =
        pLCFVehInput->sLcfSVehInputFromSenData.sCSCLTAData.fLeCridBndPosX0_met;
    pTPInput->fLeCridBndPosY0_met =
        pLCFVehInput->sLcfSVehInputFromSenData.sCSCLTAData.fLeCridBndPosY0_met;
    pTPInput->fLeCridBndHeadAng_rad = pLCFVehInput->sLcfSVehInputFromSenData
                                          .sCSCLTAData.fLeCridBndHeadAng_rad;
    pTPInput->fLeCridBndCrv_1pm =
        pLCFVehInput->sLcfSVehInputFromSenData.sCSCLTAData.fLeCridBndCrv_1pm;
    pTPInput->fLeCridBndCrvChng_1pm2 = pLCFVehInput->sLcfSVehInputFromSenData
                                           .sCSCLTAData.fLeCridBndCrvChng_1pm2;
    pTPInput->fLeCridBndLength_met =
        pLCFVehInput->sLcfSVehInputFromSenData.sCSCLTAData.fLeCridBndLength_met;
    pTPInput->fRiCridBndPosX0_met =
        pLCFVehInput->sLcfSVehInputFromSenData.sCSCLTAData.fRiCridBndPosX0_met;
    pTPInput->fRiCridBndPosY0_met =
        pLCFVehInput->sLcfSVehInputFromSenData.sCSCLTAData.fRiCridBndPosY0_met;
    pTPInput->fRiCridBndHeadAng_rad = pLCFVehInput->sLcfSVehInputFromSenData
                                          .sCSCLTAData.fRiCridBndHeadAng_rad;
    pTPInput->fRiCridBndCrv_1pm =
        pLCFVehInput->sLcfSVehInputFromSenData.sCSCLTAData.fRiCridBndCrv_1pm;
    pTPInput->fRiCridBndCrvChng_1pm2 = pLCFVehInput->sLcfSVehInputFromSenData
                                           .sCSCLTAData.fRiCridBndCrvChng_1pm2;
    pTPInput->fRiCridBndLength_met =
        pLCFVehInput->sLcfSVehInputFromSenData.sCSCLTAData.fRiCridBndLength_met;
    pTPInput->fTgtTrajPosX0_met =
        pLCFVehInput->sLcfSVehInputFromSenData.sCSCLTAData.fTgtTrajPosX0_met;
    pTPInput->fTgtTrajPosY0_met =
        pLCFVehInput->sLcfSVehInputFromSenData.sCSCLTAData.fTgtTrajPosY0_met;
    pTPInput->fTgtTrajHeadingAng_rad = pLCFVehInput->sLcfSVehInputFromSenData
                                           .sCSCLTAData.fTgtTrajHeadingAng_rad;
    pTPInput->fTgtTrajCurve_1pm =
        pLCFVehInput->sLcfSVehInputFromSenData.sCSCLTAData.fTgtTrajCurve_1pm;
    pTPInput->fTgtTrajCrvChng_1pm2 =
        pLCFVehInput->sLcfSVehInputFromSenData.sCSCLTAData.fTgtTrajCrvChng_1pm2;
    pTPInput->fTgtTrajLength_met =
        pLCFVehInput->sLcfSVehInputFromSenData.sCSCLTAData.fTgtTrajLength_met;
    pTPInput->bLatencyCompActivated = pLCFVehInput->sLcfSVehInputFromSenData
                                          .sCSCLTAData.bLatencyCompActivated;
    pTPInput->fSensorTimeStamp_sec =
        pLCFVehInput->sLcfSVehInputFromSenData.sCSCLTAData.fSensorTimeStamp_sec;
    pTPInput->uiTrajPlanServQu_nu =
        pLCFVehInput->sLcfSVehInputFromSenData.sCSCLTAData.uiTrajPlanServQu_nu;
    pTPInput->fWeightTgtDistY_nu =
        pLCFVehInput->sLcfSVehInputFromSenData.sCSCLTAData.fWeightTgtDistY_nu;
    pTPInput->fWeightEndTime_nu =
        pLCFVehInput->sLcfSVehInputFromSenData.sCSCLTAData.fWeightEndTime_nu;
    pTPInput->fDistYToLeTgtArea_met = pLCFVehInput->sLcfSVehInputFromSenData
                                          .sCSCLTAData.fDistYToLeTgtArea_met;
    pTPInput->fDistYToRiTgtArea_met = pLCFVehInput->sLcfSVehInputFromSenData
                                          .sCSCLTAData.fDistYToRiTgtArea_met;
    pTPInput->fFTireAclMax_mps2 =
        pLCFVehInput->sLcfSVehInputFromSenData.sCSCLTAData.fFTireAclMax_mps2;
    pTPInput->fFTireAclMin_mps2 =
        pLCFVehInput->sLcfSVehInputFromSenData.sCSCLTAData.fFTireAclMin_mps2;
    pTPInput->fObstacleVelX_mps =
        pLCFVehInput->sLcfSVehInputFromSenData.sCSCLTAData.fObstacleVelX_mps;
    pTPInput->fObstacleAccelX_mps2 =
        pLCFVehInput->sLcfSVehInputFromSenData.sCSCLTAData.fObstacleAccelX_mps2;
    pTPInput->fObstacleWidth_met =
        pLCFVehInput->sLcfSVehInputFromSenData.sCSCLTAData.fObstacleWidth_met;
    pTPInput->fObstacleDistX_met =
        pLCFVehInput->sLcfSVehInputFromSenData.sCSCLTAData.fObstacleDistX_met;
    pTPInput->fObstacleDistY_met =
        pLCFVehInput->sLcfSVehInputFromSenData.sCSCLTAData.fObstacleDistY_met;
    pTPInput->fMaxJerkAllowed_mps3 =
        pLCFVehInput->sLcfSVehInputFromSenData.sCSCLTAData.fMaxJerkAllowed_mps3;
    pTPInput->uiTrajGuiQualifier_nu = pLCFVehInput->sLcfSVehInputFromSenData
                                          .sCSCLTAData.uiTrajGuiQualifier_nu;

    // printf("------------- Inpur Rapper -----------");
    // printf("TP : pTPInput->uiControllingFunction_nu = %d\n",
    // pTPInput->uiControllingFunction_nu);
    // printf("TP : pTPInput->uiSysStateLCF_nu = %d\n",
    // pTPInput->uiSysStateLCF_nu);
    // printf("TP : pTPInput->uiLeLnQuality_per = %d\n",
    // pTPInput->uiLeLnQuality_per);
    // printf("TP : pTPInput->uiControllingFunction_nu = %d\n",
    // pTPInput->uiControllingFunction_nu);
}
/* ****************************************************************************
 Functionname:     TCOutputWrapper
 @brief:		   the output wrapper function for the TC process

 @description:	   convert TC sub-module's result to LCF Veh output structure

 @param[in]		   const sMCTLFCOut_st sMCTLFCOutput: MCTLFC
sub-module's output
 @param[out]       proLcfSenPrtList_t* LCFSenOutput: LCF Sen output pointer
 @return           -
 @pre
 @post             -
 @author
**************************************************************************** */
static void TPOutputWrapper(const TRJPLN_TrajectoryPlanInReq_t sTPInput,
                            const TRJPLN_TrajectoryPlanOutPro_t sTPOutput,
                            const TRJPLN_TrajectoryPlanDebug_t sTPDebug,
                            proLcfVehPrtList_t* pLCFVehOutput) {
    pLCFVehOutput->pLcfVehOutputData.sTPOutput.bGradLimitActive =
        sTPOutput.bGradLimitActive;
    pLCFVehOutput->pLcfVehOutputData.sTPOutput.bPlausiCheckStatus =
        sTPOutput.bPlausiCheckStatus;
    pLCFVehOutput->pLcfVehOutputData.sTPOutput.bTargetSwitch =
        sTPOutput.bTargetSwitch;
    pLCFVehOutput->pLcfVehOutputData.sTPOutput.bReplanCurValues =
        sTPOutput.bReplanCurValues;
    pLCFVehOutput->pLcfVehOutputData.sTPOutput.bUseTargetCorridor =
        sTPOutput.bUseTargetCorridor;
    pLCFVehOutput->pLcfVehOutputData.sTPOutput.fCtrlErrDistY_met =
        sTPOutput.fCtrlErrDistY_met;
    pLCFVehOutput->pLcfVehOutputData.sTPOutput.fCtrlErrHeadAglPrev_rad =
        sTPOutput.fCtrlErrHeadAglPrev_rad;
    pLCFVehOutput->pLcfVehOutputData.sTPOutput.fCtrlErrHeadingAngle_rad =
        sTPOutput.fCtrlErrHeadingAngle_rad;
    pLCFVehOutput->pLcfVehOutputData.sTPOutput.fCurDistY_met =
        sTPOutput.fCurDistY_met;
    pLCFVehOutput->pLcfVehOutputData.sTPOutput.fCurHeading_rad =
        sTPOutput.fCurHeading_rad;
    pLCFVehOutput->pLcfVehOutputData.sTPOutput.fDeltaTargetCrv_1pm =
        sTPOutput.fDeltaTargetCrv_1pm;
    pLCFVehOutput->pLcfVehOutputData.sTPOutput.fDeltaTargetHeading_rad =
        sTPOutput.fDeltaTargetHeading_rad;
    pLCFVehOutput->pLcfVehOutputData.sTPOutput.fDeltaTargetPosY0_met =
        sTPOutput.fDeltaTargetPosY0_met;
    pLCFVehOutput->pLcfVehOutputData.sTPOutput.fTrajDistY_met =
        sTPOutput.fTrajDistY_met;
    pLCFVehOutput->pLcfVehOutputData.sTPOutput.fTrajDistYPrev_met =
        sTPOutput.fTrajDistYPrev_met;
    pLCFVehOutput->pLcfVehOutputData.sTPOutput.fTrajHeadInclPrev_rad =
        sTPOutput.fTrajHeadInclPrev_rad;
    pLCFVehOutput->pLcfVehOutputData.sTPOutput.fTrajHeading_rad =
        sTPOutput.fTrajHeading_rad;
    pLCFVehOutput->pLcfVehOutputData.sTPOutput.fTrajHeadingPrev_rad =
        sTPOutput.fTrajHeadingPrev_rad;
    pLCFVehOutput->pLcfVehOutputData.sTPOutput.fTrajTgtCrvGrd_1pms =
        sTPOutput.fTrajTgtCrvGrd_1pms;
    pLCFVehOutput->pLcfVehOutputData.sTPOutput.fTrajTgtCrvGrdPrev_1pms =
        sTPOutput.fTrajTgtCrvGrdPrev_1pms;
    pLCFVehOutput->pLcfVehOutputData.sTPOutput.fTrajTgtCrvPrev_1pm =
        sTPOutput.fTrajTgtCrvPrev_1pm;
    pLCFVehOutput->pLcfVehOutputData.sTPOutput.fTrajTgtCurve_1pm =
        sTPOutput.fTrajTgtCurve_1pm;
    pLCFVehOutput->pLcfVehOutputData.sTPOutput.uiD_QuStatusTrajPlan_nu =
        sTPOutput.uiD_QuStatusTrajPlan_nu;
    pLCFVehOutput->pLcfVehOutputData.sTPOutput.uiS_QuStatusTrajPlan_nu =
        sTPOutput.uiS_QuStatusTrajPlan_nu;
    pLCFVehOutput->pLcfVehOutputData.sTPOutput.uiTrajGuiQualifier_nu =
        sTPOutput.uiTrajGuiQualifier_nu;

    pLCFVehOutput->pLcfVehOutputData.sTPInput.fEgoVelX_mps =
        sTPInput.fEgoVelX_mps;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fEgoAccelX_mps2 =
        sTPInput.fEgoAccelX_mps2;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fEgoCurve_1pm =
        sTPInput.fEgoCurve_1pm;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fKappaSumCommand_1pm =
        sTPInput.fKappaSumCommand_1pm;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fCycleTimeVeh_sec =
        sTPInput.fCycleTimeVeh_sec;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fEPSManualTrqActVal_Nm =
        sTPInput.fEPSManualTrqActVal_Nm;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.bSysStOffLatDMC =
        sTPInput.bSysStOffLatDMC;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.bSysStReqLatDMC =
        sTPInput.bSysStReqLatDMC;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.uiOdometerState_nu =
        sTPInput.uiOdometerState_nu;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fEgoYawRate_rps =
        sTPInput.fEgoYawRate_rps;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.uiVehSync4LCO_us =
        sTPInput.uiVehSync4LCO_us;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.uiSenToVehTStamp_us =
        sTPInput.uiSenToVehTStamp_us;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fPlanningHorizon_sec =
        sTPInput.fPlanningHorizon_sec;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.uiSysStateLCF_nu =
        sTPInput.uiSysStateLCF_nu;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fPredTimeCurve_sec =
        sTPInput.fPredTimeCurve_sec;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fPredTimeHeadAng_sec =
        sTPInput.fPredTimeHeadAng_sec;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.bTriggerReplan =
        sTPInput.bTriggerReplan;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fLeCridBndPosX0_met =
        sTPInput.fLeCridBndPosX0_met;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fLeCridBndPosY0_met =
        sTPInput.fLeCridBndPosY0_met;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fLeCridBndHeadAng_rad =
        sTPInput.fLeCridBndHeadAng_rad;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fLeCridBndCrv_1pm =
        sTPInput.fLeCridBndCrv_1pm;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fLeCridBndCrvChng_1pm2 =
        sTPInput.fLeCridBndCrvChng_1pm2;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fLeCridBndLength_met =
        sTPInput.fLeCridBndLength_met;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fRiCridBndPosX0_met =
        sTPInput.fRiCridBndPosX0_met;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fRiCridBndPosY0_met =
        sTPInput.fRiCridBndPosY0_met;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fRiCridBndHeadAng_rad =
        sTPInput.fRiCridBndHeadAng_rad;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fRiCridBndCrv_1pm =
        sTPInput.fRiCridBndCrv_1pm;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fRiCridBndCrvChng_1pm2 =
        sTPInput.fRiCridBndCrvChng_1pm2;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fRiCridBndLength_met =
        sTPInput.fRiCridBndLength_met;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fTgtTrajPosX0_met =
        sTPInput.fTgtTrajPosX0_met;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fTgtTrajPosY0_met =
        sTPInput.fTgtTrajPosY0_met;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fTgtTrajHeadingAng_rad =
        sTPInput.fTgtTrajHeadingAng_rad;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fTgtTrajCurve_1pm =
        sTPInput.fTgtTrajCurve_1pm;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fTgtTrajCrvChng_1pm2 =
        sTPInput.fTgtTrajCrvChng_1pm2;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fTgtTrajLength_met =
        sTPInput.fTgtTrajLength_met;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.bLatencyCompActivated =
        sTPInput.bLatencyCompActivated;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fSensorTimeStamp_sec =
        sTPInput.fSensorTimeStamp_sec;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.uiTrajPlanServQu_nu =
        sTPInput.uiTrajPlanServQu_nu;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fWeightTgtDistY_nu =
        sTPInput.fWeightTgtDistY_nu;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fWeightEndTime_nu =
        sTPInput.fWeightEndTime_nu;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fDistYToLeTgtArea_met =
        sTPInput.fDistYToLeTgtArea_met;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fDistYToRiTgtArea_met =
        sTPInput.fDistYToRiTgtArea_met;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fFTireAclMax_mps2 =
        sTPInput.fFTireAclMax_mps2;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fFTireAclMin_mps2 =
        sTPInput.fFTireAclMin_mps2;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fObstacleVelX_mps =
        sTPInput.fObstacleVelX_mps;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fObstacleAccelX_mps2 =
        sTPInput.fObstacleAccelX_mps2;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fObstacleWidth_met =
        sTPInput.fObstacleWidth_met;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fObstacleDistX_met =
        sTPInput.fObstacleDistX_met;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fObstacleDistY_met =
        sTPInput.fObstacleDistY_met;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.fMaxJerkAllowed_mps3 =
        sTPInput.fMaxJerkAllowed_mps3;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.uiTrajGuiQualifier_nu =
        sTPInput.uiTrajGuiQualifier_nu;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.uiLeLnQuality_per =
        sTPInput.uiLeLnQuality_per;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.uiRiLnQuality_per =
        sTPInput.uiRiLnQuality_per;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.uiLeCrvQuality_per =
        sTPInput.uiLeCrvQuality_per;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.uiRiCrvQuality_per =
        sTPInput.uiRiCrvQuality_per;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.uiLatCtrlMode_nu =
        sTPInput.uiLatCtrlMode_nu;
    pLCFVehOutput->pLcfVehOutputData.sTPInput.uiControllingFunction_nu =
        sTPInput.uiControllingFunction_nu;

    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pCalcEnableDebug.bTrajPlanEnble =
        sTPDebug.pCalcEnableDebug.bTrajPlanEnble;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pCalcEnableDebug.bTrigTrajReplan =
        sTPDebug.pCalcEnableDebug.bTrigTrajReplan;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pCalcEnableDebug
        .bReplanModeArcLength = sTPDebug.pCalcEnableDebug.bReplanModeArcLength;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pCalcEnableDebug
        .bReplanCurValues = sTPDebug.pCalcEnableDebug.bReplanCurValues;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pCalcEnableDebug
        .bReplanTgtValues = sTPDebug.pCalcEnableDebug.bReplanTgtValues;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pCalcEnableDebug
        .bTrigCustFctChange = sTPDebug.pCalcEnableDebug.bTrigCustFctChange;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pCalcEnableDebug
        .bTrajGuiQuChange = sTPDebug.pCalcEnableDebug.bTrajGuiQuChange;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pCalcEnableDebug
        .bTrigCustFctActn = sTPDebug.pCalcEnableDebug.bTrigCustFctActn;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pCalcEnableDebug
        .bTrigReplanTgtTraj = sTPDebug.pCalcEnableDebug.bTrigReplanTgtTraj;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pCalcEnableDebug
        .bEnblSpecPlanStrategy =
        sTPDebug.pCalcEnableDebug.bEnblSpecPlanStrategy;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pCalcEnableDebug
        .fDelayVehGui_sec = sTPDebug.pCalcEnableDebug.fDelayVehGui_sec;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pCalcEnableDebug
        .bTrigLargeDeviation = sTPDebug.pCalcEnableDebug.bTrigLargeDeviation;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pCalcEnableDebug
        .fPredictionTimeCrv_sec =
        sTPDebug.pCalcEnableDebug.fPredictionTimeCrv_sec;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pCalcEnableDebug
        .fPredictionTimeHead_sec =
        sTPDebug.pCalcEnableDebug.fPredictionTimeHead_sec;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pCalcEnableDebug
        .bCorridorJumpDetected =
        sTPDebug.pCalcEnableDebug.bCorridorJumpDetected;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pCalcEnableDebug
        .bLatDMCReqFinished = sTPDebug.pCalcEnableDebug.bLatDMCReqFinished;

    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrenetBackDebug
        .fTrajHeading_rad = sTPDebug.pFrenetBackDebug.fTrajHeading_rad;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrenetBackDebug.fTrajDistY_met =
        sTPDebug.pFrenetBackDebug.fTrajDistY_met;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrenetBackDebug
        .fTrajTgtCurve_1pm = sTPDebug.pFrenetBackDebug.fTrajTgtCurve_1pm;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrenetBackDebug
        .fTrajTgtCrvGrd_1pms = sTPDebug.pFrenetBackDebug.fTrajTgtCrvGrd_1pms;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrenetBackDebug
        .uiTrajGuiQualifier_nu =
        sTPDebug.pFrenetBackDebug.uiTrajGuiQualifier_nu;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrenetBackDebug
        .fTrajHeadingPrev_rad = sTPDebug.pFrenetBackDebug.fTrajHeadingPrev_rad;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrenetBackDebug
        .fTrajTgtCrvPrev_1pm = sTPDebug.pFrenetBackDebug.fTrajTgtCrvPrev_1pm;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrenetBackDebug.fCurHeading_rad =
        sTPDebug.pFrenetBackDebug.fCurHeading_rad;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrenetBackDebug.fCurDistY_met =
        sTPDebug.pFrenetBackDebug.fCurDistY_met;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrenetBackDebug
        .fTrajHeadInclPrev_rad =
        sTPDebug.pFrenetBackDebug.fTrajHeadInclPrev_rad;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrenetBackDebug
        .fCtrlErrDistY_met = sTPDebug.pFrenetBackDebug.fCtrlErrDistY_met;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrenetBackDebug
        .fCtrlErrHeadingAngle_rad =
        sTPDebug.pFrenetBackDebug.fCtrlErrHeadingAngle_rad;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrenetBackDebug
        .fCtrlErrHeadAglPrev_rad =
        sTPDebug.pFrenetBackDebug.fCtrlErrHeadAglPrev_rad;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrenetBackDebug
        .fTrajDistYPrev_met = sTPDebug.pFrenetBackDebug.fTrajDistYPrev_met;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrenetBackDebug
        .fDeltaTargetCrv_1pm = sTPDebug.pFrenetBackDebug.fDeltaTargetCrv_1pm;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrenetBackDebug
        .fDeltaTargetPosY0_met =
        sTPDebug.pFrenetBackDebug.fDeltaTargetPosY0_met;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrenetBackDebug
        .fDeltaTargetHeading_rad =
        sTPDebug.pFrenetBackDebug.fDeltaTargetHeading_rad;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrenetBackDebug
        .bUseTargetCorridor = sTPDebug.pFrenetBackDebug.bUseTargetCorridor;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrenetBackDebug.bTargetSwitch =
        sTPDebug.pFrenetBackDebug.bTargetSwitch;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrenetBackDebug
        .bGradLimitActive = sTPDebug.pFrenetBackDebug.bGradLimitActive;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrenetBackDebug
        .bPlausiCheckStatus = sTPDebug.pFrenetBackDebug.bPlausiCheckStatus;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrenetBackDebug
        .uiS_QuStatusTrajPlan_nu =
        sTPDebug.pFrenetBackDebug.uiS_QuStatusTrajPlan_nu;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrenetBackDebug
        .fTrajTgtCrvGrdPrev_1pms =
        sTPDebug.pFrenetBackDebug.fTrajTgtCrvGrdPrev_1pms;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrenetBackDebug
        .uiD_QuStatusTrajPlan_nu =
        sTPDebug.pFrenetBackDebug.uiD_QuStatusTrajPlan_nu;

    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrentTransfDebug.fCurDistY_met =
        sTPDebug.pFrentTransfDebug.fCurDistY_met;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrentTransfDebug
        .fCurDistY1stDeriv_mps =
        sTPDebug.pFrentTransfDebug.fCurDistY1stDeriv_mps;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrentTransfDebug
        .fCurDistY2ndDeriv_mps2 =
        sTPDebug.pFrentTransfDebug.fCurDistY2ndDeriv_mps2;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrentTransfDebug
        .fTrajVelRefCurve_mps = sTPDebug.pFrentTransfDebug.fTrajVelRefCurve_mps;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrentTransfDebug
        .fTrajAclRefCurve_mps2 =
        sTPDebug.pFrentTransfDebug.fTrajAclRefCurve_mps2;
    for (int i = 0; i < 15; i++) {
        pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrentTransfDebug
            .afTargetDistY_met[i] =
            sTPDebug.pFrentTransfDebug.afTargetDistY_met[i];
        pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrentTransfDebug
            .fTargetDistY1stDeriv_mps[i] =
            sTPDebug.pFrentTransfDebug.fTargetDistY1stDeriv_mps[i];
        pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrentTransfDebug
            .fTargetDistY2ndDeriv_mps2[i] =
            sTPDebug.pFrentTransfDebug.fTargetDistY2ndDeriv_mps2[i];
        pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrentTransfDebug
            .afTargetPoints_nu[i] =
            sTPDebug.pFrentTransfDebug.afTargetPoints_nu[i];
    }
    for (int i = 0; i < 100; i++) {
        pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrentTransfDebug
            .afLeDistY_met[i] = sTPDebug.pFrentTransfDebug.afLeDistY_met[i];
    }
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrentTransfDebug
        .fTrajDistYPrev_met = sTPDebug.pFrentTransfDebug.fTrajDistYPrev_met;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrentTransfDebug
        .fTrajDistY1stToPrev_mps =
        sTPDebug.pFrentTransfDebug.fTrajDistY1stToPrev_mps;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrentTransfDebug
        .fTrajDistY2ndToPrev_mps2 =
        sTPDebug.pFrentTransfDebug.fTrajDistY2ndToPrev_mps2;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrentTransfDebug
        .uiNumOfTgtPoints_nu = sTPDebug.pFrentTransfDebug.uiNumOfTgtPoints_nu;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrentTransfDebug
        .fTrajPlanningHorizon_sec =
        sTPDebug.pFrentTransfDebug.fTrajPlanningHorizon_sec;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrentTransfDebug
        .fDistY1stToDevHead_mps =
        sTPDebug.pFrentTransfDebug.fDistY1stToDevHead_mps;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrentTransfDebug
        .fDistY2ndToDevHead_mps2 =
        sTPDebug.pFrentTransfDebug.fDistY2ndToDevHead_mps2;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrentTransfDebug
        .fCurDistYPreview_met = sTPDebug.pFrentTransfDebug.fCurDistYPreview_met;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrentTransfDebug
        .fCurDistY1stToPrev_mps =
        sTPDebug.pFrentTransfDebug.fCurDistY1stToPrev_mps;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrentTransfDebug
        .fPreviewTimeHeading_sec =
        sTPDebug.pFrentTransfDebug.fPreviewTimeHeading_sec;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrentTransfDebug
        .fPlanHorizonVisRange_sec =
        sTPDebug.pFrentTransfDebug.fPlanHorizonVisRange_sec;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pFrentTransfDebug
        .uiNumOfPointsCridrLeft_nu =
        sTPDebug.pFrentTransfDebug.uiNumOfPointsCridrLeft_nu;

    for (int i = 0; i < 6; i++) {
        pLCFVehOutput->pLcfVehOutputData.sTPDebug.pTrajCalcDebug
            .afTrajParam_nu[i] = sTPDebug.pTrajCalcDebug.afTrajParam_nu[i];
    }
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pTrajCalcDebug.fTrajDistY_met =
        sTPDebug.pTrajCalcDebug.fTrajDistY_met;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pTrajCalcDebug
        .fTrajDistY1stDeriv_mps =
        sTPDebug.pTrajCalcDebug.fTrajDistY1stDeriv_mps;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pTrajCalcDebug
        .fTrajDistY2ndDeriv_mps2 =
        sTPDebug.pTrajCalcDebug.fTrajDistY2ndDeriv_mps2;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pTrajCalcDebug
        .fTrajDistY3rdDeriv_mps3 =
        sTPDebug.pTrajCalcDebug.fTrajDistY3rdDeriv_mps3;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pTrajCalcDebug
        .uiQuStatusTrajPlan_nu = sTPDebug.pTrajCalcDebug.uiQuStatusTrajPlan_nu;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pTrajCalcDebug.bTrajEnd =
        sTPDebug.pTrajCalcDebug.bTrajEnd;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pTrajCalcDebug.bLengthOK =
        sTPDebug.pTrajCalcDebug.bLengthOK;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pTrajCalcDebug.bMatrixInverseOK =
        sTPDebug.pTrajCalcDebug.bMatrixInverseOK;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pTrajCalcDebug
        .fEndPointTrajectory_nu =
        sTPDebug.pTrajCalcDebug.fEndPointTrajectory_nu;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pTrajCalcDebug
        .fPassedTrajLenPercent_per =
        sTPDebug.pTrajCalcDebug.fPassedTrajLenPercent_per;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pTrajCalcDebug.fMaxJerkTraj_mps3 =
        sTPDebug.pTrajCalcDebug.fMaxJerkTraj_mps3;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pTrajCalcDebug.bMaxJerkOK =
        sTPDebug.pTrajCalcDebug.bMaxJerkOK;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pTrajCalcDebug.fMaxAclTraj_mps2 =
        sTPDebug.pTrajCalcDebug.fMaxAclTraj_mps2;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pTrajCalcDebug.fOptimalCost_nu =
        sTPDebug.pTrajCalcDebug.fOptimalCost_nu;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pTrajCalcDebug
        .fWeightTargetDistY_nu = sTPDebug.pTrajCalcDebug.fWeightTargetDistY_nu;
    pLCFVehOutput->pLcfVehOutputData.sTPDebug.pTrajCalcDebug.fWeightEndTime_nu =
        sTPDebug.pTrajCalcDebug.fWeightEndTime_nu;
}
/* ****************************************************************************
 Functionname:     TCInputWrapper
 @brief: the input wrapper function for the TC process

 @description: the input wrapper function for the TC process

 @param[in]
                        reqLcfVehPrtList_t* pLCFVehInput:the input of the LCF
whole module
 @param[in]
                        reqLcfVehParams* pLcfVehParams:the parameter of the LCF
whole module
 @param[in]
                        TRJPLN_TrajectoryPlanOutPro_t* pTPOutput:the output of
the TP sub-module
 @param[out]
                        sTJATCTInReq_st* pTCInput: the input of the TC
sub-module

 @return           -
 @pre
 @post             -
 @author
**************************************************************************** */
static void TCInputWrapper(const reqLcfVehPrtList_t* pLCFVehInput,
                           const reqLcfVehParams* pLcfVehParams,
                           const TRJPLN_TrajectoryPlanOutPro_t* pTPOutput,
                           sTJATCTInReq_st* pTCInput) {
    pTCInput->TCTI_TimeSysCycle = pLcfVehParams->LCFVeh_Kf_fSysCycleTime_sec;

    pTCInput->TCTI_VehicleVelX = pLCFVehInput->sLCFVehDyn.fEgoVelX_mps;
    pTCInput->TCTI_VehYawRate = pLCFVehInput->sLCFVehDyn.fEgoYawRate_rps;
    pTCInput->TCTI_WhlSteerAngleVdy =
        pLCFVehInput->sLCFVehDyn.fWhlSteerAngleVdy_rad;
    pTCInput->TCTI_EstSelfSteerGrdnt =
        pLCFVehInput->sLCFVehDyn.fEstSelfSteerGrdnt_rads2pm;
    pTCInput->TCTI_VehCurvature = pLCFVehInput->sLCFVehDyn.fEgoCurve_1pm;

    pTCInput->TCTI_SteerAngleLaDmc =
        pLCFVehInput->sLcfVehInputFromDMCData.fSteerAngleLaDmc;

    pTCInput->TCTI_StLatCtrlMode =
        pLCFVehInput->sLcfSVehInputFromSenData.sTJASAState.uiLatCtrlMode_nu;

    pTCInput->TCTI_StCntrlFcn = pLCFVehInput->sLcfSVehInputFromSenData
                                    .sCSCLTAData.uiControllingFunction_nu;
    pTCInput->TCTI_ReqTrajCrvCsc =
        pLCFVehInput->sLcfSVehInputFromSenData.sCSCLTAData.fTgtTrajCurve_1pm;
    pTCInput->TCTI_MaxCrvGrdBuildup = pLCFVehInput->sLcfSVehInputFromSenData
                                          .sCSCLTAData.fMaxCrvGrdBuildup_1pms;
    pTCInput->TCTI_MaxCrvGrdRed =
        pLCFVehInput->sLcfSVehInputFromSenData.sCSCLTAData.fMaxCrvGrdRed_1pms;
    pTCInput->TCTI_LmtReqTrajCrvGrd = pLCFVehInput->sLcfSVehInputFromSenData
                                          .sCSCLTAData.fGrdLimitTgtCrvGC_1pms;
    pTCInput->TCTI_MaxTrajCrv = pLCFVehInput->sLcfSVehInputFromSenData
                                    .sCSCLTAData.fMaxCrvTrajGuiCtrl_1pm;
    pTCInput->TCTI_EnaLmtActCsc =
        pLCFVehInput->sLcfSVehInputFromSenData.sCSCLTAData.bLimiterActivated_nu;
    pTCInput->TCTI_TimeLmtDur =
        pLCFVehInput->sLcfSVehInputFromSenData.sCSCLTAData.fLimiterDuration_sec;
    pTCInput->TCTI_StLcfSys =
        pLCFVehInput->sLcfSVehInputFromSenData.sCSCLTAData.uiSysStateLCF_nu;

    pTCInput->TCTI_StVehOdo = pLCFVehInput->sTPRCVInReq.uiStVehOdo_nu;
    pTCInput->TCTI_StLaneLaKmc = pLCFVehInput->sTPRCVInReq.uiStLaneLaKmc_st;

    pTCInput->TCTI_EnaReplanCurValues = pTPOutput->bReplanCurValues;
    pTCInput->TCTI_BtfTrajGuiQualifier = pTPOutput->uiTrajGuiQualifier_nu;
    pTCInput->TCTI_ReqTrajDistYTpl = pTPOutput->fTrajDistY_met;
    pTCInput->TCTI_CurTrajDistYTpl = pTPOutput->fCurDistY_met;
    pTCInput->TCTI_ReqTrajCrvTpl = pTPOutput->fTrajTgtCurve_1pm;
    // pTCInput->TCTI_ReqTrajCrvTpl = pTPOutput->fTrajTgtCrvPrev_1pm;
    pTCInput->TCTI_ReqTrajHeadTpl = pTPOutput->fCurHeading_rad;
    pTCInput->TCTI_InclPrevTrajHeadTpl = pTPOutput->fTrajHeadInclPrev_rad;

    pTCInput->TestFlag_EnableFF =
        pLCFVehInput->sLCFVehCalibrations.TestFlag_EnableFF;
    pTCInput->TestFlag_EnableTP =
        pLCFVehInput->sLCFVehCalibrations.TestFlag_EnableTP;
    pTCInput->TestFlag_EnableOL =
        pLCFVehInput->sLCFVehCalibrations.TestFlag_EnableOL;

    pTCInput->Test_CoeffMainPGainLdc =
        pLCFVehInput->sLCFVehCalibrations.Test_CoeffMainPGainLdc;
    pTCInput->Test_CoeffPGainLdc =
        pLCFVehInput->sLCFVehCalibrations.Test_CoeffPGainLdc;
    pTCInput->Test_CoeffIGainLdc =
        pLCFVehInput->sLCFVehCalibrations.Test_CoeffIGainLdc;
    pTCInput->Test_CoeffDGainLdc =
        pLCFVehInput->sLCFVehCalibrations.Test_CoeffDGainLdc;
    pTCInput->Test_TimeDT1Ldc =
        pLCFVehInput->sLCFVehCalibrations.Test_TimeDT1Ldc;
    pTCInput->Test_CoeffPT1GainLdc =
        pLCFVehInput->sLCFVehCalibrations.Test_CoeffPT1GainLdc;
    pTCInput->Test_TimePT1Ldc =
        pLCFVehInput->sLCFVehCalibrations.Test_TimePT1Ldc;

    pTCInput->Test_CoeffMainPGainCac =
        pLCFVehInput->sLCFVehCalibrations.Test_CoeffMainPGainCac;
    pTCInput->Test_CoeffPGainCac =
        pLCFVehInput->sLCFVehCalibrations.Test_CoeffPGainCac;
    pTCInput->Test_CoeffIGainCac =
        pLCFVehInput->sLCFVehCalibrations.Test_CoeffIGainCac;
    pTCInput->Test_CoeffDGainCac =
        pLCFVehInput->sLCFVehCalibrations.Test_CoeffDGainCac;
    pTCInput->Test_TimeDT1Cac =
        pLCFVehInput->sLCFVehCalibrations.Test_TimeDT1Cac;
    pTCInput->Test_CoeffPT1GainCac =
        pLCFVehInput->sLCFVehCalibrations.Test_CoeffPT1GainCac;
    pTCInput->Test_TimePT1Cac =
        pLCFVehInput->sLCFVehCalibrations.Test_TimePT1Cac;
    pTCInput->Test_CDCTimeFltCurHeading =
        pLCFVehInput->sLCFVehCalibrations.Test_CDCTimeFltCurHeading;

    memcpy(pTCInput->LQR_e1_gains,
           pLCFVehInput->sLCFVehCalibrations.LQR_e1_gains,
           9 * sizeof(real32_T));
    memcpy(pTCInput->LQR_e1dot_gains,
           pLCFVehInput->sLCFVehCalibrations.LQR_e1dot_gains,
           9 * sizeof(real32_T));
    memcpy(pTCInput->LQR_e2_gains,
           pLCFVehInput->sLCFVehCalibrations.LQR_e2_gains,
           9 * sizeof(real32_T));
    memcpy(pTCInput->LQR_e2dot_gains,
           pLCFVehInput->sLCFVehCalibrations.LQR_e2dot_gains,
           9 * sizeof(real32_T));
    memcpy(pTCInput->LQR_Feedforward_gains,
           pLCFVehInput->sLCFVehCalibrations.LQR_Feedforward_gains,
           9 * sizeof(real32_T));
}

/* ****************************************************************************
 Functionname:     TCOutputWrapper
 @brief:		   the output wrapper function for the TC process

 @description:	   convert TC sub-module's result to LCF Veh output structure

 @param[in]		   const sMCTLFCOut_st sMCTLFCOutput: MCTLFC
sub-module's output
 @param[out]       proLcfSenPrtList_t* LCFSenOutput: LCF Sen output pointer
 @return           -
 @pre
 @post             -
 @author
**************************************************************************** */
static void TCOutputWrapper(const sTJATCTDebug_st sTCDebug,
                            const sTJATCTOut_st sTCOutput,
                            proLcfVehPrtList_t* pLCFVehOutput) {
    pLCFVehOutput->pLcfVehOutputData.sTCOutput.TJATCT_TimerPlauCheck =
        sTCOutput.TJATCT_TimerPlauCheck;
    pLCFVehOutput->pLcfVehOutputData.sTCOutput.TJATCT_DeltaFCmd =
        sTCOutput.TJATCT_DeltaFCmd;
    pLCFVehOutput->pLcfVehOutputData.sTCOutput.TJATCT_ReqFfcCrv =
        sTCOutput.TJATCT_ReqFfcCrv;
    pLCFVehOutput->pLcfVehOutputData.sTCOutput.TJATCT_SumCtrlCrv =
        sTCOutput.TJATCT_SumCtrlCrv;
    pLCFVehOutput->pLcfVehOutputData.sTCOutput.TJATCT_HldVehCrv =
        sTCOutput.TJATCT_HldVehCrv;
    pLCFVehOutput->pLcfVehOutputData.sTCOutput.TJATCT_ThdCrvPlauChkUp =
        sTCOutput.TJATCT_ThdCrvPlauChkUp;
    pLCFVehOutput->pLcfVehOutputData.sTCOutput.TJATCT_ThdCrvPlauChkLow =
        sTCOutput.TJATCT_ThdCrvPlauChkLow;
    pLCFVehOutput->pLcfVehOutputData.sTCOutput.TJATCT_ReqFbcDcCrv =
        sTCOutput.TJATCT_ReqFbcDcCrv;
    pLCFVehOutput->pLcfVehOutputData.sTCOutput.TJATCT_LmtReqFfcCrv =
        sTCOutput.TJATCT_LmtReqFfcCrv;
    pLCFVehOutput->pLcfVehOutputData.sTCOutput.TJATCT_BtfQulifierTrajCtrl =
        sTCOutput.TJATCT_BtfQulifierTrajCtrl;
    pLCFVehOutput->pLcfVehOutputData.sTCOutput.TJATCT_EnaUnplauRequest =
        sTCOutput.TJATCT_EnaUnplauRequest;
    pLCFVehOutput->pLcfVehOutputData.sTCOutput.TJATCT_EnaSetDegrReq =
        sTCOutput.TJATCT_EnaSetDegrReq;
    pLCFVehOutput->pLcfVehOutputData.sTCOutput.TJATCT_EnaRstDegrReq =
        sTCOutput.TJATCT_EnaRstDegrReq;
    pLCFVehOutput->pLcfVehOutputData.sTCOutput.TJATCT_EnaPlausibilityCheck =
        sTCOutput.TJATCT_EnaPlausibilityCheck;
    pLCFVehOutput->pLcfVehOutputData.sTCOutput.TJATCT_EnaHldVehCrv =
        sTCOutput.TJATCT_EnaHldVehCrv;
    pLCFVehOutput->pLcfVehOutputData.sTCOutput.TJATCT_EnaLmtWarn =
        sTCOutput.TJATCT_EnaLmtWarn;
    pLCFVehOutput->pLcfVehOutputData.sTCOutput.TJATCT_EnaPlauCheck =
        sTCOutput.TJATCT_EnaPlauCheck;
    pLCFVehOutput->pLcfVehOutputData.sTCOutput.TJATCT_EnaPlauCheck =
        sTCOutput.TJATCT_EnaPlauCheck;
    pLCFVehOutput->pLcfVehOutputData.sTCOutput.LGC_EnableCtrl_nu =
        sTCOutput.LGC_EnableCtrl_nu;
    pLCFVehOutput->pLcfVehOutputData.sTCOutput.CDC_CtrlErrDistY =
        sTCOutput.CDC_CtrlErrDistY;
    pLCFVehOutput->pLcfVehOutputData.sTCOutput.CDC_CtrlErrHeading =
        sTCOutput.CDC_CtrlErrHeading;
    pLCFVehOutput->pLcfVehOutputData.sTCOutput.data_log_0 =
        sTCOutput.data_log_0;
    pLCFVehOutput->pLcfVehOutputData.sTCOutput.data_log_1 =
        sTCOutput.data_log_1;
    pLCFVehOutput->pLcfVehOutputData.sTCOutput.data_log_2 =
        sTCOutput.data_log_2;
    pLCFVehOutput->pLcfVehOutputData.sTCOutput.data_log_3 =
        sTCOutput.data_log_3;
    pLCFVehOutput->pLcfVehOutputData.sTCOutput.data_log_4 =
        sTCOutput.data_log_4;
    pLCFVehOutput->pLcfVehOutputData.sTCOutput.data_log_5 =
        sTCOutput.data_log_5;
    pLCFVehOutput->pLcfVehOutputData.sTCOutput.data_log_6 =
        sTCOutput.data_log_6;
    pLCFVehOutput->pLcfVehOutputData.sTCOutput.data_log_7 =
        sTCOutput.data_log_7;
    pLCFVehOutput->pLcfVehOutputData.sTCOutput.data_log_8 =
        sTCOutput.data_log_8;
    pLCFVehOutput->pLcfVehOutputData.sTCOutput.data_log_9 =
        sTCOutput.data_log_9;
    memcpy(&(pLCFVehOutput->pLcfVehOutputData.sTCDebug), &sTCDebug,
           sizeof(sTJATCTDebug_st));
}
static void TCInputOutWrapper(const sTJATCTInReq_st pTCInput,
                              proLcfVehPrtList_t* pLCFVehOutput) {
    pLCFVehOutput->pLcfVehOutputData.pTCInput.TCTI_BtfTrajGuiQualifier =
        pTCInput.TCTI_BtfTrajGuiQualifier;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.TCTI_CurTrajDistYTpl =
        pTCInput.TCTI_CurTrajDistYTpl;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.TCTI_EnaLmtActCsc =
        pTCInput.TCTI_EnaLmtActCsc;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.TCTI_EnaReplanCurValues =
        pTCInput.TCTI_EnaReplanCurValues;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.TCTI_EstSelfSteerGrdnt =
        pTCInput.TCTI_EstSelfSteerGrdnt;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.TCTI_InclPrevTrajHeadTpl =
        pTCInput.TCTI_InclPrevTrajHeadTpl;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.TCTI_LmtReqTrajCrvGrd =
        pTCInput.TCTI_LmtReqTrajCrvGrd;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.TCTI_MaxCrvGrdBuildup =
        pTCInput.TCTI_MaxCrvGrdBuildup;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.TCTI_MaxCrvGrdRed =
        pTCInput.TCTI_MaxCrvGrdRed;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.TCTI_MaxTrajCrv =
        pTCInput.TCTI_MaxTrajCrv;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.TCTI_ReqTrajCrvCsc =
        pTCInput.TCTI_ReqTrajCrvCsc;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.TCTI_ReqTrajCrvTpl =
        pTCInput.TCTI_ReqTrajCrvTpl;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.TCTI_ReqTrajDistYTpl =
        pTCInput.TCTI_ReqTrajDistYTpl;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.TCTI_ReqTrajHeadTpl =
        pTCInput.TCTI_ReqTrajHeadTpl;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.TCTI_StCntrlFcn =
        pTCInput.TCTI_StCntrlFcn;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.TCTI_SteerAngleLaDmc =
        pTCInput.TCTI_SteerAngleLaDmc;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.TCTI_StLaneLaKmc =
        pTCInput.TCTI_StLaneLaKmc;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.TCTI_StLatCtrlMode =
        pTCInput.TCTI_StLatCtrlMode;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.TCTI_StLcfSys =
        pTCInput.TCTI_StLcfSys;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.TCTI_StVehOdo =
        pTCInput.TCTI_StVehOdo;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.TCTI_TimeLmtDur =
        pTCInput.TCTI_TimeLmtDur;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.TCTI_TimeSysCycle =
        pTCInput.TCTI_TimeSysCycle;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.TCTI_VehCurvature =
        pTCInput.TCTI_VehCurvature;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.TCTI_VehicleVelX =
        pTCInput.TCTI_VehicleVelX;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.TCTI_VehYawRate =
        pTCInput.TCTI_VehYawRate;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.TCTI_WhlSteerAngleVdy =
        pTCInput.TCTI_WhlSteerAngleVdy;

    pLCFVehOutput->pLcfVehOutputData.pTCInput.TestFlag_EnableFF =
        pTCInput.TestFlag_EnableFF;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.TestFlag_EnableTP =
        pTCInput.TestFlag_EnableTP;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.TestFlag_EnableOL =
        pTCInput.TestFlag_EnableOL;

    pLCFVehOutput->pLcfVehOutputData.pTCInput.Test_CoeffMainPGainLdc =
        pTCInput.Test_CoeffMainPGainLdc;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.Test_CoeffPGainLdc =
        pTCInput.Test_CoeffPGainLdc;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.Test_CoeffIGainLdc =
        pTCInput.Test_CoeffIGainLdc;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.Test_CoeffDGainLdc =
        pTCInput.Test_CoeffDGainLdc;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.Test_TimeDT1Ldc =
        pTCInput.Test_TimeDT1Ldc;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.Test_CoeffPT1GainLdc =
        pTCInput.Test_CoeffPT1GainLdc;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.Test_TimePT1Ldc =
        pTCInput.Test_TimePT1Ldc;

    pLCFVehOutput->pLcfVehOutputData.pTCInput.Test_CoeffMainPGainCac =
        pTCInput.Test_CoeffMainPGainCac;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.Test_CoeffPGainCac =
        pTCInput.Test_CoeffPGainCac;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.Test_CoeffIGainCac =
        pTCInput.Test_CoeffIGainCac;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.Test_CoeffDGainCac =
        pTCInput.Test_CoeffDGainCac;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.Test_TimeDT1Cac =
        pTCInput.Test_TimeDT1Cac;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.Test_CoeffPT1GainCac =
        pTCInput.Test_CoeffPT1GainCac;
    pLCFVehOutput->pLcfVehOutputData.pTCInput.Test_TimePT1Cac =
        pTCInput.Test_TimePT1Cac;

    pLCFVehOutput->pLcfVehOutputData.pTCInput.Test_CDCTimeFltCurHeading =
        pTCInput.Test_CDCTimeFltCurHeading;
}

/* ****************************************************************************
 Functionname:     LCKInputWrapper
 @brief: the input wrapper function for the LCK process

 @description: the input wrapper function for the LCK process

 @param[in]
                        reqLcfVehPrtList_t* pLCFVehInput:the input of the LCF
whole module
 @param[in]
                        reqLcfVehParams* pLcfVehParams:the parameter of the LCF
whole module
 @param[out]
                        sLCKInput_t* pLCKInput: the input of the LCK sub-module

 @return           -
 @pre
 @post             -
 @author
**************************************************************************** */
static void LCKInputWrapper(const reqLcfVehPrtList_t* pLCFVehInput,
                            const reqLcfVehParams* pLcfVehParams,
                            sLCKInput_t* pLCKInput) {
    static float32 fLCKLaneCurvLast = 0.f;

    // eLCKMode
    switch (pLCFVehInput->sLcfSVehInputFromSenData.sLDPSAData.LDPSC_SysOut_St) {
        case LDP_SYSOUT_Disable:
            pLCKInput->eLCKMode = LCK_MODE_OFF;
            break;

        case LDP_SYSOUT_RampOut:
            pLCKInput->eLCKMode = LCK_MODE_OFF_NOW;
            break;

        case LDP_SYSOUT_Request:
            /* here we can switch between different controller */
            pLCKInput->eLCKMode = LCK_MODE_ON_SP; /* state space controller */
            break;

        case LDP_SYSOUT_Control:
            pLCKInput->eLCKMode = LCK_MODE_ON_SP; /* state space controller */
            break;

        default:
            /* by default switch off lateral control and output error code */
            pLCKInput->eLCKMode = LCK_MODE_OFF;
            break;
    }

    // fCurvature and fHeading
    switch (
        pLCFVehInput->sLcfSVehInputFromSenData.sLDPSAData.LDPSC_DgrSide_St) {
        case LDP_LANE_DEPARTURE_LEFT:
            pLCKInput->fCurvature = pLCFVehInput->sLcfSVehInputFromSenData
                                        .sLDPSAData.LDPDT_LnCltdCurvLf_ReMi;
            pLCKInput->fHeading = pLCFVehInput->sLcfSVehInputFromSenData
                                      .sLDPSAData.LDPDT_LnHeadLf_Rad;
            break;
        case LDP_LANE_DEPARTURE_RIGHT:
            pLCKInput->fCurvature = pLCFVehInput->sLcfSVehInputFromSenData
                                        .sLDPSAData.LDPDT_LnCltdCurvRi_ReMi;
            pLCKInput->fHeading = pLCFVehInput->sLcfSVehInputFromSenData
                                      .sLDPSAData.LDPDT_LnHeadRi_Rad;
            break;
        default:
            pLCKInput->fCurvature = 0.0f;
            pLCKInput->fHeading = 0.0f;
            break;
    }
    pLCKInput->fCurvature =
        pLCKInput->fCurvature * LDP_CAMERA_INPUT_SIGN_CORRECT;
    pLCKInput->fHeading = pLCKInput->fHeading * LDP_CAMERA_INPUT_SIGN_CORRECT;

    TUE_CML_LowPassFilter(&fLCKLaneCurvLast, pLCKInput->fCurvature, 0.05f);
    pLCKInput->fCurvature = fLCKLaneCurvLast;

    // fCycleTime_sec
    pLCKInput->fCycleTime_sec = pLcfVehParams->LCFVeh_Kf_fSysCycleTime_sec;

    // eOpMode
    pLCKInput->eOpMode = LCKOpMode_Running;  //待确认

    pLCKInput->fDrvTrq = 0.f;
    pLCKInput->fLatDev =
        0.5f *
        (pLCFVehInput->sLcfSVehInputFromSenData.sLDPSAData.LDPDT_LnPstnLf_Mi +
         pLCFVehInput->sLcfSVehInputFromSenData.sLDPSAData.LDPDT_LnPstnRi_Mi) *
        LDP_CAMERA_INPUT_SIGN_CORRECT;

    // fSteerAng;
    if (TUE_CML_Abs(pLcfVehParams->LCFVeh_Kf_SteeringRatio_nu) >
        LCK_INIT_F_ZERO) {
        pLCKInput->fSteerAng = pLCFVehInput->sLCFVehDyn.fWhlSteerAngleVdy_rad /
                               pLcfVehParams->LCFVeh_Kf_SteeringRatio_nu;
    } else {
        // check steering ratio value range
        pLCKInput->fSteerAng = LCK_INIT_F_ZERO;
    }

    //
    pLCKInput->fSteerAngVel = 0.f;
    pLCKInput->fVhclSpeed = pLCFVehInput->sLCFVehDyn.fEgoVelX_mps;
    pLCKInput->fYawRate = pLCFVehInput->sLCFVehDyn.fEgoYawRate_rps;
}

/* ****************************************************************************
 Functionname:     LCDInputWrapper
 @brief: the input wrapper function for the LCD process

 @description: the input wrapper function for the LCD process

 @param[in]
                        reqLcfVehPrtList_t* pLCFVehInput:the input of the LCF
whole module
 @param[in]
                        reqLcfVehParams* pLcfVehParams:the parameter of the LCF
whole module
 @param[in]
                        sLCKOutput_t* pLCKOutput: the output of the LCD
sub-module
 @param[out]
                        sLCDInput_t* pLCDInput: the input of the LCD sub-module

 @return           -
 @pre
 @post             -
 @author
**************************************************************************** */
static void LCDInputWrapper(const reqLcfVehPrtList_t* pLCFVehInput,
                            const reqLcfVehParams* pLcfVehParams,
                            const sLCKInput_t* pLCKInput,
                            const sLCKOutput_t* pLCKOutput,
                            sLCDInput_t* pLCDInput) {
    eLCKLCDCtrlMode_t eLCKLCDCtrlMode;

    //
    pLCDInput->eOpMode = LCDOpMode_Running;  //待确认

    pLCDInput->fDrvTrq = 0.f;
    pLCDInput->fVhclSpeed = pLCFVehInput->sLCFVehDyn.fEgoVelX_mps;

    pLCDInput->fCycleTime_sec = pLcfVehParams->LCFVeh_Kf_fSysCycleTime_sec;

    pLCDInput->fSteerAng = pLCKInput->fSteerAng;
    pLCDInput->fSteerAngVel = 0.f;

    // fRefSteerAng
    if (pLCKOutput->bRefStrAnglReq == TRUE) {
        /* enable dynamic controller */
        eLCKLCDCtrlMode = LCK_LCD_MODE_ON;
        pLCDInput->fRefSteerAng = pLCKOutput->fRefStrAng;
    } else {
        /* disable dynamic controller */
        eLCKLCDCtrlMode = LCK_LCD_MODE_OFF;
        pLCDInput->fRefSteerAng = pLCKInput->fSteerAng;
    }

    /* overwrite enabling/disabling of dynamic controller
       if immediate cancelation is requested */
    if (pLCKInput->eLCKMode == LCK_MODE_OFF_NOW) {
        eLCKLCDCtrlMode = LCK_LCD_MODE_OFF_NOW;
        pLCDInput->fRefSteerAng = pLCKInput->fSteerAng;
    }

    // eLCDMode
    switch (eLCKLCDCtrlMode) {
        case LCK_LCD_MODE_OFF:
            pLCDInput->eLCDMode = LCD_MODE_OFF;
            break;

        case LCK_LCD_MODE_OFF_NOW:
            pLCDInput->eLCDMode = LCD_MODE_OFF_NOW;
            break;

        case LCK_LCD_MODE_ON:
            /* here we can switch between different controller */
            if (LCD_PAR_SELECT_CTRL == 1u) {
                pLCDInput->eLCDMode =
                    LCD_MODE_ON_DEV; /* development controller */
            } else {
                pLCDInput->eLCDMode = LCD_MODE_ON_PID; /* PID controller */
            }
            break;

        default:
            /* by default switch off lateral control and output error code */
            pLCDInput->eLCDMode = LCD_MODE_OFF;
            break;
    }
}

/* ****************************************************************************
 Functionname:     TCOutputWrapper
 @brief:		   the output wrapper function for the TC process

 @description:	   convert TC sub-module's result to LCF Veh output structure

 @param[in]		   const sMCTLFCOut_st sMCTLFCOutput: MCTLFC
sub-module's output
 @param[out]       proLcfSenPrtList_t* LCFSenOutput: LCF Sen output pointer
 @return           -
 @pre
 @post             -
 @author
**************************************************************************** */
static void LCDOutputWrapper(const sLCDOutput_t sLCDOutput,
                             proLcfVehPrtList_t* pLCFVehOutput) {
    pLCFVehOutput->pLcfVehOutputData.sLCDOutput.bTorqueReq =
        sLCDOutput.bTorqueReq;
    pLCFVehOutput->pLcfVehOutputData.sLCDOutput.fTorque = sLCDOutput.fTorque;
    pLCFVehOutput->pLcfVehOutputData.sLCDOutput.fTorqueGrad =
        sLCDOutput.fTorqueGrad;
    pLCFVehOutput->pLcfVehOutputData.sLCDOutput.uiLCDState =
        sLCDOutput.uiLCDState;
}

/* ****************************************************************************
 Functionname:     HODInputWrapper
 @brief: the input wrapper function for the HOD process

 @description: the input wrapper function for the HOD process

 @param[in]reqLcfVehPrtList_t:the input of the LCF whole module

 @param[out]pHODInput: the input of the HOD sub-module

 @return           -
 @pre
 @post             -
 @author
**************************************************************************** */
static void HODInputWrapper(const reqLcfVehPrtList_t* pLCFVehInput,
                            const reqLcfVehParams* pLcfVehParams,
                            sHODInput_t* pHODInput) {
    // printf("--------------------HODInput--------------------\n");
    pHODInput->HOD_BtfAvailableFct = 2u;
    pHODInput->HOD_BtfControlFct = 1u;
    pHODInput->HOD_CycleTime_Sec = pLcfVehParams->LCFVeh_Kf_fSysCycleTime_sec;
    pHODInput->HOD_EnaActTorque = 1u;
    pHODInput->HOD_StActFctLevel = 0u;
    pHODInput->HOD_TrqActMan = pLCFVehInput->sLCFVehDyn.fManuActuTrqEPS_nm;
    pHODInput->HOD_VelXVeh = pLCFVehInput->sLCFVehDyn.fEgoVelX_mps;
}

/* ****************************************************************************
 Functionname:     HODOutputWrapper
 @brief:		   the output wrapper function for the HOD process

 @description:	   convert HOD sub-module's result to LCF Veh output structure

 @param[in]		   const sHODOutput_t g_LCF_HOD_Output: HOD sub-module's
output
 @param[out]       proLcfVehPrtList_t* pLCFVehOutput: LCF VEH output pointer
 @return           -
 @pre
 @post             -
 @author
**************************************************************************** */
static void HODOutputWrapper(sHODDebug_t sHODDebug,
                             sHODOutput_t sHODOutput,
                             proLcfVehPrtList_t* pLCFVehOutput) {
    // printf("--------------------HODOutput--------------------\n");
    pLCFVehOutput->pLcfVehOutputData.sHODOutput.HOD_EnaHandsOffCnfm =
        sHODOutput.HOD_EnaHandsOffCnfm;
    pLCFVehOutput->pLcfVehOutputData.sHODOutput.HOD_StSysWarning =
        sHODOutput.HOD_StSysWarning;
    pLCFVehOutput->pLcfVehOutputData.sHODOutput.HOD_HandsOffAbuseWarn =
        sHODOutput.HOD_HandsOffAbuseWarn;
    pLCFVehOutput->pLcfVehOutputData.sHODOutput.HOD_RatDrvAttention =
        sHODOutput.HOD_RatDrvAttention;
    pLCFVehOutput->pLcfVehOutputData.sHODOutput.HOD_StEstHandTrq =
        sHODOutput.HOD_StEstHandTrq;
    memcpy(&pLCFVehOutput->pLcfVehOutputData.sHODDebug, &sHODDebug,
           sizeof(sHODDebug_t));
}

/* ****************************************************************************
 Functionname:     LCF_VEH_ResetParams
 @brief: set parameters input data only while reset process triggered

 @description: set parameters input data only while reset process triggered

 @param[in]
                        const reqLcfVehParams* pLCFInputParam: system paramters
from RTE data
 @param[out]
                         TRJPLN_TrajectoryPlanParam_t* pTPParam: algo parameters
of TP sub-module

 @return           -
 @pre
 @post             -
 @author
**************************************************************************** */
static void LCF_VEH_ResetParams(const reqLcfVehParams* pLCFInputParam,
                                TRJPLN_TrajectoryPlanParam_t* pTPParam,
                                sTJATCTParam_st* pTCParam,
                                sLCKParam_t* pLCKParams,
                                sLCDParams_t* pLCDParams) {
    // TP
    pTPParam->fCycleTime_sec = pLCFInputParam->LCFVeh_Kf_fSysCycleTime_sec;
    pTPParam->fEgoVehLength_met = pLCFInputParam->LCFVeh_Kf_fEgoVehLength_met;
    pTPParam->fEgoVehWidth_met = pLCFInputParam->LCFVeh_Kf_fEgoVehWidth_met;

    // TC
    pTCParam->tmp = 0u;

    // LCK
    pLCKParams->fCrnStiffFr = pLCFInputParam->LCFVeh_Kf_CrnStiffFr;
    pLCKParams->fCrnStiffRe = pLCFInputParam->LCFVeh_Kf_CrnStiffRe;
    pLCKParams->fDistToFrontAxle =
        pLCFInputParam->LCFVeh_Kf_DistToFrontAxle_met;
    pLCKParams->fDistToRearAxle = pLCFInputParam->LCFVeh_Kf_DistToRearAxle_met;
    pLCKParams->fMass = pLCFInputParam->LCFVeh_Kf_Mass_kg;
    pLCKParams->fSelfStrGrad = pLCFInputParam->LCFVeh_Kf_SelfStrGrad;
    pLCKParams->fSteeringRatio = pLCFInputParam->LCFVeh_Kf_SteeringRatio_nu;

    // LCD
    pLCDParams->uiVersionNum = 0u;
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */