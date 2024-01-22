/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "lcf_sen_main.h"
//#include "Rte_CtApLCFSEN.h"
#ifdef UBUNTU_SYSTEM
//#include "Rte_CtApLCFSEN.h"
//#include "dataType_Temp.h"
#endif
//#include "Rte_TypeNeedAdd.h"
#include "string.h"
/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE2_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
static reqLcfSenPrtList_t g_LCFSEN_reqPorts = {0};
static reqLcfSenParams g_LCFSEN_reqParams = {0};
static proLcfSenPrtList_t g_LCFSEN_proPorts = {0};
static reqLcfSenDebug_t g_LCFSEN_proDebugs = {0};
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"

#define ASW_QM_CORE2_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
static boolean gLCFSEN_bFirstInitial;
static boolean gLCFVEH_bFirstInitial;
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"

#define LCF_SEN_DEBUG

#ifndef LCF_SEN_DEBUG

// RTE signal declaration
#if 1
boolean g_VCU_LDPSwitchEn_B = 1;
boolean g_VCU_LDWSwitchEn_B = 1;
uint8_T g_VCU_LDPMod_St = 3;
uint8_T g_VCU_LDWMod_St = 3;
#else
extern boolean g_VCU_LDPSwitchEn_B;
extern boolean g_VCU_LDWSwitchEn_B;
extern uint8_T g_VCU_LDPMod_St;
extern uint8_T g_VCU_LDWMod_St;
#endif

#ifndef eTurnIndicator_Off
#define eTurnIndicator_Off 0U
#endif
#ifndef eTurnIndicator_Left
#define eTurnIndicator_Left 1U
#endif
#ifndef eTurnIndicator_Right
#define eTurnIndicator_Right 2U
#endif
#ifndef eTurnIndicator_Both
#define eTurnIndicator_Both 3U
#endif
#ifndef eTurnIndicator_Invalid
#define eTurnIndicator_Invalid 4U
#endif

// function protypes used for compile, need to be deleted,只为编译通过，需要删除
// void
// Rte_Read_CtApLCFSEN_PpVED_VehDyn_t_info_DeVED_VehDyn_t_buf(Dt_RECORD_VED_VehDyn_t*
// ps_VehDyn) {}  //VehDyn
// void
// Rte_Read_CtApLCFSEN_PpEnvm_SRRGenObjList_t_info_DeEnvm_SRRGenObjList_t_buf(Dt_RECORD_Envm_SRRGenObjList_t*
// pSSRObjList) {} //SSRObjList
// void
// Rte_Write_CtApLCFSEN_PpLCF_SenAlgoCompState_t_info_DeLCF_SenAlgoCompState_t_buf(Dt_RECORD_LCF_SenAlgoCompState_t*
// pSenAlgoCompState) {} //
// void
// Rte_Write_CtApLCFSEN_PpLCF_SenGenericOutputs_t_info_DeLCF_SenGenericOutputs_t_buf(Dt_RECORD_LCF_SenGenericOutputs_t*
// pLCF_SenGenericOutputs) {} //
extern uint32 Norm_SYS_GetCurrentTime(void);  //待确认

Dt_RECORD_VLC_SenAccOOI_t Dt_RECORD_VLC_SenAccOOI_buf = {0};
Dt_RECORD_Envm_GenObjectList Dt_RECORD_Envm_GenObjectList_buf = {0};
Dt_RECORD_Envm_CRObjectList Dt_RECORD_Envm_CRObjectList_buf = {0};
Dt_RECORD_VED_VehDyn_t Dt_RECORD_VED_VehDyn_buf = {0};

//待确认删除
//#ifdef LINUX_SYS
// extern uint32 Norm_SYS_GetCurrentTime(void);
//#else
// extern uint32 MPC5748G_TimeGetSinceBoot_us(void);
//#endif
#endif

/*****************************************************************************
  Functionname:LCF_SenReset                                            */ /*!

                 @brief: external reset function for system calling

                 @description: the LCF module should reset while first
               initialize
                                               or system error happened

                 @param[in]
                 @param[out]

                 @return
               *****************************************************************************/
void LCF_SenReset(void) {
    memset(&g_LCFSEN_reqPorts, 0, sizeof(reqLcfSenPrtList_t));
    memset(&g_LCFSEN_reqParams, 0, sizeof(reqLcfSenParams));
    memset(&g_LCFSEN_proPorts, 0, sizeof(proLcfSenPrtList_t));
    memset(&g_LCFSEN_proDebugs, 0, sizeof(reqLcfSenDebug_t));

    LcfSenSetParams(&g_LCFSEN_reqParams);
    LcfSenReset(&g_LCFSEN_reqParams);
}

#if 0
extern boolean ReadCurrent_bTorqueReq(void);
extern float32 ReadCurrent_fTorque(void);

extern uint8 LDPSC_DgrSide_St;

void EP30_SWC_LKA_VehControl(void)
{
	static uint8 MsgCounter = 0;
//	static uint8 FristExec = 1;
	static uint32 TOIActDelay = 0;
	FCTLKSOutputLCD_t FCTLKSOutputLCD_buf = {0};
	FCTLKSOutputLCD_t	 *FCT_pLCDOutputGeneric = &FCTLKSOutputLCD_buf;  /*!<internal pointer to external LCD gneric OUTPUT interface. Generic LKS output Dynamic.*/

	uint8 checksum = 0;

	uint16 FLC_LKA_StrToqReq = 0;
	float32 fSteerTrq = 0;
	static float32 lastTrq = 0;	

	Dt_RECORD_FLC_Fr01_0EA MsgFLC_Fr01_0EA = {0};
	Dt_RECORD_FLC_Fr02_0FA MsgFLC_Fr02_0FA = {0};

	MsgFLC_Fr01_0EA.FLC_Status = 0;
//	sFLC_Fr01.B.FLC_LDWLKA_Symbol = LDPSC_SysOut_St;

	//		: FLC_LDWLKA_Mode = 0x0 Off;
	//		: FLC_LDWLKA_Mode = 0x01 LDW;
	//		: FLC_LDWLKA_Mode = 0x2 LKA;
	MsgFLC_Fr01_0EA.FLC_LDWLKA_Mode = LDWLKA_Mode;

	if (MsgFLC_Fr01_0EA.FLC_LDWLKA_Mode == 0x02) {
		if (LDWLKA_Warn == 0x0) {
			VCU_LDPMod_St = 0x1;
			MsgFLC_Fr01_0EA.FLC_LDWLKA_WarnSetSta = 0x1;
		} else if(LDWLKA_Warn == 0x1){
			VCU_LDPMod_St = 0x2;
			MsgFLC_Fr01_0EA.FLC_LDWLKA_WarnSetSta = 0x2;
		}else if(LDWLKA_Warn == 0x2){
			VCU_LDPMod_St = 0x3;
			MsgFLC_Fr01_0EA.FLC_LDWLKA_WarnSetSta = 0x3;
		}else {
			VCU_LDPMod_St = 0x1;
			MsgFLC_Fr01_0EA.FLC_LDWLKA_WarnSetSta = 0x1;
		}

		if(LDPSC_SysOut_St == 0 || LDPSC_SysOut_St == 1) {
			MsgFLC_Fr01_0EA.FLC_LDWLKA_Symbol = 2;

		} else if(LDPSC_SysOut_St == 2) {
			MsgFLC_Fr01_0EA.FLC_LDWLKA_Symbol = 2;

		} else if(LDPSC_SysOut_St == 3 || LDPSC_SysOut_St == 4) {
			//MsgFLC_Fr01_0EA.FLC_LKA_TOIAct = FCT_pLCDOutputGeneric->bSteerTrqReq;

			MsgFLC_Fr01_0EA.FLC_LKA_TOIAct = ReadCurrent_bTorqueReq();//proLcfVehPrtList_t_PST->pLcfVehOutputData.sLCDOutput.bTorqueReq;
			

			if(MsgFLC_Fr01_0EA.FLC_LKA_TOIAct) {
				TOIActDelay++;
				MsgFLC_Fr01_0EA.FLC_LDWLKA_Symbol = 5;
			} else {
				MsgFLC_Fr01_0EA.FLC_LDWLKA_Symbol = 4;
				TOIActDelay = 0;
			}
			if((TOIActDelay > 2)) {
				fSteerTrq = ReadCurrent_fTorque();//proLcfVehPrtList_t_PST->pLcfVehOutputData.sLCDOutput.fTorque;
				if (LDPSC_DgrSide_St == 1u)
				{
					fSteerTrq = -(TUE_CML_Abs(fSteerTrq));
				}
				else if (LDPSC_DgrSide_St == 2u)
				{
					fSteerTrq = TUE_CML_Abs(fSteerTrq);
				}
				else
				{
				}
			}

			if(fSteerTrq - lastTrq > 0.0625 ) {
				fSteerTrq = lastTrq + 0.0625;
			} else if(fSteerTrq - lastTrq < -0.0625 ) {
				fSteerTrq = lastTrq -0.0625; 
			} 

			lastTrq = fSteerTrq;	
		} else {
			MsgFLC_Fr01_0EA.FLC_LDWLKA_Symbol = 2;
		}
		MsgFLC_Fr01_0EA.FLC_LDWLKA_Recog_SysState = 3;

	} else if(MsgFLC_Fr01_0EA.FLC_LDWLKA_Mode ==  0x01) {

		if (LDWLKA_Warn == 0x0) {
			VCU_LDWMod_St = 0x1;
			MsgFLC_Fr01_0EA.FLC_LDWLKA_WarnSetSta = 0x1;
		} else if(LDWLKA_Warn == 0x1){
			VCU_LDWMod_St = 0x2;
			MsgFLC_Fr01_0EA.FLC_LDWLKA_WarnSetSta = 0x2;
		}else if(LDWLKA_Warn == 0x2){
			VCU_LDWMod_St = 0x3;
			MsgFLC_Fr01_0EA.FLC_LDWLKA_WarnSetSta = 0x3;
		}else {
			VCU_LDWMod_St = 0x1;
			MsgFLC_Fr01_0EA.FLC_LDWLKA_WarnSetSta = 0x1;
		}


		if(LDWC_DgrSide_St == 1) {
			MsgFLC_Fr01_0EA.FLC_LDWLKA_Recog_SysState = 1;
			MsgFLC_Fr01_0EA.FLC_LDWLKA_LHWarning = 1;
			MsgFLC_Fr01_0EA.FLC_LDWLKA_RHWarning = 0;
			MsgFLC_Fr01_0EA.FLC_LDWLKA_Symbol = 3;

		} else if(LDWC_DgrSide_St == 2) {
			MsgFLC_Fr01_0EA.FLC_LDWLKA_Recog_SysState = 2;
			MsgFLC_Fr01_0EA.FLC_LDWLKA_LHWarning = 0;
			MsgFLC_Fr01_0EA.FLC_LDWLKA_RHWarning = 1;
			MsgFLC_Fr01_0EA.FLC_LDWLKA_Symbol = 3;

		} else if(LDWC_DgrSide_St == 3) {
			MsgFLC_Fr01_0EA.FLC_LDWLKA_Recog_SysState = 3;
			MsgFLC_Fr01_0EA.FLC_LDWLKA_LHWarning = 1;
			MsgFLC_Fr01_0EA.FLC_LDWLKA_RHWarning = 1;
			MsgFLC_Fr01_0EA.FLC_LDWLKA_Symbol = 3;

		} else {
			MsgFLC_Fr01_0EA.FLC_LDWLKA_Recog_SysState = 0;
			MsgFLC_Fr01_0EA.FLC_LDWLKA_LHWarning = 0;
			MsgFLC_Fr01_0EA.FLC_LDWLKA_RHWarning = 0;
			MsgFLC_Fr01_0EA.FLC_LDWLKA_Symbol = 1;
		}

	} else {
			VCU_LDPMod_St = 0x1;
			VCU_LDWMod_St = 0x1;
			if (LDWLKA_Warn == 0x0) {
				MsgFLC_Fr01_0EA.FLC_LDWLKA_WarnSetSta = 0x1;
			} else if(LDWLKA_Warn == 0x1){
				MsgFLC_Fr01_0EA.FLC_LDWLKA_WarnSetSta = 0x2;
			}else if(LDWLKA_Warn == 0x2){
				MsgFLC_Fr01_0EA.FLC_LDWLKA_WarnSetSta = 0x3;
			}else {
				MsgFLC_Fr01_0EA.FLC_LDWLKA_WarnSetSta = 0x1;
			}
			MsgFLC_Fr01_0EA.FLC_LDWLKA_Symbol = 0;
	}

	if(MsgFLC_Fr01_0EA.FLC_LDWLKA_Symbol != 5) {
		lastTrq = 0;

	}

	MsgFLC_Fr01_0EA.FLC_LKA_StrToqReq = fSteerTrq;
	MsgFLC_Fr01_0EA.FLC_Fr01_MsgCounter = MsgCounter&0x0f;

//	checksum = Veh_EP30_Checksum_Calc(&sFLC_Fr01.dataMsg[0], 8, 7);
	MsgFLC_Fr01_0EA.FLC_Fr01_Checksum = checksum;



	MsgFLC_Fr02_0FA.FLC_Fr02_MsgCounter = MsgCounter&0x0f;
//	checksum = Veh_EP30_Checksum_Calc(&sFLC_Fr02.dataMsg[0], 8, 7);
	MsgFLC_Fr02_0FA.FLC_Fr02_Checksum = checksum;

	//MsgFLC_Fr01_0EA.FLC_LDWLKA_RHWarning = MsgCounter&0x03;

	Rte_Write_PpCOM_FLC_Fr01_0EA_10ms_DeFLC_Fr01_0EA(&MsgFLC_Fr01_0EA); 
	Rte_Write_PpCOM_FLC_Fr02_0FA_10ms_DeFLC_Fr02_0FA(&MsgFLC_Fr02_0FA); 

	MsgCounter++;
}

#endif

/*****************************************************************************
Functionname:LCF_SenProcess                                            */ /*!

                       @brief: LCF main function whilc would be invoked by BSW

                       @description: LCF main function whilc would be invoked by
                     BSW

                       @param[in]
                       @param[out]

                       @return
                     *****************************************************************************/
#define SWC_LCFSEN_ONOFF 1
#if 0
uint32 LCF_SenProcess(void)
{
	uint32 uiRetCode = BASE_RETURN_ERROR;
	boolean bPortsCheckResult = FALSE;
	if(SWC_LCFSEN_ONOFF) {
		if (gLCFSEN_bFirstInitial)
		{
			//RTE and input check
			bPortsCheckResult = LcfSenPortsCheck(&g_LCFSEN_reqPorts, &g_LCFSEN_reqParams, &g_LCFSEN_proPorts, &g_LCFSEN_proDebugs);

			if (bPortsCheckResult)
			{
				//RTE to LCF input wrapper
				LcfRTESenInputWrapper(&g_LCFSEN_reqPorts);
				LcfSenExec(&g_LCFSEN_reqPorts, &g_LCFSEN_reqParams, &g_LCFSEN_proPorts, &g_LCFSEN_proDebugs);
				LcfRTESenOutputWrapper(&g_LCFSEN_proPorts);
				//EP30_SWC_LKA_VehControl();

				/* Runtime measurement is done via sections of code in "lcf_sen_service.h" */
				uiRetCode = BASE_RETURN_OK;
			}
			else
			{
				LCF_SenReset();
				uiRetCode = BASE_RETURN_ERROR;
			}
		}
		else
		{
			LCF_SenReset();
			gLCFSEN_bFirstInitial = TRUE;
			uiRetCode = BASE_RETURN_OK;
		}

		/*  in case error is returned, all provide ports are set to 0 */
		if (uiRetCode == BASE_RETURN_ERROR)
		{
			memset(&g_LCFSEN_proPorts, 0, sizeof(proLcfSenPrtList_t));
		}
		else {}

	}
	return uiRetCode;
}
#endif
/*****************************************************************************
  Functionname:LcfSenSetParams                                            */ /*!

        @brief: set system parameters based on RTE signal

        @description: set system parameters based on RTE signal

        @param[in/out]
                              reqLcfSenParams* LCFSenParams: the pointer of LCF
      Sen
      parameters
        @param[out]

        @return
      *****************************************************************************/
static void LcfSenSetParams(reqLcfSenParams* LCFSenParams) {
#ifndef LCF_SEN_DEBUG

    //	extern VehPar_t *
    // Rte_IWriteRef_SWCNorm_RunEntNorm_PPortVehPar_DEPVehPar(void);
    VehPar_t* ps_VehPar =
        Rte_IWriteRef_SWCNorm_RunEntNorm_PPortVehPar_DEPVehPar();
    // Rte_IRead_SWCVdyAdapt_RunEntVdyMain_RPortVehPar_DEPVehPar(ps_VehPar);
    // //待修改

    if (ps_VehPar->sSigHeader.eSigStatus == AL_SIG_STATE_OK) {
        LCFSenParams->LCFSen_Kf_fEgoLength_met =
            ps_VehPar->VehParAdd.VehicleLength;
        // LCFSenParams->LCFSen_Kf_fEgoWidth_met         =
        // ps_VehPar->VehParAdd.VehicleWidth;
        LCFSenParams->LCFSen_Kf_fEgoWidth_met = 1.84f;
        LCFSenParams->LCFSen_Kf_SysCycleTime_sec = 0.05f;
        LCFSenParams->LCFSen_Kf_fVehOverhangFront_met =
            ps_VehPar->VehParAdd.OverhangFront;
        LCFSenParams->LCFSen_Kf_fVehWheelBase_met =
            ps_VehPar->VehParMain.WheelBase;
    }

#endif
}

/*****************************************************************************
  Functionname:LcfRTESenInputWrapper */ /*!

@brief: set LCF SEN input pointer based on the value of RTE signals

@description: set LCF SEN input pointer based on the value of RTE signals

@param[in/out]
            reqLcfSenPrtList_t* g_LCFSEN_reqPorts: LCF SEN input pointer
@param[out]

@return
*****************************************************************************/
void LcfRTESenInputWrapper(reqLcfSenPrtList_t* g_LCFSEN_reqPorts) {
#ifndef LCF_SEN_DEBUG

    VehPar_t* ps_VehPar =
        Rte_IWriteRef_SWCNorm_RunEntNorm_PPortVehPar_DEPVehPar();   //待修改
    Dt_RECORD_VED_VehDyn_t* ps_VehDyn = &Dt_RECORD_VED_VehDyn_buf;  // VehDyn
    //	Dt_RECORD_Envm_SRRGenObjList_t* pSSRObjList = NULL;   //SSRObjList
    DIMInputGeneric_t* ps_DimInput =
        Rte_IWriteRef_SWCNorm_RunEntNorm_PPortFctDimInputGeneric_DEPFctDimInputGeneric();  //待修改
    Dt_RECORD_VLC_SenAccOOI_t* pACCOOIObjData =
        &Dt_RECORD_VLC_SenAccOOI_buf;  //待修改
    Dt_RECORD_Envm_GenObjectList* pGenObjectList =
        &Dt_RECORD_Envm_GenObjectList_buf;  //待修改
    Dt_RECORD_Envm_CRObjectList* pCRObjectList =
        &Dt_RECORD_Envm_CRObjectList_buf;  //待修改
    CamObjectList* pCamObject = Rte_IWriteRef_SWCNorm_CamObjectList();  //待修改

    ps_VehPar = Rte_IWriteRef_SWCNorm_RunEntNorm_PPortVehPar_DEPVehPar();

    // Rte_IRead_SWCVdyAdapt_RunEntVdyMain_RPortVehPar_DEPVehPar(ps_VehPar);
    // //待修改
    Rte_Read_PpVED_VehDyn_t_info_DeVED_VehDyn_t_buf(ps_VehDyn);  // VehDyn
    //	Rte_Read_PpEnvm_SRRGenObjList_t_info_DeEnvm_SRRGenObjList_t_buf(pSSRObjList);
    ////SSRObjList
    ps_DimInput =
        Rte_IWriteRef_SWCNorm_RunEntNorm_PPortFctDimInputGeneric_DEPFctDimInputGeneric();  //待修改
    // ACC Input;  //待修改
    Rte_Read_PpVLC_SenAccOOI_t_info_DeVLC_SenAccOOI_t_buf(pACCOOIObjData);
    Rte_Read_PpVLC_SenAccOOI_t_info_DeVLC_SenAccOOI_t_buf(pGenObjectList);
    //	Rte_IWriteRef_SWCEnvmAdapt_RunEntEnvmMain_PPortGenObjectList_DEPGenObjectList(pGenObjectList);
    ////待修改
    Rte_Read_PpEnvm_CRObjectList_t_buf_DeEnvm_CRObjectList_buf(
        pCRObjectList);  //待修改
    // Rte_Camera_Infomation_Object_Addr(pCamObject);//待修改

    pCamObject = Rte_IWriteRef_SWCNorm_CamObjectList();

    memset(g_LCFSEN_reqPorts, 0, sizeof(reqLcfSenPrtList_t));

    // calculate cycle time
    static uint32 uiTimeStampLastCycle = 0u;
    uint32 uiCurrentCycleTime_us = 0u;
    uint32 u32globalTimeStamp = Norm_SYS_GetCurrentTime();  //待确认

    if (uiTimeStampLastCycle == 0u) {
        uiCurrentCycleTime_us = TASK_CYCLE_TIME_50 * 1000000;
    } else {
        if (u32globalTimeStamp >= uiTimeStampLastCycle) {
            uiCurrentCycleTime_us = (u32globalTimeStamp - uiTimeStampLastCycle);
        } else {
            uiCurrentCycleTime_us =
                (0xFFFFFFFF - uiTimeStampLastCycle + u32globalTimeStamp);
        }
    }

    uiCurrentCycleTime_us = MAX(0, MIN(1000000, (uiCurrentCycleTime_us)));
    uiTimeStampLastCycle = u32globalTimeStamp;

    // base control data input wrapper
    g_LCFSEN_reqPorts->sBaseCtrlData.bLDPSwitchEn = g_VCU_LDPSwitchEn_B;
    g_LCFSEN_reqPorts->sBaseCtrlData.bLDWSwitchEn = g_VCU_LDWSwitchEn_B;
    g_LCFSEN_reqPorts->sBaseCtrlData.eOpMode = BASE_OM_RUN;
    g_LCFSEN_reqPorts->sBaseCtrlData.fSystemSenCylceTime_sec =
        ((float32)uiCurrentCycleTime_us) / 1000000.f;
    g_LCFSEN_reqPorts->sBaseCtrlData.uiVersionNumber =
        LCF_SEN_FRAMEWORK_VERSION;
    g_LCFSEN_reqPorts->sBaseCtrlData.uLDPMode_nu = g_VCU_LDPMod_St;
    g_LCFSEN_reqPorts->sBaseCtrlData.uLDWMode_nu = g_VCU_LDWMod_St;

    LcfSenSetSigHeader(&(g_LCFSEN_reqPorts->sBaseCtrlData.sSigHeader),
                       ps_VehDyn->sSigHeader.eSigStatus,
                       ps_VehDyn->sSigHeader.uiTimeStamp,
                       ps_VehDyn->sSigHeader.uiMeasurementCounter,
                       ps_VehDyn->sSigHeader.uiCycleCounter);

    // ACC OOI data input wrapper
    LcfSenSetSigHeader(&(g_LCFSEN_reqPorts->sLCFACCOOIData.sSigHeader),
                       pACCOOIObjData->sSigHeader.eSigStatus,
                       pACCOOIObjData->sSigHeader.uiTimeStamp,
                       pACCOOIObjData->sSigHeader.uiMeasurementCounter,
                       pACCOOIObjData->sSigHeader.uiCycleCounter);

    sint8 siACCOOIObjId = pACCOOIObjData->AccOOINextLong.Attributes.uiObjectID;

    if (siACCOOIObjId >= 0 &&
        pACCOOIObjData->sSigHeader.eSigStatus == AL_SIG_STATE_OK) {
        g_LCFSEN_reqPorts->sLCFACCOOIData.sACCOOIData.bObjectDetected_bool =
            TRUE;
        g_LCFSEN_reqPorts->sLCFACCOOIData.sACCOOIData.fObjPosXStdDev_met =
            pGenObjectList->aObject[siACCOOIObjId].Kinematic.fDistXStd;
        g_LCFSEN_reqPorts->sLCFACCOOIData.sACCOOIData.fObjPosX_met =
            pACCOOIObjData->AccOOINextLong.Kinematic.fDistX;
        g_LCFSEN_reqPorts->sLCFACCOOIData.sACCOOIData.fObjPosYStdDev_met =
            pGenObjectList->aObject[siACCOOIObjId].Kinematic.fDistYStd;
        g_LCFSEN_reqPorts->sLCFACCOOIData.sACCOOIData.fObjPosY_met =
            pACCOOIObjData->AccOOINextLong.Kinematic.fDistY;
        g_LCFSEN_reqPorts->sLCFACCOOIData.sACCOOIData.fObjRelAclX_mps2 =
            pACCOOIObjData->AccOOINextLong.Kinematic.fArelX;
        g_LCFSEN_reqPorts->sLCFACCOOIData.sACCOOIData.fObjRelAclY_mps2 =
            pACCOOIObjData->AccOOINextLong.Kinematic.fArelY;
        g_LCFSEN_reqPorts->sLCFACCOOIData.sACCOOIData.fObjRelHeadingAngle_rad =
            pCRObjectList->aObject[siACCOOIObjId].Geometry.fOrientation;
        g_LCFSEN_reqPorts->sLCFACCOOIData.sACCOOIData.fObjRelVelX_mps =
            pACCOOIObjData->AccOOINextLong.Kinematic.fVabsX;
        g_LCFSEN_reqPorts->sLCFACCOOIData.sACCOOIData.fObjRelVelY_mps =
            pACCOOIObjData->AccOOINextLong.Kinematic.fVabsY;
        g_LCFSEN_reqPorts->sLCFACCOOIData.sACCOOIData.fObjWidth_met =
            pCRObjectList->aObject[siACCOOIObjId].Geometry.fWidth;
        g_LCFSEN_reqPorts->sLCFACCOOIData.sACCOOIData.uiObjClassType_nu =
            pGenObjectList->aObject[siACCOOIObjId].Attributes.eClassification;
        g_LCFSEN_reqPorts->sLCFACCOOIData.sACCOOIData.uiObjID_nu =
            siACCOOIObjId;
        g_LCFSEN_reqPorts->sLCFACCOOIData.sACCOOIData.uiObjMeasState_nu =
            pGenObjectList->aObject[siACCOOIObjId].General.eMaintenanceState;
        g_LCFSEN_reqPorts->sLCFACCOOIData.sACCOOIData.uiObjQuality_perc =
            pGenObjectList->aObject[siACCOOIObjId].Qualifiers.uiAccObjQuality;
        g_LCFSEN_reqPorts->sLCFACCOOIData.sACCOOIData.uiObjSensorSource_btf =
            FOP_FUSION_RADAR_FRONT_NU;
        g_LCFSEN_reqPorts->sLCFACCOOIData.sACCOOIData.uiObjTimeStamp_usec =
            pACCOOIObjData->sSigHeader.uiTimeStamp;
    }

    // lane data input wrapper
    LcfSenSetSigHeader(&(g_LCFSEN_reqPorts->sLCFLaneData.sSigHeader),
                       pCamObject->sSigHeader.eSigStatus,
                       pCamObject->sSigHeader.uiTimeStamp,
                       pCamObject->sSigHeader.uiMeasurementCounter,
                       pCamObject->sSigHeader.uiCycleCounter);

    if (pCamObject->sSigHeader.eSigStatus == AL_SIG_STATE_OK) {
        for (uint8 i = 0; i < LCF_SEN_CAMERA_LANE_NUM; i++) {
            g_LCFSEN_reqPorts->sLCFLaneData.abAvailable[i] =
                pCamObject->sLaneLines[i].Quality > 0;
            g_LCFSEN_reqPorts->sLCFLaneData.afCurvatureChange_1pm2[i] =
                pCamObject->sLaneLines[i].CurvatureDer;
            g_LCFSEN_reqPorts->sLCFLaneData.afCurvature_1pm[i] =
                pCamObject->sLaneLines[i].Curvature;
            g_LCFSEN_reqPorts->sLCFLaneData.afEventDistance_met[i] = 0.f;
            g_LCFSEN_reqPorts->sLCFLaneData.afHeadingAngle_rad[i] =
                pCamObject->sLaneLines[i].HeadingAngle;
            g_LCFSEN_reqPorts->sLCFLaneData.afPosY0_met[i] =
                pCamObject->sLaneLines[i].Position;
            g_LCFSEN_reqPorts->sLCFLaneData.afStdDevCurvatureChange_1pm2[i] =
                0.f;
            g_LCFSEN_reqPorts->sLCFLaneData.afStdDevCurvature_1pm[i] = 0.f;
            g_LCFSEN_reqPorts->sLCFLaneData.afStdDevHeadingAngle_rad[i] = 0.f;
            g_LCFSEN_reqPorts->sLCFLaneData.afStdDevPosY0_met[i] = 0.f;
            g_LCFSEN_reqPorts->sLCFLaneData.afValidLength_met[i] =
                pCamObject->sLaneLines[i].ViewRangeEnd;
            g_LCFSEN_reqPorts->sLCFLaneData.auColor_nu[i] = 1u;
            g_LCFSEN_reqPorts->sLCFLaneData.auEventQuality_nu[i] = 0u;
            g_LCFSEN_reqPorts->sLCFLaneData.auEventType_nu[i] = 0u;
            g_LCFSEN_reqPorts->sLCFLaneData.auMarkerType_nu[i] = 1u;
            g_LCFSEN_reqPorts->sLCFLaneData.auQuality_nu[i] =
                pCamObject->sLaneLines[i].Quality;
        }
        g_LCFSEN_reqPorts->sLCFLaneData.uiCompState_nu = 0u;
        g_LCFSEN_reqPorts->sLCFLaneData.bLDPControlEnable =
            g_LCFSEN_reqPorts->sBaseCtrlData.bLDPSwitchEn;
        g_LCFSEN_reqPorts->sLCFLaneData.bLDPLeControlEnable =
            g_LCFSEN_reqPorts->sBaseCtrlData.bLDPSwitchEn;
        g_LCFSEN_reqPorts->sLCFLaneData.bLDPRiControlEnable =
            g_LCFSEN_reqPorts->sBaseCtrlData.bLDPSwitchEn;
        g_LCFSEN_reqPorts->sLCFLaneData.bLDWControlEnable =
            g_LCFSEN_reqPorts->sBaseCtrlData.bLDWSwitchEn;
        g_LCFSEN_reqPorts->sLCFLaneData.bLDWLeControlEnable =
            g_LCFSEN_reqPorts->sBaseCtrlData.bLDWSwitchEn;
        g_LCFSEN_reqPorts->sLCFLaneData.bLDWRiControlEnable =
            g_LCFSEN_reqPorts->sBaseCtrlData.bLDWSwitchEn;
        g_LCFSEN_reqPorts->sLCFLaneData.bParallelModelActiv = FALSE;
        g_LCFSEN_reqPorts->sLCFLaneData.fSineWaveDtct_nu = FALSE;
        g_LCFSEN_reqPorts->sLCFLaneData.fVertSlopeChange_nu = 0.f;

        g_LCFSEN_reqPorts->sLCFLaneData.uiCamStuatuQualifier_nu = 0u;
        g_LCFSEN_reqPorts->sLCFLaneData.uLaneChange_nu = 0u;
        g_LCFSEN_reqPorts->sLCFLaneData.uLeftIndex_nu = 0u;
        g_LCFSEN_reqPorts->sLCFLaneData.uRightIndex_nu = 0u;
        g_LCFSEN_reqPorts->sLCFLaneData.uRoadWorks_btf = 0u;
        g_LCFSEN_reqPorts->sLCFLaneData.uWeatherCond_nu = 0u;
    }

    // ego dynamic data input wrapper
    LcfSenSetSigHeader(&(g_LCFSEN_reqPorts->sLCFVehDyn.sSigHeader),
                       ps_VehDyn->sSigHeader.eSigStatus,
                       ps_VehDyn->sSigHeader.uiTimeStamp,
                       ps_VehDyn->sSigHeader.uiMeasurementCounter,
                       ps_VehDyn->sSigHeader.uiCycleCounter);

    if (ps_VehDyn->sSigHeader.eSigStatus == AL_SIG_STATE_OK) {
        g_LCFSEN_reqPorts->sLCFVehDyn.fEgoAccelX_mpss =
            ps_VehDyn->Longitudinal.AccelCorr.corrAccel;
        g_LCFSEN_reqPorts->sLCFVehDyn.fEgoAccelY_mpss =
            ps_VehDyn->Lateral.Accel.LatAccel;
        g_LCFSEN_reqPorts->sLCFVehDyn.fEgoCurve_1pm =
            ps_VehDyn->Lateral.Curve.Curve;
        g_LCFSEN_reqPorts->sLCFVehDyn.fEgoVehYawRateStd_rpss =
            TUE_CML_SqrtApprox(ps_VehDyn->Lateral.YawRate.Variance);
        g_LCFSEN_reqPorts->sLCFVehDyn.fEgoVelX_mps =
            ps_VehDyn->Longitudinal.VeloCorr.corrVelo;
        g_LCFSEN_reqPorts->sLCFVehDyn.fEgoYawRate_rps =
            ps_VehDyn->Lateral.YawRate.YawRate;
        g_LCFSEN_reqPorts->sLCFVehDyn.fFrontAxleSteerAgl_rad =
            ps_VehDyn->Lateral.OffCompStWheelAngle;
        g_LCFSEN_reqPorts->sLCFVehDyn.fManuActuTrqEPS_nm = 0.f;
        g_LCFSEN_reqPorts->sLCFVehDyn.fSteerWheelAglChng_rps =
            TUE_DEG2RAD(ps_DimInput->fSteeringWheelAngleGrad);
        g_LCFSEN_reqPorts->sLCFVehDyn.fSteerWheelAgl_rad =
            TUE_DEG2RAD(ps_DimInput->fSteeringWheelAngle);
        g_LCFSEN_reqPorts->sLCFVehDyn.uEgoMotionState_nu =
            ps_VehDyn->MotionState.MotState;
    }

    // veh signal input wrapper
    LcfSenSetSigHeader(&(g_LCFSEN_reqPorts->sLCFVehSig.sSigHeader),
                       ps_VehDyn->sSigHeader.eSigStatus,
                       ps_VehDyn->sSigHeader.uiTimeStamp,
                       ps_VehDyn->sSigHeader.uiMeasurementCounter,
                       ps_VehDyn->sSigHeader.uiCycleCounter);

    if (ps_VehDyn->sSigHeader.eSigStatus == AL_SIG_STATE_OK &&
        ps_DimInput->sSigHeader.eSigStatus == AL_SIG_STATE_OK) {
        g_LCFSEN_reqPorts->sLCFVehSig.bCtrlStAvlbABS = TRUE;
        g_LCFSEN_reqPorts->sLCFVehSig.bCtrlStAvlbACC = TRUE;
        g_LCFSEN_reqPorts->sLCFVehSig.bCtrlStAvlbEBA = FALSE;
        g_LCFSEN_reqPorts->sLCFVehSig.bCtrlStAvlbESC = TRUE;
        g_LCFSEN_reqPorts->sLCFVehSig.bCtrlStAvlbTCS = TRUE;
        g_LCFSEN_reqPorts->sLCFVehSig.bCtrlStAvlbVSM = TRUE;
        g_LCFSEN_reqPorts->sLCFVehSig.bCtrlStEnABS =
            ps_DimInput->eDriverBraking;
        g_LCFSEN_reqPorts->sLCFVehSig.bCtrlStEnACC = FALSE;
        g_LCFSEN_reqPorts->sLCFVehSig.bCtrlStEnEBA = FALSE;
        g_LCFSEN_reqPorts->sLCFVehSig.bCtrlStEnESC = FALSE;
        g_LCFSEN_reqPorts->sLCFVehSig.bCtrlStEnTCS = FALSE;
        g_LCFSEN_reqPorts->sLCFVehSig.bCtrlStEnVSM = FALSE;
        g_LCFSEN_reqPorts->sLCFVehSig.bDoorOpen = FALSE;
        g_LCFSEN_reqPorts->sLCFVehSig.bDrvNoBuckledUp = FALSE;
        g_LCFSEN_reqPorts->sLCFVehSig.bDtctOverSte = FALSE;
        g_LCFSEN_reqPorts->sLCFVehSig.bDtctRollerBench = FALSE;
        g_LCFSEN_reqPorts->sLCFVehSig.bDtctUnderSte = FALSE;
        g_LCFSEN_reqPorts->sLCFVehSig.bGrNeu = FALSE;
        g_LCFSEN_reqPorts->sLCFVehSig.bGrPark = FALSE;
        g_LCFSEN_reqPorts->sLCFVehSig.bGrRvs = FALSE;
        g_LCFSEN_reqPorts->sLCFVehSig.bLDPErrCondition = FALSE;
        g_LCFSEN_reqPorts->sLCFVehSig.bLDWErrCondition = FALSE;
        g_LCFSEN_reqPorts->sLCFVehSig.bStErrABS = FALSE;
        g_LCFSEN_reqPorts->sLCFVehSig.bStErrACC = FALSE;
        g_LCFSEN_reqPorts->sLCFVehSig.bStErrEBA = FALSE;
        g_LCFSEN_reqPorts->sLCFVehSig.bStErrESC = FALSE;
        g_LCFSEN_reqPorts->sLCFVehSig.bStErrLatDMC = FALSE;
        g_LCFSEN_reqPorts->sLCFVehSig.bStErrTSC = FALSE;
        g_LCFSEN_reqPorts->sLCFVehSig.bStErrVDY = FALSE;
        g_LCFSEN_reqPorts->sLCFVehSig.bStErrVSM = FALSE;
        g_LCFSEN_reqPorts->sLCFVehSig.bTrailerExs = FALSE;
        g_LCFSEN_reqPorts->sLCFVehSig.bTrnSglEnLf =
            (ps_DimInput->eTurnIndicator == eTurnIndicator_Left);
        g_LCFSEN_reqPorts->sLCFVehSig.bTrnSglEnRi =
            (ps_DimInput->eTurnIndicator == eTurnIndicator_Right);
        g_LCFSEN_reqPorts->sLCFVehSig.bTrnSglHarLigEn = FALSE;
        g_LCFSEN_reqPorts->sLCFVehSig.bVehRdyToSta = TRUE;
        g_LCFSEN_reqPorts->sLCFVehSig.fAccPedPstn_Per =
            ps_DimInput->fAccelPedalPos;
        g_LCFSEN_reqPorts->sLCFVehSig.fSpeedometerVelocity_kmh =
            ps_VehDyn->Longitudinal.VeloCorr.corrVelo * 3.6f;

        g_LCFSEN_reqPorts->sLCFVehSig.uLCFAdasSysWarn_nu = FALSE;
        g_LCFSEN_reqPorts->sLCFVehSig.uStageWiper_St = FALSE;
        g_LCFSEN_reqPorts->sLCFVehSig.uStateWiper_St = FALSE;
        g_LCFSEN_reqPorts->sLCFVehSig.uStBrightness_nu = FALSE;
    }

// side radar object list
//	LcfSenSetSigHeader(&(g_LCFSEN_reqPorts->sSSRObjectList.sSigHeader),
//		pSSRObjList->sSigHeader.eSigStatus,
//		pSSRObjList->sSigHeader.uiTimeStamp,
//		pSSRObjList->sSigHeader.uiMeasurementCounter,
//		pSSRObjList->sSigHeader.uiCycleCounter);
//
//	if (pSSRObjList->sSigHeader.eSigStatus == AL_SIG_STATE_OK)
//	{
//		g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.uiNumOfObject =
// MIN(pSSRObjList->aRearLeftObjects.uiNumOfObjects +
//			pSSRObjList->aRearRightObjects.uiNumOfObjects,
// ILE_INPUT_RADAR_OBJECT_NUM);

//		uint8 uiInsertedObjNum = 0u;

//		//rear left radar object insert
//		for (uint8 i = 0; i <
// pSSRObjList->aRearLeftObjects.uiNumOfObjects
//&& uiInsertedObjNum <
// g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.uiNumOfObject; i++)
//		{
//			g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.ObjectArray[uiInsertedObjNum].fPosX_met
//= pSSRObjList->aRearLeftObjects.aObjects[i].sKinematic.fDistX_met;
//			g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.ObjectArray[uiInsertedObjNum].fPosY_met
//= pSSRObjList->aRearLeftObjects.aObjects[i].sKinematic.fDistY_met;
//			g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.ObjectArray[uiInsertedObjNum].fRelVelX_mps
//= pSSRObjList->aRearLeftObjects.aObjects[i].sKinematic.fVrelX_mps;
//			g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.ObjectArray[uiInsertedObjNum].fRelVelY_mps
//= pSSRObjList->aRearLeftObjects.aObjects[i].sKinematic.fVrelY_mps;
//			g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.ObjectArray[uiInsertedObjNum].fShapePoint0PosX_met
//= pSSRObjList->aRearLeftObjects.aObjects[i].sGeometry.fLengthFront_met;
//			g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.ObjectArray[uiInsertedObjNum].fShapePoint0PosY_met
//= pSSRObjList->aRearLeftObjects.aObjects[i].sGeometry.fWidthLeft_met;
//			g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.ObjectArray[uiInsertedObjNum].fShapePoint1PosX_met
//= pSSRObjList->aRearLeftObjects.aObjects[i].sGeometry.fLengthFront_met;
//			g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.ObjectArray[uiInsertedObjNum].fShapePoint1PosY_met
//= -pSSRObjList->aRearLeftObjects.aObjects[i].sGeometry.fWidthRight_met;
//			g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.ObjectArray[uiInsertedObjNum].fShapePoint2PosX_met
//= -pSSRObjList->aRearLeftObjects.aObjects[i].sGeometry.fLengthRear_met;
//			g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.ObjectArray[uiInsertedObjNum].fShapePoint2PosY_met
//= -pSSRObjList->aRearLeftObjects.aObjects[i].sGeometry.fWidthRight_met;
//			g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.ObjectArray[uiInsertedObjNum].fShapePoint3PosX_met
//= -pSSRObjList->aRearLeftObjects.aObjects[i].sGeometry.fLengthRear_met;
//			g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.ObjectArray[uiInsertedObjNum].fShapePoint3PosY_met
//= pSSRObjList->aRearLeftObjects.aObjects[i].sGeometry.fWidthLeft_met;
//			g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.ObjectArray[uiInsertedObjNum].uiExistProb_nu
//=
//(uint8)(pSSRObjList->aRearLeftObjects.aObjects[i].sQualifier.fProbabilityOfExistence_per
//* 100.f);
//			g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.ObjectArray[uiInsertedObjNum].uiShapePoint0State_nu
//= TRUE;
//			g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.ObjectArray[uiInsertedObjNum].uiShapePoint1State_nu
//= TRUE;
//			g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.ObjectArray[uiInsertedObjNum].uiShapePoint2State_nu
//= TRUE;
//			g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.ObjectArray[uiInsertedObjNum].uiShapePoint3State_nu
//= TRUE;
//			g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.ObjectArray[uiInsertedObjNum].uiID_nu
//= pSSRObjList->aRearLeftObjects.aObjects[i].sGeneral.uiID_nu;
//			uiInsertedObjNum++;
//		}
//		//rear right radar object insert
//		for (uint8 i = 0; i <
// pSSRObjList->aRearRightObjects.uiNumOfObjects && uiInsertedObjNum <
// g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.uiNumOfObject; i++)
//		{
//			g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.ObjectArray[uiInsertedObjNum].fPosX_met
//= pSSRObjList->aRearRightObjects.aObjects[i].sKinematic.fDistX_met;
//			g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.ObjectArray[uiInsertedObjNum].fPosY_met
//= pSSRObjList->aRearRightObjects.aObjects[i].sKinematic.fDistY_met;
//			g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.ObjectArray[uiInsertedObjNum].fRelVelX_mps
//= pSSRObjList->aRearRightObjects.aObjects[i].sKinematic.fVrelX_mps;
//			g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.ObjectArray[uiInsertedObjNum].fRelVelY_mps
//= pSSRObjList->aRearRightObjects.aObjects[i].sKinematic.fVrelY_mps;
//			g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.ObjectArray[uiInsertedObjNum].fShapePoint0PosX_met
//= pSSRObjList->aRearRightObjects.aObjects[i].sGeometry.fLengthFront_met;
//			g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.ObjectArray[uiInsertedObjNum].fShapePoint0PosY_met
//= pSSRObjList->aRearRightObjects.aObjects[i].sGeometry.fWidthLeft_met;
//			g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.ObjectArray[uiInsertedObjNum].fShapePoint1PosX_met
//= pSSRObjList->aRearRightObjects.aObjects[i].sGeometry.fLengthFront_met;
//			g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.ObjectArray[uiInsertedObjNum].fShapePoint1PosY_met
//= -pSSRObjList->aRearRightObjects.aObjects[i].sGeometry.fWidthRight_met;
//			g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.ObjectArray[uiInsertedObjNum].fShapePoint2PosX_met
//= -pSSRObjList->aRearRightObjects.aObjects[i].sGeometry.fLengthRear_met;
//			g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.ObjectArray[uiInsertedObjNum].fShapePoint2PosY_met
//= -pSSRObjList->aRearRightObjects.aObjects[i].sGeometry.fWidthRight_met;
//			g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.ObjectArray[uiInsertedObjNum].fShapePoint3PosX_met
//= -pSSRObjList->aRearRightObjects.aObjects[i].sGeometry.fLengthRear_met;
//			g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.ObjectArray[uiInsertedObjNum].fShapePoint3PosY_met
//= pSSRObjList->aRearRightObjects.aObjects[i].sGeometry.fWidthLeft_met;
//			g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.ObjectArray[uiInsertedObjNum].uiExistProb_nu
//=
//(uint8)(pSSRObjList->aRearRightObjects.aObjects[i].sQualifier.fProbabilityOfExistence_per
//* 100.f);
//			g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.ObjectArray[uiInsertedObjNum].uiShapePoint0State_nu
//= TRUE;
//			g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.ObjectArray[uiInsertedObjNum].uiShapePoint1State_nu
//= TRUE;
//			g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.ObjectArray[uiInsertedObjNum].uiShapePoint2State_nu
//= TRUE;
//			g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.ObjectArray[uiInsertedObjNum].uiShapePoint3State_nu
//= TRUE;
//			g_LCFSEN_reqPorts->sSSRObjectList.sSSRObjList.ObjectArray[uiInsertedObjNum].uiID_nu
//= pSSRObjList->aRearRightObjects.aObjects[i].sGeneral.uiID_nu;
//			uiInsertedObjNum++;
//		}
//	}
#endif
}

/*****************************************************************************
Functionname:LcfRTESenOutputWrapper */ /*!

@brief: set LCF SEN output pointer based on the value of RTE signals

@description: set LCF SEN output pointer based on the value of RTE signals

@param[in/out]
                     proLcfSenPrtList_t* pLcfSenOutput
@param[out]

@return
*****************************************************************************/
#ifndef LCF_SEN_DEBUG
Dt_RECORD_LCF_SenGenericOutputs_t Dt_RECORD_LCF_SenGenericOutputs_buf2;
Dt_RECORD_LCF_SenGenericOutputs_t LCF_SenGenericOutputs_buf;
uint8 LCF_VEH_Temp_LDPSC_SysOut_St;
uint8 LCF_VEH_Temp_LDPSC_DgrSide_St;
float32 LCF_VEH_Temp_LDPDT_LnCltdCurvLf_ReMi;
float32 LCF_VEH_Temp_LDPDT_LnHeadLf_Rad;
float32 LCF_VEH_Temp_LDPDT_LnCltdCurvRi_ReMi;
float32 LCF_VEH_Temp_LDPDT_LnHeadRi_Rad;
float32 LCF_VEH_Temp_LDPDT_LnPstnLf_Mi;
float32 LCF_VEH_Temp_LDPDT_LnPstnRi_Mi;
#endif

void LcfRTESenOutputWrapper(proLcfSenPrtList_t* pLcfSenOutput) {
    // static uint8 cnt = 0;

#ifndef LCF_SEN_DEBUG

    Dt_RECORD_LCF_SenGenericOutputs_t* psLCF_SenGenericOutputs =
        &LCF_SenGenericOutputs_buf;
    //	static Dt_RECORD_LCF_SenToVeh_t sLCF_SenToVeh;
    Dt_RECORD_LCF_SenAlgoCompState_t LCF_SenAlgoCompState_buf;

    Dt_RECORD_LCF_SenAlgoCompState_t* psSenAlgoCompState =
        &LCF_SenAlgoCompState_buf;

    // Rte_Write_CtApLCFSEN_PpLCF_SenGenericOutputs_t_info_DeLCF_SenGenericOutputs_t_buf(psLCF_SenGenericOutputs);

    // SEN State return values of the algo component
    memcpy(psSenAlgoCompState->AlgoVersionInfo,
           pLcfSenOutput->pAlgoCompState.AlgoVersionInfo,
           (size_t)sizeof(pLcfSenOutput->pAlgoCompState.AlgoVersionInfo));

    cnt++;

    psSenAlgoCompState->eCompState = pLcfSenOutput->pAlgoCompState.eCompState;
    psSenAlgoCompState->eErrorCode = pLcfSenOutput->pAlgoCompState.eErrorCode;
    psSenAlgoCompState->eGenAlgoQualifier =
        pLcfSenOutput->pAlgoCompState.eGenAlgoQualifier;
    psSenAlgoCompState->eShedulerSubModeRequest =
        pLcfSenOutput->pAlgoCompState.eShedulerSubModeRequest;

    psSenAlgoCompState->sSigHeader.eSigStatus =
        pLcfSenOutput->pAlgoCompState.sSigHeader.eSigStatus;
    psSenAlgoCompState->sSigHeader.uiCycleCounter =
        pLcfSenOutput->pAlgoCompState.sSigHeader.uiCycleCounter;
    psSenAlgoCompState->sSigHeader.uiMeasurementCounter =
        pLcfSenOutput->pAlgoCompState.sSigHeader.uiMeasurementCounter;
    psSenAlgoCompState->sSigHeader.uiTimeStamp =
        pLcfSenOutput->pAlgoCompState.sSigHeader.uiTimeStamp;

    psSenAlgoCompState->uiAlgoVersionNumber =
        pLcfSenOutput->pAlgoCompState.uiAlgoVersionNumber;
    psSenAlgoCompState->uiApplicationNumber =
        pLcfSenOutput->pAlgoCompState.uiApplicationNumber;
    psSenAlgoCompState->uiVersionNumber =
        pLcfSenOutput->pAlgoCompState.uiVersionNumber;

    // LDPSA output
    psLCF_SenGenericOutputs->sSigHeader.eSigStatus =
        pLcfSenOutput->pLcfSenOutputData.sSigHeader.eSigStatus;
    psLCF_SenGenericOutputs->sSigHeader.uiCycleCounter =
        pLcfSenOutput->pLcfSenOutputData.sSigHeader.uiCycleCounter;
    psLCF_SenGenericOutputs->sSigHeader.uiMeasurementCounter =
        pLcfSenOutput->pLcfSenOutputData.sSigHeader.uiMeasurementCounter;
    psLCF_SenGenericOutputs->sSigHeader.uiTimeStamp =
        pLcfSenOutput->pLcfSenOutputData.sSigHeader.uiTimeStamp;

    psLCF_SenGenericOutputs->uiVersionNumber =
        pLcfSenOutput->pLcfSenOutputData.uiVersionNumber;

    psLCF_SenGenericOutputs->sLDPSAOutput.LDPDT_LnCltdCurvLf_ReMi =
        pLcfSenOutput->pLcfSenOutputData.sLDPSAOutput.LDPDT_LnCltdCurvLf_ReMi;
    psLCF_SenGenericOutputs->sLDPSAOutput.LDPDT_LnCltdCurvRi_ReMi =
        pLcfSenOutput->pLcfSenOutputData.sLDPSAOutput.LDPDT_LnCltdCurvRi_ReMi;
    psLCF_SenGenericOutputs->sLDPSAOutput.LDPDT_LnHeadLf_Rad =
        pLcfSenOutput->pLcfSenOutputData.sLDPSAOutput.LDPDT_LnHeadLf_Rad;
    psLCF_SenGenericOutputs->sLDPSAOutput.LDPDT_LnHeadRi_Rad =
        pLcfSenOutput->pLcfSenOutputData.sLDPSAOutput.LDPDT_LnHeadRi_Rad;
    psLCF_SenGenericOutputs->sLDPSAOutput.LDPDT_LnPstnLf_Mi =
        pLcfSenOutput->pLcfSenOutputData.sLDPSAOutput.LDPDT_LnPstnLf_Mi;
    psLCF_SenGenericOutputs->sLDPSAOutput.LDPDT_LnPstnRi_Mi =
        pLcfSenOutput->pLcfSenOutputData.sLDPSAOutput.LDPDT_LnPstnRi_Mi;
    psLCF_SenGenericOutputs->sLDPSAOutput.LDPSC_DgrSide_St =
        pLcfSenOutput->pLcfSenOutputData.sLDPSAOutput.LDPSC_DgrSide_St;
    psLCF_SenGenericOutputs->sLDPSAOutput.LDPSC_RdyToTrig_B =
        pLcfSenOutput->pLcfSenOutputData.sLDPSAOutput.LDPSC_RdyToTrig_B;
    psLCF_SenGenericOutputs->sLDPSAOutput.LDPSC_StateLf_St =
        pLcfSenOutput->pLcfSenOutputData.sLDPSAOutput.LDPSC_StateLf_St;
    psLCF_SenGenericOutputs->sLDPSAOutput.LDPSC_StateRi_St =
        pLcfSenOutput->pLcfSenOutputData.sLDPSAOutput.LDPSC_StateRi_St;
    psLCF_SenGenericOutputs->sLDPSAOutput.LDPSC_SysOut_St =
        pLcfSenOutput->pLcfSenOutputData.sLDPSAOutput.LDPSC_SysOut_St;
    psLCF_SenGenericOutputs->sLDPSAOutput.LDWC_DgrSide_St =
        pLcfSenOutput->pLcfSenOutputData.sLDPSAOutput.LDWC_DgrSide_St;
    psLCF_SenGenericOutputs->sLDPSAOutput.LDWC_RdyToTrig_B =
        pLcfSenOutput->pLcfSenOutputData.sLDPSAOutput.LDWC_RdyToTrig_B;
    psLCF_SenGenericOutputs->sLDPSAOutput.LDWC_StateLf_St =
        pLcfSenOutput->pLcfSenOutputData.sLDPSAOutput.LDWC_StateLf_St;
    psLCF_SenGenericOutputs->sLDPSAOutput.LDWC_StateRi_St =
        pLcfSenOutput->pLcfSenOutputData.sLDPSAOutput.LDWC_StateRi_St;
    psLCF_SenGenericOutputs->sLDPSAOutput.LDWC_SysOut_St =
        pLcfSenOutput->pLcfSenOutputData.sLDPSAOutput.LDWC_SysOut_St;

#if 0
	Dt_RECORD_LCF_SenGenericOutputs_buf2.sLDPSAOutput.LDPSC_SysOut_St = pLcfSenOutput->pLcfSenOutputData.sLDPSAOutput.LDPSC_SysOut_St;
	Dt_RECORD_LCF_SenGenericOutputs_buf2.sLDPSAOutput.LDPSC_DgrSide_St = pLcfSenOutput->pLcfSenOutputData.sLDPSAOutput.LDPSC_DgrSide_St;
	Dt_RECORD_LCF_SenGenericOutputs_buf2.sLDPSAOutput.LDPDT_LnCltdCurvLf_ReMi = pLcfSenOutput->pLcfSenOutputData.sLDPSAOutput.LDPDT_LnCltdCurvLf_ReMi;
	Dt_RECORD_LCF_SenGenericOutputs_buf2.sLDPSAOutput.LDPDT_LnHeadLf_Rad = pLcfSenOutput->pLcfSenOutputData.sLDPSAOutput.LDPDT_LnHeadLf_Rad;
	Dt_RECORD_LCF_SenGenericOutputs_buf2.sLDPSAOutput.LDPDT_LnCltdCurvRi_ReMi = pLcfSenOutput->pLcfSenOutputData.sLDPSAOutput.LDPDT_LnCltdCurvRi_ReMi;
	Dt_RECORD_LCF_SenGenericOutputs_buf2.sLDPSAOutput.LDPDT_LnHeadRi_Rad = pLcfSenOutput->pLcfSenOutputData.sLDPSAOutput.LDPDT_LnHeadRi_Rad;
	Dt_RECORD_LCF_SenGenericOutputs_buf2.sLDPSAOutput.LDPDT_LnPstnLf_Mi = pLcfSenOutput->pLcfSenOutputData.sLDPSAOutput.LDPDT_LnPstnLf_Mi;
	Dt_RECORD_LCF_SenGenericOutputs_buf2.sLDPSAOutput.LDPDT_LnPstnRi_Mi = pLcfSenOutput->pLcfSenOutputData.sLDPSAOutput.LDPDT_LnPstnRi_Mi;
	Dt_RECORD_LCF_SenGenericOutputs_buf2.sLDPSAOutput.LDWC_DgrSide_St =pLcfSenOutput->pLcfSenOutputData.sLDPSAOutput.LDWC_DgrSide_St;
#endif

    LCF_VEH_Temp_LDPSC_SysOut_St =
        pLcfSenOutput->pLcfSenOutputData.sLDPSAOutput.LDPSC_SysOut_St;
    LCF_VEH_Temp_LDPSC_DgrSide_St =
        pLcfSenOutput->pLcfSenOutputData.sLDPSAOutput.LDPSC_DgrSide_St;
    LCF_VEH_Temp_LDPDT_LnCltdCurvLf_ReMi =
        pLcfSenOutput->pLcfSenOutputData.sLDPSAOutput.LDPDT_LnCltdCurvLf_ReMi;
    LCF_VEH_Temp_LDPDT_LnHeadLf_Rad =
        pLcfSenOutput->pLcfSenOutputData.sLDPSAOutput.LDPDT_LnHeadLf_Rad;
    LCF_VEH_Temp_LDPDT_LnCltdCurvRi_ReMi =
        pLcfSenOutput->pLcfSenOutputData.sLDPSAOutput.LDPDT_LnCltdCurvRi_ReMi;
    LCF_VEH_Temp_LDPDT_LnHeadRi_Rad =
        pLcfSenOutput->pLcfSenOutputData.sLDPSAOutput.LDPDT_LnHeadRi_Rad;
    LCF_VEH_Temp_LDPDT_LnPstnLf_Mi =
        pLcfSenOutput->pLcfSenOutputData.sLDPSAOutput.LDPDT_LnPstnLf_Mi;
    LCF_VEH_Temp_LDPDT_LnPstnRi_Mi =
        pLcfSenOutput->pLcfSenOutputData.sLDPSAOutput.LDPDT_LnPstnRi_Mi;

    // LCF_SenGenericOutputs_buf.sLDPSAOutput.LDPDT_LnPstnRi_Mi = 1.2f;
    psLCF_SenGenericOutputs->sLDPSAOutput.LDWC_SysOut_St = 12;
    psLCF_SenGenericOutputs->sLDPSAOutput.LDPDT_LnPstnLf_Mi = 1.2f;

    Rte_Write_CtApLCFSEN_PpLCF_SenGenericOutputs_t_info_DeLCF_SenGenericOutputs_t_buf(
        psLCF_SenGenericOutputs);

    Rte_Read_CtApLCFVEH_PpLCF_SenGenericOutputs_t_info_DeLCF_SenGenericOutputs_t_buf(
        &Dt_RECORD_LCF_SenGenericOutputs_buf2);

    Rte_Write_CtApLCFSEN_PpLCF_SenAlgoCompState_t_info_DeLCF_SenAlgoCompState_t_buf(
        psSenAlgoCompState);

    // SEN to VEH
    // Rte_Write_CtApLCFSEN_PpLCF_SenToVeh_t_info_DeLCF_SenToVeh_t_buf(&sLCF_SenToVeh);
    // //待修改，Dt_RECORD_LCF_SenToVeh_t中缺少SEN to VEH的部分信号

#endif
}

/*****************************************************************************
  Functionname:LcfSenPortsCheck                                            */ /*!

     @brief: NULL pointer check and signal status check for used pointers

     @description: NULL pointer check and signal status check for used pointers

     @param[in]
     @param[out]

     @return
   *****************************************************************************/
static boolean LcfSenPortsCheck(reqLcfSenPrtList_t* pLcfSenInput,
                                reqLcfSenParams* pLcfSenParam,
                                proLcfSenPrtList_t* pLcfSenOutput,
                                reqLcfSenDebug_t* pLcFSenDebug) {
#ifndef LCF_SEN_DEBUG

    //	Dt_RECORD_Envm_SRRGenObjList_t* pSSRObjList = NULL;   //SSRObjList
    VehPar_t* ps_VehPar =
        Rte_IWriteRef_SWCNorm_RunEntNorm_PPortVehPar_DEPVehPar();  //待修改

    Dt_RECORD_VED_VehDyn_t* ps_VehDyn = &Dt_RECORD_VED_VehDyn_buf;  // VehDyn
    //	Dt_RECORD_Envm_SRRGenObjList_t* pSSRObjList = NULL;   //SSRObjList
    DIMInputGeneric_t* ps_DimInput =
        Rte_IWriteRef_SWCNorm_RunEntNorm_PPortFctDimInputGeneric_DEPFctDimInputGeneric();  //待修改

    Dt_RECORD_VLC_SenAccOOI_t* pACCOOIObjData =
        &Dt_RECORD_VLC_SenAccOOI_buf;  //待修改
    Dt_RECORD_Envm_GenObjectList* pGenObjectList =
        &Dt_RECORD_Envm_GenObjectList_buf;  //待修改
    Dt_RECORD_Envm_CRObjectList* pCRObjectList =
        &Dt_RECORD_Envm_CRObjectList_buf;  //待修改
    CamObjectList* pCamObject = Rte_IWriteRef_SWCNorm_CamObjectList();  //待修改

    ps_VehPar = Rte_IWriteRef_SWCNorm_RunEntNorm_PPortVehPar_DEPVehPar();

    // Rte_IRead_SWCVdyAdapt_RunEntVdyMain_RPortVehPar_DEPVehPar(ps_VehPar);
    // //待修改
    Rte_Read_PpVED_VehDyn_t_info_DeVED_VehDyn_t_buf(ps_VehDyn);  // VehDyn
    //	Rte_Read_PpEnvm_SRRGenObjList_t_info_DeEnvm_SRRGenObjList_t_buf(pSSRObjList);
    ////SSRObjList
    ps_DimInput =
        Rte_IWriteRef_SWCNorm_RunEntNorm_PPortFctDimInputGeneric_DEPFctDimInputGeneric();  //待修改
    // ACC Input;  //待修改
    Rte_Read_PpVLC_SenAccOOI_t_info_DeVLC_SenAccOOI_t_buf(pACCOOIObjData);
    Rte_Read_PpVLC_SenAccOOI_t_info_DeVLC_SenAccOOI_t_buf(pGenObjectList);
    //	Rte_IWriteRef_SWCEnvmAdapt_RunEntEnvmMain_PPortGenObjectList_DEPGenObjectList(pGenObjectList);
    ////待修改
    Rte_Read_PpEnvm_CRObjectList_t_buf_DeEnvm_CRObjectList_buf(
        pCRObjectList);  //待修改
    // Rte_Camera_Infomation_Object_Addr(pCamObject);//待修改

    pCamObject = Rte_IWriteRef_SWCNorm_CamObjectList();

    boolean bCheckResult = TRUE;
    if (pLcfSenInput == NULL || pLcfSenParam == NULL || pLcfSenOutput == NULL ||
        pLcFSenDebug == NULL) {
        bCheckResult = FALSE;
    }

    if (ps_VehDyn == NULL || ps_VehPar == NULL || pCamObject == NULL ||
        ps_DimInput == NULL || pACCOOIObjData == NULL ||
        pGenObjectList == NULL || pCRObjectList == NULL ||
        ps_VehPar->sSigHeader.eSigStatus != AL_SIG_STATE_OK) {
        bCheckResult = FALSE;
    }

    return bCheckResult;
#else
    return TRUE;
#endif
}

/*****************************************************************************
  Functionname:Rte_IRead_SWLCF_ProLcfSen_Out */ /*!

@brief: LCF output rte wrapper, other module could get LCF sen output data from
this interface

@description

@param[in]
@param[out]

@return
*****************************************************************************/
const proLcfSenPrtList_t* Rte_IRead_SWLCF_ProLcfSen_Out(void) {
    return &g_LCFSEN_proPorts;
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */