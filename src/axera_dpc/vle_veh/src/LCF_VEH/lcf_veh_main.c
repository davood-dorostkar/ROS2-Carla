/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "lcf_veh_main.h"
// #include "lcf_veh_ext.h"
// #ifdef UBUNTU_SYSTEM
// #include "Rte_CtApLCFVEH.h"
// #include "Rte_TypeNeedAdd.h"
// #endif
/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE2_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
static reqLcfVehPrtList_t g_LCFVEH_reqPorts = {0};
static reqLcfVehParams g_LCFVEH_reqParams = {0};
static proLcfVehPrtList_t g_LCFVEH_proPorts = {0};
static reqLcfVehDebug_t g_LCFVEH_proDebugs = {0};
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"

#define ASW_QM_CORE2_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
static boolean gLCFSEN_bFirstInitial;
static boolean gLCFVEH_bFirstInitial;
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"

// boolean ReadCurrent_bTorqueReq(void)
// {
//   return g_LCFVEH_proPorts.pLcfVehOutputData.sLCDOutput.bTorqueReq;
// }
//
// float32 ReadCurrent_fTorque(void)
// {
//	return g_LCFVEH_proPorts.pLcfVehOutputData.sLCDOutput.fTorque;
// }

#define LCF_VEH_DEBUG

#ifndef LCF_VEH_DEBUG

// function protypes used for compile, need to be deleted,待删除
// void
// Rte_Read_CtApLCFVEH_PpVED_VehDyn_t_info_DeVED_VehDyn_t_buf(Dt_RECORD_VED_VehDyn_t*
// ps_VehDyn) {}  //VehDyn
// void
// Rte_Read_CtApLCFVEH_PpLCF_SenToVeh_t_info_DeLCF_SenToVeh_t_buf(Dt_RECORD_LCF_SenToVeh_t*
// pSenToVeh) {}
// void
// Rte_Read_CtApLCFVEH_PpLCF_SenGenericOutputs_t_info_DeLCF_SenGenericOutputs_t_buf(Dt_RECORD_LCF_SenGenericOutputs_t*
// pSenGenOut) {}

#endif

/*****************************************************************************
  Functionname:  LCF_VehReset                                          */ /*!

              @brief: external reset function for system calling

              @description: the LCF module should reset while first initialize
                                            or system error happened

              @param[in]
              @param[out]

              @return
            *****************************************************************************/
void LCF_VehReset(void) {
    memset(&g_LCFVEH_reqPorts, 0, sizeof(reqLcfVehPrtList_t));
    memset(&g_LCFVEH_reqParams, 0, sizeof(reqLcfVehParams));
    memset(&g_LCFVEH_proPorts, 0, sizeof(proLcfVehPrtList_t));
    memset(&g_LCFVEH_proDebugs, 0, sizeof(reqLcfVehDebug_t));

    LcfVehSetParams(&g_LCFVEH_reqParams);
    LcfVehReset(&g_LCFVEH_reqParams);
}

/*****************************************************************************
Functionname:LCF_VehProcess                                            */ /*!

                    @brief: LCF main function whilc would be invoked by BSW

                    @description: LCF main function whilc would be invoked by
                  BSW

                    @param[in]
                    @param[out]

                    @return
                  *****************************************************************************/
#define SWC_LCFVEH_ONOFF 1
#if 0
uint32 LCF_VehProcess(void)
{
	uint32 uiRetCode = BASE_RETURN_ERROR;
	boolean bPortsCheckResult = FALSE;
	if(SWC_LCFVEH_ONOFF) {
		if (gLCFVEH_bFirstInitial)
		{
			//RTE and input check
			bPortsCheckResult = LcfVehPortsCheck(&g_LCFVEH_reqPorts, &g_LCFVEH_reqParams, &g_LCFVEH_proPorts, &g_LCFVEH_proDebugs);

			if (bPortsCheckResult)
			{
				//RTE to LCF input wrapper
				LcfRTEVehInputWrapper(&g_LCFVEH_reqPorts);
				LcfVehExec(&g_LCFVEH_reqPorts, &g_LCFVEH_reqParams, &g_LCFVEH_proPorts, &g_LCFVEH_proDebugs);
				LcfRTEVehOutputWrapper(&g_LCFVEH_proPorts);

				/* Runtime measurement is done via sections of code in "lcf_sen_service.h" */
				uiRetCode = BASE_RETURN_OK;
			}
			else
			{
				LCF_VehReset();
				uiRetCode = BASE_RETURN_ERROR;
			}
		}
		else
		{
			LCF_VehReset();
			gLCFVEH_bFirstInitial = TRUE;
			uiRetCode = BASE_RETURN_OK;
		}

		/*  in case error is returned, all provide ports are set to 0 */
		if (uiRetCode == BASE_RETURN_ERROR)
		{
			memset(&g_LCFVEH_proPorts, 0, sizeof(proLcfVehPrtList_t));
		}
		else {}
	}
	return uiRetCode;
}
#endif
/*****************************************************************************
  Functionname:LcfVehSetParams                                            */ /*!

        @brief: set system parameters based on RTE signal

        @description: set system parameters based on RTE signal

        @param[in/out]
                              reqLcfVehParams* LCFVehParams: the pointer of LCF
      Veh
      parameters
        @param[out]

        @return
      *****************************************************************************/
void LcfVehSetParams(reqLcfVehParams* LCFVehParams) {
#ifndef LCF_VEH_DEBUG

    VehPar_t* ps_VehPar =
        Rte_IWriteRef_SWCNorm_RunEntNorm_PPortVehPar_DEPVehPar();
    // Rte_IRead_SWCVdyAdapt_RunEntVdyMain_RPortVehPar_DEPVehPar(ps_VehPar);
    // //待修改

    if (ps_VehPar->sSigHeader.eSigStatus == AL_SIG_STATE_OK) {
        LCFVehParams->LCFVeh_Kf_fSysCycleTime_sec = 0.02f;
        LCFVehParams->LCFVeh_Kf_fEgoVehLength_met =
            ps_VehPar->VehParAdd.VehicleLength;
        LCFVehParams->LCFVeh_Kf_fEgoVehWidth_met = 1.84f;
        ps_VehPar->VehParAdd.VehicleWidth;

        LCFVehParams->LCFVeh_Kf_Mass_kg = ps_VehPar->VehParAdd.CurbWeight;
        LCFVehParams->LCFVeh_Kf_CrnStiffFr = ps_VehPar->VehParMain.FrCrnrStiff;
        LCFVehParams->LCFVeh_Kf_CrnStiffRe = ps_VehPar->VehParMain.ReCrnrStiff;

        /* check if distance to front/rear axle is in plausible range?*/
        LCFVehParams->LCFVeh_Kf_DistToRearAxle_met =
            ps_VehPar->VehParMain.WheelBase *
                ps_VehPar->VehParMain.AxisLoadDistr +
            0.1f;
        // LCFVehParams->LCFVeh_Kf_DistToFrontAxle_met =
        // ps_VehPar->VehParMain.WheelBase -
        // LCFVehParams->LCFVeh_Kf_DistToRearAxle_met;
        LCFVehParams->LCFVeh_Kf_SteeringRatio_nu =
            ps_VehPar->VehParMain.SteeringRatio.swa.rat[1];
        LCFVehParams->LCFVeh_Kf_DistToFrontAxle_met =
            ps_VehPar->VehParMain.WheelBase -
            LCFVehParams->LCFVeh_Kf_DistToRearAxle_met;

        if (LCK_PAR_SSG_SEL == 1u) {
            //   Cornering stiffness at front/rear tires (cr, cf) has to be
            //   greater
            //   then zero, otherwise there is a division by zero. This is
            //   checked
            //   in LCK wrapper.
            LCFVehParams->LCFVeh_Kf_SelfStrGrad =
                (LCFVehParams->LCFVeh_Kf_Mass_kg *
                 ((LCFVehParams->LCFVeh_Kf_CrnStiffRe *
                   LCFVehParams->LCFVeh_Kf_DistToRearAxle_met) -
                  (LCFVehParams->LCFVeh_Kf_CrnStiffFr *
                   LCFVehParams->LCFVeh_Kf_DistToFrontAxle_met))) /
                ((LCFVehParams->LCFVeh_Kf_CrnStiffRe *
                  LCFVehParams->LCFVeh_Kf_CrnStiffFr) *
                 (LCFVehParams->LCFVeh_Kf_DistToRearAxle_met +
                  LCFVehParams->LCFVeh_Kf_DistToFrontAxle_met));
        }

        else if (LCK_PAR_SSG_SEL == 2u) {
            LCFVehParams->LCFVeh_Kf_SelfStrGrad =
                ps_VehPar->VehParMain.SelfSteerGrad;
        } else {
            /*Default Value*/
            LCFVehParams->LCFVeh_Kf_SelfStrGrad =
                ps_VehPar->VehParMain.SelfSteerGrad;
        }
    }

#endif
}

/*****************************************************************************
  Functionname:LcfRTEVehInputWrapper */ /*!

                                        @brief: set LCF VEH input pointer based
                                        on the value of RTE signals

                                        @description: set LCF VEH input pointer
                                        based on the value of RTE signals

                                        @param[in/out]
                                                             reqLcfSenPrtList_t*
                                        g_LCFSEN_reqPorts: LCF SEN input
                                        pointer
                                        @param[out]

                                        @return
                                        *****************************************************************************/
// extern Dt_RECORD_LCF_SenGenericOutputs_t
// Dt_RECORD_LCF_SenGenericOutputs_buf2;
extern uint8 LCF_VEH_Temp_LDPSC_SysOut_St;
extern uint8 LCF_VEH_Temp_LDPSC_DgrSide_St;
extern float32 LCF_VEH_Temp_LDPDT_LnCltdCurvLf_ReMi;
extern float32 LCF_VEH_Temp_LDPDT_LnHeadLf_Rad;
extern float32 LCF_VEH_Temp_LDPDT_LnCltdCurvRi_ReMi;
extern float32 LCF_VEH_Temp_LDPDT_LnHeadRi_Rad;
extern float32 LCF_VEH_Temp_LDPDT_LnPstnLf_Mi;
extern float32 LCF_VEH_Temp_LDPDT_LnPstnRi_Mi;

void LcfRTEVehInputWrapper(reqLcfVehPrtList_t* g_LCFVEH_reqPorts) {
#ifndef LCF_VEH_DEBUG

    Dt_RECORD_VED_VehDyn_t Dt_RECORD_VED_VehDyn_buf;
    Dt_RECORD_LCF_SenToVeh_t Dt_RECORD_LCF_SenToVeh_buf;
    Dt_RECORD_LCF_SenGenericOutputs_t Dt_RECORD_LCF_SenGenericOutputs_buf;

    Dt_RECORD_VED_VehDyn_t* ps_VehDyn = &Dt_RECORD_VED_VehDyn_buf;  // VehDyn
    Dt_RECORD_LCF_SenToVeh_t* pSenToVeh = &Dt_RECORD_LCF_SenToVeh_buf;
    Dt_RECORD_LCF_SenGenericOutputs_t* pSenGenOut =
        &Dt_RECORD_LCF_SenGenericOutputs_buf;

    Rte_Read_CtApLCFVEH_PpVED_VehDyn_t_info_DeVED_VehDyn_t_buf(
        ps_VehDyn);  // VehDyn
    Rte_Read_CtApLCFVEH_PpLCF_SenToVeh_t_info_DeLCF_SenToVeh_t_buf(pSenToVeh);
    Rte_Read_CtApLCFVEH_PpLCF_SenGenericOutputs_t_info_DeLCF_SenGenericOutputs_t_buf(
        pSenGenOut);

    g_LCFVEH_reqPorts->sBaseCtrlData.eOpMode = BASE_OM_RUN;

    // VehDyn
    g_LCFVEH_reqPorts->sLCFVehDyn.fEgoVelX_mps =
        ps_VehDyn->Longitudinal.VeloCorr.corrVelo;
    g_LCFVEH_reqPorts->sLCFVehDyn.fEgoYawRate_rps =
        ps_VehDyn->Lateral.YawRate.YawRate;
    g_LCFVEH_reqPorts->sLCFVehDyn.fWhlSteerAngleVdy_rad =
        ps_VehDyn->Lateral.OffCompStWheelAngle;

// LDPSA output To Veh
#if 0
	g_LCFVEH_reqPorts->sLcfSVehInputFromSenData.sLDPSAData.LDPSC_SysOut_St         = pSenGenOut->sLDPSAOutput.LDPSC_SysOut_St;
	g_LCFVEH_reqPorts->sLcfSVehInputFromSenData.sLDPSAData.LDPSC_DgrSide_St        = pSenGenOut->sLDPSAOutput.LDPSC_DgrSide_St;
	g_LCFVEH_reqPorts->sLcfSVehInputFromSenData.sLDPSAData.LDPDT_LnCltdCurvLf_ReMi = pSenGenOut->sLDPSAOutput.LDPDT_LnCltdCurvLf_ReMi;
	g_LCFVEH_reqPorts->sLcfSVehInputFromSenData.sLDPSAData.LDPDT_LnHeadLf_Rad      = pSenGenOut->sLDPSAOutput.LDPDT_LnHeadLf_Rad;
	g_LCFVEH_reqPorts->sLcfSVehInputFromSenData.sLDPSAData.LDPDT_LnCltdCurvRi_ReMi = pSenGenOut->sLDPSAOutput.LDPDT_LnCltdCurvRi_ReMi;
	g_LCFVEH_reqPorts->sLcfSVehInputFromSenData.sLDPSAData.LDPDT_LnHeadRi_Rad      = pSenGenOut->sLDPSAOutput.LDPDT_LnHeadRi_Rad;
	g_LCFVEH_reqPorts->sLcfSVehInputFromSenData.sLDPSAData.LDPDT_LnPstnLf_Mi       = pSenGenOut->sLDPSAOutput.LDPDT_LnPstnLf_Mi;
	g_LCFVEH_reqPorts->sLcfSVehInputFromSenData.sLDPSAData.LDPDT_LnPstnRi_Mi       = pSenGenOut->sLDPSAOutput.LDPDT_LnPstnRi_Mi;
	g_LCFVEH_reqPorts->sLcfSVehInputFromSenData.sLDPSAData.LDWC_DgrSide_St         = pSenGenOut->sLDPSAOutput.LDWC_DgrSide_St;
#endif

#if 0
	//pSenGenOut = &Dt_RECORD_LCF_SenGenericOutputs_buf2;
	g_LCFVEH_reqPorts->sLcfSVehInputFromSenData.sLDPSAData.LDPSC_SysOut_St         = Dt_RECORD_LCF_SenGenericOutputs_buf2.sLDPSAOutput.LDPSC_SysOut_St;
	g_LCFVEH_reqPorts->sLcfSVehInputFromSenData.sLDPSAData.LDPSC_DgrSide_St        = Dt_RECORD_LCF_SenGenericOutputs_buf2.sLDPSAOutput.LDPSC_DgrSide_St;
	g_LCFVEH_reqPorts->sLcfSVehInputFromSenData.sLDPSAData.LDPDT_LnCltdCurvLf_ReMi = Dt_RECORD_LCF_SenGenericOutputs_buf2.sLDPSAOutput.LDPDT_LnCltdCurvLf_ReMi;
	g_LCFVEH_reqPorts->sLcfSVehInputFromSenData.sLDPSAData.LDPDT_LnHeadLf_Rad      = Dt_RECORD_LCF_SenGenericOutputs_buf2.sLDPSAOutput.LDPDT_LnHeadLf_Rad;
	g_LCFVEH_reqPorts->sLcfSVehInputFromSenData.sLDPSAData.LDPDT_LnCltdCurvRi_ReMi = Dt_RECORD_LCF_SenGenericOutputs_buf2.sLDPSAOutput.LDPDT_LnCltdCurvRi_ReMi;
	g_LCFVEH_reqPorts->sLcfSVehInputFromSenData.sLDPSAData.LDPDT_LnHeadRi_Rad      = Dt_RECORD_LCF_SenGenericOutputs_buf2.sLDPSAOutput.LDPDT_LnHeadRi_Rad;
	g_LCFVEH_reqPorts->sLcfSVehInputFromSenData.sLDPSAData.LDPDT_LnPstnLf_Mi       = Dt_RECORD_LCF_SenGenericOutputs_buf2.sLDPSAOutput.LDPDT_LnPstnLf_Mi;
	g_LCFVEH_reqPorts->sLcfSVehInputFromSenData.sLDPSAData.LDPDT_LnPstnRi_Mi       = Dt_RECORD_LCF_SenGenericOutputs_buf2.sLDPSAOutput.LDPDT_LnPstnRi_Mi;
	g_LCFVEH_reqPorts->sLcfSVehInputFromSenData.sLDPSAData.LDWC_DgrSide_St         = Dt_RECORD_LCF_SenGenericOutputs_buf2.sLDPSAOutput.LDWC_DgrSide_St;
#endif
    g_LCFVEH_reqPorts->sLcfSVehInputFromSenData.sLDPSAData.LDPSC_SysOut_St =
        LCF_VEH_Temp_LDPSC_SysOut_St;
    g_LCFVEH_reqPorts->sLcfSVehInputFromSenData.sLDPSAData.LDPSC_DgrSide_St =
        LCF_VEH_Temp_LDPSC_DgrSide_St;
    g_LCFVEH_reqPorts->sLcfSVehInputFromSenData.sLDPSAData
        .LDPDT_LnCltdCurvLf_ReMi = LCF_VEH_Temp_LDPDT_LnCltdCurvLf_ReMi;
    g_LCFVEH_reqPorts->sLcfSVehInputFromSenData.sLDPSAData.LDPDT_LnHeadLf_Rad =
        LCF_VEH_Temp_LDPDT_LnHeadLf_Rad;
    g_LCFVEH_reqPorts->sLcfSVehInputFromSenData.sLDPSAData
        .LDPDT_LnCltdCurvRi_ReMi = LCF_VEH_Temp_LDPDT_LnCltdCurvRi_ReMi;
    g_LCFVEH_reqPorts->sLcfSVehInputFromSenData.sLDPSAData.LDPDT_LnHeadRi_Rad =
        LCF_VEH_Temp_LDPDT_LnHeadRi_Rad;
    g_LCFVEH_reqPorts->sLcfSVehInputFromSenData.sLDPSAData.LDPDT_LnPstnLf_Mi =
        LCF_VEH_Temp_LDPDT_LnPstnLf_Mi;
    g_LCFVEH_reqPorts->sLcfSVehInputFromSenData.sLDPSAData.LDPDT_LnPstnRi_Mi =
        LCF_VEH_Temp_LDPDT_LnPstnRi_Mi;

    // TJASA output To Veh, Reserved
    // pLcfVehInput->sTJASAState.uiLatCtrlMode_nu			=
    // pLcfSenOutput->pLcfSenOutputToVehData.sTJASAOutput.TJASTM_LatCtrlMode_nu;

    // LBP output To Veh, Reserved
    // pLcfVehInput->sLBPData_t.uiLeCrvQuality_per			=
    // pLcfSenOutput->pLcfSenOutputData.sLBPOutput.uCrvQualityLf;
    // pLcfVehInput->sLBPData_t.uiLeLnQuality_per			=
    // pLcfSenOutput->pLcfSenOutputData.sLBPOutput.uOverallQualityLf;
    // pLcfVehInput->sLBPData_t.uiRiCrvQuality_per			=
    // pLcfSenOutput->pLcfSenOutputData.sLBPOutput.uCrvQualityRi;
    // pLcfVehInput->sLBPData_t.uiRiLnQuality_per			=
    // pLcfSenOutput->pLcfSenOutputData.sLBPOutput.uOverallQualityRi;

    // CSCLTA output To Veh, Reserved
    // pLcfVehInput->sCSCLTAData.uiControllingFunction_nu	=
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_ControllingFunction_nu;
    // pLcfVehInput->sCSCLTAData.fPlanningHorizon_sec		=
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_PlanningHorzion_sec;
    // pLcfVehInput->sCSCLTAData.uiSysStateLCF_nu			=
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_SysStateLCF;
    // pLcfVehInput->sCSCLTAData.fPredTimeCurve_sec		    =
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_PredTimeCrv_sec;
    // pLcfVehInput->sCSCLTAData.fPredTimeHeadAng_sec		=
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_PredTimeHeadAng_sec;
    // pLcfVehInput->sCSCLTAData.bTriggerReplan			    =
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_TriggerReplan_nu;
    // pLcfVehInput->sCSCLTAData.fLeCridBndPosX0_met		    =
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_LeCridrBndPosX0_met;
    // pLcfVehInput->sCSCLTAData.fLeCridBndPosY0_met		    =
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_LeCridrBndPosY0_met;
    // pLcfVehInput->sCSCLTAData.fLeCridBndHeadAng_rad		=
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_LeCridrHeadAng_rad;
    // pLcfVehInput->sCSCLTAData.fLeCridBndCrv_1pm			=
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_LeCridrBndCrv_1pm;
    // pLcfVehInput->sCSCLTAData.fLeCridBndCrvChng_1pm2	    =
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_LeCridrBndCrvChng_1pm2;
    // pLcfVehInput->sCSCLTAData.fLeCridBndLength_met		=
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_LeCridrBndLength_met;
    // pLcfVehInput->sCSCLTAData.fRiCridBndPosX0_met		    =
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_RiCridrBndPosX0_met;
    // pLcfVehInput->sCSCLTAData.fRiCridBndPosY0_met		    =
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_RiCridrBndPosY0_met;
    // pLcfVehInput->sCSCLTAData.fRiCridBndHeadAng_rad		=
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_RiCridrHeadAng_rad;
    // pLcfVehInput->sCSCLTAData.fRiCridBndCrv_1pm			=
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_RiCridrBndCrv_1pm;
    // pLcfVehInput->sCSCLTAData.fRiCridBndCrvChng_1pm2	    =
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_RiCridrBndCrvChng_1pm2;
    // pLcfVehInput->sCSCLTAData.fRiCridBndLength_met		=
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_RiCridrBndLength_met;
    // pLcfVehInput->sCSCLTAData.fTgtTrajPosX0_met			=
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_TgtTrajPosX0_met;
    // pLcfVehInput->sCSCLTAData.fTgtTrajPosY0_met			=
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_TgtTrajPosY0_met;
    // pLcfVehInput->sCSCLTAData.fTgtTrajHeadingAng_rad	    =
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_TgtTrajHeadAng_rad;
    // pLcfVehInput->sCSCLTAData.fTgtTrajCurve_1pm			=
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_TgtTrajCrv_1pm;
    // pLcfVehInput->sCSCLTAData.fTgtTrajCrvChng_1pm2		=
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_TgtTrajCrvChng_1pm2;
    // pLcfVehInput->sCSCLTAData.fTgtTrajLength_met		    =
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_TgtTrajLength_met;
    // pLcfVehInput->sCSCLTAData.bLatencyCompActivated		=
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_LatencyCompActivated_bool;
    // pLcfVehInput->sCSCLTAData.fSensorTimeStamp_sec		=
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_SensorTStamp_sec;
    // pLcfVehInput->sCSCLTAData.uiTrajPlanServQu_nu		    =
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_TgtPlanServQu_nu;
    // pLcfVehInput->sCSCLTAData.fWeightTgtDistY_nu		    =
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_WeightTgtDistY_nu;
    // pLcfVehInput->sCSCLTAData.fWeightEndTime_nu			=
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_WeightEndTime_nu;
    // pLcfVehInput->sCSCLTAData.fDistYToLeTgtArea_met		=
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_DistYToILeTgtArea_met;
    // pLcfVehInput->sCSCLTAData.fDistYToRiTgtArea_met		=
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_DistYToIRiTgtArea_met;
    // pLcfVehInput->sCSCLTAData.fFTireAclMax_mps2			=
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_FTireAclMax_mps2;
    // pLcfVehInput->sCSCLTAData.fFTireAclMin_mps2			=
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_FTireAclMin_mps2;
    // pLcfVehInput->sCSCLTAData.fObstacleVelX_mps			=
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_ObstacleVelX_mps;
    // pLcfVehInput->sCSCLTAData.fObstacleAccelX_mps2		=
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_ObstacleAclX_mps2;
    // pLcfVehInput->sCSCLTAData.fObstacleWidth_met		    =
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_ObstacleWidth_met;
    // pLcfVehInput->sCSCLTAData.fObstacleDistX_met		    =
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_ObstacleDistX_met;
    // pLcfVehInput->sCSCLTAData.fObstacleDistY_met		    =
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_ObstacleDistY_met;
    // pLcfVehInput->sCSCLTAData.fMaxJerkAllowed_mps3		=
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_MaxJerkAllowed_mps3;
    // pLcfVehInput->sCSCLTAData.uiTrajGuiQualifier_nu		=
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_TrajGuiQualifier_nu;
    // pLcfVehInput->sCSCLTAData.fMaxCrvGrdBuildup_1pms	    =
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_MaxCrvGrdBuildup_1pms;
    // pLcfVehInput->sCSCLTAData.fMaxCrvGrdRed_1pms		    =
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_MaxCrvGrdRed_1pms;
    // pLcfVehInput->sCSCLTAData.fGrdLimitTgtCrvGC_1pms	    =
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_GrdLimitTgtCrvGC_1pms;
    // pLcfVehInput->sCSCLTAData.fMaxCrvTrajGuiCtrl_1pm	    =
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_MaxCrvTrajGuiCtrl_1pm;
    // pLcfVehInput->sCSCLTAData.bLimiterActivated_nu		=
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_LimiterActivated_nu;
    // pLcfVehInput->sCSCLTAData.fLimiterDuration_sec		=
    // pLcfSenOutput->pLcfSenOutputToVehData.sMCTLFCOut.CSCLTA_LimiterDuration_sec;

#endif
}

/*****************************************************************************
  Functionname:LcfRTEVehOutputWrapper */ /*!

                                       @brief: set LCF VEH output pointer based
                                       on the value of RTE signals

                                       @description: set LCF VEH output pointer
                                       based on the value of RTE signals

                                       @param[in/out]
                                                           reqLcfSenPrtList_t*
                                       g_LCFSEN_reqPorts: LCF SEN input pointer
                                       @param[out]

                                       @return
                                       *****************************************************************************/
void LcfRTEVehOutputWrapper(proLcfVehPrtList_t* pLcfVehOutput) {
#ifndef LCF_VEH_DEBUG

    // Rte_CtApLCFVEH.h中未定义Rte_Write方法？

    // Dt_RECORD_LCF_SenAlgoCompState_t sVehAlgoCompState;

    //// SEN State return values of the algo component
    // memcpy(sVehAlgoCompState.AlgoVersionInfo,
    // pLcfVehOutput->pAlgoCompState.AlgoVersionInfo,
    // (size_t)sizeof(pLcfVehOutput->pAlgoCompState.AlgoVersionInfo));

    // sVehAlgoCompState.eCompState                      =
    // pLcfVehOutput->pAlgoCompState.eCompState;
    // sVehAlgoCompState.eErrorCode                      =
    // pLcfVehOutput->pAlgoCompState.eErrorCode;
    // sVehAlgoCompState.eGenAlgoQualifier               =
    // pLcfVehOutput->pAlgoCompState.eGenAlgoQualifier;
    // sVehAlgoCompState.eShedulerSubModeRequest         =
    // pLcfVehOutput->pAlgoCompState.eShedulerSubModeRequest;
    //
    // sVehAlgoCompState.sSigHeader.eSigStatus           =
    // pLcfVehOutput->pAlgoCompState.sSigHeader.eSigStatus;
    // sVehAlgoCompState.sSigHeader.uiCycleCounter       =
    // pLcfVehOutput->pAlgoCompState.sSigHeader.uiCycleCounter;
    // sVehAlgoCompState.sSigHeader.uiMeasurementCounter =
    // pLcfVehOutput->pAlgoCompState.sSigHeader.uiMeasurementCounter;
    // sVehAlgoCompState.sSigHeader.uiTimeStamp          =
    // pLcfVehOutput->pAlgoCompState.sSigHeader.uiTimeStamp;
    //
    // sVehAlgoCompState.uiAlgoVersionNumber             =
    // pLcfVehOutput->pAlgoCompState.uiAlgoVersionNumber;
    // sVehAlgoCompState.uiApplicationNumber             =
    // pLcfVehOutput->pAlgoCompState.uiApplicationNumber;
    // sVehAlgoCompState.uiVersionNumber                 =
    // pLcfVehOutput->pAlgoCompState.uiVersionNumber;

    // LCD Output

#endif
}

/*****************************************************************************
  Functionname:LcfVehPortsCheck                                            */ /*!

      @brief: NULL pointer check and signal status check for used pointers

      @description: NULL pointer check and signal status check for used pointers

      @param[in]
      @param[out]

      @return
    *****************************************************************************/
static boolean LcfVehPortsCheck(reqLcfVehPrtList_t* pLcfVehInput,
                                reqLcfVehParams* pLcfVehParam,
                                proLcfVehPrtList_t* pLcfVehOutput,
                                reqLcfVehDebug_t* pLcFVehDebug) {
    return TRUE;

#if 0
#ifndef LCF_VEH_DEBUG

	

	Dt_RECORD_VED_VehDyn_t ps_VehDynVal;//              = NULL;    //VehDyn
	Dt_RECORD_LCF_SenToVeh_t pSenToVehVal;//            = NULL;
	Dt_RECORD_LCF_SenGenericOutputs_t pSenGenOutVal;//  = NULL;

	Dt_RECORD_VED_VehDyn_t* ps_VehDyn             = &ps_VehDynVal;    //VehDyn
	Dt_RECORD_LCF_SenToVeh_t* pSenToVeh           = &pSenToVehVal;
	Dt_RECORD_LCF_SenGenericOutputs_t* pSenGenOut = &pSenGenOutVal;

	Rte_Read_CtApLCFVEH_PpVED_VehDyn_t_info_DeVED_VehDyn_t_buf(ps_VehDyn);	//VehDyn
	Rte_Read_CtApLCFVEH_PpLCF_SenToVeh_t_info_DeLCF_SenToVeh_t_buf(pSenToVeh);
	Rte_Read_CtApLCFVEH_PpLCF_SenGenericOutputs_t_info_DeLCF_SenGenericOutputs_t_buf(pSenGenOut);

	boolean bCheckResult = TRUE;
	if (pLcfVehInput == NULL ||
		pLcfVehParam == NULL ||
		pLcfVehOutput == NULL ||
		pLcFVehDebug == NULL)
	{
		bCheckResult = FALSE;
	}

	if (ps_VehDyn == NULL ||
		pSenToVeh == NULL ||
		pSenGenOut == NULL)
	{
		bCheckResult = FALSE;
	}

	return bCheckResult;

#else
	return TRUE;
#endif
#endif
}

/*****************************************************************************
  Functionname:Rte_IRead_SWLCF_ProLcfVeh_Out */ /*!

                                @brief: LCF output rte wrapper, other module
                                could get LCF veh output data from
                                this interface

                                @description

                                @param[in]
                                @param[out]

                                @return
                                *****************************************************************************/
const proLcfVehPrtList_t* Rte_IRead_SWLCF_ProLcfVeh_Out(void) {
    return &g_LCFVEH_proPorts;
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
