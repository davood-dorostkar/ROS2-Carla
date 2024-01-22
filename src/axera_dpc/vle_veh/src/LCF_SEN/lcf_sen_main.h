#ifndef LCF_SEN_MAIN_H
#define LCF_SEN_MAIN_H
#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
  MACROS
*****************************************************************************/
//#define LCF_SEN_DEBUG
#define UBUNTU_SYSTEM

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#ifndef _DEBUG
//#include "Rte_BswM_Type.h"
//#include "vdy_ext.h"
//#include "Platform_Types.h"

//#include "TM_PlatF_Types.h"
//#include "Compiler.h"

//#include "Compiler_Cfg.h"
//#include "TM_PlatF_Types.h"
//#include "TM_Soc_Ips.h"
//#include "glob.h"
//#include "Project_specific_Types.h"
//#include "TM_Std_Types.h"
//#include "TM_CErrRecover.h"
//#include "TM_SWErr.h"
//#include "TM_Soc_Ips.h"
//#include "TM_Reg_Sys.h"
//#include "TM_Mcal.h"
//#include "TM_Dio_Env_Cfg.h"
//#include "TM_Dio_Cfg.h"
//#include "TM_MamMap.h"
//#include "stdio.h"
//#include "TM_Algo_Const.h"
//#include "stddef.h"
//#include "VS_Dev_Conf.h"
//#include "SensorSimulationDataConfig_60ms.h"
//#include "Tuerme.h"
//#include "TM_Rte.h"
//#include "TM_Rte_Types.h"
//#include "TM_Rte_EcuM_Types.h"
//#include "TM_Rte_Nvm_Types.h"
//#include "TM_Rte_Algn_Types.h"
//#include "TM_Rte_Bsw_Types.h"
//#include "TM_Rte_Dem_Types.h"
//#include "TM_Rte_Diag_Types.h"
//#include "TM_Rte_DiagSrv_Types.h"
//#include "TM_Rte_OpsAdapter_Tyeps.h"
//#include "TM_Rte_Eth_Types.h"
//#include "TM_Rte_SensAdapter_Types.h"
//#include "TM_Rte_VehicleAdapter_Types.h"
//#include "TM_Rte_Frame_Types.h"
//#include "TM_Rte_FSMonitor_Types.h"
//#include "TM_Rte_Dlogger_Types.h"
//#include "TM_Rte_Norm_Types.h"
//#include "Rte_SWCRcc_Type.h"
//#include "TM_Rte_SPMAdapter_Types.h"
//#include "TM_Rte_SWCtrl_Types.h"
//#include "TM_Ret_VED_Adapter_Types.h"
//#include "TM_Rte_Xcp_Types.h"
//#include "TM_Rte_Common.h"
//#include "TM_Algo_Sens_Par.h"
//#include "TM_Algo_Cfg.h"
//#include "TM_Algo_Types.h"
//#include "TM_Base_Bayes.h"
//#include "assert.h"
//#include "TM_Base_Cfg.h"
//#include "TM_Base_Complex.h"
//#include "TM_Base_Const.h"
//#include "TM_Base_Emul.h"
//#include "TM_Base_Fourier.h"
//#include "TM_Base_Kafi.h"
//#include "TM_Base_Mapping.h"
//#include "TM_Base_Mat.h"
//#include "TM_Base_Misc.h"
//#include "TM_Base_Mtrx.h"
//#include "TM_Base_Stat.h"
//#include "TM_Base_Trigo.h"
//#include "TM_Base_Vector.h"
//#include "TM_Base_Interpol.h"
//#include "TM_Base_Ver.h"
//#include "TM_Base_Ext.h"
//#include "TM_Global_CompID.h"
//#include "TM_Algo_Global.h"
//#include "TM_Envm_cfg.h"
//#include "TM_Envm_Objlist.h"
//#include "TM_Envm_Meas.h"
//#include "TM_Dlogger_App.h"
//#include "RTA_Pub_Cfg.h"
//#include "TM_RTA_Types.h"
//#include "TM_RTA_Prv_Cfg.h"
//#include "sys_interrupt.h"
//#include "Sys_Mem.h"
//#include "TM_RTA.h"
//#include "TM_Algo_Service_Types.h"
//#include "TM_Envm_ext.h"
//#include "fct_veh_ext.h"
//#include "fct_sen_ext.h"
//#include "ADPVDY_Simulation_Process_20ms.h"
//#include "head.h"
//#include "TM_Envm_ext.h"
//#include "TM_Envm_par.h"
//#include "TM_Envm_State.h"
//#include "TM_Envm_Signal_Check.h"
//#include "TM_Envm_Err.h"
//#include "TM_Envm.h"
//#include "TM_Envm_Macro.h"

#ifdef LINUX_SYS
#include "BSW_DataLog_Server.h"
#endif
#endif
#include "lcf_sen_local_main.h"
#include "lcf_sen_ext.h"

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
void LcfRTESenInputWrapper(reqLcfSenPrtList_t* g_LCFSEN_reqPorts);
void LcfRTESenOutputWrapper(proLcfSenPrtList_t* pLcfSenOutput);

static void LcfSenSetParams(reqLcfSenParams* LCFSenParams);

static boolean LcfSenPortsCheck(reqLcfSenPrtList_t* pLcfSenInput,
                                reqLcfSenParams* pLcfSenParam,
                                proLcfSenPrtList_t* pLcfSenOutput,
                                reqLcfSenDebug_t* pLcFSenDebug);

#ifdef __cplusplus
}
#endif

#endif
