/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU1_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

#include "vlcVeh_ext.h"
#include "vlcVeh_consts.h"
#include "vlcVeh_common_utils.h"

#include "vlc_veh.h"

/*****************************************************************************
  extern function
*****************************************************************************/
uint32 Norm_SYS_GetCurrentTime(void) {}
//void SYS_u_TimeGetSinceBoot_ms(void) {}
/*****************************************************************************
  MODULGLOBALE VARIABLEN
*****************************************************************************/
// boolean isVehMainInitializedForSimulation = FALSE;
/*vlc veh and it's subcomponents initialization status*/
// SET_MEMSEC_VAR(VLCVehIsInitialized)
// static boolean VLCVehIsInitialized = FALSE;                 /*!<flag showing
// once completed algo cycle (!=shutdown).*/ static VLCVeh_SyncRef_t
// VLCVehSyncRef;                /*!<struct that stores signal headers of all
// input interfaces for simulation synchronization.*/ static const
// VLCCtrlData_t*
// VLCVeh_pCtrlData;        /*!<internal pointer to relevant external control
// data INPUT interface.*/
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
VLCVehFrame_t
    VLCVehFrame; /*!<internal structure that stores the status of the Algo.*/
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
const VED_VehDyn_t* VLCVEH_pEgoDynRaw; /*!<internal pointer to external raw ego
                                          dynamics INPUT interface.*/
const VED_VehPar_t*
    VLCVEH_pGlobEgoStatic; /*!<internal pointer to external vehicle parameters
                              INPUT interface.*/
const VLC_HypothesisIntf_t*
    VLC_pCDHypothesesVeh; /*!<internal pointer to external hypothesis INPUT
                             interface.*/
VLC_DIMOutputCustom_t* VLC_pDIMCustDataOut; /*!<internal pointer to external DIM
                                               custom OUTPUT interface.*/
const VLC_DIMInputCustom_t*
    VLC_pDIMCustDataIn; /*!<internal pointer to external DIM custom INPUT
                           interface.input.*/
const VLC_DIMInputGeneric_t*
    VLC_pDIMGenericDataIn; /*!<internal pointer to external DIM generic INPUT
                              interface. VLC DIM generic input.*/
VLC_SADOutputCustom_t*
    VLC_pHEADCustDataOut; /*!<internal pointer to external HEAD custom OUTPUT
                             interface. VLC HEAD custom output.*/
VLC_SADOutputGeneric_t*
    VLC_pHEADGenericDataOut; /*!<internal pointer to external HEAD generic
                                OUTPUT interface. VLC HEAD generic output.*/
const VLC_SADInputGeneric_t*
    VLC_pHEADGenericDataIn; /*!<internal pointer to external HEAD generic INPUT
                               interface. VLC HEAD generic input.*/
const VLC_SADInputCustom_t*
    VLC_pHEADCustDataIn; /*!<internal pointer to external HEAD custom INPUT
                            interface. VLC HEAD custom input.*/

const Com_AlgoParameters_t*
    VLCVEH_pBswAlgoParameters; /*!< Input algo parameters from BSW */
const VLC_Parameters_t*
    VLCVEH_pCPAR_VLC_Parameters; /*!<internal pointer to external EBA CParameter
                                    INPUT interface. VLC Coding Parameters.*/
const VED_VehSig_t*
    VLCVEH_pVehSig; /*!<internal pointer to external VDY sensor signals INPUT.
                       Pointer to raw vehicle signals.*/
const PowerTrain_t* VLCVEH_pVehSigPowerTrain;
const VLC_AccLeverInput_t* VLCVEH_pAccLever;
const VLC_LongCtrlInput_t* VLCVEH_pLongCtrlResp;
const VLC_acc_object_t* VLCVEH_pAccDisplayObj;
const VLC_acc_output_data_t* VLCVEH_pAccOutput;
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
VLC_DFV2SenInfo_t* VLCVEH_pDFVLongOut;
VLC_LongCtrlOutput_t* VLCVEH_pLongCtrlOutput;
VLC_LODMCOutput_t* VLCVEH_pLODMCOutput;
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
const VLCSenAccOOI_t* VLCVEH_pAccOOIData;
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
VLC_DFVOutArbitrated_t* VLCVEH_pVLCVehOutArbitrated;
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
// temp for vlcVeh

/*
const Com_AlgoParameters_t*
    VLCSEN_pBswAlgoParameters;
*/

// temp for head
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
HEADState_t HEADState;
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
void HEADSetOpMode(eHEADOpMode_t OpMode) {}

// temp for dim
// DIMHypothesisList_t DIMHypothesisList;
// DIMState_t DIMState;

const VLCVeh_Parameters_t* pVLCVEH_Parameters;

/* end of definition for local alias for external ports */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU1_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */