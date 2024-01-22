/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

#include "vlcSen_ext.h"
#include "vlcSen_consts.h"
#include "vlc_sen.h"

/*****************************************************************************
  MODULGLOBALE VARIABLEN
*****************************************************************************/
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/*vlc sen and it's subcomponents initialization status*/
boolean VLCSenIsInitialized = FALSE;
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/* Input Port Measurement Counter struct */
/* Meas freeze reference */
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
VLCSen_SyncRef_t VLCSenSyncRef;
/* distance from Bumper to Center of Gravity */
float32 VLC_fBumperToCoG;
/* vehicle width */
float32 VLC_fEgoVehicleWidth;
/* vehicle length */
float32 VLC_fEgoVehicleLength;
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"

/* Pointers to the input/output data of the VLC component */
const VLCCtrlData_t* VLCSEN_pSenCtrlData;
const ECAMtCyclEnvmode_t* VLCSEN_pECAMtCyclEnvmode;
const Envm_t_GenObjectList* VLCSEN_pEmGenObjList;
const Envm_t_CRObjectList* VLCSEN_pEmARSObjList;
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
AssessedObjList_t* VLCSEN_pPubFctObjList;
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
const VED_VehDyn_t* VLCSEN_pEgoDynObjSync;
const VED_VehDyn_t* VLCSEN_pEgoDynRaw;
const VED_VehPar_t* VLCSEN_pGlobEgoStatic;

/* customer specific input/output */
const VLCCustomInput_t* VLCSEN_pCustomInput;
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
VLCCustomOutput_t* VLCSEN_pCustomOutput;
VLCCDOutputCustom_t* VLCSEN_pCDCustomOutput;
VLC_HypothesisIntf_t* VLC_pCDHypothesesSen; /*!< VLC CD hypotheses */
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
#ifndef BswAlgoParameters_define
#define BswAlgoParameters_define
const Com_AlgoParameters_t*
    VLCSEN_pBswAlgoParameters; /*!< Input algo parameters from BSW */
#endif
const VLC_Parameters_t*
    VLCSEN_pCPAR_VLC_Parameters;               /*!< VLC Coding Parameters */
const t_CamLaneInputData* VLCSEN_pCamLaneData; /*!< Camera lane input data */

#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/* Provide OOI Objects from SEN to VEH */
VLCSenAccOOI_t* VLCSEN_pAccOOIData;

/*! The VLC private object data */
VLCPrivObjectList_t VLCObjectList;
/* frame (cycle time, cycle counter, opmode ...) */
VLCSenFrame_t VLCSenFrame;
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
// float g_fPreBrakeDecel;
const VLCSen_Parameters_t* pVLCSEN_Parameters;

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */