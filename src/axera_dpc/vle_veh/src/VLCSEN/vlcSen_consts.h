#pragma once

#ifndef VLCSEN_CONSTS_H
#define VLCSEN_CONSTS_H
#ifdef __cplusplus
extern "C" {
#endif
#include "vlcSen_ext.h"
// #include "TM_Global_TypeDefs.h"

//===============
/*! @brief Simulation sync structure contains Signal Headers of every Input Port
 */
typedef struct VLCSen_SyncRef {
    SignalHeader_t sSigHeader;  /*!<sSigHeader */
    SignalHeader_t SenCtrlData; /*!< SenCtrlData */
    SignalHeader_t
        EgoDynObjSync;        /*!< The dynamic vehicle ego data object sync */
    SignalHeader_t EgoDynRaw; /*!< The dynamic vehicle ego data raw */
    SignalHeader_t EgoStaticData;  /*!< the static vehicle ego data */
    SignalHeader_t EmFctCycleMode; /*!< The global sensor state */
    SignalHeader_t EmGenObjList;   /*!< EM generic object list */
    SignalHeader_t EmARSObjList; /*!< EM ARS-technology-specific object list */
    /* Longitudinal control input ports */
    SignalHeader_t DFVLongOut; /*!< Internal communication struct from VLC_VEH
                                  to VLC_SEN */
    /* customer specific input/output */
    SignalHeader_t VLCCustomInput; /*!< Custom input */
    /* algo parameters from BSW */
    SignalHeader_t BswAlgoParameters;   /*!< Input algo parameters from BSW */
    SignalHeader_t CPAR_VLC_Parameters; /*!< VLC Coding Parameters */
    SignalHeader_t CamLaneData;         /*!< Camera lane input data */
} VLCSen_SyncRef_t; /*!< @vaddr:VLC_MEAS_ID_SEN_INPUT_SIGHEADERS
                       @cycleid:VLC_ENV @vname:VLCSen_SyncRef */

/*****************************************************************************
  MODULGLOBALE VARIABLEN
*****************************************************************************/
/*vlc sen and it's subcomponents initialization status*/
// boolean VLCSenIsInitialized = FALSE;
/* Input Port Measurement Counter struct */
/* Meas freeze reference */
extern VLCSen_SyncRef_t VLCSenSyncRef;
/* distance from Bumper to Center of Gravity */
extern float32 VLC_fBumperToCoG;
/* vehicle width */
extern float32 VLC_fEgoVehicleWidth;
/* vehicle length */
extern float32 VLC_fEgoVehicleLength;

/* Pointers to the input/output data of the VLC component */
extern const VLCCtrlData_t* VLCSEN_pSenCtrlData;
extern const ECAMtCyclEnvmode_t* VLCSEN_pECAMtCyclEnvmode;
extern const Envm_t_GenObjectList* VLCSEN_pEmGenObjList;
extern const Envm_t_CRObjectList* VLCSEN_pEmARSObjList;
extern AssessedObjList_t* VLCSEN_pPubFctObjList;
extern const VED_VehDyn_t* VLCSEN_pEgoDynObjSync;
extern const VED_VehDyn_t* VLCSEN_pEgoDynRaw;
extern const VED_VehPar_t* VLCSEN_pGlobEgoStatic;

/* customer specific input/output */
extern const VLCCustomInput_t* VLCSEN_pCustomInput;
extern VLCCustomOutput_t* VLCSEN_pCustomOutput;
extern VLCCDOutputCustom_t* VLCSEN_pCDCustomOutput;
extern VLC_HypothesisIntf_t* VLC_pCDHypothesesSen; /*!< VLC CD hypotheses */
extern const Com_AlgoParameters_t*
    VLCSEN_pBswAlgoParameters; /*!< Input algo parameters from BSW */
extern const VLC_Parameters_t*
    VLCSEN_pCPAR_VLC_Parameters; /*!< VLC Coding Parameters */
extern const t_CamLaneInputData*
    VLCSEN_pCamLaneData; /*!< Camera lane input data */

/* Provide OOI Objects from SEN to VEH */
extern VLCSenAccOOI_t* VLCSEN_pAccOOIData;

// extern const VLC_LongCtrlInput_t   * VLCVEH_pLongCtrlResp;

extern const VLCSen_Parameters_t* pVLCSEN_Parameters;
#ifdef __cplusplus
}
#endif

#endif