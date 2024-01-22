/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU1_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "stddef.h"
#include "TM_Global_Types.h"
#include "vlcVeh_consts.h"
#include "vlc_veh.h"
#include "vlc_par.h"
#include "vlc_inhibit_ext.h"
#include "vlc_long_veh_ext.h"
// #include "LongVLC_interface.h"

#define VLC_HMI_INTFVER_MIN_G30_I390 0x03

#define VLC_LD_OUT_INTFVER_MIN_G30_I390 0x03

#define VLC_CB_OUT_INTFVER_MIN_G30_I390 0x07

#define VLC_LKA_OUT_GEN_INTFVER_MIN_G30_I390 0x0F

/*****************************************************************************
  MODULGLOBALE VARIABLEN
*****************************************************************************/
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
boolean isVehMainInitializedForSimulation = FALSE;
/*vlc veh and it's subcomponents initialization status*/
SET_MEMSEC_VAR(VLCVehIsInitialized)
static boolean VLCVehIsInitialized =
    FALSE; /*!<flag showing once completed algo cycle (!=shutdown).*/
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"

#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/* Meas freeze reference */
static VLCVeh_SyncRef_t
    VLCVehSyncRef; /*!<struct that stores signal headers of all input interfaces
                      for simulation synchronization.*/

/* Meas freeze control data */
static const VLCCtrlData_t*
    VLCVeh_pCtrlData; /*!<internal pointer to relevant external control data
                         INPUT interface.*/
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"

uint8 flag_LoDMC_PDSimple = 2;    // Simple PD controller for LoDMC - Default 1, Long VLC 2
uint8 Simple_PD_LoDMC_flag = 0;
float error_previous = 0;

///* frame (cycle time, cycle counter, opmode ...) */
// VLCVehFrame_t VLCVehFrame;                            /*!<internal structure
// that stores the status of the Algo.*/
//
///* start of definition for local alias for external ports (shall be same same
/// than provide and receive ports */
//
// const VED_VehDyn_t              * VLCVEH_pEgoDynRaw;      /*!<internal
// pointer to external raw ego dynamics INPUT interface.*/
//
// const VED_VehPar_t              * VLCVEH_pGlobEgoStatic;  /*!<internal
// pointer to external vehicle parameters INPUT interface.*/
//
//
// const VLC_HypothesisIntf_t      * VLC_pCDHypothesesVeh;   /*!<internal
// pointer to external hypothesis INPUT interface.*/
//
//
//
// VLC_DIMOutputCustom_t           * VLC_pDIMCustDataOut;    /*!<internal
// pointer to external DIM custom OUTPUT interface.*/
//
// const VLC_DIMInputCustom_t      * VLC_pDIMCustDataIn;     /*!<internal
// pointer to external DIM custom INPUT interface.input.*/
//
// const VLC_DIMInputGeneric_t      * VLC_pDIMGenericDataIn;  /*!<internal
// pointer to external DIM generic INPUT interface. VLC DIM generic input.*/
//
//
//
// VLC_SADOutputCustom_t          * VLC_pHEADCustDataOut;    /*!<internal
// pointer to external HEAD custom OUTPUT interface. VLC HEAD custom output.*/
//
// VLC_SADOutputGeneric_t         * VLC_pHEADGenericDataOut; /*!<internal
// pointer to external HEAD generic OUTPUT interface. VLC HEAD generic output.*/
//
// const VLC_SADInputGeneric_t    * VLC_pHEADGenericDataIn;  /*!<internal
// pointer to external HEAD generic INPUT interface. VLC HEAD generic input.*/
//
// const VLC_SADInputCustom_t     * VLC_pHEADCustDataIn;     /*!<internal
// pointer to external HEAD custom INPUT interface. VLC HEAD custom input.*/
//
//
//
// const Com_AlgoParameters_t * VLCVEH_pBswAlgoParameters;  /*!< Input algo
// parameters from BSW */
//
//
//
// const VLC_Parameters_t * VLCVEH_pCPAR_VLC_Parameters;    /*!<internal pointer
// to external EBA CParameter INPUT interface. VLC Coding Parameters.*/
//
//
// const VehSig_t *              VLCVEH_pVehSig;             /*!<internal
// pointer to external VDY sensor signals INPUT. Pointer to raw vehicle
// signals.*/
//
//
//
//
//
//  const PowerTrain_t                * VLCVEH_pVehSigPowerTrain;
//
//
//  const VLC_AccLeverInput_t             * VLCVEH_pAccLever;
//
//  const VLC_LongCtrlInput_t                * VLCVEH_pLongCtrlResp;
//
//
//  const VLC_acc_object_t                  * VLCVEH_pAccDisplayObj;
//
//  const VLC_acc_output_data_t            * VLCVEH_pAccOutput;
//
//  VLC_DFV2SenInfo_t            * VLCVEH_pDFVLongOut;
//
//
//  VLC_LongCtrlOutput_t            * VLCVEH_pLongCtrlOutput;
//
//
//
//  const VLCSenAccOOI_t        * VLCVEH_pAccOOIData;
//
//
//
//  VLC_DFVOutArbitrated_t       * VLCVEH_pVLCVehOutArbitrated;
//
//
///* end of definition for local alias for external ports */

/*****************************************************************************
  MODULLOKALE SYMBOLISCHE KONSTANTEN
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE MAKROS
*****************************************************************************/

/* to add states of new function read from bottom to top. Add new ones on top.
   remember also to adjust VLCVehSetStates function prototype and
   implementation.*/

/*! macro for scheduler control NOT using UDWState*/
#define VLC_VEH_STATE_PARAM(_UDWSTATE_, _LCDSTATE_, _LCKSTATE_, _LKSSTATE_, \
                            _LKASTATE_, _DIMSTATE_, _HEADSTATE_,            \
                            _VLCVEHSTATE_)                                  \
    VLC_VEH_STATE_PARAM_POST_UDW((_LCDSTATE_), (_LCKSTATE_), (_LKSSTATE_),  \
                                 (_LKASTATE_), (_DIMSTATE_), (_HEADSTATE_), \
                                 (_VLCVEHSTATE_))

/*! macro for scheduler control NOT using LCDState*/
#define VLC_VEH_STATE_PARAM_POST_UDW(_LCDSTATE_, _LCKSTATE_, _LKSSTATE_,   \
                                     _LKASTATE_, _DIMSTATE_, _HEADSTATE_,  \
                                     _VLCVEHSTATE_)                        \
    VLC_VEH_STATE_PARAM_POST_LCD((_LCKSTATE_), (_LKSSTATE_), (_LKASTATE_), \
                                 (_DIMSTATE_), (_HEADSTATE_), (_VLCVEHSTATE_))

/*! macro for scheduler control NOT using LCKState*/
#define VLC_VEH_STATE_PARAM_POST_LCD(_LCKSTATE_, _LKSSTATE_, _LKASTATE_,     \
                                     _DIMSTATE_, _HEADSTATE_, _VLCVEHSTATE_) \
    VLC_VEH_STATE_PARAM_POST_LCK((_LKSSTATE_), (_LKASTATE_), (_DIMSTATE_),   \
                                 (_HEADSTATE_), (_VLCVEHSTATE_))

/*! macro for scheduler control NOT using LKSState*/
#define VLC_VEH_STATE_PARAM_POST_LCK(_LKSSTATE_, _LKASTATE_, _DIMSTATE_,    \
                                     _HEADSTATE_, _VLCVEHSTATE_)            \
    VLC_VEH_STATE_PARAM_POST_LKS((_LKASTATE_), (_DIMSTATE_), (_HEADSTATE_), \
                                 (_VLCVEHSTATE_))

/*! macro for scheduler control NOT using LKAState*/
#define VLC_VEH_STATE_PARAM_POST_LKS(_LKASTATE_, _DIMSTATE_, _HEADSTATE_, \
                                     _VLCVEHSTATE_)                       \
    VLC_VEH_STATE_PARAM_POST_LKA((_DIMSTATE_), (_HEADSTATE_), (_VLCVEHSTATE_))

/*! macro for scheduler control using additionally DIMState*/
#define VLC_VEH_STATE_PARAM_POST_LKA(_DIMSTATE_, _HEADSTATE_, _VLCVEHSTATE_) \
    (_DIMSTATE_), VLC_VEH_STATE_PARAM_POST_DIM((_HEADSTATE_), (_VLCVEHSTATE_))

/*! macro for scheduler control using VEHState and HEADState*/
#define VLC_VEH_STATE_PARAM_POST_DIM(_HEADSTATE_, _VLCVEHSTATE_) \
    (_HEADSTATE_), (_VLCVEHSTATE_)

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/

static void VLCVehAlgoInit(void);
static void VLCVehNvmRead(const reqVLCVehPrtList_t* pRequirePorts);
static void VLCVehNvmWrite(const reqVLCVehPrtList_t* pRequirePorts,
                           const proVLCVehPrtList_t* pProvidePorts);
static void VLCVehSetStates(DIMState_t StDIM,
                            HEADState_t StHEAD,
                            VLCVehState_t StVLC);
static void VLCVehProcessStates(VLC_OP_MODE_t eOpMode);

static void VLCVehMeasCallback(void);
static void VLCVehFrameFreeze(void);

static void VLCSetVehFrameData(const reqVLCVehPrtList_t* pRequirePorts);
static void VLCVehCheckPorts(const reqVLCVehPrtList_t* pRequirePorts,
                             const proVLCVehPrtList_t* pProvidePorts);
static void VLCVehSetupPorts(const reqVLCVehPrtList_t* pRequirePorts,
                             const proVLCVehPrtList_t* pProvidePorts);
static void VLCVehSignalErrorShutdown(void);
static void VLCVehSetupSyncRef(const reqVLCVehPrtList_t* pRequirePorts);
static void VLCVehSetProvideHeader(const reqVLCVehPrtList_t* pRequirePorts,
                                   const proVLCVehPrtList_t* pProvidePorts);
static void VLCVehSetProvideHeaderStates(
    const proVLCVehPrtList_t* pProvidePorts, AlgoSignalState_t eSigState);
static void VLCVehSetSigHeaderState(SignalHeader_t* const pSigHeader,
                                    AlgoSignalState_t eSigState);
static void VLCVehSetSigHeaderError(SignalHeader_t* const pSigHeader);
static void VLCVehFillSigHeader(SignalHeader_t* const pSigHeader,
                                const SignalHeader_t* const pSourceHdr);
static void VLCVehSetErrorProvidePorts(const proVLCVehPrtList_t* pProvidePorts);

void SAD_Process(void);
void LODMCProcess(void);
void LODMCSimplePD(void);
/*****************************************************************************
  FUNCTIONS
*****************************************************************************/

/*************************************************************************************************************************
  Functionname:    VLCVehSetErrorProvidePorts */
static void VLCVehSetErrorProvidePorts(
    const proVLCVehPrtList_t* pProvidePorts) {
    /* Provide ports */
    if (pProvidePorts != NULL) {
        if (pProvidePorts->pDFVLongOut != NULL) {
            VLCVehSetSigHeaderError(&pProvidePorts->pDFVLongOut->sSigHeader);
        }
        if (pProvidePorts->pLongCtrlOutput != NULL) {
            pProvidePorts->pLongCtrlOutput->uiVersionNumber = VLC_VEH_INTFVER;
            VLCVehSetSigHeaderError(
                &pProvidePorts->pLongCtrlOutput->sSigHeader);
        }

        if (pProvidePorts->pVLCVehOutArbitrated != NULL) {
            pProvidePorts->pVLCVehOutArbitrated->uiVersionNr =
                VLC_VEH_OUT_ARBIT_INTFVER;
            VLCVehSetSigHeaderError(
                &pProvidePorts->pVLCVehOutArbitrated->sSigHeader);
        }

        if (pProvidePorts->pDIMOutputCustom != NULL) {
            pProvidePorts->pDIMOutputCustom->uiVersionNumber = VLC_VEH_INTFVER;
            VLCVehSetSigHeaderError(
                &pProvidePorts->pDIMOutputCustom->sSigHeader);
        }

        if (pProvidePorts->pHEADOutputGeneric != NULL) {
            pProvidePorts->pHEADOutputGeneric->uiVersionNumber =
                VLC_VEH_INTFVER;
            VLCVehSetSigHeaderError(
                &pProvidePorts->pHEADOutputGeneric->sSigHeader);
        }
        if (pProvidePorts->pHEADOutputCustom != NULL) {
            pProvidePorts->pHEADOutputCustom->uiVersionNumber = VLC_VEH_INTFVER;
            VLCVehSetSigHeaderError(
                &pProvidePorts->pHEADOutputCustom->sSigHeader);
        }

        if (pProvidePorts->pErrorOut != NULL) {
            pProvidePorts->pErrorOut->uiVersionNumber = VLC_VEH_INTFVER;
            VLCVehSetSigHeaderError(&pProvidePorts->pErrorOut->sSigHeader);
        }

        if (pProvidePorts->pVLCVehOutArbitrated != NULL) {
            pProvidePorts->pVLCVehOutArbitrated->uiVersionNr = VLC_VEH_INTFVER;
            VLCVehSetSigHeaderError(
                &pProvidePorts->pVLCVehOutArbitrated->sSigHeader);
        }

    } else {
    }
}

/*************************************************************************************************************************
  Functionname:    VLCVehSetupSyncRef */

static void VLCVehSetupSyncRef(const reqVLCVehPrtList_t* pRequirePorts) {
    /*setting whole syncref to zero */
    (void)memset(&VLCVehSyncRef, 0, sizeof(VLCVehSyncRef));

    if (pRequirePorts != NULL) {
        /*pVehCtrlData*/
        if (pRequirePorts->pVehCtrlData != NULL) {
            VLCVehSyncRef.VLCCtrlData = pRequirePorts->pVehCtrlData->sSigHeader;
        } else {
            VLCVehSyncRef.sSigHeader.eSigStatus = AL_SIG_STATE_INVALID;
        }

        /*pEgoDynRaw*/
        if (pRequirePorts->pEgoDynRaw != NULL) {
            VLCVehSyncRef.VehDyn = pRequirePorts->pEgoDynRaw->sSigHeader;
        } else {
            VLCVehSyncRef.sSigHeader.eSigStatus = AL_SIG_STATE_INVALID;
        }

        /*pEgoStaticData*/
        if (pRequirePorts->pEgoStaticData != NULL) {
            VLCVehSyncRef.VehPar = pRequirePorts->pEgoStaticData->sSigHeader;
        } else {
            VLCVehSyncRef.sSigHeader.eSigStatus = AL_SIG_STATE_INVALID;
        }

        /*pAccLever*/
        if (pRequirePorts->pAccDisplayObj != NULL) {
            VLCVehSyncRef.AccLeverInput = pRequirePorts->pAccLever->sSigHeader;
        } else {
            VLCVehSyncRef.sSigHeader.eSigStatus = AL_SIG_STATE_INVALID;
        }
        /*pLongCtrlResp*/
        if (pRequirePorts->pLongCtrlResp != NULL) {
            VLCVehSyncRef.LongCtrlInput =
                pRequirePorts->pLongCtrlResp->sSigHeader;
        } else {
            VLCVehSyncRef.sSigHeader.eSigStatus = AL_SIG_STATE_INVALID;
        }

        /*pAccDisplayObj*/
        if (pRequirePorts->pAccDisplayObj != NULL) {
            VLCVehSyncRef.acc_object =
                pRequirePorts->pAccDisplayObj->sSigHeader;
        } else {
            VLCVehSyncRef.sSigHeader.eSigStatus = AL_SIG_STATE_INVALID;
        }
        /*pAccOutput*/
        if (pRequirePorts->pAccOutput != NULL) {
            VLCVehSyncRef.acc_output_data =
                pRequirePorts->pAccOutput->sSigHeader;
        } else {
            VLCVehSyncRef.sSigHeader.eSigStatus = AL_SIG_STATE_INVALID;
        }

        /*pVLCAccOOIData*/
        if (pRequirePorts->pVLCAccOOIData != NULL) {
            VLCVehSyncRef.VLCAccOOIData =
                pRequirePorts->pVLCAccOOIData->sSigHeader;
        } else {
            VLCVehSyncRef.sSigHeader.eSigStatus = AL_SIG_STATE_INVALID;
        }

        /*pDIMInputGeneric*/
        if (pRequirePorts->pDIMInputGeneric != NULL) {
            VLCVehSyncRef.DIMInputGeneric =
                pRequirePorts->pDIMInputGeneric->sSigHeader;
        } else {
            VLCVehSyncRef.sSigHeader.eSigStatus = AL_SIG_STATE_INVALID;
        }

        /*pDIMInputCustom*/
        if (pRequirePorts->pDIMInputCustom != NULL) {
            VLCVehSyncRef.DIMInputCustom =
                pRequirePorts->pDIMInputCustom->sSigHeader;
        } else {
            VLCVehSyncRef.sSigHeader.eSigStatus = AL_SIG_STATE_INVALID;
        }

        /*pVLCCDHypotheses*/
        if (pRequirePorts->pVLCCDHypotheses != NULL) {
            VLCVehSyncRef.HypothesisIntf =
                pRequirePorts->pVLCCDHypotheses->sSigHeader;
        } else {
            VLCVehSyncRef.sSigHeader.eSigStatus = AL_SIG_STATE_INVALID;
        }

        /*pSADInputGeneric*/
        if (pRequirePorts->pSADInputGeneric != NULL) {
            VLCVehSyncRef.HEADInputGeneric =
                pRequirePorts->pSADInputGeneric->sSigHeader;
        } else {
            VLCVehSyncRef.sSigHeader.eSigStatus = AL_SIG_STATE_INVALID;
        }

        if (pRequirePorts->pSADInputCustom != NULL) {
            VLCVehSyncRef.HEADInputCustom =
                pRequirePorts->pSADInputCustom->sSigHeader;
        } else {
            VLCVehSyncRef.sSigHeader.eSigStatus = AL_SIG_STATE_INVALID;
        }

        { VLCVehSyncRef.sSigHeader.eSigStatus = AL_SIG_STATE_INVALID; }

        /* algo parameters from BSW */
        /* Currently BSW algo parameters has no signal header */
        /*VLCVehSyncRef.BSW_s_AlgoParameters   =
         * pRequirePorts->pBswAlgoParameters->sSigHeader;*/
        VLCVehSetSigHeaderError(&VLCVehSyncRef.BSW_s_AlgoParameters);

        /*pCPAR_VLC_Parameters*/
        if (pRequirePorts->pCPAR_VLC_Parameters != NULL) {
            VLCVehSyncRef.CPAR_VLC_Parameters =
                pRequirePorts->pCPAR_VLC_Parameters->sSigHeader;
        } else {
            VLCVehSyncRef.sSigHeader.eSigStatus = AL_SIG_STATE_INVALID;
        }

        /*pVehSig*/
        if (pRequirePorts->pVehSig != NULL) {
            VLCVehSyncRef.VehSig = pRequirePorts->pVehSig->sSigHeader;
        } else {
            VLCVehSyncRef.sSigHeader.eSigStatus = AL_SIG_STATE_INVALID;
        }

        if (pRequirePorts->pVehCtrlData != NULL) {
            /* fill signal header of SyncRef with valid data from Control Input
             */
            VLCVehFillSigHeader(&VLCVehSyncRef.sSigHeader,
                                &pRequirePorts->pVehCtrlData->sSigHeader);
            VLCVehSetSigHeaderState(&VLCVehSyncRef.sSigHeader, AL_SIG_STATE_OK);
        } else {
            VLCVehSyncRef.sSigHeader.eSigStatus = AL_SIG_STATE_INVALID;
        }
    } else {
        VLCVehSyncRef.sSigHeader.eSigStatus = AL_SIG_STATE_INVALID;
    }
}

/*************************************************************************************************************************
  Functionname:    VLCVehAlgoInit */
static void VLCVehAlgoInit(void) {
    /*--- VARIABLES ---*/
    VLCVehFrame.bFirstCycleDone = FALSE;
}

/*************************************************************************************************************************
  Functionname:    VLCVehNvmRead */
static void VLCVehNvmRead(const reqVLCVehPrtList_t* pRequirePorts) {
    // pRequirePorts->pLongCtrlResp->KinCtrlDynInput.acc_enable =
    //     pRequirePorts->pVLCVehNvRams->VLCVEH_Nb_ACCOnOffSwitch;
    pRequirePorts->pLongCtrlResp->KinCtrlDynInput.acc_enable =
        TRUE;
    pRequirePorts->pLongCtrlResp->Custom.LongCtrlInputCustom.ISA_Active =
        pRequirePorts->pVLCVehNvRams->VLCVEH_Nb_SLFOnOffSwitch;
    pRequirePorts->pAebVehSig->CDCS_AEB_OnOffSet = 0;
    // pRequirePorts->pVLCVehNvRams->VLCVEH_Nb_AEBOnOffSwitch;
    pRequirePorts->pAebVehSig->CDCS_FCW_OnOffSet = 0;
    // pRequirePorts->pVLCVehNvRams->VLCVEH_Nb_FCWOnOffSwitch;
    pRequirePorts->pAebVehSig->FCW_SensitiveLevel =
        pRequirePorts->pVLCVehNvRams->VLCVEH_Nu_FCWSensitivity;
}

/*************************************************************************************************************************
  Functionname:    VLCVehNvmWrite */
static void VLCVehNvmWrite(const reqVLCVehPrtList_t* pRequirePorts,
                           const proVLCVehPrtList_t* pProvidePorts) {
    // pProvidePorts->pVLCVehNvRams->VLCVEH_Nb_ACCOnOffSwitch =
    //     pRequirePorts->pLongCtrlResp->KinCtrlDynInput.acc_enable;
    pProvidePorts->pVLCVehNvRams->VLCVEH_Nb_ACCOnOffSwitch =
        TRUE;
    pProvidePorts->pVLCVehNvRams->VLCVEH_Nb_SLFOnOffSwitch =
        pRequirePorts->pLongCtrlResp->Custom.LongCtrlInputCustom.ISA_Active;
    // pProvidePorts->pVLCVehNvRams->VLCVEH_Nb_AEBOnOffSwitch =
    //     pRequirePorts->pAebVehSig->CDCS_AEB_OnOffSet;
    // pProvidePorts->pVLCVehNvRams->VLCVEH_Nb_FCWOnOffSwitch =
    //     pRequirePorts->pAebVehSig->CDCS_FCW_OnOffSet;
    pProvidePorts->pVLCVehNvRams->VLCVEH_Nu_FCWSensitivity =
        pRequirePorts->pAebVehSig->FCW_SensitiveLevel;
}

/*************************************************************************************************************************
  Functionname:    VLCVehSetProvideHeader */
static void VLCVehSetProvideHeader(const reqVLCVehPrtList_t* pRequirePorts,
                                   const proVLCVehPrtList_t* pProvidePorts) {
    VLCVehFillSigHeader(&pProvidePorts->pDFVLongOut->sSigHeader,
                        &pRequirePorts->pLongCtrlResp->sSigHeader);
    VLCVehFillSigHeader(&pProvidePorts->pLongCtrlOutput->sSigHeader,
                        &pRequirePorts->pLongCtrlResp->sSigHeader);

    VLCVehFillSigHeader(&pProvidePorts->pDIMOutputCustom->sSigHeader,
                        &pRequirePorts->pDIMInputGeneric->sSigHeader);

    VLCVehFillSigHeader(&pProvidePorts->pHEADOutputGeneric->sSigHeader,
                        &pRequirePorts->pVLCCDHypotheses->sSigHeader);
    VLCVehFillSigHeader(&pProvidePorts->pHEADOutputCustom->sSigHeader,
                        &pRequirePorts->pVLCCDHypotheses->sSigHeader);

    VLCVehFillSigHeader(&pProvidePorts->pErrorOut->sSigHeader,
                        &pRequirePorts->pEgoDynRaw->sSigHeader);

    VLCVehFillSigHeader(&pProvidePorts->pVLCVehOutArbitrated->sSigHeader,
                        &pRequirePorts->pEgoDynRaw->sSigHeader);
}

/*************************************************************************************************************************
  Functionname:    VLCVehSetProvideHeaderStates */
static void VLCVehSetProvideHeaderStates(
    const proVLCVehPrtList_t* pProvidePorts, AlgoSignalState_t eSigState) {
    VLCVehSetSigHeaderState(&pProvidePorts->pDFVLongOut->sSigHeader, eSigState);
    VLCVehSetSigHeaderState(&pProvidePorts->pLongCtrlOutput->sSigHeader,
                            eSigState);

    VLCVehSetSigHeaderState(&pProvidePorts->pDIMOutputCustom->sSigHeader,
                            eSigState);

    VLCVehSetSigHeaderState(&pProvidePorts->pHEADOutputGeneric->sSigHeader,
                            eSigState);
    VLCVehSetSigHeaderState(&pProvidePorts->pHEADOutputCustom->sSigHeader,
                            eSigState);

    VLCVehSetSigHeaderState(&pProvidePorts->pErrorOut->sSigHeader, eSigState);

    VLCVehSetSigHeaderState(&pProvidePorts->pVLCVehOutArbitrated->sSigHeader,
                            eSigState);
}

/*************************************************************************************************************************
  Functionname:    VLCVehSetStates */
static void VLCVehSetStates(DIMState_t StDIM,

                            HEADState_t StHEAD,

                            VLCVehState_t StVLC) {
    DIMState = StDIM;

    HEADState = StHEAD;

    VLCVehFrame.eVLCState = StVLC;
}

/*************************************************************************************************************************
  Functionname:    VLCVehProcessStates */
static void VLCVehProcessStates(VLC_OP_MODE_t eOpMode) {
    if ((VLCVehIsInitialized == FALSE) && (eOpMode != VLC_MOD_SHUTDOWN)) {
        eOpMode = VLC_MOD_INIT;
    }

    switch (eOpMode) {
        case (VLC_OP_MODE_t)VLC_MOD_STARTUP:
            VLCVehSetStates(VLC_VEH_STATE_PARAM(
                UDW_STATE_INIT, LCD_STATE_INIT, LCK_STATE_INIT, LKS_STATE_INIT,
                LKA_STATE_INIT, DIM_STATE_INIT, HEAD_STATE_INIT, VLC_VEH_INIT));
            VLCVehIsInitialized = FALSE;
            break;
        case (VLC_OP_MODE_t)VLC_MOD_INIT:
            VLCVehSetStates(VLC_VEH_STATE_PARAM(
                UDW_STATE_INIT, LCD_STATE_INIT, LCK_STATE_INIT, LKS_STATE_INIT,
                LKA_STATE_INIT, DIM_STATE_INIT, HEAD_STATE_INIT, VLC_VEH_INIT));
            VLCVehIsInitialized = FALSE;
            break;
        case (VLC_OP_MODE_t)VLC_MOD_RUNNING:
            VLCVehSetStates(VLC_VEH_STATE_PARAM(
                UDW_STATE_OK, LCD_STATE_OK, LCK_STATE_OK, LKS_STATE_OK,
                LKA_STATE_OK, DIM_STATE_OK, HEAD_STATE_OK, VLC_VEH_RUN));
            break;
        case (VLC_OP_MODE_t)VLC_MOD_SHUTDOWN:
        default:
            VLCVehSetStates(VLC_VEH_STATE_PARAM(
                UDW_STATE_INIT, LCD_STATE_INIT, LCK_STATE_INIT, LKS_STATE_INIT,
                LKA_STATE_INIT, DIM_STATE_INIT, HEAD_STATE_INIT,
                VLC_VEH_SHUTDOWN));
            break;
    }

    /*set HEAD opmode @todo: Remove this later, so only the normal Head
     * operation mode is used! */
    switch (VLCVehFrame.eVLCState) {
        case VLC_VEH_RUN:
            HEADSetOpMode(HEADOpMode_Running);
            break;
        case VLC_VEH_INIT:
            HEADSetOpMode(HEADOpMode_ShutDown);
            break;
        case VLC_VEH_SHUTDOWN:
            HEADSetOpMode(HEADOpMode_Stop);
            break;
        default:
            HEADSetOpMode(HEADOpMode_Stop);
            break;
    } /* endswitch */
}

/*************************************************************************************************************************
  Functionname:    VLCVehSignalErrorShutdown */
static void VLCVehSignalErrorShutdown(void) {
    VLCVehProcessStates(VLC_MOD_SHUTDOWN);
    VLCVehIsInitialized = FALSE;
}

/*************************************************************************************************************************
  Functionname:    VLCVehMeasCallback */
static void VLCVehMeasCallback(void)

{
    return;
}

/*************************************************************************************************************************
  Functionname:    VLCSetVehFrameData */
static void VLCSetVehFrameData(const reqVLCVehPrtList_t* pRequirePorts) {
    if ((pRequirePorts != NULL) && (pRequirePorts->pVehCtrlData != NULL)) {
        // VLCVehFrame.eVLCOpMode = pRequirePorts->pVehCtrlData->OpMode;
        VLCVehFrame.eVLCOpMode =
            VLC_MOD_RUNNING;  // pRequirePorts->pVehCtrlData->OpMode;
    } else {
        VLCVehFrame.eVLCOpMode = VLC_MOD_SHUTDOWN;
        VLCVehSignalErrorShutdown();
    }
    VLCVehFrame.uiCycleCounter++;

    VLCVehFrame.Versions.uiHEAD = HEAD_AUTOVERSION;

    VLCVehFrame.Versions.uiDIM = DIM_AUTOVERSION;

    VLCVehFrame.Versions.FctVersionNumVar = VLCALL_SW_VERSION_NUMBER;

    VLCVehFrame.Versions.uiVLCVEH = VLCALL_SW_VERSION_NUMBER;
}

/*************************************************************************************************************************
  Functionname:    VLCVehSetupPorts */
static void VLCVehSetupPorts(const reqVLCVehPrtList_t* pRequirePorts,
                             const proVLCVehPrtList_t* pProvidePorts) {
    GET_EGO_RAW_DATA_PTR = pRequirePorts->pEgoDynRaw;
    GET_EGO_STATIC_DATA_PTR = pRequirePorts->pEgoStaticData;

    GET_VLC_DIM_CUST_IN_DATA_PTR = pRequirePorts->pDIMInputCustom;
    GET_VLC_DIM_GENERIC_IN_DATA_PTR = pRequirePorts->pDIMInputGeneric;
    GET_VLC_DIM_CUST_OUT_DATA_PTR = pProvidePorts->pDIMOutputCustom;
    GET_VLC_DIM_CUST_OUT_DATA_PTR->uiVersionNumber = VLC_VEH_INTFVER;

    GET_VLC_HEAD_GENERIC_IN_DATA_PTR = pRequirePorts->pSADInputGeneric;
    GET_VLC_HEAD_CUST_IN_DATA_PTR = pRequirePorts->pSADInputCustom;

    GET_VLC_HEAD_CUST_OUT_DATA_PTR = pProvidePorts->pHEADOutputCustom;
    GET_VLC_HEAD_GENERIC_OUT_DATA_PTR = pProvidePorts->pHEADOutputGeneric;

    GET_VLC_HEAD_CUST_OUT_DATA_PTR->uiVersionNumber = VLC_VEH_INTFVER;
    GET_VLC_HEAD_GENERIC_OUT_DATA_PTR->uiVersionNumber = VLC_VEH_INTFVER;

    VLC_pCDHypothesesVeh = pRequirePorts->pVLCCDHypotheses;

    VLCVEH_pBswAlgoParameters = pRequirePorts->pBswAlgoParameters;

    VLCVEH_pCPAR_VLC_Parameters = pRequirePorts->pCPAR_VLC_Parameters;

    VLCVEH_pVehSig = pRequirePorts->pVehSig;

    VLCVeh_pCtrlData = pRequirePorts->pVehCtrlData;

    VLCVEH_pVehSigPowerTrain = &pRequirePorts->pVehSig->PowerTrain;

    VLCVEH_pAccLever = pRequirePorts->pAccLever;
    VLCVEH_pLongCtrlResp = pRequirePorts->pLongCtrlResp;

    VLCVEH_pAccDisplayObj = pRequirePorts->pAccDisplayObj;
    VLCVEH_pAccOutput = pRequirePorts->pAccOutput;
    VLCVEH_pDFVLongOut = pProvidePorts->pDFVLongOut;

    VLCVEH_pLongCtrlOutput = pProvidePorts->pLongCtrlOutput;

    VLCVEH_pLODMCOutput = pProvidePorts->pLODMCOutput;

    VLCVEH_pAccOOIData = pRequirePorts->pVLCAccOOIData;

    VLCVEH_pVLCVehOutArbitrated = pProvidePorts->pVLCVehOutArbitrated;
}

/*************************************************************************************************************************
  Functionname:    VLCVehCheckPorts */
static void VLCVehCheckPorts(const reqVLCVehPrtList_t* pRequirePorts,
                             const proVLCVehPrtList_t* pProvidePorts) {
    /* Verify that all request port pointers are set to non-null */
    if ((pRequirePorts == NULL) || (pRequirePorts->pVehCtrlData == NULL) ||
        (pRequirePorts->pEgoDynRaw == NULL) ||
        (pRequirePorts->pEgoStaticData == NULL) ||
        (pRequirePorts->pAccLever == NULL) ||
        (pRequirePorts->pLongCtrlResp == NULL) ||
        (pRequirePorts->pAccDisplayObj == NULL) ||
        (pRequirePorts->pAccOutput == NULL) ||
        (pRequirePorts->pVLCAccOOIData == NULL) ||
        (pRequirePorts->pDIMInputCustom == NULL) ||
        (pRequirePorts->pDIMInputGeneric == NULL) ||
        (pRequirePorts->pVLCCDHypotheses == NULL) ||
        (pRequirePorts->pSADInputGeneric == NULL) ||
        (pRequirePorts->pSADInputCustom == NULL) ||
        (pRequirePorts->pBswAlgoParameters == NULL) ||
        (pRequirePorts->pCPAR_VLC_Parameters == NULL) ||
        (pRequirePorts->pVehSig == NULL)) {
        /* Some request port pointer is NULL => DEM and shutdown */

        /* shutdown the system */
        VLCVehSignalErrorShutdown();
    } else {
        /* Verify that provide port buffer pointers are set (non-null) */
        if ((pProvidePorts == NULL) || (pProvidePorts->pDFVLongOut == NULL) ||
            (pProvidePorts->pLongCtrlOutput == NULL) ||
            (pProvidePorts->pDIMOutputCustom == NULL) ||
            (pProvidePorts->pHEADOutputGeneric == NULL) ||
            (pProvidePorts->pHEADOutputCustom == NULL) ||
            (pProvidePorts->pErrorOut == NULL) ||
            (pProvidePorts->pVLCVehOutArbitrated == NULL)) {
            VLCVehSignalErrorShutdown();
        } else {
        }
    }
}

void LODMCSimplePD() {

   // Input data pointers
   const VED_VehDyn_t* pVehDyn = VLCVEH_pEgoDynRaw;
   const VLC_LongCtrlOutput_t* pAccOut = VLCVEH_pLongCtrlOutput;
   const VED_VehSig_t* pVehSig = VLCVEH_pVehSig; // Not used here and in the coordinator

   // Output data pointers
   VLC_LODMCOutput_t* pLODMCOut = VLCVEH_pLODMCOutput; //define VLCVEH_pLODMCOutput

   // Internal data
   float acceleration_actual;
   float acceleration_request;
   float k_p = 1;
   float k_d = 1;
   float error;
   float torque_request;
   float torque_min = 0.0;
   float torque_max = 2500.0;


//    acceleration_actual = pVehDyn->Longitudinal.AccelCorr.corrAccel; // maybe wrong
   acceleration_actual = pVehDyn->Longitudinal.MotVar.Accel; // based on justin's feedback
//    acceleration_request = pAccOut->Custom.CustomOutput.RequestedLongAccelRaw;
   acceleration_request = pAccOut->KinOutput.MinRequestedLongAcceleration; // justin_note

   if (Simple_PD_LoDMC_flag == 0) {
       error = acceleration_request - acceleration_actual;
       error_previous = error;
       torque_request = k_p * error + 0 * (error - error_previous);
   }
   else {
       error = acceleration_request - acceleration_actual;
       torque_request = k_p * error + k_d * (error - error_previous)/0.1;
       error_previous = error;
   }

   if (torque_request < torque_min) {
           torque_request = torque_min;
   }

   else if (torque_request > torque_max) {
	   torque_request = torque_max;
   }

   //justin_note: to do add Calibration parameters for different velocity 
   if(acceleration_request > 0){
        pLODMCOut->TorqueActiveFlag = (boolean)1;
        pLODMCOut->EngineTRQReq = torque_request;
        pLODMCOut->BrakeActiveFlag = (boolean)0;  // Ignoring uphill/downhill brake request from LoDMC 
        pLODMCOut->DecelReq = 0;
   }else{
        pLODMCOut->TorqueActiveFlag = (boolean)0;
        pLODMCOut->EngineTRQReq = torque_request;
        pLODMCOut->BrakeActiveFlag = (boolean)1; 
        pLODMCOut->DecelReq = acceleration_request;
   }

}

/*************************************************************************************************************************
  Functionname:    VLCVeh_Exec */

void VLCVeh_Exec(const reqVLCVehPrtList_t* pRequirePorts,
                 const VLCVeh_Parameters_t* pVLCParameters,
                 const proVLCVehPrtList_t* pProvidePorts,
                 const reqVLCVehDebugList_t* pVLCVehDebugPorts) {
#if 0  
  for(int index = 0; index < 1; index++)
  {
    if(pRequirePorts->pVLCCDHypotheses->Hypothesis[index].uiObjectProbability != 0 || pRequirePorts->pVLCCDHypotheses->Hypothesis[index].uiHypothesisProbability != 0)
    {
      printf("ObjectProbability-->%d\n", pRequirePorts->pVLCCDHypotheses->Hypothesis[index].uiObjectProbability);
      printf("HypothesisProbability-->%d\n", pRequirePorts->pVLCCDHypotheses->Hypothesis[index].uiHypothesisProbability);
  
      printf("ttc-->%f\n", pRequirePorts->pVLCCDHypotheses->Hypothesis[index].fTTC);
      printf("ttc2-->%f\n", pRequirePorts->pVLCCDHypotheses->Hypothesis[index].fTTC2);
      printf("ttc3-->%f\n", pRequirePorts->pVLCCDHypotheses->Hypothesis[index].fTTC3);
      printf("ttc4-->%f\n", pRequirePorts->pVLCCDHypotheses->Hypothesis[index].fTTC4);

      printf("ttb_pre-->%f\n", pRequirePorts->pVLCCDHypotheses->Hypothesis[index].fTTBPre);
      printf("ttb_acute-->%f\n", pRequirePorts->pVLCCDHypotheses->Hypothesis[index].fTTBAcute);

      printf("tts_pre-->%f\n", pRequirePorts->pVLCCDHypotheses->Hypothesis[index].fTTSPre);
      printf("tts_acute-->%f\n", pRequirePorts->pVLCCDHypotheses->Hypothesis[index].fTTSAcute);

      printf("LongNecAccel-->%f\n", pRequirePorts->pVLCCDHypotheses->Hypothesis[index].fLongNecAccel);
      printf("DistX-->%f\n", pRequirePorts->pVLCCDHypotheses->Hypothesis[index].fDistX);

      printf("fVrelX-->%f\n", pRequirePorts->pVLCCDHypotheses->Hypothesis[index].fVrelX);
      printf("ArelX-->%f\n", pRequirePorts->pVLCCDHypotheses->Hypothesis[index].fArelX);
      printf("ClosingVelocity-->%f\n", pRequirePorts->pVLCCDHypotheses->Hypothesis[index].fClosingVelocity);     
    } 
  }
#endif
    // set parameters pointer input
    GET_VLCVEH_PARAMETERS = pVLCParameters;

    /* update Frame Data and set OpMode */
    VLCSetVehFrameData(pRequirePorts);

    /* set the process states for all subcomponents */
    VLCVehProcessStates(VLCVehFrame.eVLCOpMode);

    /* check for NULLpointer => trigger ErrorShutdown */
    VLCVehCheckPorts(pRequirePorts, pProvidePorts);
    /* setup the input port header sync structure */
    VLCVehSetupSyncRef(pRequirePorts);

    if ((VLCVehFrame.eVLCOpMode != VLC_MOD_SHUTDOWN) &&
        (VLCVehFrame.eVLCState != VLC_VEH_SHUTDOWN)) {
        /*Init in init Mode*/
        if (VLCVehFrame.eVLCState == VLC_VEH_INIT ||
            isVehMainInitializedForSimulation == FALSE) {
            VLCVehAlgoInit();
            VLCVehNvmRead(pRequirePorts);
            isVehMainInitializedForSimulation = TRUE;
        } else {
            VLCVehFrame.bFirstCycleDone = TRUE;
        }

        /* Opmode indicates liveliness => setup port pointers */
        VLCVehSetupPorts(pRequirePorts, pProvidePorts);

        /* set all the signal headers of all provide ports to invalid*/
        VLCVehSetProvideHeaderStates(pProvidePorts, AL_SIG_STATE_INVALID);
        VLCVehSetProvideHeader(pRequirePorts, pProvidePorts);

        /*---------------------------------------------------------------------------*/
        /* Start DIM */
        /*---------------------------------------------------------------------------*/
        /* Sends RTA-Start event for Driver Intention Monitoring runtime */
        DIMProcess(VLC_VEH_CYCLE_TIME);
        /* Sends RTA-End event for Driver Intention Monitoring runtime */
        //(JINXIN) add AEB Switch judgement here. if AEB switch is
        // off(eMainSwitch = 0),SAD will not be called
        // printf("[func:%s,line:%d] AEB Switch-->%d\n", __func__, __LINE__,
        //        pRequirePorts->pSADInputGeneric->eMainSwitch);
        // if (1 == pRequirePorts->pSADInputGeneric
        //              ->eMainSwitch)  // eMainSwitch:0(AEB Off), 1(AEB On)
        // {
        // SAD_Process();
        // }
        /* Sends RTA-Start event for Longitudinal control runtime */

        pProvidePorts->pLongCtrlOutput->uiVersionNumber = VLC_VEH_INTFVER;

        VLC_LC_EXEC((times_t)(VLC_VEH_CYCLE_TIME * 1000), pVLCVehDebugPorts,
                    pRequirePorts->pEgoDynRaw,
                    &pRequirePorts->pVehSig->PowerTrain,
                    pRequirePorts->pAccLever, pRequirePorts->pLongCtrlResp,
                    pRequirePorts->pAccDisplayObj, pRequirePorts->pAccOutput,
                    pRequirePorts->pVLCAccOOIData, pRequirePorts->pCamLaneData,
                    pProvidePorts->pDFVLongOut, pProvidePorts->pLongCtrlOutput,
                    pProvidePorts->p_isa_info, pProvidePorts->p_pacc_info);

        #if (VLC == 0)
            // Call coordinator LoDMC here - the generated from autoode! 
            uint8 fake_coordinator_loDMC = 0;
            // LODMCProcess();
        #elif (VLC == 1)
            LODMCSimplePD();
            // Implement PD controller: input: error acceleration, output: torque
            Simple_PD_LoDMC_flag = 1;
        #endif
        
        VLCVEHProcessCustomOutput();

        VLCVehFillErrorOut(pProvidePorts->pErrorOut);

        /* Set all signal header values again to avoid errors if a component
         * accidentialy overwrite the SigHeader*/
        VLCVehSetProvideHeader(pRequirePorts, pProvidePorts);

        /* Process meas freezes */
        {
            //      VLCVehProcessMeasFreeze(pProvidePorts);
        }

        /*computation chain ran through. VLC Vehicle is initialized or
         * running.*/
        VLCVehIsInitialized = TRUE;
    }

    else /************************* end of non-error path, beginning of error
            path *************************/
    {
        /* VLC_MOD_SHUTDOWN => fill invalidate the available provide ports and
         * fill whole sigHeader */
        VLCVehSetErrorProvidePorts(pProvidePorts);

        /* Init Algo */
        VLCVehAlgoInit();

        /*make sure in error case init of sub components is performed next
         * cylcle*/
        VLCVehIsInitialized = FALSE;
    }

    VLCVehNvmWrite(pRequirePorts, pProvidePorts);
    // pProvidePorts->pDIMOutputCustom->eDriverAttentionState = 10u;
}

/*************************************************************************************************************************
  Functionname:    VLCVehSetSigHeaderError */
static void VLCVehSetSigHeaderError(SignalHeader_t* const pSigHeader) {
    pSigHeader->uiCycleCounter = VLCVehFrame.uiCycleCounter;
    pSigHeader->eSigStatus = AL_SIG_STATE_INVALID;
}

/*************************************************************************************************************************
  Functionname:    VLCVehFillSigHeader */
static void VLCVehFillSigHeader(SignalHeader_t* const pSigHeader,
                                const SignalHeader_t* const pSourceHdr) {
    pSigHeader->uiTimeStamp = pSourceHdr->uiTimeStamp;
    pSigHeader->uiMeasurementCounter = pSourceHdr->uiMeasurementCounter;
    pSigHeader->uiCycleCounter = VLCVehFrame.uiCycleCounter;
}

/*************************************************************************************************************************
  Functionname:    VLCVehSetSigHeaderState */
static void VLCVehSetSigHeaderState(SignalHeader_t* const pSigHeader,
                                    AlgoSignalState_t eSigState) {
    pSigHeader->eSigStatus = eSigState;
}
/* ************************************************************************* */
/*   Copyright                                               */
/* ************************************************************************* */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU1_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
