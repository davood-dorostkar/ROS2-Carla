/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include <string.h>
#include "vlc_sen.h"
#include "frame_sen_wrapper.h"

#include "vlcSen_ext.h"
#include "cd_ext.h"
#include "vlc_long_sen_ext.h"
/*****************************************************************************
  MODULGLOBALE KONSTANTEN
*****************************************************************************/

/*****************************************************************************
  SYMBOLISCHE KONSTANTEN
*****************************************************************************/

#define VLC_HMI_INTFVER_MIN_G30_I390 0x03

#define VLC_LD_OUT_INTFVER_MIN_G30_I390 0x03

#define VLC_CB_OUT_INTFVER_MIN_G30_I390 0x07

#define VLC_LKA_OUT_GEN_INTFVER_MIN_G30_I390 0x0F

/*****************************************************************************
  MODULLOKALE SYMBOLISCHE KONSTANTEN
*****************************************************************************/

///*! The VLC private object data */
// VLCPrivObjectList_t VLCObjectList;
///* frame (cycle time, cycle counter, opmode ...) */
// VLCSenFrame_t VLCSenFrame;
/*****************************************************************************
  MODULLOKALE MAKROS
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE TYPEN
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE KONSTANTEN
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/

static void VLCSenAlgoInit(void);
static void VLCSenInitAssessedObjList(AssessedObjList_t* pFctObjList,
                                      AlgoSignalState_t eSigState);
static void VLCMergeDeleteObj(const Envm_t_GenObjectList* pObjList);
static void VLCSenCheckPorts(const reqVLCSenPrtList_t* pRequirePorts,
                             const proVLCSenPrtList_t* pProvidePorts);

static void VLCSetSenFrameData(const reqVLCSenPrtList_t* pRequirePorts);

static void VLCSenPreProcessing(const reqVLCSenPrtList_t* pRequirePorts,
                                const proVLCSenPrtList_t* pProvidePorts);
static void VLCSenPostProcessing(const reqVLCSenPrtList_t* pRequirePorts,
                                 const proVLCSenPrtList_t* pProvidePorts);
static void VLCSenSetInterfaceVersionProvidePorts(
    const proVLCSenPrtList_t* pProvidePorts);
static void VLCSenSetupPorts(const reqVLCSenPrtList_t* pRequirePorts,
                             const proVLCSenPrtList_t* pProvidePorts);

static void VLCSenSetupSyncRef(const reqVLCSenPrtList_t* pRequirePorts);
static void VLCSenSetErrorProvidePorts(const proVLCSenPrtList_t* pProvidePorts);
static void VLCSenSetProvideHeader(const reqVLCSenPrtList_t* pRequirePorts,
                                   const proVLCSenPrtList_t* pProvidePorts);
static void VLCSenSetProvideHeaderStates(
    const proVLCSenPrtList_t* pProvidePorts, AlgoSignalState_t eSigState);

/*! Utility function to set signal header to error */
static void VLCSenSetSigHeaderError(SignalHeader_t* const pSigHeader);

/*! Utility function to default fill signal header */
static void VLCSenFillSigHeader(SignalHeader_t* const pSigHeader,
                                const SignalHeader_t* const pSourceHdr);
static void VLCSenSetSigHeaderState(SignalHeader_t* const pSigHeader,
                                    AlgoSignalState_t eSigState);
/*! @endcond Doxygen_Suppress */

/*****************************************************************************
  FUNCTIONS
*****************************************************************************/

/*************************************************************************************************************************
  Functionname:    VLCSEN_SERVICE_ADD_EVENT */
void VLCSEN_SERVICE_ADD_EVENT(const AS_t_RtaEventType RtaEvtType,
                              const uint8 u_IdLocal,
                              const uint8 u_OptData) {
    _PARAM_UNUSED(RtaEvtType);
    _PARAM_UNUSED(u_IdLocal);
    _PARAM_UNUSED(u_OptData);
}

/*************************************************************************************************************************
  Functionname:    VLCSenSetErrorProvidePorts */
static void VLCSenSetErrorProvidePorts(
    const proVLCSenPrtList_t* pProvidePorts) {
    /* Provide ports */
    if (pProvidePorts != NULL) {
        if (pProvidePorts->pPubFctObj != NULL) {
            GET_VLC_PUB_OBJ_DATA_PTR = pProvidePorts->pPubFctObj;
            pProvidePorts->pPubFctObj->uiVersionNumber = VLC_SEN_INTFVER;
            VLCSenSetSigHeaderError(&pProvidePorts->pPubFctObj->sSigHeader);
        } else {
            GET_VLC_PUB_OBJ_DATA_PTR = NULL;
        }

        if (pProvidePorts->pCollDetOutput != NULL) {
            VLCSEN_pCDCustomOutput = pProvidePorts->pCollDetOutput;
            pProvidePorts->pCollDetOutput->uiVersionNumber = VLC_SEN_INTFVER;
            VLCSenSetSigHeaderError(&pProvidePorts->pCollDetOutput->sSigHeader);
        } else {
            VLCSEN_pCDCustomOutput = NULL;
        }

        if (pProvidePorts->pVLCCDHypotheses != NULL) {
            VLC_pCDHypothesesSen = pProvidePorts->pVLCCDHypotheses;
            pProvidePorts->pVLCCDHypotheses->uiVersionNumber = VLC_SEN_INTFVER;
            VLCSenSetSigHeaderError(
                &pProvidePorts->pVLCCDHypotheses->sSigHeader);
        } else {
            VLC_pCDHypothesesSen = NULL;
        }

        if (pProvidePorts->pAccDisplayObj != NULL) {
#if (defined(_MSC_VER))
#pragma message(__FILE__ "(" STRING_QUOTE( \
    __LINE__) "): warning TODO: Check if error management is working!")
#endif
            pProvidePorts->pAccDisplayObj->uiVersionNumber = VLC_SEN_INTFVER;
            VLCSenSetSigHeaderError(&pProvidePorts->pAccDisplayObj->sSigHeader);
        }
        if (pProvidePorts->pAccOutput != NULL) {
            pProvidePorts->pAccOutput->uiVersionNumber = VLC_SEN_INTFVER;
            VLCSenSetSigHeaderError(&pProvidePorts->pAccOutput->sSigHeader);
        }

        if (pProvidePorts->pVLCCustomOutput != NULL) {
#if (defined(_MSC_VER))
#pragma message(__FILE__ "(" STRING_QUOTE( \
    __LINE__) "): warning TODO: Check if error management is working!")

#endif
            /* customer specific input/output */
            pProvidePorts->pVLCCustomOutput->uiVersionNumber = VLC_SEN_INTFVER;
            VLCSenSetSigHeaderError(
                &pProvidePorts->pVLCCustomOutput->sSigHeader);
        }

        if (pProvidePorts->pErrorOut != NULL) {
#if (defined(_MSC_VER))
#pragma message(__FILE__ "(" STRING_QUOTE( \
    __LINE__) "): warning TODO: Check if error management is working!")
#endif
            pProvidePorts->pErrorOut->uiVersionNumber = VLC_SEN_INTFVER;
            VLCSenSetSigHeaderError(&pProvidePorts->pErrorOut->sSigHeader);
        }

        /* Provide OOI Objects from SEN to VEH */
        if (pProvidePorts->pVLCAccOOIData != NULL) {
            GET_VLC_AVLC_OOI_DATA_PTR = pProvidePorts->pVLCAccOOIData;
            pProvidePorts->pVLCAccOOIData->uiVersionNumber = VLC_SEN_INTFVER;
            VLCSenSetSigHeaderError(&pProvidePorts->pVLCAccOOIData->sSigHeader);
        }

    } else {
        GET_VLC_PUB_OBJ_DATA_PTR = NULL;
        VLCSEN_pCDCustomOutput = NULL;
        VLC_pCDHypothesesSen = NULL;
    }
}

/*************************************************************************************************************************
  Functionname:    VLCSenSetupSyncRef */
static void VLCSenSetupSyncRef(const reqVLCSenPrtList_t* pRequirePorts) {
    /*setting whole syncref to zero */
    (void)memset(&VLCSenSyncRef, 0, sizeof(VLCSenSyncRef));

    if (pRequirePorts != NULL) {
        /*pSenCtrlData*/
        if (pRequirePorts->pSenCtrlData != NULL) {
            VLCSenSyncRef.SenCtrlData = pRequirePorts->pSenCtrlData->sSigHeader;
        } else {
            VLCSenSyncRef.sSigHeader.eSigStatus = AL_SIG_STATE_INVALID;
        }

        /*pEgoDynObjSync*/
        if (pRequirePorts->pEgoDynObjSync != NULL) {
            VLCSenSyncRef.EgoDynObjSync =
                pRequirePorts->pEgoDynObjSync->sSigHeader;
            // and by LiuYang 2019-05-30 for eSigStatus must be 1
            VLCSenSyncRef.EgoDynObjSync.eSigStatus = 1;
        } else {
            VLCSenSyncRef.sSigHeader.eSigStatus = AL_SIG_STATE_INVALID;
        }

        /*EgoDynRaw*/
        if (pRequirePorts->pEgoDynRaw != NULL) {
            VLCSenSyncRef.EgoDynRaw = pRequirePorts->pEgoDynRaw->sSigHeader;
        } else {
            VLCSenSyncRef.sSigHeader.eSigStatus = AL_SIG_STATE_INVALID;
        }

        /*EgoStaticData*/
        if (pRequirePorts->pEgoStaticData != NULL) {
            VLCSenSyncRef.EgoStaticData =
                pRequirePorts->pEgoStaticData->sSigHeader;
        } else {
            VLCSenSyncRef.sSigHeader.eSigStatus = AL_SIG_STATE_INVALID;
        }

        /*pECAMtCyclEnvmode*/
        /* Cycle mode struct currently has no signal header, reset to zero */
        /*VLCSenSyncRef.EmFctCycleMode        =
         * pRequirePorts->pECAMtCyclEnvmode->sSigHeader;*/
        VLCSenSetSigHeaderError(&VLCSenSyncRef.EmFctCycleMode);

        /*pEmGenObjList*/
        if (pRequirePorts->pEmGenObjList != NULL) {
            VLCSenSyncRef.EmGenObjList =
                pRequirePorts->pEmGenObjList->sSigHeader;
        } else {
            VLCSenSyncRef.sSigHeader.eSigStatus = AL_SIG_STATE_INVALID;
        }

        /*pEmARSObjList*/
        if (pRequirePorts->pEmARSObjList != NULL) {
            VLCSenSyncRef.EmARSObjList =
                pRequirePorts->pEmARSObjList->sSigHeader;
        } else {
            VLCSenSyncRef.sSigHeader.eSigStatus = AL_SIG_STATE_INVALID;
        }

        /*pDFVLongOut*/
        if (pRequirePorts->pDFVLongOut != NULL) {
            VLCSenSyncRef.DFVLongOut = pRequirePorts->pDFVLongOut->sSigHeader;
        } else {
            VLCSenSyncRef.sSigHeader.eSigStatus = AL_SIG_STATE_INVALID;
        }

        /*pVLCCustomInput*/
        if (pRequirePorts->pVLCCustomInput != NULL) {
            VLCSenSyncRef.VLCCustomInput =
                pRequirePorts->pVLCCustomInput->sSigHeader;
        } else {
            VLCSenSyncRef.sSigHeader.eSigStatus = AL_SIG_STATE_INVALID;
        }

        /* BSW algo parameters currently has no signal header */
        /*VLCSenSyncRef.BswAlgoParameters       =
         * pRequirePorts->pBswAlgoParameters->sSigHeader;*/
        VLCSenSetSigHeaderError(&VLCSenSyncRef.BswAlgoParameters);

        /*pCPAR_VLC_Parameters*/
        if (pRequirePorts->pCPAR_VLC_Parameters != NULL) {
            VLCSenSyncRef.CPAR_VLC_Parameters =
                pRequirePorts->pCPAR_VLC_Parameters->sSigHeader;
        } else {
            VLCSenSyncRef.sSigHeader.eSigStatus = AL_SIG_STATE_INVALID;
        }

        /*pCamLaneData*/
        if (pRequirePorts->pCamLaneData != NULL) {
            VLCSenSyncRef.CamLaneData = pRequirePorts->pCamLaneData->sSigHeader;
        } else {
            VLCSenSyncRef.sSigHeader.eSigStatus = AL_SIG_STATE_INVALID;
        }

        if (pRequirePorts->pSenCtrlData != NULL) {
            /* fill signal header of SyncRef with valid data from Control Input
             */
            VLCSenFillSigHeader(&VLCSenSyncRef.sSigHeader,
                                &pRequirePorts->pSenCtrlData->sSigHeader);
            VLCSenSetSigHeaderState(&VLCSenSyncRef.sSigHeader, AL_SIG_STATE_OK);
        } else {
            VLCSenSyncRef.sSigHeader.eSigStatus = AL_SIG_STATE_INVALID;
        }
    } else {
        VLCSenSyncRef.sSigHeader.eSigStatus = AL_SIG_STATE_INVALID;
    }
}

/*************************************************************************************************************************
  Functionname:    VLCSenAlgoInit */
static void VLCSenAlgoInit(void) {
    /* reset FirstCycleDone */
    VLCSenFrame.bFirstCycleDone = FALSE;
    /* initialize custom Data for object list */
    VLCInitCustomObjectList();

    VLCSenIsInitialized = FALSE;

    /* Initialize a default non-zero Sensor Position Relative to CoG
     (real distance will be calculated each cycle when not in init mode) */
    VLC_fBumperToCoG = VLC_SEN_BUMPER2COG_DIST_DEFAULT;
}

/*************************************************************************************************************************
  Functionname:    VLCSenInitAssessedObjList */
static void VLCSenInitAssessedObjList(AssessedObjList_t* pFctObjList,
                                      AlgoSignalState_t eSigState) {
    eObjOOI_t i;
    ObjNumber_t j;
    pFctObjList->uiVersionNumber = VLC_SEN_INTFVER;
    pFctObjList->HeaderAssessedObjList.iNumOfUsedObjects = 0;

    pFctObjList->sSigHeader.uiCycleCounter = VLCSenFrame.uiCycleCounter;
    pFctObjList->sSigHeader.eSigStatus = eSigState;

    /* Reset header array with indices of OOI objects */
    for (i = OBJ_NEXT_OOI; i < OBJ_NEXT_LAT_RIGHT_OOI; i++) {
        pFctObjList->HeaderAssessedObjList.aiOOIList[i] = OBJ_INDEX_NO_OBJECT;
    }
    /* Default object loss information is none */
    pFctObjList->HeaderAssessedObjList.eRelObjLossReason = OBJ_LOSS_NO_INFO;
    /* Go through all objects */
    for (j = 0; j < Envm_N_OBJECTS; j++) {
        /* Reset lane information */
        pFctObjList->ObjList[j].LaneInformation.eAssociatedLane =
            ASSOC_LANE_UNKNOWN;
        pFctObjList->ObjList[j].LaneInformation.uiCutInProbability = 0u;
        pFctObjList->ObjList[j].LaneInformation.uiCutOutProbability = 0u;
        /* Reset object of interest information */
        pFctObjList->ObjList[j].ObjOfInterest.cExternalID = 255u;
        pFctObjList->ObjList[j].ObjOfInterest.eObjOOI = OBJ_NOT_OOI;
    }
}

/*************************************************************************************************************************
  Functionname:    VLCMergeDeleteObj */
#if (defined(_MSC_VER))
#pragma message(__FILE__ "(" STRING_QUOTE( \
    __LINE__) "): warning TODO: Split Logic is missing completely, unclear behaviour in case of split!")
#endif
static void VLCMergeDeleteObj(const Envm_t_GenObjectList* pObjList) {
    ObjNumber_t iObj;
    /*! First: Delete the VLC-content of "NEW"-objects which have no merge
       information from the last cycle (eSplitMergeState =
       Envm_GEN_OBJECT_SM_STATE_NONE).
        Necessary, if another object has a merge reference to a "NEW" object. */
    for (iObj = (ObjNumber_t)0; iObj < Envm_N_OBJECTS; iObj++) {
        if ((pObjList->aObject[iObj].General.eMaintenanceState ==
             Envm_GEN_OBJECT_MT_STATE_NEW) &&
            (pObjList->aObject[iObj].General.eSplitMergeState ==
             Envm_GEN_OBJECT_SM_STATE_NONE)) {
            SIDeleteObject(iObj);
            CDDeleteObject(iObj);
        }
    }
    /*! Second: Do the "merge" operation (copy information of the last cycle if
     * necessary) */
    for (iObj = (ObjNumber_t)0; iObj < Envm_N_OBJECTS; iObj++) {
        const Envm_t_GenObject* pCurObj = &(pObjList->aObject[iObj]);
        const uint8 uiObjMergeFlag = pCurObj->General.uiMergeID;
        /* VLC_USE_EM_GENERIC_OBJECT_LIST */
        /*! If the object's merge-split ID is a valid ID (range between 0 and
        Envm_N_OBJECTS -1), then a merge operation took place in the current EM
        cycle. If the merge ID is not equal the object ID iObj, the object is
        merged into another object. If the merge ID is equal the object ID iObj,
        the object in merged into a new object on the same VLC-list position and
        a special handing is necessary. If in case if no merge the merge ID is
        Envm_GEN_OBJECT_SM_ID_NONE, in case of merge to unknown object the merge
        ID is Envm_GEN_OBJECT_SM_ID_NONE.
        Both cases are handled as if no merge took place since no information
        about the object in VLC available. */

        /*! Merge, if valid ID in uiMergeID and if uiSplitMergeID != ID of
         * object */
        if ((uiObjMergeFlag >= Envm_N_OBJECTS) ||
            (uiObjMergeFlag != OBJ_GET_ID_I(iObj))) {
            /* For safeties sake verify that the merge flag is valid */
            if (uiObjMergeFlag < Envm_N_OBJECTS) {
                /* Cast is save as validity is checked */
                const ObjNumber_t iObjToKeep = (ObjNumber_t)uiObjMergeFlag;
                SIMergeObjects(iObjToKeep, iObj);
                CDMergeObjects(iObjToKeep, iObj);
            }
            /* If the object is set to deleted, then delete any information
             * associated */
            if ((OBJ_IS_DELETED(iObj)) || (OBJ_IS_NEW(iObj))) {
                SIDeleteObject(iObj);
                CDDeleteObject(iObj);
            }
        } else {
            /*! The old object is merged into the new object at the same
              position in the VLC-list
              ->  First, the information which is copied during the merge, has
              to be stored locally.
                  Than, the old VLC-object can be deleted and afterwards the
              "merge"-information can be considered. */
            SIMergeDeleteObjectSameVLCID(iObj);
            CDMergeDeleteObjectsSameVLCID(iObj);
        }
    }
}

/*************************************************************************************************************************
  Functionname:    VLCSenCheckPorts */
static void VLCSenCheckPorts(const reqVLCSenPrtList_t* pRequirePorts,
                             const proVLCSenPrtList_t* pProvidePorts) {
    /* Verify that all request port pointers are set to non-null */
    if ((pRequirePorts == NULL) || (pRequirePorts->pSenCtrlData == NULL) ||
        (pRequirePorts->pECAMtCyclEnvmode == NULL) ||
        (pRequirePorts->pEmGenObjList == NULL) ||
        (pRequirePorts->pEmARSObjList == NULL) ||
        (pRequirePorts->pEgoDynObjSync == NULL) ||
        (pRequirePorts->pEgoDynRaw == NULL) ||
        (pRequirePorts->pEgoStaticData == NULL) ||
        (pRequirePorts->pDFVLongOut == NULL) ||
        (pRequirePorts->pVLCCustomInput == NULL) ||
        (pRequirePorts->pBswAlgoParameters == NULL) ||
        (pRequirePorts->pCPAR_VLC_Parameters == NULL) ||
        (pRequirePorts->pCamLaneData == NULL)) {
        /* Some request port pointer is NULL => DEM and shutdown */

        /*validity of service pointer isn't clear here, anyway assign internal
         * global alias*/

        VLCSenSignalErrorShutdown(FALSE);
    } else {
        /*SigStatus not invalid in all interfaces*/

        /* Verify that provide port buffer pointers are set (non-null) */
        if ((pProvidePorts == NULL) || (pProvidePorts->pPubFctObj == NULL) ||
            (pProvidePorts->pAccDisplayObj == NULL) ||
            (pProvidePorts->pAccOutput == NULL) ||
            (pProvidePorts->pVLCCustomOutput == NULL) ||
            (pProvidePorts->pCollDetOutput == NULL) ||
            (pProvidePorts->pVLCCDHypotheses == NULL) ||
            (pProvidePorts->pErrorOut == NULL) ||
            (pProvidePorts->pVLCAccOOIData == NULL)) {
            /* Some provide port pointer is NULL => DEM -> shutdown */
            VLCSenSignalErrorShutdown(FALSE);
        } else {
            /* receive and provide ports are fine */
        }
    }
}

/*************************************************************************************************************************
  Functionname:    VLCSetSenFrameData */
static void VLCSetSenFrameData(const reqVLCSenPrtList_t* pRequirePorts) {
    /* Store operation mode received from outside in sen frame for freezing */
    if ((pRequirePorts != NULL) && (pRequirePorts->pSenCtrlData != NULL)) {
        // VLCSenFrame.eVLCOpMode = pRequirePorts->pSenCtrlData->OpMode;
        VLCSenFrame.eVLCOpMode = VLC_MOD_RUNNING;
    } else {
        VLCSenFrame.eVLCOpMode = VLC_MOD_SHUTDOWN;
        VLCSenSignalErrorShutdown(FALSE);
    }
    VLCSenFrame.uiCycleCounter++;

    VLCSenFrame.Versions.uiCP = CP_SW_VERSION_NUMBER;
    VLCSenFrame.Versions.uiSI = SI_SW_VERSION_NUMBER;
    VLCSenFrame.Versions.uiCD = CD_SW_VERSION_NUMBER;
    VLCSenFrame.Versions.uiFIP = FIP_SW_VERSION_NUMBER;
    VLCSenFrame.Versions.uProjectID = VLCALL_SW_PROJ_ID;
    VLCSenFrame.Versions.FctVersionNumVar = VLCALL_SW_VERSION_NUMBER;
    VLCSenFrame.Versions.uiVLCSEN = VLCALL_SW_VERSION_NUMBER;
}

/*************************************************************************************************************************
  Functionname:    VLCSenSetProvideHeader */
static void VLCSenSetProvideHeader(const reqVLCSenPrtList_t* pRequirePorts,
                                   const proVLCSenPrtList_t* pProvidePorts) {
    VLCSenFillSigHeader(&pProvidePorts->pPubFctObj->sSigHeader,
                        &pRequirePorts->pEmGenObjList->sSigHeader);
    VLCSenFillSigHeader(&pProvidePorts->pAccDisplayObj->sSigHeader,
                        &pRequirePorts->pEmGenObjList->sSigHeader);
    VLCSenFillSigHeader(&pProvidePorts->pAccOutput->sSigHeader,
                        &pRequirePorts->pEmGenObjList->sSigHeader);
    VLCSenFillSigHeader(&pProvidePorts->pVLCCustomOutput->sSigHeader,
                        &pRequirePorts->pEmGenObjList->sSigHeader);
    VLCSenFillSigHeader(&pProvidePorts->pCollDetOutput->sSigHeader,
                        &pRequirePorts->pEmGenObjList->sSigHeader);
    VLCSenFillSigHeader(&pProvidePorts->pVLCCDHypotheses->sSigHeader,
                        &pRequirePorts->pEmGenObjList->sSigHeader);
    VLCSenFillSigHeader(&pProvidePorts->pErrorOut->sSigHeader,
                        &pRequirePorts->pEmGenObjList->sSigHeader);
    VLCSenFillSigHeader(&pProvidePorts->pVLCAccOOIData->sSigHeader,
                        &pRequirePorts->pEmGenObjList->sSigHeader);
}

/*************************************************************************************************************************
  Functionname:    VLCSenSetProvideHeaderStates */
static void VLCSenSetProvideHeaderStates(
    const proVLCSenPrtList_t* pProvidePorts, AlgoSignalState_t eSigState) {
    VLCSenSetSigHeaderState(&pProvidePorts->pPubFctObj->sSigHeader, eSigState);
    VLCSenSetSigHeaderState(&pProvidePorts->pAccDisplayObj->sSigHeader,
                            eSigState);
    VLCSenSetSigHeaderState(&pProvidePorts->pAccOutput->sSigHeader, eSigState);
    VLCSenSetSigHeaderState(&pProvidePorts->pVLCCustomOutput->sSigHeader,
                            eSigState);
    VLCSenSetSigHeaderState(&pProvidePorts->pCollDetOutput->sSigHeader,
                            eSigState);
    VLCSenSetSigHeaderState(&pProvidePorts->pVLCCDHypotheses->sSigHeader,
                            eSigState);
    VLCSenSetSigHeaderState(&pProvidePorts->pErrorOut->sSigHeader, eSigState);
    VLCSenSetSigHeaderState(&pProvidePorts->pVLCAccOOIData->sSigHeader,
                            eSigState);
}

/*************************************************************************************************************************
  Functionname:    VLCSenPreProcessing */
static void VLCSenPreProcessing(const reqVLCSenPrtList_t* pRequirePorts,
                                const proVLCSenPrtList_t* pProvidePorts)

{
    VLCSenSetInterfaceVersionProvidePorts(pProvidePorts);

    /* If operation mode indicates run, but we our object list is invalid, then
    force to init mode. Note: VLC_SEN_RUN Implies that InputPort pointers are
    valid. */

    if ((VLCSenFrame.eVLCState == VLC_SEN_RUN) &&
        (pRequirePorts->pEmGenObjList->sSigHeader.eSigStatus !=
         AL_SIG_STATE_OK)) {
        /*changed by LiuYang 2019-05-30 for debug start */
        //    VLCSenSignalErrorShutdown(TRUE); /*Shutdown without error*/
        /*changed by LiuYang 2019-05-30 for debug end */
    }

    VLCSenProcessInput(TASK_CYCLE_TIME, pRequirePorts->pEgoDynObjSync);

    VLCPreProcessObjectList(&VLCObjectList, pRequirePorts);

    /*! Do VLC Input Preprocessing */
    FIPProcess();

    switch (VLCSenFrame.eVLCState) {
        case VLC_SEN_RUN:
        case VLC_SEN_RG_HIGH:
            /*! Set information: cycle was processed */
            VLCSenFrame.bFirstCycleDone = TRUE;
            /* Merge and Delete or Delete vlc Object data*/
            VLCMergeDeleteObj(pRequirePorts->pEmGenObjList);
            break;

        case VLC_SEN_INIT:
            /* init TP frame */
            VLCSenAlgoInit();
            break;

        case VLC_SEN_SHUTDOWN:
            /* Keep object list up-to-date even in shutdown: merge and delete or
             * delete vlc Object data*/
            VLCMergeDeleteObj(pRequirePorts->pEmGenObjList);
            break;

        case VLC_SEN_ERROR:
        default:
            /* init EM frame */
            VLCSenAlgoInit();
            break;
    } /* endswitch */
}

/*************************************************************************************************************************
  Functionname:    VLCSenPostProcessing */
static void VLCSenPostProcessing(const reqVLCSenPrtList_t* pRequirePorts,
                                 const proVLCSenPrtList_t* pProvidePorts) {
    _PARAM_UNUSED(pRequirePorts);

    switch (VLCSenFrame.eVLCState) {
        case VLC_SEN_RUN:
        case VLC_SEN_RG_HIGH:
            break;
        case VLC_SEN_INIT:
            /* In shutdown/init mode set the status of the assessed object list
             * to init */
            VLCSenInitAssessedObjList(pProvidePorts->pPubFctObj,
                                      AL_SIG_STATE_INIT);
            break;
        case VLC_SEN_SHUTDOWN:
            /* In shutdown/init mode set the status of the assessed object list
             * to invalid */
            VLCSenInitAssessedObjList(pProvidePorts->pPubFctObj,
                                      AL_SIG_STATE_INVALID);
            break;
        case VLC_SEN_ERROR:
        default:
            break;
    } /* endswitch */
}

/*************************************************************************************************************************
  Functionname:    VLCSenSetupPorts */
static void VLCSenSetupPorts(const reqVLCSenPrtList_t* pRequirePorts,
                             const proVLCSenPrtList_t* pProvidePorts) {
    /* Initialize global pointers used by macros */
    /* Request Ports */
    VLCSEN_pSenCtrlData = pRequirePorts->pSenCtrlData;
    GET_Envm_VLC_CYCLE_MODE_PTR = pRequirePorts->pECAMtCyclEnvmode;

    VLCSEN_pEmGenObjList = pRequirePorts->pEmGenObjList;

    VLCSEN_pEmARSObjList = pRequirePorts->pEmARSObjList;

    GET_EGO_OBJ_SYNC_DATA_PTR = pRequirePorts->pEgoDynObjSync;
    GET_EGO_RAW_DATA_PTR = pRequirePorts->pEgoDynRaw;
    GET_EGO_STATIC_DATA_PTR = pRequirePorts->pEgoStaticData;

    /* customer specific input/output */
    VLCSEN_pCustomOutput = pProvidePorts->pVLCCustomOutput;
    VLCSEN_pCustomInput = pRequirePorts->pVLCCustomInput;

    VLCSEN_pBswAlgoParameters = pRequirePorts->pBswAlgoParameters;

    VLCSEN_pCPAR_VLC_Parameters = pRequirePorts->pCPAR_VLC_Parameters;

    VLCSEN_pCamLaneData = pRequirePorts->pCamLaneData;

    /* Provide ports */
    GET_VLC_PUB_OBJ_DATA_PTR = pProvidePorts->pPubFctObj;

    VLC_pCDHypothesesSen = pProvidePorts->pVLCCDHypotheses;

    VLCSEN_pCDCustomOutput = pProvidePorts->pCollDetOutput;

    /* Provide OOI Objects from SEN to VEH */
    VLCSEN_pAccOOIData = pProvidePorts->pVLCAccOOIData;
}

/*************************************************************************************************************************
  Functionname:    VLCSenSetInterfaceVersionProvidePorts */
static void VLCSenSetInterfaceVersionProvidePorts(
    const proVLCSenPrtList_t* pProvidePorts) {
    pProvidePorts->pPubFctObj->uiVersionNumber = VLC_SEN_INTFVER;

    pProvidePorts->pAccOutput->uiVersionNumber = VLC_SEN_INTFVER;
    pProvidePorts->pAccDisplayObj->uiVersionNumber = VLC_SEN_INTFVER;

    pProvidePorts->pVLCCustomOutput->uiVersionNumber = VLC_SEN_INTFVER;

    pProvidePorts->pCollDetOutput->uiVersionNumber = VLC_SEN_INTFVER;

    pProvidePorts->pVLCCDHypotheses->uiVersionNumber = VLC_SEN_INTFVER;

    pProvidePorts->pErrorOut->uiVersionNumber = VLC_SEN_INTFVER;

    pProvidePorts->pVLCAccOOIData->uiVersionNumber = VLC_SEN_INTFVER;
}

/*************************************************************************************************************************
  Functionname:    VLCSen_Exec */
void VLCSen_Exec(const reqVLCSenPrtList_t* pRequirePorts,
                 const VLCSen_Parameters_t* pVLCParameters,
                 const proVLCSenPrtList_t* pProvidePorts) {
    boolean bSensorBlocked = FALSE;

    /*---------------------------------------------------------------------------*/
    /* Start data preparation */
    /*---------------------------------------------------------------------------*/

    // set parameters pointer input
    GET_VLCSEN_PARAMETERS = pVLCParameters;

    /* set the FrameData for Sensor */
    VLCSetSenFrameData(pRequirePorts);
    /* set process state from CtrlData - will be overwritten by other procedures
     */
    /* setup opmodes for subfuctions */
    VLCSenProcessStates(VLCSenFrame.eVLCOpMode);

    VLCSenCheckPorts(pRequirePorts, pProvidePorts);
    /* setup sync ref no matter what (sync in running and non running mode)*/
    VLCSenSetupSyncRef(pRequirePorts);

    if (VLCSenFrame.eVLCState != VLC_SEN_ERROR) {
        /* Ports OK, CtrlData indicates no error */
        VLCSenSetupPorts(pRequirePorts, pProvidePorts);

        /* set all the signal headers of all provide ports to invalid*/
        VLCSenSetProvideHeaderStates(pProvidePorts, AL_SIG_STATE_INVALID);
        VLCSenSetProvideHeader(pRequirePorts, pProvidePorts);

        /*---------------------------------------------------------------------------*/
        /* Start Pre-Processing (VLC) */
        /*---------------------------------------------------------------------------*/
        /* VLC Main Time */
        VLCSenPreProcessing(pRequirePorts, pProvidePorts);

        /*---------------------------------------------------------------------------*/
        /* Start SituationAnalysis (SA / CP) */
        /*---------------------------------------------------------------------------*/

        VLCCPProcess();

        /*---------------------------------------------------------------------------*/
        /* Start SituationAnalysis (SI) */
        /*---------------------------------------------------------------------------*/

        VLCSIProcess();

        /*---------------------------------------------------------------------------*/
        /* Start APIA Analysis (Collision Detection) */
        /*---------------------------------------------------------------------------*/

        VLCCDProcess();

        /*---------------------------------------------------------------------------*/
        /* Start longitudinal controller processing */
        /*---------------------------------------------------------------------------*/

        VLC_LONG_EXEC((times_t)(TASK_CYCLE_TIME * Time_s), bSensorBlocked,
                      pRequirePorts->pEmGenObjList, pProvidePorts->pPubFctObj,
                      pRequirePorts->pEgoDynRaw, pRequirePorts->pDFVLongOut,
                      pRequirePorts->pVLCCustomInput,
                      pProvidePorts->pAccDisplayObj,
                      pProvidePorts->pVLCAccOOIData, pProvidePorts->pAccOutput,
                      pProvidePorts->p_overtake_assist_info);

        /*---------------------------------------------------------------------------*/
        /* Start Nachbearbeitung Targetprocessing */
        /*---------------------------------------------------------------------------*/

        VLCSenFillErrorOut(pProvidePorts->pErrorOut);
        VLCSenPostProcessing(pRequirePorts, pProvidePorts);

        /* setup all provide port headers (cyclecounter, measurementcounter,
         * timestamp) */
        VLCSenSetProvideHeader(pRequirePorts, pProvidePorts);

        /* Output Meas Freezes */
        /* Process other meas freezes */
        VLCSenProcessMeasFreeze(pProvidePorts);

        /*computation chain ran through. VLC Sensor is initialized or running.*/
        VLCSenIsInitialized = TRUE;

        /* VLC Main Time */

    } /*(VLCSenState != VLC_SEN_ERROR) */
    else {
        /* error occured, set available provide ports */
        VLCSenSetErrorProvidePorts(pProvidePorts);

        VLCSenAlgoInit();

        /*make sure in error case init of sub components is performed next
         * cycle*/
        VLCSenIsInitialized = FALSE;
    }
}

/*************************************************************************************************************************
  Functionname:    VLCSenSetSigHeaderError */
static void VLCSenSetSigHeaderError(SignalHeader_t* const pSigHeader) {
    pSigHeader->uiTimeStamp = 0u;
    pSigHeader->uiMeasurementCounter = 0u;
    pSigHeader->uiCycleCounter = VLCSenFrame.uiCycleCounter;
    pSigHeader->eSigStatus = AL_SIG_STATE_INVALID;
}

/*************************************************************************************************************************
  Functionname:    VLCSenFillSigHeader */
static void VLCSenFillSigHeader(SignalHeader_t* const pSigHeader,
                                const SignalHeader_t* const pSourceHdr) {
    pSigHeader->uiTimeStamp = pSourceHdr->uiTimeStamp;
    pSigHeader->uiMeasurementCounter = pSourceHdr->uiMeasurementCounter;
    pSigHeader->uiCycleCounter = VLCSenFrame.uiCycleCounter;
}

/*************************************************************************************************************************
  Functionname:    VLCSenSetSigHeaderState */
static void VLCSenSetSigHeaderState(SignalHeader_t* const pSigHeader,
                                    AlgoSignalState_t eSigState) {
    pSigHeader->eSigStatus = eSigState;
}
/* ************************************************************************* */
/*   Copyright                                               */
/* ************************************************************************* */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */