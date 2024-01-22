/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "vlc_sen.h"
#include "cd_ext.h"

/*****************************************************************************
  MODULGLOBALE KONSTANTEN
*****************************************************************************/

/*****************************************************************************
  MODULGLOBALE VARIABLEN
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE SYMBOLISCHE KONSTANTEN
*****************************************************************************/

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

/*****************************************************************************
  FUNCTIONS
*****************************************************************************/

/*************************************************************************************************************************
  Functionname:    VLCSenFillErrorOut */
void VLCSenFillErrorOut(DFSErrorOut_t* pDest) {
    pDest->uiVersionNumber = VLC_SEN_INTFVER;
    pDest->sSigHeader.eSigStatus = AL_SIG_STATE_OK;
    /* Fill failure outputs with default unknown (i.e.: untested) state */
    pDest->ObjectNotMeasured = AL_ERR_STATE_UNKNOWN;
    pDest->ObjectNotMeasTmp = AL_ERR_STATE_UNKNOWN;
    pDest->Blockage = AL_ERR_STATE_UNKNOWN;
    pDest->IncreaseBlockage = AL_ERR_STATE_UNKNOWN;
    pDest->DecreaseBlockage = AL_ERR_STATE_UNKNOWN;
    pDest->BelowMinPerfDist_AZ = AL_ERR_STATE_UNKNOWN;
    pDest->BelowMinPerfDist_EL = AL_ERR_STATE_UNKNOWN;
    pDest->BelowMinPerfDist_VED = AL_ERR_STATE_UNKNOWN;
    /* Depending on VLC SI sub-component activation states and assessed object
    list signal status set object selection active/not active state */
    if (((SIState == SI_OK) || (SIState == SI_RED_QUAL)) &&
        (GET_VLC_PUB_OBJ_DATA_PTR->sSigHeader.eSigStatus == AL_SIG_STATE_OK)) {
        pDest->bObjSelectionActive = TRUE;
    } else {
        pDest->bObjSelectionActive = FALSE;
    }
    if (CDState == CD_STATE_OK) {
        pDest->bCollisionAvoidActive = TRUE;
    } else {
        pDest->bCollisionAvoidActive = FALSE;
    }
    pDest->bRequestRoadBeam = FALSE;
    /* Call SPM to fill in errors */
}

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */