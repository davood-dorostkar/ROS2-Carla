/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "si.h"
#include "si_par.h"

/*****************************************************************************
  MODULGLOBALE KONSTANTEN
*****************************************************************************/

/*****************************************************************************
  MODULGLOBALE VARIABLEN
*****************************************************************************/
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
SET_MEMSEC_VAR(SILastCycleOOIObjID)
ObjNumber_t SILastCycleOOIObjID[SiAnzOOI];
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
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
  MODULLOKALE VARIABLEN
*****************************************************************************/

void SISelectObjectInOOI(const sint8 nr, const ObjNumber_t CurObjId) {
    /* The object was already one of the objects of interest in the
            previous cycle, so any state transition processing needed */
    switch (nr) {
        /* Reset potentials if object changed lane from lateral to host lane.
            Reason: Potential on lateral lanes is Cut In potential, potential on
           host lane is Cut out potential, so these are differnt things and have
           to be calculated totally new.
            @todo: respect eventually new variable for Cut Out potential !!!!!
            @todo: or better yet, don't do filtering of value, instead
           recalculate in each cycle: that way no feedback loops exist! */
        case OBJ_NEXT_OOI:
        case OBJ_HIDDEN_NEXT_OOI:
            if (OBJ_GET_OOI_POS(CurObjId) > OBJ_HIDDEN_NEXT_OOI) {
                OBJ_GET_CUT_IN_POTENTIAL(CurObjId) = 0u;
            }
            break;

        case OBJ_NEXT_LONG_LEFT_OOI:
        case OBJ_NEXT_LONG_RIGHT_OOI:
        case OBJ_NEXT_LAT_LEFT_OOI:
        case OBJ_NEXT_LAT_RIGHT_OOI:
            if (OBJ_GET_OOI_POS(CurObjId) <= OBJ_HIDDEN_NEXT_OOI) {
                OBJ_GET_CUT_IN_POTENTIAL(CurObjId) = 0u;

                /* @todo: Balkon eventuell wieder einreissen, wenn neuer Tracker
                   verfuegbar Notwendig, um Vorschau bei Spurwechsel auch fuer
                   Objekte jenseits des Grenzwinkels aktiv zu halten. Sonst
                   bekommen Objekte, die gerade von der Hostspur auf die
                   Nachbarspur gewechselt haben trotz la- teraler
                   Geschwindigkeit vom Host weg Cut-In potential !!! */
            }
            break;

        default:
            break;
    }
}

/*************************************************************************************************************************
  Functionname:    SISelectBaseObjectsOfInterest */
void SISelectBaseObjectsOfInterest(void) {
    sint8 nr;
    ObjNumber_t NewObjId;

    /* save Object IDs of last cycle decision */
    for (nr = 0; nr < SiAnzOOI; nr++) {
        SILastCycleOOIObjID[nr] = OBJ_GET_OOI_LIST_OBJ_IDX(nr);
    }

    /*caution: order of execution is relevant!*/
    /*select next objects at host lane*/
    NewObjId = SISelectNextObjectMoving(
        ASSOC_LANE_EGO, SiLatDisplPredictionTime, SI_OBJ_SELECTION_NEXT_LONG);
    SISelectStationaryObject(&NewObjId, OBJ_NEXT_OOI);
    SISelectOncomingObject(&NewObjId, OBJ_NEXT_OOI);
    /* Check if the relevant object has changed from the previous cycle */
    if ((NewObjId < 0) ||
        (NewObjId != OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_NEXT_OOI))) {
        /* Get previous relevant object ID and check if there was one */
        const ObjNumber_t PrevRelObjId = OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_NEXT_OOI);
        if ((PrevRelObjId >= 0) && (PrevRelObjId < Envm_N_OBJECTS)) {
            /* There was a previous rel object, reset it's relevant info */
            SI_OBJ_SET_OBJ_OF_INTEREST(PrevRelObjId, OBJ_NOT_OOI);
        }
    }
    OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_NEXT_OOI) = NewObjId;
    NewObjId = SISelectNextObjectMoving(
        ASSOC_LANE_EGO, SiLatDisplPredictionTime, SI_OBJ_SELECTION_NEXT_LONG);
    SISelectStationaryObject(&NewObjId, OBJ_HIDDEN_NEXT_OOI);

    OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_HIDDEN_NEXT_OOI) = NewObjId;

    /*Check path occupation*/
    SISelectCorridorObjects(&OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_NEXT_OOI),
                            &OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_HIDDEN_NEXT_OOI));

    /* Determine relevant object loss reason. Note: this has to be called before
    'SIRelObject' is updated
    with new selections information. The call in this position assumes that it
    has not yet been updated
    since the previous cycle. Previously this called was made directly after
    single object selection.
    Moved here so that object loss reason is set _after_ blocked path decision
    was also made to make
    object loss signal consistent with object selection. */
    SIObReObRelObjLossReason(OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_NEXT_OOI),
                             &SIRelObject, OBJ_NEXT_OOI);

    /*select next distance object on adjacent lanes*/
    NewObjId = SISelectNextObjectMoving(
        ASSOC_LANE_LEFT, SiLatDisplPredictionTime, SI_OBJ_SELECTION_NEXT_LONG);
    SISelectStationaryObject(&NewObjId, OBJ_NEXT_LONG_LEFT_OOI);
    if (NewObjId < 0) {
        SISelectOncomingObject(&NewObjId, OBJ_NEXT_LONG_LEFT_OOI);
    }
    OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_NEXT_LONG_LEFT_OOI) = NewObjId;

    NewObjId = SISelectNextObjectMoving(
        ASSOC_LANE_RIGHT, SiLatDisplPredictionTime, SI_OBJ_SELECTION_NEXT_LONG);
    SISelectStationaryObject(&NewObjId, OBJ_NEXT_LONG_RIGHT_OOI);
    if (NewObjId < 0) {
        SISelectOncomingObject(&NewObjId, OBJ_NEXT_LONG_RIGHT_OOI);
    }
    OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_NEXT_LONG_RIGHT_OOI) = NewObjId;

    /*select next lateral displacement object on adjacent lanes*/
    NewObjId =
        SISelectNextObjectMoving(ASSOC_LANE_LEFT, SiLatDisplPredictionTime,
                                 SI_OBJ_SELECTION_NEXT_LATERAL);
    SISelectStationaryObject(&NewObjId, OBJ_NEXT_LAT_LEFT_OOI);
    if (NewObjId < 0 && OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_NEXT_LONG_LEFT_OOI) >= 0 &&
        OBJ_DYNAMIC_PROPERTY(OBJ_GET_OOI_LIST_OBJ_IDX(
            OBJ_NEXT_LONG_LEFT_OOI)) == CR_OBJECT_PROPERTY_ONCOMING) {
        SISelectOncomingObject(&NewObjId, OBJ_NEXT_LAT_LEFT_OOI);
    }
    OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_NEXT_LAT_LEFT_OOI) = NewObjId;

    NewObjId =
        SISelectNextObjectMoving(ASSOC_LANE_RIGHT, SiLatDisplPredictionTime,
                                 SI_OBJ_SELECTION_NEXT_LATERAL);
    SISelectStationaryObject(&NewObjId, OBJ_NEXT_LAT_RIGHT_OOI);
    if (NewObjId < 0 &&
        OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_NEXT_LONG_RIGHT_OOI) >= 0 &&
        OBJ_DYNAMIC_PROPERTY(OBJ_GET_OOI_LIST_OBJ_IDX(
            OBJ_NEXT_LONG_RIGHT_OOI)) == CR_OBJECT_PROPERTY_ONCOMING) {
        SISelectOncomingObject(&NewObjId, OBJ_NEXT_LAT_RIGHT_OOI);
    }
    OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_NEXT_LAT_RIGHT_OOI) = NewObjId;

    /*save relevant object data*/
    if (OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_NEXT_OOI) >= 0) {
        SI_OBJ_SET_OBJ_OF_INTEREST(OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_NEXT_OOI),
                                   OBJ_NEXT_OOI);
    }

    SIReInit();

    /*set relevantObjNr and object speed/accel*/
    for (nr = 0; nr < SiAnzOOI; nr++) {
        const ObjNumber_t CurObjId = OBJ_GET_OOI_LIST_OBJ_IDX(nr);
        if (CurObjId >= 0) {
            /* Set the AlreadyOOI flag, so that in the next loop of processing
            we
            can see that it has already been selected (even without the
            selection
            marks which are cleared at the begining of each cycle) */
            OBJ_GET_SI(CurObjId).Bool.AlreadyOOI = 1u;

            /* Check what object position the object had in the previous cycle
             */
            if (OBJ_GET_OOI_POS(CurObjId) != OBJ_NOT_OOI) {
                SISelectObjectInOOI(nr, CurObjId);
            }

            /* Store the relevant object number associated with the object */
            SI_OBJ_SET_OBJ_OF_INTEREST(CurObjId, nr);
        }
    }
    /* Fill in relevant object loss reason in external interface */
    GET_VLC_PUB_OBJ_DATA_PTR->HeaderAssessedObjList.eRelObjLossReason =
        SIObOOIGetOOILossReason(OBJ_NEXT_OOI);
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  AUFHEBUNG MODULLOKALER SYMBOLISCHE KONSTANTEN
*****************************************************************************/
