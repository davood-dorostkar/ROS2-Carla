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
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  FUNCTION
*****************************************************************************/

static ObjNumber_t SISelectNextStatObjForLane(eAssociatedLane_t Lane);

/*************************************************************************************************************************
  Functionname:    SISelectStationaryObject */
void SISelectStationaryObject(ObjNumber_t* const pNewObjId,
                              SIRelObjEnum_t eRelObjType) {
    ObjNumber_t SiIndexStehend;
    eAssociatedLane_t Lane;

    /* Look for stationary objects */

    /* Determine Lane */
    if (eRelObjType <= OBJ_HIDDEN_NEXT_OOI) {
        Lane = ASSOC_LANE_EGO;
    } else if ((eRelObjType == OBJ_NEXT_LONG_LEFT_OOI) ||
               (eRelObjType == OBJ_NEXT_LAT_LEFT_OOI)) {
        Lane = ASSOC_LANE_LEFT;
    } else {
        Lane = ASSOC_LANE_RIGHT;
    }

    /* Select next stationary object for given lane */
    SiIndexStehend = SISelectNextStatObjForLane(Lane);

    /* Note: detailed comparison of the old complex code showed that tracking
    the OOI object
    states with a state machine was completely overblown as each state had the
    same logic (but
    in different order of evaluation, which in this context has no influence) */
    if (*pNewObjId >= 0) /* moving object found */
    {
        if (SiIndexStehend >= 0) /* stationary object found */
        {
            /* Both stationary and moving object were found : select the closer
             * one */
            if (OBJ_LONG_DISPLACEMENT(*pNewObjId) >
                OBJ_LONG_DISPLACEMENT(SiIndexStehend)) {
                /* if stationary object is closer than moving object, take
                   stationary and release
                    moving object as OOI, first unselect old object */
                OBJ_GET_SI(*pNewObjId).Bool.SelectedAsOOI = 0u;
                /* Store new object ID in location pointed to by caller */
                *pNewObjId = SiIndexStehend;
                /* And set the new object's selection marks */
                OBJ_GET_SI(SiIndexStehend).Bool.SelectedAsOOI = 1u;
            } else {
                /* moving object is closer, so keep it */
            }
        } else {
            /* no stationary object found, so keep moving object */
        }
    } else /* no moving object found */
    {
        if (SiIndexStehend >= 0) /* stationary object found */
        {
            /* Take stationary object */
            *pNewObjId = SiIndexStehend;
            OBJ_GET_SI(SiIndexStehend).Bool.SelectedAsOOI = 1u;
        } else {
            /* no object found at all, do nothing */
        }
    }
}

/*************************************************************************************************************************
  Functionname:    SISelectNextStatObjForLane */
static ObjNumber_t SISelectNextStatObjForLane(eAssociatedLane_t Lane) {
    ObjNumber_t i;
    ObjNumber_t iStatObj = OBJ_INDEX_NO_OBJECT;

    /* Go through entire object list */
    for (i = (ObjNumber_t)(Envm_N_OBJECTS - 1); i >= 0; i--) {
        /* This is for selection of stationary objects, thus only take those;
         * but only if objects were not seen as oncoming and stationary only */
        if (OBJ_DYNAMIC_PROPERTY(i) == CR_OBJECT_PROPERTY_STATIONARY) {
            /* The stationary classified objects shall not be crossing; if
              the object was aleady OBJ_NEXT_OOI, OBJ_NEXT_LONG_LEFT_OOI,
              OBJ_NEXT_LONG_RIGHT_OOI,
              also consider crossing stationary objects due to a toggling false
              crossing
              classification of stationary objects */
            if ((OBJ_DYNAMIC_SUB_PROPERTY(i) != CR_OBJECT_SUBPROP_CROSSING) ||
                (SILastCycleOOIObjID[OBJ_NEXT_OOI] == i) ||
                (SILastCycleOOIObjID[OBJ_NEXT_LONG_LEFT_OOI] == i) ||
                (SILastCycleOOIObjID[OBJ_NEXT_LONG_RIGHT_OOI] == i)) {
                /* Only take objects that were assigned the lane we are looking
                 * at */
                if (OBJ_GET_FUNC_LANE(i) == Lane) {
                    /* Only consider objects not already selected as OOI */
                    if (OBJ_GET_SI(i).Bool.SelectedAsOOI == FALSE) {
                        if (iStatObj >= 0) {
                            if (OBJ_LONG_DISPLACEMENT(i) <
                                OBJ_LONG_DISPLACEMENT(iStatObj)) {
                                iStatObj = OBJ_GET_ID_I(i);
                            }
                        } else {
                            iStatObj = OBJ_GET_ID_I(i);
                        }
                    }
                }
            }
        }
    }
    return (iStatObj);
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  AUFHEBUNG MODULLOKALER SYMBOLISCHE KONSTANTEN
*****************************************************************************/
/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */
