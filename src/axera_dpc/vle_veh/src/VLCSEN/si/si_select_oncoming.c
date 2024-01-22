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

/*************************************************************************************************************************
  Functionname:    SISelectOncomingObject */
void SISelectOncomingObject(ObjNumber_t* const pNewObjId,
                            SIRelObjEnum_t eRelObjType) {
    ObjNumber_t i;
    ObjNumber_t iOncomeObj = OBJ_INDEX_NO_OBJECT;
    eAssociatedLane_t Lane, object_lane;
    float32 DistanceToTraj;
    float32 DistanceToTrajVar;

    /* Determine Lane */
    if (eRelObjType <= OBJ_HIDDEN_NEXT_OOI) {
        Lane = ASSOC_LANE_EGO;
    } else if ((eRelObjType == OBJ_NEXT_LONG_LEFT_OOI) ||
               (eRelObjType == OBJ_NEXT_LAT_LEFT_OOI)) {
        Lane = ASSOC_LANE_LEFT;
    } else {
        Lane = ASSOC_LANE_RIGHT;
    }

    /* Check for oncoming objects */
    for (i = (ObjNumber_t)(Envm_N_OBJECTS - 1); i >= 0; i--) {
        if (OBJ_DYNAMIC_PROPERTY(i) == CR_OBJECT_PROPERTY_ONCOMING) {
            /* crossing traffic already selected as moving object !!!! */
            if (OBJ_DYNAMIC_SUB_PROPERTY(i) != CR_OBJECT_SUBPROP_CROSSING) {
                /* Get distance to trajectory to decide if object is on left or
                 * right side of lane */
                SITrajGetObjToRefDistance(i, &DistanceToTraj,
                                          &DistanceToTrajVar);
                if (DistanceToTraj < 0.0f) {
                    object_lane = ASSOC_LANE_RIGHT;
                } else {
                    object_lane = ASSOC_LANE_LEFT;
                }
                /* If object was associated with the given lane we are looking
                at or the simplified
                lane assignment based on it being left/right of ego course */
                if (((object_lane == Lane) || (OBJ_GET_FUNC_LANE(i) == Lane)) &&
                    (OBJ_GET_FUNC_LANE(i) != ASSOC_LANE_UNKNOWN)) {
                    /* Only consider objects not yet selected as OOI */
                    if (OBJ_GET_SI(i).Bool.SelectedAsOOI == FALSE) {
                        /* If we already have a prior selected oncoming object,
                        check if the
                        new object has a higher priority (closer X distance),
                        and only replace
                        it if it does. Otherwise if no oncoming object selected
                        so far, then
                        always take the new one */
                        if (iOncomeObj >= 0) {
                            if (OBJ_LONG_DISPLACEMENT(i) <
                                OBJ_LONG_DISPLACEMENT(iOncomeObj)) {
                                iOncomeObj = i;
                            }
                        } else {
                            iOncomeObj = i;
                        }
                    }
                }
            }
        }
    }

    /* check if the loop over all objects selected an oncoming object */
    if (iOncomeObj >= 0) {
        /* Check, if previous algorithms for moving and stationary object
         * already selected an object */
        if (*pNewObjId >= 0) {
            /* Verify that oncoming traffic closer than previously selected one
             */
            if (OBJ_LONG_DISPLACEMENT(*pNewObjId) >
                OBJ_LONG_DISPLACEMENT(iOncomeObj)) {
                /* Replace previously selected stationary/moving with oncomming
                 * object */
                OBJ_GET_SI(*pNewObjId).Bool.SelectedAsOOI = 0u;
                *pNewObjId = iOncomeObj;
                OBJ_GET_SI(iOncomeObj).Bool.SelectedAsOOI = 1u;
            }
        } else {
            *pNewObjId = iOncomeObj;
            OBJ_GET_SI(iOncomeObj).Bool.SelectedAsOOI = 1u;
        }
    }
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */
