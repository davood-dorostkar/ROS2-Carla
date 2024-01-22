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
//#include "vlc_par.h"

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
static boolean SICheckObjectDetected(const ObjNumber_t ObjId);
static boolean SICheckObjectNextLong(const ObjNumber_t NewObjId,
                                     const ObjNumber_t OldObjId);
static boolean SICheckObjectNextLat(const ObjNumber_t NewObjId,
                                    const ObjNumber_t OldObjId,
                                    fTime_t fPredictionTime);

/*************************************************************************************************************************
  Functionname:    SISelectNextObjectMoving */
ObjNumber_t SISelectNextObjectMoving(eAssociatedLane_t Lane,
                                     fTime_t fPredictionTime,
                                     SIObjSelectionType_t SelectionType) {
    ObjNumber_t returnObject = OBJ_INDEX_NO_OBJECT;
    ObjNumber_t nr;

    /*search for nearest object*/
    for (nr = (ObjNumber_t)(Envm_N_OBJECTS - 1); nr >= (ObjNumber_t)0; nr--) {
        /*object available and usable for this function*/
        if (SICheckObjectDetected(nr) == TRUE) {
            /*check if the lane is the given lane*/
            if (OBJ_GET_FUNC_LANE(nr) == Lane) {
                /*compare to returnObject using the right compare function*/
                switch (SelectionType) {
                    /*return next longitudinal object*/
                    case SI_OBJ_SELECTION_NEXT_LONG:
                        if (SICheckObjectNextLong(nr, returnObject) == TRUE) {
                            returnObject = nr;
                        }
                        break;

                    /*return next lateral object*/
                    case SI_OBJ_SELECTION_NEXT_LATERAL:
                        if (SICheckObjectNextLat(nr, returnObject,
                                                 fPredictionTime) == TRUE) {
                            returnObject = nr;
                        }
                        break;

                    default:
                        /*do nothing*/
                        break;
                }
            }
        }
    }

    /*if object was found --> mark object as selected*/
    if (returnObject >= 0) {
        OBJ_GET_SI(returnObject).Bool.SelectedAsOOI = 1u;
    }

    /*return the object if available*/
    return returnObject;
}

/*************************************************************************************************************************
  Functionname:    SICheckObjectDetected */
static boolean SICheckObjectDetected(const ObjNumber_t ObjId) {
    boolean returnVal = FALSE;

    /* crossing traffic only in distances smaller
     * SI_MAX_DIST_FOR_CROSSING_REPORT ( defined in si_par.h) */
    if ((OBJ_DYNAMIC_SUB_PROPERTY(ObjId) != CR_OBJECT_SUBPROP_CROSSING) ||
        (OBJ_LONG_DISPLACEMENT(ObjId) < SI_MAX_DIST_FOR_CROSSING_REPORT)) {
        /* select only moving objects in the same direction or crossing
         * traffic*/
        if ((OBJ_DYNAMIC_PROPERTY(ObjId) == CR_OBJECT_PROPERTY_MOVING) ||
            (OBJ_DYNAMIC_SUB_PROPERTY(ObjId) == CR_OBJECT_SUBPROP_CROSSING)) {
            /*check if object is not selected*/
            if (OBJ_GET_SI(ObjId).Bool.SelectedAsOOI == FALSE) {
                returnVal = TRUE;
            }
        }
    }
    return returnVal;
}

/*************************************************************************************************************************
  Functionname:    SICheckObjectNextLong */
static boolean SICheckObjectNextLong(const ObjNumber_t NewObjId,
                                     const ObjNumber_t OldObjId) {
    boolean returnVal;
    returnVal = FALSE;

    if (OldObjId >= 0) {
        /*if distance to new object is shorter than distance to old object -->
         * new object is more relevant*/
        if (OBJ_LONG_DISPLACEMENT(NewObjId) < OBJ_LONG_DISPLACEMENT(OldObjId)) {
            returnVal = TRUE;
        }
    } else {
        /*no object was previously selected --> select new object*/
        returnVal = TRUE;
    }

    return returnVal;
}

/*************************************************************************************************************************
  Functionname:    SICheckObjectNextLat */
static boolean SICheckObjectNextLat(const ObjNumber_t NewObjId,
                                    const ObjNumber_t OldObjId,
                                    fTime_t fPredictionTime) {
    boolean returnVal;
    fDistance_t predicted_distance_to_lane;
    float32 DistanceToTraj;
    float32 DistanceToTrajVar;
    float32 VRelToTraj;
    float32 VRelToTrajVar;
    returnVal = FALSE;

    /*calculate predicted distance for new object*/
    SITrajGetObjToRefDistance(NewObjId, &DistanceToTraj, &DistanceToTrajVar);
    SITrajGetObjToRefDistanceGradient(NewObjId, &VRelToTraj, &VRelToTrajVar);

    predicted_distance_to_lane =
        (DistanceToTraj) + (fPredictionTime * ((VRelToTraj)));

    /* save predicted distance value in SI structure of object list for further
     * comparison */
    OBJ_GET_SI(NewObjId).fPredictedLatDispl = predicted_distance_to_lane;

    /* Already object selected before? */
    if (OldObjId >= 0) {
        /* differ between left and right side case */
        if (DistanceToTraj > 0.0f) {
            /* In left side case, distances are positive, so closer distance is
             * mathematically smaller */
            /*if lateral displacement to new object is shorter than distance to
             * old object --> new object is more relevant*/
            if (OBJ_GET_SI(NewObjId).fPredictedLatDispl <
                OBJ_GET_SI(OldObjId).fPredictedLatDispl) {
                returnVal = TRUE;
            }
        } else {
            /* In right side case, distances are negative, so closer distance is
             * mathematically bigger */
            /*if lateral displacement to new object is shorter than distance to
             * old object --> new object is more relevant*/
            if (OBJ_GET_SI(NewObjId).fPredictedLatDispl >
                OBJ_GET_SI(OldObjId).fPredictedLatDispl) {
                returnVal = TRUE;
            }
        }
    } else {
        /*no previously object was selected --> select new object*/
        returnVal = TRUE;
    }
    return returnVal;
}
/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */