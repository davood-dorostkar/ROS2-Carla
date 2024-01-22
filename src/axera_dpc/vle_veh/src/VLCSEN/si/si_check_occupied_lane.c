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
#define LAT_LANECHANGE_TIME (1.4f)
#define LAT_LANECHANGE_MINSPEED (5.0f)
#define STATIC_OBJECT_OCCUPATION_RIGHT_ENTRY (2u)
#define STATIC_OBJECT_OCCUPATION_LEFT_ENTRY (3u)
#define STATIC_OBJECT_OCCUPATION_FREE_ENTRY (4u)
#define SI_PATH_OCCUPATION_LENGTH_TRAJEGO_FRONT_TO_OBJECT (250.0f)
#define SI_VALID_OBJECT_WIDTH_IN_LANE_FOR_ARSOBJ_CLASS_POINT (0.8f)
#define SI_OBJECT_WIDTH_IN_LANE_FOR_NON_ARSOBJ_CLASS_POINT (1.8f)
/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/
SET_MEMSEC_CONST(CorrSelOffsetYObj2ObjDistX)
static const GDBLFunction_t CorrSelOffsetYObj2ObjDistX = {

    SI_CORRIDOR_SEL_Y_OFFSET_MIN, /* Ausgangswert A1 */
    SI_CORRIDOR_SEL_Y_OFFSET_MAX, /* Ausgangswert A2 */
    (SI_CORRIDOR_SEL_Y_OFFSET_MAX - SI_CORRIDOR_SEL_Y_OFFSET_MIN) /
        (SI_CORSELYOFF_Obj2ObjDistX_MAX - SI_CORSELYOFF_Obj2ObjDistX_MIN),
    /* Achsabschnitt der Anpassungsgerade:   A1 - (A2-A1)/(E2-E1) * E1 */
    SI_CORRIDOR_SEL_Y_OFFSET_MIN -
        (((SI_CORRIDOR_SEL_Y_OFFSET_MAX - SI_CORRIDOR_SEL_Y_OFFSET_MIN) /
          (SI_CORSELYOFF_Obj2ObjDistX_MAX - SI_CORSELYOFF_Obj2ObjDistX_MIN)) *
         SI_CORSELYOFF_Obj2ObjDistX_MIN)};

SET_MEMSEC_CONST(CorrSelOffsetYTTCGapWeight)
static const GDBLFunction_t CorrSelOffsetYTTCGapWeight = {

    1.0F, /* Ausgangswert A1 */
    0.0F, /* Ausgangswert A2 */
    (-1.0F) / (SI_CORSELYOFF_TTCGAP_MAX - SI_CORSELYOFF_TTCGAP_MIN),
    /* Achsabschnitt der Anpassungsgerade:   A1 - (A2-A1)/(E2-E1) * E1 */
    1.0F - (((-1.0F) / (SI_CORSELYOFF_TTCGAP_MAX - SI_CORSELYOFF_TTCGAP_MIN)) *
            SI_CORSELYOFF_TTCGAP_MIN)};

SET_MEMSEC_CONST(CorrSelOffsetYTimeGapWeight)
static const GDBLFunction_t CorrSelOffsetYTimeGapWeight = {

    SI_CORSEL_Y_HYST_MIN, /* Ausgangswert A1 */
    SI_CORSEL_Y_HYST_MAX, /* Ausgangswert A2 */
    (SI_CORSEL_Y_HYST_MAX - SI_CORSEL_Y_HYST_MIN) /
        (SI_CORSEL_Y_HYST_TIMEGAP_MAX - SI_CORSEL_Y_HYST_TIMEGAP_MIN),
    /* Achsabschnitt der Anpassungsgerade:   A1 - (A2-A1)/(E2-E1) * E1 */
    SI_CORSEL_Y_HYST_MIN -
        (((SI_CORSEL_Y_HYST_MAX - SI_CORSEL_Y_HYST_MIN) /
          (SI_CORSEL_Y_HYST_TIMEGAP_MAX - SI_CORSEL_Y_HYST_TIMEGAP_MIN)) *
         SI_CORSEL_Y_HYST_TIMEGAP_MIN)};

SET_MEMSEC_CONST(CorrSelPathwidthPassableMov)
static const GDBLFunction_t CorrSelPathwidthPassableMov = {

    SI_MIN_PATHWIDTH_PASSABLE_MOV_MIN, /* Ausgangswert A1 */
    SI_MIN_PATHWIDTH_PASSABLE_MOV_MAX, /* Ausgangswert A2 */
    (SI_MIN_PATHWIDTH_PASSABLE_MOV_MAX - SI_MIN_PATHWIDTH_PASSABLE_MOV_MIN) /
        (SI_MIN_PATHWIDTH_PASSABLE_MOV_VOBJ_MAX -
         SI_MIN_PATHWIDTH_PASSABLE_MOV_VOBJ_MIN),
    /* Achsabschnitt der Anpassungsgerade:   A1 - (A2-A1)/(E2-E1) * E1 */
    SI_MIN_PATHWIDTH_PASSABLE_MOV_MIN -
        (((SI_MIN_PATHWIDTH_PASSABLE_MOV_MAX -
           SI_MIN_PATHWIDTH_PASSABLE_MOV_MIN) /
          (SI_MIN_PATHWIDTH_PASSABLE_MOV_VOBJ_MAX -
           SI_MIN_PATHWIDTH_PASSABLE_MOV_VOBJ_MIN)) *
         SI_MIN_PATHWIDTH_PASSABLE_MOV_VOBJ_MIN)};

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  FUNCTION
*****************************************************************************/

/*****************************************************************************
  FUNCTION PROTOTYPES DECLARATIONS
*****************************************************************************/

static float32 SICalcDriveAroundDist(float32 EgoSpeedXObjSync);
static float32 SIGetObjectWidthInLane(ObjNumber_t iObject);
static void SIInitSiPathOccupation(SIPathOccupationArray_t SiPathOccupation,
                                   float32 HalfLaneWidth);
static void SIFillStat2StatPathOccupation(
    SIPathOccupationArray_t SiPathOccupation);
static void SIFillMov2MovPathOccupation(
    SIPathOccupationArray_t SiPathOccupation);
static void SICalcOccPathWidth(
    const SiPathOccupation_t SiPathOcc[SI_PATH_OVLC_ENTRIES],
    SIPathOccResultArray_t SiPathhOccResult);
static void SIInsertObjInPathOcc(
    SIPathOccupationArray_t SiPathOccupation,
    uint32* ActualEntry,
    uint32* NextFree,
    const SiPathOccInsertObjData_t* SiPathOccInsertObjDataPtr);
static void SIInitSiPathOccResult(SIPathOccResultArray_t SiPathhOccResult,
                                  float32 HalfLaneWidth);
static void SICheckForNarrowPath(
    const SiPathOccResult_t SiPathhOccResult[SI_PATH_OVLC_ENTRIES],
    ObjNumber_t* const pCloseObjId,
    ObjNumber_t* const pFarObjId);
static void SIResetBlockedPathDecision(void);
static void SIUpdateTimerBlockedPathDecision(void);
static void SISelectOOI(ObjNumber_t ObjectToSelect);
static void SIUnSelectOOI(ObjNumber_t ObjectToUnSelect);

/*************************************************************************************************************************
  Functionname:    SIInitBlockedPath */
void SIInitBlockedPath(void) {}

/*************************************************************************************************************************
  Functionname:    SIFarObjCheck */
void SIFarObjCheck(ObjNumber_t CloseObjId,
                   ObjNumber_t FarObjId,
                   float32 CloseObjDistTraj,
                   float32 FarObjDistTraj,
                   ObjNumber_t* const pNextObjId,
                   ObjNumber_t* const pHiddenObjId) {
    /*far object was relevant object in last cycle -> DistY Hysteresis*/
    if (FarObjDistTraj > CloseObjDistTraj) {
        /* Unselect single inlane objects */
        if (*pNextObjId >= 0) {
            SIUnSelectOOI(*pNextObjId);
        }
        /* far object relevance changes to close object only if far
        object dist to trajectory is twice as big as close object
        --> close object is relevant object, far object is next
        hidden */
        *pNextObjId = CloseObjId;
        SISelectOOI(CloseObjId);
    } else {
        /* next object gets hidden next, if existing */
        if (*pNextObjId != FarObjId) {
            if (*pHiddenObjId >= 0) {
                /* Unselect hidden object */
                SIUnSelectOOI(*pHiddenObjId);
            }
            *pHiddenObjId = *pNextObjId;
        }
        /* keep relevance of far object, leave close object free for
         * neigbour object selection */
        *pNextObjId = FarObjId;
        SISelectOOI(FarObjId);
    }
}

/*************************************************************************************************************************
  Functionname:    SIObjDiffCheck */
void SIObjDiffCheck(ObjNumber_t CloseObjId,
                    ObjNumber_t FarObjId,
                    float32 CloseObjDistTraj,
                    float32 FarObjDistTraj,
                    ObjNumber_t* const pNextObjId,
                    ObjNumber_t* const pHiddenObjId) {
    if ((CloseObjId != *pNextObjId) && (FarObjId != *pNextObjId)) {
        /* check for next object */
        if ((*pNextObjId < 0) || (OBJ_LONG_DISPLACEMENT(*pNextObjId) >=
                                  OBJ_LONG_DISPLACEMENT(FarObjId))) {
            /* no relevant in lane single object selection in actual cycle */
            /* or relevant in lane single object farer away than blocked path
             * objects */
            if ((FarObjId == SILastCycleOOIObjID[OBJ_NEXT_OOI]) ||
                (*pNextObjId == FarObjId)) {
                SIFarObjCheck(CloseObjId, FarObjId, CloseObjDistTraj,
                              FarObjDistTraj, pNextObjId, pHiddenObjId);
            } else if (CloseObjId ==
                       SILastCycleOOIObjID
                           [OBJ_NEXT_OOI]) { /*close object was relevant object
                                                in last cycle -> DistY
                                                Hysteresis*/
                if (CloseObjDistTraj > FarObjDistTraj) {
                    if (*pNextObjId != FarObjId)

                    {
                        if (*pHiddenObjId >= 0) {
                            /* Unselect hidden object */
                            SIUnSelectOOI(*pHiddenObjId);
                        }
                        *pHiddenObjId = *pNextObjId;
                    }

                    /* close object relevance changes to far object only if
                    close object dist to trajectory is twice as big as far
                    object
                    --> far object is relevant object, close object is neighbour
                    object */
                    *pNextObjId = FarObjId;
                    SISelectOOI(FarObjId);
                } else {
                    /* Unselect single inlane objects */
                    if (*pNextObjId >= 0)

                    {
                        SIUnSelectOOI(*pNextObjId);
                    }
                    /* keep relevance of close object, far object is next hidden
                     */
                    *pNextObjId = CloseObjId;
                    SISelectOOI(CloseObjId);
                }

            } else { /* no in lane  object in last cycle */
                if (FarObjDistTraj > CloseObjDistTraj) {
                    /* Unselect single inlane objects */
                    if (*pNextObjId >= 0)

                    {
                        SIUnSelectOOI(*pNextObjId);
                    }
                    /* close object only gets relevant if far object dist to
                    trajectory is twice as big as close object
                    --> close object is relevant object, far object is next
                    hidden */
                    *pNextObjId = CloseObjId;
                    SISelectOOI(CloseObjId);
                } else {
                    /* next object gets hidden next, if existing */
                    if (*pHiddenObjId >= 0)

                    {
                        /* Unselect hidden object */
                        SIUnSelectOOI(*pHiddenObjId);
                    }

                    *pHiddenObjId = *pNextObjId;

                    /* far object gets relevant, leave close object free for
                     * neigbour object selection */
                    *pNextObjId = FarObjId;
                    SISelectOOI(FarObjId);
                }
            }
        }
    } /* End if Far/Close object not selected by single-object selection */
}

/*************************************************************************************************************************
  Functionname:    SISelectCorridorObjects */
void SISelectCorridorObjects(ObjNumber_t* const pNextObjId,
                             ObjNumber_t* const pHiddenObjId) {
    /* stationary and slow moving objects are inserted here.
       relationships between slow moving, stopped and stationary
       are assumed to be the same since there is nearly no relative
       movement between objects. object longitudinal expansion
       depends only on own vehicles speed. */
    SIPathOccupationArray_t SiPathOccStat2Stat;
    SIPathOccResultArray_t SiPathOccResultStat2Stat;

    /* moving objects are inserted here.
    relationships between moving objects depend on relative speed
    between them, but this is neglected. only relative speed
    towards own vehicle is taken into account.*/
    SIPathOccupationArray_t SiPathOccMov2Mov;
    SIPathOccResultArray_t SiPathOccResultMov2Mov = {0};

    ObjNumber_t CloseObjId = OBJ_INDEX_NO_OBJECT;
    ObjNumber_t FarObjId = OBJ_INDEX_NO_OBJECT;
    ObjNumber_t StatCloseObjId = OBJ_INDEX_NO_OBJECT;
    ObjNumber_t StatFarObjId = OBJ_INDEX_NO_OBJECT;

    float32 FarObjDistTraj;
    float32 CloseObjDistTraj;
    float32 fTTCGap, fTimeGap, fObj2ObjDistX, fCorridorSelectionOffsetY,
        fRelObjectHysteresisY;
    const fCurve_t ActualCurve = EGO_CURVE_OBJ_SYNC;
    const fVelocity_t ActualSpeed = EGO_SPEED_X_OBJ_SYNC;

    /* Reset the blocked path decision bits for all objects */
    SIResetBlockedPathDecision();

    if (fABS(ActualCurve) < SI_MAX_CURVE_FOR_BLOCKED_PATH_SELECTION) {
        /* insert stationary objects into path occupation list */
        SIFillStat2StatPathOccupation(SiPathOccStat2Stat);
        /* insert moving objects into path occupation list */
        SIFillMov2MovPathOccupation(SiPathOccMov2Mov);

        /* calculate resulting path widths of neigbouring objects within path */
        SICalcOccPathWidth(SiPathOccStat2Stat, SiPathOccResultStat2Stat);
        /* calculate resulting path widths of neigbouring objects within path */
        SICalcOccPathWidth(SiPathOccMov2Mov, SiPathOccResultMov2Mov);

        /* check for blocked stat2atat path ahead */
        SICheckForNarrowPath(SiPathOccResultStat2Stat, &StatCloseObjId,
                             &StatFarObjId);
        /* check for blocked mov2mov path ahead */
        SICheckForNarrowPath(SiPathOccResultMov2Mov, &CloseObjId, &FarObjId);

        /*overwrite mov2mov narrow path with stat2stat narrowpath if closer */
        if ((CloseObjId < 0) ||
            ((StatCloseObjId >= 0) && (OBJ_LONG_DISPLACEMENT(StatCloseObjId) <
                                       OBJ_LONG_DISPLACEMENT(CloseObjId)))) {
            CloseObjId = StatCloseObjId;
            FarObjId = StatFarObjId;
        }

        /* Objects blocking path detected ? */
        if ((CloseObjId >= 0) && (FarObjId >= 0)) {
            float32 fFarObjWidth, fCloseObjWidth;
            boolean b_LaneChangeConsideration = FALSE;
            const SIScaleBracketState_t LC_State =
                SIReturnStateScaleBracket(); /*!< Get lane change state */
            const float32 f_Dist2TrajFarObj =
                CPTrajGetObjToRefDistMeas(&(OBJ_GET_CP(FarObjId).TrajDist));
            const float32 f_Dist2TrajCloseObj =
                CPTrajGetObjToRefDistMeas(&(OBJ_GET_CP(CloseObjId).TrajDist));
            /* Set FarObjId and CloseObjId in case of a Lane Change;
              set the objects in the direction of the lane change to the
              CloseObjId */
            if (LC_State == PRE_LANE_CHANGE_LEFT) {
                if (f_Dist2TrajFarObj > f_Dist2TrajCloseObj) {
                    const ObjNumber_t ObjTemp = CloseObjId;
                    CloseObjId = FarObjId;
                    FarObjId = ObjTemp;
                }
                b_LaneChangeConsideration = TRUE;
            } else if (LC_State == PRE_LANE_CHANGE_RIGHT)

            {
                if (f_Dist2TrajFarObj < f_Dist2TrajCloseObj) {
                    const ObjNumber_t ObjTemp = CloseObjId;
                    CloseObjId = FarObjId;
                    FarObjId = ObjTemp;
                }
                b_LaneChangeConsideration = TRUE;
            }

            /* Use Hysteresis in case of no lane change */
            if (b_LaneChangeConsideration == FALSE) {
                fFarObjWidth = SIGetObjectWidthInLane(FarObjId);
                fCloseObjWidth = SIGetObjectWidthInLane(CloseObjId);

                /*calculate corridor objects inner edge distance to trajectory*/
                FarObjDistTraj =
                    CPTrajGetObjToRefDistMeas(&(OBJ_GET_CP(FarObjId).TrajDist));
                FarObjDistTraj = fABS(FarObjDistTraj) - (0.5f * fFarObjWidth);
                FarObjDistTraj = MAX_FLOAT(0.0f, FarObjDistTraj);

                CloseObjDistTraj = CPTrajGetObjToRefDistMeas(
                    &(OBJ_GET_CP(CloseObjId).TrajDist));
                CloseObjDistTraj =
                    fABS(CloseObjDistTraj) - (0.5f * fCloseObjWidth);
                CloseObjDistTraj = MAX_FLOAT(0.0f, CloseObjDistTraj);

                /*calculate x-dist depending y-hysteresis between corridor
                 * objects */
                fTTCGap =
                    MIN_FLOAT(-OBJ_LONG_DISPLACEMENT(CloseObjId) /
                                  MIN_FLOAT(OBJ_LONG_VREL(CloseObjId), -1.0f),
                              -OBJ_LONG_DISPLACEMENT(FarObjId) /
                                  MIN_FLOAT(OBJ_LONG_VREL(FarObjId), -1.0f));

                fObj2ObjDistX = OBJ_LONG_DISPLACEMENT(FarObjId) -
                                OBJ_LONG_DISPLACEMENT(CloseObjId);

                fCorridorSelectionOffsetY = dGDBmathLineareFunktion(
                    &CorrSelOffsetYObj2ObjDistX, fObj2ObjDistX);
                fCorridorSelectionOffsetY *= dGDBmathLineareFunktion(
                    &CorrSelOffsetYTTCGapWeight, fTTCGap);

                /*apply dist-x hysteresis*/
                FarObjDistTraj += fCorridorSelectionOffsetY;
                CloseObjDistTraj -= fCorridorSelectionOffsetY;

                if (FarObjId == SILastCycleOOIObjID[OBJ_NEXT_OOI]) {
                    fTimeGap = OBJ_LONG_DISPLACEMENT(FarObjId) /
                               MAX_FLOAT(ActualSpeed, 0.1f);
                    /* calculate relevant object depending hysteresis between
                     * corridor objects*/
                    fRelObjectHysteresisY = dGDBmathLineareFunktion(
                        &CorrSelOffsetYTimeGapWeight, fTimeGap);
                    /*apply rel hysteresis*/
                    FarObjDistTraj -= fRelObjectHysteresisY;
                    CloseObjDistTraj += fRelObjectHysteresisY;
                } else if (CloseObjId == SILastCycleOOIObjID[OBJ_NEXT_OOI]) {
                    fTimeGap = OBJ_LONG_DISPLACEMENT(CloseObjId) /
                               MAX_FLOAT(ActualSpeed, 0.1f);
                    /* calculate relevant object depending hysteresis between
                     * corridor objects*/
                    fRelObjectHysteresisY = dGDBmathLineareFunktion(
                        &CorrSelOffsetYTimeGapWeight, fTimeGap);
                    /*apply rel hysteresis*/
                    FarObjDistTraj += fRelObjectHysteresisY;
                    CloseObjDistTraj -= fRelObjectHysteresisY;
                } else {
                    /*No RelObject Selection Hysteresis*/
                }
            } else {
                /* In case of an lane change don't appy any hysteresis,
                   manipulate the distance to the acc trajectory in order
                   to select the object on the next lane */
                fCloseObjWidth = SIGetObjectWidthInLane(CloseObjId);

                CloseObjDistTraj = CPTrajGetObjToRefDistMeas(
                    &(OBJ_GET_CP(CloseObjId).TrajDist));
                CloseObjDistTraj =
                    fABS(CloseObjDistTraj) - (0.5f * fCloseObjWidth);
                CloseObjDistTraj = MAX_FLOAT(0.0f, CloseObjDistTraj);

                FarObjDistTraj = CloseObjDistTraj + 1.f;
            }

            /* Yes, increment blocked path decision timer for both of the
             * participating objects*/
            OBJ_GET_SI(CloseObjId).BlockedPathDecision.PathSelectionTimer++;
            OBJ_GET_SI(FarObjId).BlockedPathDecision.PathSelectionTimer++;
            if ((OBJ_GET_SI(CloseObjId).BlockedPathDecision.PathSelectionTimer >
                 SI_BLOCKED_PATH_DECISION_TIME) ||
                (OBJ_GET_SI(FarObjId).BlockedPathDecision.PathSelectionTimer >
                 SI_BLOCKED_PATH_DECISION_TIME)) {
                /* blocked path decision time is over, so check if object shall
                be inserted as next and next
                hiddden object */
                OBJ_GET_SI(CloseObjId).BlockedPathDecision.PathSelectionTimer =
                    SI_BLOCKED_PATH_DECISION_TIME;
                OBJ_GET_SI(FarObjId).BlockedPathDecision.PathSelectionTimer =
                    SI_BLOCKED_PATH_DECISION_TIME;

                /* Are objects different from single object selection selected
                 * object */
                SIObjDiffCheck(CloseObjId, FarObjId, CloseObjDistTraj,
                               FarObjDistTraj, pNextObjId, pHiddenObjId);

            } /* end if blocked path timer ? */
        }
    }

    /* Update blocked path decision timers */
    SIUpdateTimerBlockedPathDecision();
}

/*************************************************************************************************************************
  Functionname:    SIFillStat2StatPathOccupation */
static void SIFillStat2StatPathOccupation(
    SIPathOccupationArray_t SiPathOccupation) {
    ObjNumber_t i, j;
    uint32 ActualR = STATIC_OBJECT_OCCUPATION_RIGHT_ENTRY; /* staring point for
                                                              right entries is
                                                              number 2 */
    uint32 ActualL = STATIC_OBJECT_OCCUPATION_LEFT_ENTRY;  /* staring point for
                                                              left entries is
                                                              number 3 */
    uint32 NextFree =
        STATIC_OBJECT_OCCUPATION_FREE_ENTRY; /* so first free entry
                                                is number 4 */
    const fVelocity_t TempVehicleSpeed = EGO_SPEED_X_OBJ_SYNC;

    SiPathOccInsertObjData_t SiPathOccInsertObjData;

    /* set path width for all objects */
    SiPathOccInsertObjData.HalbeSpurbreite = SI_HALF_LANEOCCUPATION_WIDTH;

    /* initialize path occupation list */
    SIInitSiPathOccupation(SiPathOccupation,
                           SiPathOccInsertObjData.HalbeSpurbreite);

    /* search sorted  object list for objects touching track */
    for (j = 0; j < OBJ_NUMBER_OF_OBJ_USED; j++) {
        i = Envm_OBJ_INDEX_DISTX_SORTED[j];
        /* do not use empty objects or not preselected objects or objects of
         * different dynamic property */
        /* slow moving objects are handled as stationary objects; definition of
           slow moving is constant
           SI_CORR_SEL_MOVING_AS_STAT_SPEED */
        /* crossing objects are excluded from corridor selection list */
        if (((OBJ_DYNAMIC_PROPERTY(i) == CR_OBJECT_PROPERTY_STATIONARY) ||
             ((OBJ_DYNAMIC_PROPERTY(i) == CR_OBJECT_PROPERTY_MOVING) &&
              ((TempVehicleSpeed + OBJ_LONG_VREL(i)) <
               SI_CORR_SEL_MOVING_AS_STAT_SPEED))) &&
            (SIBasePreselObjList[i] == TRUE) &&
            (OBJ_DYNAMIC_SUB_PROPERTY(i) != CR_OBJECT_SUBPROP_CROSSING)) {
            const float32 fObjHalfWidth = 0.5f * SIGetObjectWidthInLane(i);
            const float32 fObjLength = SIGetObjectLength(i);
            /* set object data structure for path occupation list entry */
            SiPathOccInsertObjData.DistTrajToObjEdge =
                CPTrajGetObjToRefDistMeas(&OBJ_GET_CP(i).TrajDist);
            SiPathOccInsertObjData.ObjID = i;
            SiPathOccInsertObjData.CalculatedObjLength =
                SICalcDriveAroundDist(TempVehicleSpeed) + fObjLength +
                VLC_WHEELBASE_DEFAULT;
            SiPathOccInsertObjData.ObjDistOnTraj =
                CPTrajGetObjDistOnTraj(&OBJ_GET_CP(i).TrajDist);

            /* left side or right side ? */
            if (SiPathOccInsertObjData.DistTrajToObjEdge < 0.f) {
                /* right side, so insert to right path of list */
                /* recalculate distance to trajectory using absolute value and
                 * respecting object width */
                SiPathOccInsertObjData.DistTrajToObjEdge =
                    fABS(SiPathOccInsertObjData.DistTrajToObjEdge) -
                    fObjHalfWidth;
                SIInsertObjInPathOcc(SiPathOccupation, &ActualR, &NextFree,
                                     &SiPathOccInsertObjData);
            } else {
                /* left side, so insert to left path of list */
                /* recalculate distance to trajectory using absolute value and
                 * respecting object width */
                SiPathOccInsertObjData.DistTrajToObjEdge =
                    fABS(SiPathOccInsertObjData.DistTrajToObjEdge) -
                    fObjHalfWidth;
                SIInsertObjInPathOcc(SiPathOccupation, &ActualL, &NextFree,
                                     &SiPathOccInsertObjData);
            }
        }
    }
}

/*************************************************************************************************************************
  Functionname:    SIFillMov2MovPathOccupation */
static void SIFillMov2MovPathOccupation(
    SIPathOccupationArray_t SiPathOccupation) {
    ObjNumber_t i, j;
    uint32 ActualR = STATIC_OBJECT_OCCUPATION_RIGHT_ENTRY; /* staring point for
                                                              right entries is
                                                              number 2 */
    uint32 ActualL = STATIC_OBJECT_OCCUPATION_LEFT_ENTRY;  /* staring point for
                                                              left entries is
                                                              number 3 */
    uint32 NextFree =
        STATIC_OBJECT_OCCUPATION_FREE_ENTRY; /* so first free entry
                                                is number 4 */

    SiPathOccInsertObjData_t SiPathOccInsertObjData;

    /* set path width for all objects */
    SiPathOccInsertObjData.HalbeSpurbreite = SI_HALF_LANEOCCUPATION_WIDTH;

    /* initialize path occupation list */
    SIInitSiPathOccupation(SiPathOccupation,
                           SiPathOccInsertObjData.HalbeSpurbreite);

    /* search sorted  object list for objects touching track */
    for (j = 0; j < OBJ_NUMBER_OF_OBJ_USED; j++) {
        i = Envm_OBJ_INDEX_DISTX_SORTED[j];
        /* do not use empty objects or not preselected objects or objects of
         * different dynamic property */
        /* crossing objects are excluded from corridor selection list */
        if ((OBJ_DYNAMIC_PROPERTY(i) == CR_OBJECT_PROPERTY_MOVING) &&
            (SIBasePreselObjList[i] == TRUE) &&
            (OBJ_DYNAMIC_SUB_PROPERTY(i) != CR_OBJECT_SUBPROP_CROSSING)) {
            const float32 fObjHalfWidth = 0.5f * SIGetObjectWidthInLane(i);
            const float32 fObjLength = SIGetObjectLength(i);

            /* set object data structure for path occupation list entry */
            SiPathOccInsertObjData.DistTrajToObjEdge =
                CPTrajGetObjToRefDistMeas(&OBJ_GET_CP(i).TrajDist);
            SiPathOccInsertObjData.ObjID = i;
            SiPathOccInsertObjData.CalculatedObjLength =
                SICalcDriveAroundDist(-OBJ_LONG_VREL(i)) + fObjLength +
                VLC_WHEELBASE_DEFAULT;
            SiPathOccInsertObjData.ObjDistOnTraj =
                CPTrajGetObjDistOnTraj(&OBJ_GET_CP(i).TrajDist);

            /* left side or right side ? */
            if (SiPathOccInsertObjData.DistTrajToObjEdge < 0.f) {
                /* right side, so insert to right path of list */
                /* recalculate distance to trajectory using absolute value and
                 * respecting object width */
                SiPathOccInsertObjData.DistTrajToObjEdge =
                    fABS(SiPathOccInsertObjData.DistTrajToObjEdge) -
                    fObjHalfWidth;
                SIInsertObjInPathOcc(SiPathOccupation, &ActualR, &NextFree,
                                     &SiPathOccInsertObjData);
            } else {
                /* left side, so insert to left path of list */
                /* recalculate distance to trajectory using absolute value and
                 * respecting object width */
                SiPathOccInsertObjData.DistTrajToObjEdge =
                    fABS(SiPathOccInsertObjData.DistTrajToObjEdge) -
                    fObjHalfWidth;
                SIInsertObjInPathOcc(SiPathOccupation, &ActualL, &NextFree,
                                     &SiPathOccInsertObjData);
            }
        }
    }
}

/*************************************************************************************************************************
  Functionname:    SIInitSiPathOccupation */
static void SIInitSiPathOccupation(SIPathOccupationArray_t SiPathOccupation,
                                   float32 HalfLaneWidth) {
    uint32 i;
    /* SiPathOccupation entries 0 and 1 are starting points of the list.
    SiPathOccupation[0] is starting point for right side with Distance to
    Trajectorie of
    half trackwidth.
    SiPathOccupation[1] is starting point for left side with Distance to
    Trajectorie of
    half trackwidth.
    This is to find the first entry of narrow track and to keep information of
    trackwidth within the list. */

    /*initialize entire list */
    for (i = 0u; i < (uint32)SI_PATH_OVLC_ENTRIES; i++) {
        SiPathOccupation[i].LengthTrajEgoFrontToObj = 0.0f;
        SiPathOccupation[i].DistTrajToObjEdge = HalfLaneWidth;
        SiPathOccupation[i].ObjID = OBJ_INDEX_NO_OBJECT;
        SiPathOccupation[i].prev = NULL;
        SiPathOccupation[i].next = NULL;
    }
    /* set starting points */
    SiPathOccupation[0].next = &SiPathOccupation[2];
    SiPathOccupation[1].next = &SiPathOccupation[3];

    SiPathOccupation[2].prev = &SiPathOccupation[0];
    SiPathOccupation[3].prev = &SiPathOccupation[1];

    SiPathOccupation[2].LengthTrajEgoFrontToObj =
        SI_PATH_OCCUPATION_LENGTH_TRAJEGO_FRONT_TO_OBJECT;
    SiPathOccupation[3].LengthTrajEgoFrontToObj =
        SI_PATH_OCCUPATION_LENGTH_TRAJEGO_FRONT_TO_OBJECT;
}

static void SILookForPosToInsertSecondEdge1(
    SIPathOccupationArray_t SiPathOccupation,
    uint32* ActualEntry,
    uint32* NextFree,
    const SiPathOccInsertObjData_t* SiPathOccInsertObjDataPtr,
    SiPathOccupation_t** List_insert_ptr_ptr,
    SiPathOccupation_t* last_deleted) {
    if ((*List_insert_ptr_ptr)->LengthTrajEgoFrontToObj <
        (SiPathOccInsertObjDataPtr->ObjDistOnTraj +
         SiPathOccInsertObjDataPtr->CalculatedObjLength)) {
        if ((*List_insert_ptr_ptr)->DistTrajToObjEdge <=
            SiPathOccInsertObjDataPtr->DistTrajToObjEdge) {
            /* this point is nearer but narrower than actual
             * entry, so keep it */

            /* next entry in list */
            (*List_insert_ptr_ptr) = (*List_insert_ptr_ptr)->next;

            /* do not forget to increment Pointer to next free
             * space in SiPathOccupation */
            if ((*List_insert_ptr_ptr) == NULL) {
                (*NextFree)++;
            }
        } else {
            /* this entry is nearer and wider than new entry */
            if ((*List_insert_ptr_ptr)->prev->DistTrajToObjEdge <
                SiPathOccInsertObjDataPtr->DistTrajToObjEdge) {
                /*This point once was endpoint of an object,
                narrower than actual, so keep it and just give
                it new ObjID and DistTrajToObjEdge data */
                (*List_insert_ptr_ptr)->ObjID =
                    SiPathOccInsertObjDataPtr->ObjID;
                (*List_insert_ptr_ptr)->DistTrajToObjEdge =
                    SiPathOccInsertObjDataPtr->DistTrajToObjEdge;

                /* next entry in list */
                (*List_insert_ptr_ptr) = (*List_insert_ptr_ptr)->next;

                /* do not forget to increment Pointer to next
                 * free space in SiPathOccupation */
                if ((*List_insert_ptr_ptr) == NULL) {
                    (*NextFree)++;
                }
            } else {
                /* seems to be a point wider than actual entry
                 * so delete it from list (open chain)*/
                if ((*List_insert_ptr_ptr)->next != NULL) {
                    SiPathOccupation[*NextFree].next =
                        (*List_insert_ptr_ptr)->next;
                    (*List_insert_ptr_ptr)->next->prev =
                        &SiPathOccupation[*NextFree];

                    last_deleted = (*List_insert_ptr_ptr);

                    /* next entry in list */
                    (*List_insert_ptr_ptr) = (*List_insert_ptr_ptr)->next;
                } else {
                    (*NextFree)++;
                    /* insert second point of object (object
                     * length)*/
                    SiPathOccupation[*ActualEntry].prev->next =
                        &SiPathOccupation[*NextFree];
                    SiPathOccupation[*NextFree].prev =
                        SiPathOccupation[*ActualEntry].prev;
                    SiPathOccupation[*NextFree].LengthTrajEgoFrontToObj =
                        SiPathOccInsertObjDataPtr->ObjDistOnTraj +
                        SiPathOccInsertObjDataPtr->CalculatedObjLength;
                    SiPathOccupation[*NextFree].DistTrajToObjEdge =
                        SiPathOccInsertObjDataPtr->HalbeSpurbreite;
                    SiPathOccupation[*NextFree].ObjID = OBJ_INDEX_NO_OBJECT;
                    SiPathOccupation[*NextFree].next = NULL;

                    /* next entry in list */
                    (*List_insert_ptr_ptr) = NULL;

                    /* prepare next edge entry */
                    *ActualEntry = *NextFree;
                    (*NextFree)++;
                }
            }
        }
    } else {
        if ((*List_insert_ptr_ptr)->prev->DistTrajToObjEdge <
            SiPathOccInsertObjDataPtr->DistTrajToObjEdge) {
            /* do not insert entry, because t is wider than
             * already saved point */
            /* break condition for while loop */
            (*List_insert_ptr_ptr) = NULL;
            (*NextFree)++;
        } else {
            /*found insertion point for new entry */
            (*NextFree)++;
            /* insert second point of object (object length)*/
            (*List_insert_ptr_ptr)->prev->next = &SiPathOccupation[*NextFree];
            SiPathOccupation[*NextFree].prev = (*List_insert_ptr_ptr)->prev;
            SiPathOccupation[*NextFree].LengthTrajEgoFrontToObj =
                SiPathOccInsertObjDataPtr->ObjDistOnTraj +
                SiPathOccInsertObjDataPtr->CalculatedObjLength;
            if (last_deleted != NULL) {
                SiPathOccupation[*NextFree].DistTrajToObjEdge =
                    last_deleted->DistTrajToObjEdge;
                SiPathOccupation[*NextFree].ObjID = last_deleted->ObjID;
            } else {
                SiPathOccupation[*NextFree].DistTrajToObjEdge =
                    SiPathOccInsertObjDataPtr->HalbeSpurbreite;
                SiPathOccupation[*NextFree].ObjID = OBJ_INDEX_NO_OBJECT;
            }
            SiPathOccupation[*NextFree].next = (*List_insert_ptr_ptr);
            (*List_insert_ptr_ptr)->prev = &SiPathOccupation[*NextFree];

            /* break condition for while loop */
            (*List_insert_ptr_ptr) = NULL;

            /* prepare next edge entry */
            (*NextFree)++;
        }
    }
}

static void SILookForPosToInsertSecondEdge2(
    SIPathOccupationArray_t SiPathOccupation,
    uint32* ActualEntry,
    uint32* NextFree,
    const SiPathOccInsertObjData_t* SiPathOccInsertObjDataPtr,
    SiPathOccupation_t** List_insert_ptr_ptr,
    SiPathOccupation_t* last_deleted) {
    if ((*List_insert_ptr_ptr)->LengthTrajEgoFrontToObj <
        (SiPathOccInsertObjDataPtr->ObjDistOnTraj +
         SiPathOccInsertObjDataPtr->CalculatedObjLength)) {
        if ((*List_insert_ptr_ptr)->DistTrajToObjEdge <=
            SiPathOccInsertObjDataPtr->DistTrajToObjEdge) {
            /* this point is nearer but narrower than actual
             * entry, so keep it */

            /* next entry in list */
            (*List_insert_ptr_ptr) = (*List_insert_ptr_ptr)->next;
        } else {
            /* this entry is nearer and wider than new entry */
            if ((*List_insert_ptr_ptr)->prev->DistTrajToObjEdge <
                SiPathOccInsertObjDataPtr->DistTrajToObjEdge) {
                /*This point once was endpoint of an object,
                narrower than actual, so keep it and just give
                it new ObjID and DistTrajToObjEdge data */
                (*List_insert_ptr_ptr)->ObjID =
                    SiPathOccInsertObjDataPtr->ObjID;
                (*List_insert_ptr_ptr)->DistTrajToObjEdge =
                    SiPathOccInsertObjDataPtr->DistTrajToObjEdge;

                /* next entry in list */
                (*List_insert_ptr_ptr) = (*List_insert_ptr_ptr)->next;
            } else {
                /* seems to be a point wider than actual entry
                 * so delete it from list (open chain)*/
                if ((*List_insert_ptr_ptr)->next != NULL)

                {
                    (*List_insert_ptr_ptr)->next->prev =
                        (*List_insert_ptr_ptr)->prev;
                    (*List_insert_ptr_ptr)->prev->next =
                        (*List_insert_ptr_ptr)->next;

                    last_deleted = (*List_insert_ptr_ptr);
                    /* next entry in list */
                    (*List_insert_ptr_ptr) = (*List_insert_ptr_ptr)->next;
                } else {
                    /* insert second point of object (object
                     * length)*/
                    SiPathOccupation[*ActualEntry].prev->next =
                        &SiPathOccupation[*NextFree];
                    SiPathOccupation[*NextFree].prev =
                        SiPathOccupation[*ActualEntry].prev;
                    SiPathOccupation[*NextFree].LengthTrajEgoFrontToObj =
                        SiPathOccInsertObjDataPtr->ObjDistOnTraj +
                        SiPathOccInsertObjDataPtr->CalculatedObjLength;
                    SiPathOccupation[*NextFree].DistTrajToObjEdge =
                        SiPathOccInsertObjDataPtr->HalbeSpurbreite;
                    SiPathOccupation[*NextFree].ObjID = OBJ_INDEX_NO_OBJECT;
                    SiPathOccupation[*NextFree].next = NULL;

                    /* next entry in list */
                    (*List_insert_ptr_ptr) = NULL;

                    /* prepare next edge entry */
                    *ActualEntry = *NextFree;
                    (*NextFree)++;
                }
            }
        }
    } else {
        if ((*List_insert_ptr_ptr)->prev->DistTrajToObjEdge <
            SiPathOccInsertObjDataPtr->DistTrajToObjEdge) {
            /* do not insert entry, because t is wider than
             * already saved point */
            /* break condition for while loop */
            (*List_insert_ptr_ptr) = NULL;
        } else {
            /* insert second point of object (object length)*/
            (*List_insert_ptr_ptr)->prev->next = &SiPathOccupation[*NextFree];
            SiPathOccupation[*NextFree].prev = (*List_insert_ptr_ptr)->prev;
            SiPathOccupation[*NextFree].LengthTrajEgoFrontToObj =
                SiPathOccInsertObjDataPtr->ObjDistOnTraj +
                SiPathOccInsertObjDataPtr->CalculatedObjLength;
            if (last_deleted != NULL) {
                SiPathOccupation[*NextFree].DistTrajToObjEdge =
                    last_deleted->DistTrajToObjEdge;
                SiPathOccupation[*NextFree].ObjID = last_deleted->ObjID;
            } else {
                SiPathOccupation[*NextFree].DistTrajToObjEdge =
                    SiPathOccInsertObjDataPtr->HalbeSpurbreite;
                SiPathOccupation[*NextFree].ObjID = OBJ_INDEX_NO_OBJECT;
            }
            SiPathOccupation[*NextFree].next = (*List_insert_ptr_ptr);
            (*List_insert_ptr_ptr)->prev = &SiPathOccupation[*NextFree];

            /* break condition for while loop */
            (*List_insert_ptr_ptr) = NULL;

            /* prepare next edge entry */
            (*NextFree)++;
        }
    }
}

/*************************************************************************************************************************
  Functionname:    SIInsertObjInPathOcc */
static void SIInsertObjInPathOcc(
    SIPathOccupationArray_t SiPathOccupation,
    uint32* ActualEntry,
    uint32* NextFree,
    const SiPathOccInsertObjData_t* SiPathOccInsertObjDataPtr) {
    SiPathOccupation_t* List_insert_ptr = NULL;
    SiPathOccupation_t* last_deleted = NULL;

    /* last entry at this side nearer than new entry ? */
    if (SiPathOccupation[*ActualEntry].LengthTrajEgoFrontToObj <
        SiPathOccInsertObjDataPtr->ObjDistOnTraj) {
        /* this is the simple case: add new edge at the end of the list */
        /* insert first point of object */
        SiPathOccupation[*ActualEntry].next = &SiPathOccupation[*NextFree];
        SiPathOccupation[*NextFree].prev = &SiPathOccupation[*ActualEntry];
        SiPathOccupation[*NextFree].LengthTrajEgoFrontToObj =
            SiPathOccInsertObjDataPtr->ObjDistOnTraj;
        SiPathOccupation[*NextFree].DistTrajToObjEdge =
            SiPathOccInsertObjDataPtr->DistTrajToObjEdge;
        SiPathOccupation[*NextFree].ObjID = SiPathOccInsertObjDataPtr->ObjID;

        /* prepare second point entry */
        *ActualEntry = *NextFree;
        (*NextFree)++;

        /* insert second point of object (object length)*/
        SiPathOccupation[*ActualEntry].next = &SiPathOccupation[*NextFree];
        SiPathOccupation[*NextFree].prev = &SiPathOccupation[*ActualEntry];
        SiPathOccupation[*NextFree].LengthTrajEgoFrontToObj =
            SiPathOccInsertObjDataPtr->ObjDistOnTraj +
            SiPathOccInsertObjDataPtr->CalculatedObjLength;
        SiPathOccupation[*NextFree].DistTrajToObjEdge =
            SiPathOccInsertObjDataPtr->HalbeSpurbreite;
        SiPathOccupation[*NextFree].ObjID = OBJ_INDEX_NO_OBJECT;
        SiPathOccupation[*NextFree].next = NULL;

        /* prepare next edge entry */
        *ActualEntry = *NextFree;
        (*NextFree)++;
    } /* end simple case */
    else {
        /* now it gets complex: the new entry must be inserted between the
         * previous found entries*/
        /* first look for distance to insert new edge */
        List_insert_ptr = &SiPathOccupation[*ActualEntry];
        while ((List_insert_ptr->LengthTrajEgoFrontToObj >
                SiPathOccInsertObjDataPtr->ObjDistOnTraj) &&
               (List_insert_ptr->prev != NULL)) {
            List_insert_ptr = List_insert_ptr->prev;
        }

        /* ListInsertPtr now stands at list entry nearer than new edge */
        /* now look for path width of new entry */
        while ((List_insert_ptr->DistTrajToObjEdge <
                SiPathOccInsertObjDataPtr->DistTrajToObjEdge) &&
               (List_insert_ptr->next != NULL)) {
            List_insert_ptr = List_insert_ptr->next;
        }

        /* ListInsertPtr now stands at list entry wider than new edge */
        /* if new edge is farer away than this entry insert new edge as follower
         * of ListInsertPtr*/
        if (SiPathOccInsertObjDataPtr->ObjDistOnTraj >
            List_insert_ptr->LengthTrajEgoFrontToObj) {
            SiPathOccupation[*NextFree].LengthTrajEgoFrontToObj =
                SiPathOccInsertObjDataPtr->ObjDistOnTraj;
            SiPathOccupation[*NextFree].DistTrajToObjEdge =
                SiPathOccInsertObjDataPtr->DistTrajToObjEdge;
            SiPathOccupation[*NextFree].ObjID =
                SiPathOccInsertObjDataPtr->ObjID;
            SiPathOccupation[*NextFree].prev = List_insert_ptr;
            SiPathOccupation[*NextFree].next = List_insert_ptr->next;
            List_insert_ptr->next = &SiPathOccupation[*NextFree];

            if (SiPathOccupation[*NextFree].next == NULL) {
                /* new entry is last in list, so directly insert second point */
                *ActualEntry = *NextFree;
                (*NextFree)++;
                /* insert second point of object (object length)*/
                SiPathOccupation[*ActualEntry].next =
                    &SiPathOccupation[*NextFree];
                SiPathOccupation[*NextFree].prev =
                    &SiPathOccupation[*ActualEntry];
                SiPathOccupation[*NextFree].LengthTrajEgoFrontToObj =
                    SiPathOccInsertObjDataPtr->ObjDistOnTraj +
                    SiPathOccInsertObjDataPtr->CalculatedObjLength;
                SiPathOccupation[*NextFree].DistTrajToObjEdge =
                    SiPathOccInsertObjDataPtr->HalbeSpurbreite;
                SiPathOccupation[*NextFree].ObjID =
                    SiPathOccInsertObjDataPtr->ObjID;
                SiPathOccupation[*NextFree].next = NULL;

                /* prepare next edge entry */
                *ActualEntry = *NextFree;
                (*NextFree)++;
            } else {
                /* new entry was not last in list, so look for position to
                 * insert second edge of entry */
                SiPathOccupation[*NextFree].next->prev =
                    &SiPathOccupation[*NextFree];
                List_insert_ptr = SiPathOccupation[*NextFree].next;

                while ((List_insert_ptr != NULL)) {
                    SILookForPosToInsertSecondEdge1(
                        SiPathOccupation, ActualEntry, NextFree,
                        SiPathOccInsertObjDataPtr, &List_insert_ptr,
                        last_deleted);
                }
            }
        } else {
            /* new entry is nearer than found wider edge --> do not save first
             * point of object !!*/
            /* ListInsertPtr still stands at list entry wider than new edge */
            if (List_insert_ptr->LengthTrajEgoFrontToObj >
                (SiPathOccInsertObjDataPtr->ObjDistOnTraj +
                 SiPathOccInsertObjDataPtr->CalculatedObjLength)) {
                /* second edge of object is even nearer than found wider edge so
                 * even do not save that point */
            } else if (List_insert_ptr->next == NULL) {
                /* refill DistTrajToObjEdge of new previous with actual
                 * DistTrajToObjEdge */
                List_insert_ptr->DistTrajToObjEdge =
                    SiPathOccInsertObjDataPtr->DistTrajToObjEdge;

                /* the found entry was last in list so directly insert second
                 * point after that entry*/
                List_insert_ptr->next = &SiPathOccupation[*NextFree];
                SiPathOccupation[*NextFree].prev = List_insert_ptr;
                SiPathOccupation[*NextFree].LengthTrajEgoFrontToObj =
                    SiPathOccInsertObjDataPtr->ObjDistOnTraj +
                    SiPathOccInsertObjDataPtr->CalculatedObjLength;
                SiPathOccupation[*NextFree].DistTrajToObjEdge =
                    SiPathOccInsertObjDataPtr->HalbeSpurbreite;
                SiPathOccupation[*NextFree].ObjID =
                    SiPathOccInsertObjDataPtr->ObjID;
                SiPathOccupation[*NextFree].next = NULL;

                /* prepare next edge entry */
                *ActualEntry = *NextFree;
                (*NextFree)++;
            } else {
                /* new entry was not last in list, so look for position to
                 * insert second edge of entry */
                List_insert_ptr = List_insert_ptr->next;

                while ((List_insert_ptr != NULL)) {
                    SILookForPosToInsertSecondEdge2(
                        SiPathOccupation, ActualEntry, NextFree,
                        SiPathOccInsertObjDataPtr, &List_insert_ptr,
                        last_deleted);
                }
            }
        }
    } /* end of complex case */
}

/*************************************************************************************************************************
  Functionname:    SICalcDriveAroundDist */
static float32 SICalcDriveAroundDist(float32 EgoSpeedXObjSync) {
    return MAX_FLOAT(EgoSpeedXObjSync, LAT_LANECHANGE_MINSPEED) *
           LAT_LANECHANGE_TIME;
}

/*************************************************************************************************************************
  Functionname:    SICalcOccPathWidth */
static void SICalcOccPathWidth(
    const SiPathOccupation_t SiPathOcc[SI_PATH_OVLC_ENTRIES],
    SIPathOccResultArray_t SiPathhOccResult) {
    const SiPathOccupation_t* ActualR =
        &SiPathOcc[0]; /* right start point is entry 0 */
    const SiPathOccupation_t* ActualL =
        &SiPathOcc[1]; /* left start point is entry 1 */
    SiPathOccResult_t* NextResult =
        &SiPathhOccResult[0]; /* result structure start point is 0 */

    /* initialize result structure */
    SIInitSiPathOccResult(SiPathhOccResult, ActualR->DistTrajToObjEdge);

    /* as long as there are entries on any side */
    while ((ActualR != NULL) && (ActualL != NULL)) {
        /* Calculate path width at actual entry point */
        NextResult->PathWidth =
            ActualR->DistTrajToObjEdge + ActualL->DistTrajToObjEdge;
        /* save object IDs of participating objects */
        NextResult->ObjIDR = ActualR->ObjID;
        NextResult->ObjIDL = ActualL->ObjID;
        /* save distance from sensor of actual entry point */
        NextResult->LengthTrajEgoFrontToObj = MAX_FLOAT(
            ActualR->LengthTrajEgoFrontToObj, ActualL->LengthTrajEgoFrontToObj);

        /*prepare next entry*/
        NextResult++;

        if ((ActualR->next != NULL) && (ActualL->next != NULL)) {
            /* Look if next closest entry point is on right or left side */
            if (ActualR->next->LengthTrajEgoFrontToObj >
                ActualL->next->LengthTrajEgoFrontToObj) {
                /* next entry is left */
                ActualL = ActualL->next;
            } else if (ActualR->next->LengthTrajEgoFrontToObj <
                       ActualL->next->LengthTrajEgoFrontToObj) {
                /* next entry is right */
                ActualR = ActualR->next;
            } else {
                /* if entry points on both sides are of equal distance, set both
                 * of them */
                ActualL = ActualL->next;
                ActualR = ActualR->next;
            }
        } else if (ActualR->next != NULL) {
            /* no more entries on left side, so set right entry as next */
            ActualR = ActualR->next;
        } else if (ActualL->next != NULL) {
            /* no more entries on right side, so set left entry as next */
            ActualL = ActualL->next;
        } else {
            /* no more entries on both sides, set both to break while loop */
            ActualL = ActualL->next;
            ActualR = ActualR->next;
        }
    }
}

/*************************************************************************************************************************
  Functionname:    SIInitSiPathOccResult */
static void SIInitSiPathOccResult(SIPathOccResultArray_t SiPathhOccResult,
                                  float32 HalfLaneWidth) {
    uint32 i;
    const float32 LaneWidth = 2.f * HalfLaneWidth;

    for (i = 0u; i < (uint32)SI_PATH_OVLC_ENTRIES; i++) {
        SiPathhOccResult[i].LengthTrajEgoFrontToObj =
            SI_PATH_OCCUPATION_LENGTH_TRAJEGO_FRONT_TO_OBJECT;
        SiPathhOccResult[i].PathWidth = LaneWidth;
        SiPathhOccResult[i].ObjIDL = OBJ_INDEX_NO_OBJECT;
        SiPathhOccResult[i].ObjIDR = OBJ_INDEX_NO_OBJECT;
    }
}

/*************************************************************************************************************************
  Functionname:    SICheckForNarrowPath */
static void SICheckForNarrowPath(
    const SiPathOccResult_t SiPathhOccResult[SI_PATH_OVLC_ENTRIES],
    ObjNumber_t* const pCloseObjId,
    ObjNumber_t* const pFarObjId) {
    const fVelocity_t fEgoSpeed = EGO_SPEED_X_OBJ_SYNC;
    const SiPathOccResult_t* NextResult = &SiPathhOccResult[0];
    float32 fSpeedSlowObj;
    float32 fPathwidthPassableMov;

    *pCloseObjId = OBJ_INDEX_NO_OBJECT;

    /* SiPathhOccResult is sorted by distance to sensor, so first detected
     * blocked path is closest */
    while ((NextResult->LengthTrajEgoFrontToObj <
            SI_PATH_OCCUPATION_LENGTH_TRAJEGO_FRONT_TO_OBJECT) &&
           (*pCloseObjId < 0)) {
        if ((NextResult->ObjIDL != OBJ_INDEX_NO_OBJECT) &&
            (NextResult->ObjIDR != OBJ_INDEX_NO_OBJECT)) {
            /*calculate passable corridor width depending on speed of corridor
             * objects*/
            fSpeedSlowObj =
                fEgoSpeed + MIN_FLOAT(OBJ_LONG_VREL(NextResult->ObjIDL),
                                      OBJ_LONG_VREL(NextResult->ObjIDR));
            fPathwidthPassableMov = dGDBmathLineareFunktion(
                &CorrSelPathwidthPassableMov, fSpeedSlowObj);
            /* path is build up by two objects */
            if (/* check for moving - moving constellation --> Moving pathwidth
                   is used */
                (((OBJ_DYNAMIC_PROPERTY(NextResult->ObjIDL) ==
                   CR_OBJECT_PROPERTY_MOVING) ||
                  (OBJ_IS_MOVING_TO_STATIONARY(NextResult->ObjIDL))) &&
                 ((OBJ_DYNAMIC_PROPERTY(NextResult->ObjIDR) ==
                   CR_OBJECT_PROPERTY_MOVING) ||
                  (OBJ_IS_MOVING_TO_STATIONARY(NextResult->ObjIDR))) &&
                 (NextResult->PathWidth < fPathwidthPassableMov))
                /* check for moving - moving constellation with former relevance
                   --> moving hysteresis pathwidth is used */
                ||
                ((((OBJ_DYNAMIC_PROPERTY(NextResult->ObjIDL) ==
                    CR_OBJECT_PROPERTY_MOVING) ||
                   (OBJ_IS_MOVING_TO_STATIONARY(NextResult->ObjIDL))) &&
                  ((OBJ_DYNAMIC_PROPERTY(NextResult->ObjIDR) ==
                    CR_OBJECT_PROPERTY_MOVING) ||
                   (OBJ_IS_MOVING_TO_STATIONARY(NextResult->ObjIDR)))) &&
                 ((NextResult->ObjIDR == SILastCycleOOIObjID[OBJ_NEXT_OOI]) ||
                  (NextResult->ObjIDL == SILastCycleOOIObjID[OBJ_NEXT_OOI])) &&
                 (NextResult->PathWidth <
                  (fPathwidthPassableMov +
                   SI_MIN_PATHWIDTH_PASSABLE_MOV_HYST_ADD)))
                /* minimum one of the objects is stationary use stationary
                   pathwidths */
                /* with former relevance....*/
                ||
                (((NextResult->ObjIDR == SILastCycleOOIObjID[OBJ_NEXT_OOI]) ||
                  (NextResult->ObjIDL == SILastCycleOOIObjID[OBJ_NEXT_OOI])) &&
                 (NextResult->PathWidth < SI_MIN_PATHWIDTH_PASSABLE_STAT_HYST))
                /* ...or without former relevance */
                || (NextResult->PathWidth < SI_MIN_PATHWIDTH_PASSABLE_STAT)) {
                /* found path width is to small */
                OBJ_GET_SI(NextResult->ObjIDR).Bool.SelectedByPathDecision = 1u;
                OBJ_GET_SI(NextResult->ObjIDL).Bool.SelectedByPathDecision = 1u;

                /* check for closer object of both and report it as narrow
                 * object*/
                if (OBJ_LONG_DISPLACEMENT(NextResult->ObjIDR) <
                    OBJ_LONG_DISPLACEMENT(NextResult->ObjIDL)) {
                    *pCloseObjId = NextResult->ObjIDR;
                    *pFarObjId = NextResult->ObjIDL;
                } else {
                    *pCloseObjId = NextResult->ObjIDL;
                    *pFarObjId = NextResult->ObjIDR;
                }
            }
        }
        NextResult++;
    }
}

/*************************************************************************************************************************
  Functionname:    SIResetBlockedPathDecision */
static void SIResetBlockedPathDecision(void) {
    ObjNumber_t i;

    for (i = 0; i < Envm_N_OBJECTS; i++) {
        /* Reset selection bit for blocked path decission */
        OBJ_GET_SI(i).Bool.SelectedByPathDecision = 0u;
    }
}

/*************************************************************************************************************************
  Functionname:    SIUpdateTimerBlockedPathDecision */
static void SIUpdateTimerBlockedPathDecision(void) {
    ObjNumber_t i;

    for (i = 0; i < Envm_N_OBJECTS; i++) {
        /* Check if given object selected by blocked path decission in this
        cycle. If not, then decrease the path selection timer */
        if (OBJ_GET_SI(i).Bool.SelectedByPathDecision == 0u) {
            /* Object not selected by blocked path decission in this cycle,
            update
            timer */
            if (OBJ_GET_SI(i).BlockedPathDecision.PathSelectionTimer > 1u) {
                OBJ_GET_SI(i).BlockedPathDecision.PathSelectionTimer -= 2u;
            } else {
                OBJ_GET_SI(i).BlockedPathDecision.PathSelectionTimer = 0u;
            }
        }
    }
}

/*************************************************************************************************************************
  Functionname:    SIGetObjectWidthInLane */
static float32 SIGetObjectWidthInLane(ObjNumber_t iObject) {
    float32 fObjectWidth;

    if (OBJ_CLASSIFICATION(iObject) == CR_OBJCLASS_POINT) {
        fObjectWidth = SI_VALID_OBJECT_WIDTH_IN_LANE_FOR_ARSOBJ_CLASS_POINT;
    } else {
        fObjectWidth = SI_OBJECT_WIDTH_IN_LANE_FOR_NON_ARSOBJ_CLASS_POINT;
    }
    return fObjectWidth;
}

/*************************************************************************************************************************
  Functionname:    SISelectOOI */
static void SISelectOOI(ObjNumber_t ObjectToSelect) {
    OBJ_GET_SI(ObjectToSelect).Bool.SelectedAsOOI = 1u;
}

/*************************************************************************************************************************
  Functionname:    SIUnSelectOOI */
static void SIUnSelectOOI(ObjNumber_t ObjectToUnSelect) {
    OBJ_GET_SI(ObjectToUnSelect).Bool.SelectedAsOOI = 0u;
}

/* ************************************************************************* */
/*   Copyright                                              */
/* ************************************************************************* */

/* ************************************************************************* */
/*   Copyright                                           */
/* ************************************************************************* */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */