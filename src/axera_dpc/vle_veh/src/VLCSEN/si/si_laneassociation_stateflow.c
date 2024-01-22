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
#define SI_IN2OUTLANE_MAX_TRANSITIONTIME \
    (42u) /*!< 42 cycles, that means about three seconds */

/*! Check if object is associated to ego lane based on the camera information */

/*! Parameters to determine when a stationary object is not selected as OOI
    since it is an oncoming and stopped object */
#define SI_STATOBJ_ONCOMINGCOUNTER_MIN (30u)
#define SI_STATOBJ_ONCOMING_XDIST_HYSTERESIS (5.f)
#define SI_PAR_STOPPED_ONCOMING_INLANE_DISTX_MAX \
    (2.f * SI_PAR_ONCOMMING_KEEP_RELEV_DISTX_MAX)

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  FUNCTION
*****************************************************************************/
void SIObj2TrajInit(void);

/*************************************************************************************************************************
  Functionname:    SIObj2TrajInit */
void SIObj2TrajInit(void) {
    ObjNumber_t iObj;
    CPObjDist2TrajMeas_t ObjDistMeas;

    SICourseData.fCurve = 0.0f;
    SICourseData.fCurveVar = 0.0f;
    SICourseData.fCurveGradient = 0.0f;
    SICourseData.SideSlipAngle = 0.0f;
    SICourseData.SideSlipAngleVariance = 0.0f;
    CPTrajectoryInit(&SICourseData, SI_USE_ROADESTIM, SI_USE_OBJTRACES,
                     SI_USE_CAM_LANE_FUSION, SI_USE_NAVI_PATH_FUSION,
                     &SITrajectoryData);

    ObjDistMeas.Y.f0 = 0.0f;

    ObjDistMeas.R.f00 = 0.0f;
    for (iObj = (ObjNumber_t)(Envm_N_OBJECTS - 1); iObj >= 0; iObj--) {
        CPInitObjDist2Traj(&ObjDistMeas, &(OBJ_GET_CP(iObj).TrajDist));
    }

    SICorridorInit();
}

/*************************************************************************************************************************
  Functionname:    SI_b_CheckObjLaneQuality */
boolean SI_b_CheckObjLaneQuality(const ObjNumber_t s_Obj) {
    boolean b_Ret = TRUE;
    /*  Objects which do not pass base pre-selection, may not be associated to a
     * lane */
    if (SIBasePreselObjList[s_Obj] == FALSE) {
        b_Ret = FALSE;
    }
    return b_Ret;
}

/*************************************************************************************************************************
  Functionname:    SI_b_CheckObjOncRollBack */
boolean SI_b_CheckObjOncRollBack(const ObjNumber_t s_Obj,
                                 const eAssociatedLane_t e_BaseAssocLane) {
    boolean b_Ret = TRUE;
    /* Only oncoming (AND NOT crossing) objects are considered in this check */
    if ((OBJ_DYNAMIC_PROPERTY(s_Obj) == CR_OBJECT_PROPERTY_ONCOMING) &&
        (OBJ_DYNAMIC_SUB_PROPERTY(s_Obj) != CR_OBJECT_SUBPROP_CROSSING)) {
        /* Handling for oncoming ego lane objects: These may only return true
           for the special 'relevant
           object rolling back' situation, as described above */
        if (e_BaseAssocLane == ASSOC_LANE_EGO) {
            if ((OBJ_GET_SI(s_Obj).Bool.Moving == 1u) &&
                (OBJ_GET_RELEVANT(s_Obj)) &&
                (OBJ_LONG_DISPLACEMENT(s_Obj) <
                 SI_PAR_ONCOMMING_KEEP_RELEV_DISTX_MAX) &&
                ((OBJ_LONG_VREL(s_Obj) + EGO_SPEED_X_OBJ_SYNC) >
                 SI_PAR_SPEED_SUM_ROLL_BACK_MIN)) {
                b_Ret = TRUE;
            } else {
                b_Ret = FALSE;
            }
        }
    }
    return b_Ret;
}

/*************************************************************************************************************************
  Functionname:    SI_b_CheckObjAdjLaneValidity */
boolean SI_b_CheckObjAdjLaneValidity(const ObjNumber_t s_Obj,
                                     const eAssociatedLane_t e_BaseAssocLane) {
    boolean b_Ret = TRUE;
    sint32 s_NumberOfLanes;

    /* Set the default lateral distance, up to which moving bikes or motorcycles
       may be OOI on left or right lane, even if there is no valid adjacent lane
       */
    float32 f_AbsDistBorder = SI_MOVING_CLIP_BIKE_ROAD_BORDER_ABSDIST;
    /* Get the absolute value of object's lateral distance */
    float32 f_AbsDistObj2Ref = fABS(OBJ_GET_OBJ_TO_REF_DISTANCE(s_Obj));

    /* No road estimation input to VLC available, so assume one valid adjacent
     * lane */
    s_NumberOfLanes = 1;

    /* Handling of moving and crossing objects */
    if ((OBJ_DYNAMIC_PROPERTY(s_Obj) == CR_OBJECT_PROPERTY_MOVING) ||
        (OBJ_DYNAMIC_SUB_PROPERTY(s_Obj) == CR_OBJECT_SUBPROP_CROSSING)) {
        /* Verify that there is a valid adjacent lane based on the lane matrix
           or the object was already OOI,
           or it is confirmed by the camera (thus definitely not a mirror or
           ghost). Moving objects, which
           are classified as bike or motorcyle are also selectable as OOI, even
           if there is no valid lane
           based on lane matrix. For all other moving objects this is actually a
           mirror supression.*/
        if ((s_NumberOfLanes != 0) || (OBJ_GET_SI(s_Obj).Bool.AlreadyOOI) ||
            ((f_AbsDistObj2Ref < f_AbsDistBorder) &&
             ((OBJ_CLASSIFICATION(s_Obj) == CR_OBJCLASS_BICYCLE) ||
              (OBJ_CLASSIFICATION(s_Obj) == CR_OBJCLASS_MOTORCYCLE)))) {
            b_Ret = TRUE;
        } else {
            b_Ret = FALSE;
        }
    }

    /* Handling of stationary objects */
    else if ((OBJ_DYNAMIC_PROPERTY(s_Obj) == CR_OBJECT_PROPERTY_STATIONARY) &&
             (OBJ_DYNAMIC_SUB_PROPERTY(s_Obj) != CR_OBJECT_SUBPROP_CROSSING)) {
        /* Verify that there is a valid adjacent lane based on the lane matrix
           or the object was already OOI,
           or it is confirmed by the camera (thus definitely not a mirror or
           ghost). */
        if ((s_NumberOfLanes != 0) || (OBJ_GET_SI(s_Obj).Bool.AlreadyOOI)) {
            b_Ret = TRUE;
        } else {
            b_Ret = FALSE;
        }
    }

    /* Handling of oncoming objects*/
    else if ((OBJ_DYNAMIC_PROPERTY(s_Obj) == CR_OBJECT_PROPERTY_ONCOMING) &&
             (OBJ_DYNAMIC_SUB_PROPERTY(s_Obj) != CR_OBJECT_SUBPROP_CROSSING)) {
        float32 f_DistanceToTraj;
        float32 f_DistanceToTrajVar;
        /* Get distance to trajectory to decide if object is on left or right
         * side  of lane */
        SITrajGetObjToRefDistance(s_Obj, &f_DistanceToTraj,
                                  &f_DistanceToTrajVar);

        if (e_BaseAssocLane == ASSOC_LANE_EGO) {
            /* This function checks the validity of adjacent lanes. Thus set the
               return value
               TRUE for ego lane objects. The special 'relevant object rolling
               back' situation
               for oncoming objects on the ego-lane is checked in
               SI_b_CheckObjOncRollBack. */
            b_Ret = TRUE;
        }
        /* Check objects on left adjacent lane (including objects, that are not
           associated to
           ASSOC_LANE_LEFT, but which are on the left side of the ego-course) */
        else if (((e_BaseAssocLane == ASSOC_LANE_LEFT) ||
                  (f_DistanceToTraj >= 0.0f))) {
            /* No road estimation input to VLC available, so assume valid left
             * lane */
            b_Ret = TRUE;
        }
        /* Check objects on right adjacent lane (including objects, that are not
          associated to
          ASSOC_LANE_RIGHT, but which are on the right side of the ego-course)
          */
        else if (((e_BaseAssocLane == ASSOC_LANE_RIGHT) ||
                  (f_DistanceToTraj < 0.0f))) {
            /* No road estimation input to VLC available, so assume valid right
             * lane */
            b_Ret = TRUE;
        } else {
            /* do nothing, only objects on adjacent lanes to host lane are
             * checked */
        }
    } else {
        /* do nothing, no valid dynamic property */
    }
    return b_Ret;
}

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */