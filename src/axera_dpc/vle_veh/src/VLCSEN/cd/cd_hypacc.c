
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "cd.h"

#include "vlc_glob_ext.h"

#include "TM_Global_Const.h"
#include "vlcSen_consts.h"
#include "vlcSen_common_utils.h"

// calibration
#define CAL_START_CODE
#include "Mem_Map.h"

const volatile float TRKASGN_TRACK_OVERLAP_NOW_OFFSET = -0.12f;
const volatile float TRKASGN_OBJ_OVERLAP_NOW_OFFSET = -0.12f;
const volatile float TRKASGN_TRACK_OVERLAP_PRED_OFFSET = 0.0f;
const volatile float TRKASGN_OBJ_OVERLAP_PRED_OFFSET = 0.0f;

#define CAL_STOP_CODE
#include "Mem_Map.h"

#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
static boolean bTrackAssigned_flag;

static EMPTrajOccupancy_t Occupancy_now_A, Occupancy_now_B;
static EMPTrajOccupancy_t Occupancy_pre_A, Occupancy_pre_B;
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
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
static boolean CDHypoAccObjectFilter(
    ObjNumber_t iObjectIndex,
    const CDObjectData_t *pObjectData,
    const CDInternalObject_t *currentObjInternal);

static void CDHypoAccCalculateProb(ObjNumber_t iObjectIndex,
                                   CDIntHypothesis_t *pHypothesis);

/*************************************************************************************************************************
  Functionname:    CDAssignTrackProbability */

void CDAssignTrackProbability(ObjNumber_t iObjectIndex,
                              const CDInputData_t *pInputData,
                              const CDInternalStatus_t *pInternalStatus) {
    EMPDistanceWidth_t DistWidth_now;
    EMPDistanceWidth_t DistWidth_pre;
    EMPTrajOccupancy_t Occupancy_now;
    EMPTrajOccupancy_t Occupancy_pre;

    float32 fDistTraj;
    float32 fDistTrajVar;
    float32 fDistCourse;
    const float32 fCurve = CPCDGetCurvatureEgo(0);
    const float32 fAbsCurve = fABS(fCurve);

    boolean bTrackAssigned = FALSE;
    CDInternalObject_t *const currentObjInternal =
        &(*pInternalStatus->rgObjInternal)[iObjectIndex];

    currentObjInternal->TrackAssigned =
        (currentObjInternal->TrackAssigned >> 1);
    if (!OBJ_IS_DELETED(iObjectIndex)) {
        const Envm_t_GenObjDynamicProperty eObjDynProp =
            CDGetPointer_Attributes(pInputData->pObjectData, iObjectIndex)
                ->eDynamicProperty;
        /* Local variable for object width */
        const float32 fObjWidth =
            CDGetPointer_Dimensions(pInputData->pObjectData, iObjectIndex)
                ->fWidth;
        /* Local variable for object width variance */
        const float32 fObjWidthVariance =
            SQR(CD_OBJ_WIDTH_STDDEV(pInputData->pObjectData, iObjectIndex));
        /*! due to bad road trajectory quality in curves we switch to the ego
         * trajectory for stationary objects in curves*/
        if (((eObjDynProp == (Envm_t_GenObjDynamicProperty)
                                 Envm_GEN_OBJECT_DYN_PROPERTY_STATIONARY) ||
             (eObjDynProp == (Envm_t_GenObjDynamicProperty)
                                 Envm_GEN_OBJECT_DYN_PROPERTY_STOPPED)) &&
            (fAbsCurve > (1.0f / CD_TRKASGN_MIN_RAD_FOR_ROAD_TRAJ))) {
            /* Query the object's relation to the ego trajectory once here and
             * re-use the struct */
            const CPCDObjToTrajRelation_t *sObjEgoTrajRel =
                CPCDGetObjToTrajRelationEgo(iObjectIndex);
            /* set distance structure */

            /* prediction of object position 1.5s or ttc if below */
            fDistCourse = sObjEgoTrajRel->fDistToTraj +
                          (sObjEgoTrajRel->fVelocityToTraj *
                           MIN_FLOAT(currentObjInternal->TTC,
                                     CD_TRKASGN_CURVE_OVRLAP_PRED_TIME));
            /* Set Structure for "NOW" */
            DistWidth_now.fTrajectoryWidth = CD_RUN_UP_TRACK_WIDTH;
            DistWidth_now.fTrajectoryWidthVar = 0;
            DistWidth_now.fDistanceVar =
                sObjEgoTrajRel->fDistToTrajVar + (0.25F * fObjWidthVariance);
            DistWidth_now.fDistance = sObjEgoTrajRel->fDistToTraj;
            DistWidth_now.fObjectWidth = fObjWidth;
            DistWidth_now.fObjectWidthVar = fObjWidthVariance;

            /* Set Structure for "Prediction" */
            DistWidth_pre.fTrajectoryWidth = CD_RUN_UP_TRACK_WIDTH;
            DistWidth_pre.fTrajectoryWidthVar = 0;
            /* Variance from course filter belongs to distance to trajectory.
             * variance is not changed by prediction */
            DistWidth_pre.fDistanceVar =
                sObjEgoTrajRel->fDistToTrajVar + (0.25F * fObjWidthVariance);
            DistWidth_pre.fDistance = fDistCourse;
            DistWidth_pre.fObjectWidth = fObjWidth;
            DistWidth_pre.fObjectWidthVar = fObjWidthVariance;

            /* calculate current overlap */
            EMPCPCalculateOverlap(&DistWidth_now, &Occupancy_now);
            Occupancy_now_A = Occupancy_now;
            /* calculate predicted overlap after t=MIN(ttc,
             * CD_RUN_UP_STAT_MAX_PRED_TIME ) */
            EMPCPCalculateOverlap(&DistWidth_pre, &Occupancy_pre);
            Occupancy_pre_A = Occupancy_pre;
            if (((Occupancy_now.fTrajectoryOccupancy >
                  (CD_TRKASGN_MIN_TRACK_OVERLAP)) ||
                 (Occupancy_now.fObjectOccupancy >
                  (CD_TRKASGN_MIN_OBJECT_OVERLAP))) &&
                ((Occupancy_pre.fTrajectoryOccupancy >
                  (CD_TRKASGN_MIN_TRACK_OVERLAP +
                   CD_TRKASGN_TRACK_OVERLAP_PRED_OFFSET)) ||
                 (Occupancy_pre.fObjectOccupancy >
                  (CD_TRKASGN_MIN_OBJECT_OVERLAP +
                   CD_TRKASGN_OBJ_OVERLAP_PRED_OFFSET)))) {
                bTrackAssigned = TRUE;
            } else {
                bTrackAssigned = FALSE;
            }

        } else {
            /* Non-stationary object: query fused trajectory relative
             * information for object */
            const CPCDObjToTrajRelation_t *sObjRoadTrajRel =
                CPCDGetObjToTrajRelationRoad(iObjectIndex);
            /* set distance structure */
            fDistTraj = sObjRoadTrajRel->fDistToTraj;
            fDistTrajVar = sObjRoadTrajRel->fDistToTrajVar;

            /* check if avoidance maneuver and if so release object
             * an avoidance maneuver is assumed if
             * a) object has min. required probability of existence
             * b) object has small longitudinal distance
             * c) TTC is below some threshold
             *
             * if distance and TTC condition are fulfilled (i.e. avoidance
             * maneuver) the maximum
             * of distance to trajectory or distance to course is being used as
             * distance estimate
             *
             * if distance and TTC condition are not fulfilled distance distance
             * to trajectory is used
             * and trajectory width is estimated by object distance
             */
            if (CD_EBA_OBJ_QUALITY(pInputData->pObjectData, iObjectIndex) >
                CD_COMMON_MIN_OBJ_QUALITY) {
                if ((CD_GET_DIST_X(pInputData->pObjectData, iObjectIndex) <
                     CD_TRKASGN_MAX_AVOID_DIST) &&
                    (currentObjInternal->TTC < CD_TRKASGN_MAX_AVOID_TTC)) {
                    DistWidth_now.fTrajectoryWidth = CDCalcCorridorWidth(
                        CD_GET_DIST_X(pInputData->pObjectData, iObjectIndex),
                        CD_AVLC_LENGTH, (CD_RUN_UP_TRACK_WIDTH - (0.1f)),
                        CD_AVLC_TRACK_WIDTH,
                        CD_AVLC_TO_RUNUP_LENGTH); /* Maybe other width */

                    if (fABS(CDGetPointer_Kinematic(pInputData->pObjectData,
                                                    iObjectIndex)
                                 ->fVrelY) > CD_TRKASGN_MIN_LAT_AVOID_SPEED) {
                        /* calculate obect distance to course after TTC */
                        fDistCourse =
                            CD_GET_DIST_Y(pInputData->pObjectData,
                                          iObjectIndex) +
                            (currentObjInternal->TTC *
                             CDGetPointer_Kinematic(pInputData->pObjectData,
                                                    iObjectIndex)
                                 ->fVrelY);

                        if (fABS(fDistCourse) > fABS(fDistTraj)) {
                            DistWidth_now.fDistance = fDistCourse;
                        } else {
                            DistWidth_now.fDistance = fDistTraj;
                        }
                    } else {
                        DistWidth_now.fDistance = fDistTraj;
                    }
                } else {
                    DistWidth_now.fTrajectoryWidth = CDCalcCorridorWidth(
                        CD_GET_DIST_X(pInputData->pObjectData, iObjectIndex),
                        CD_AVLC_LENGTH, CD_RUN_UP_TRACK_WIDTH,
                        CD_AVLC_TRACK_WIDTH, CD_AVLC_TO_RUNUP_LENGTH);

                    DistWidth_now.fDistance = fDistTraj;
                }

                DistWidth_now.fDistanceVar = fDistTrajVar;
                DistWidth_now.fTrajectoryWidthVar = 0;
                DistWidth_now.fObjectWidth = fObjWidth;
                DistWidth_now.fObjectWidthVar = fObjWidthVariance;

                /* calc OVERLAP  */
                EMPCPCalculateOverlap(&DistWidth_now, &Occupancy_now);
                Occupancy_now_B = Occupancy_now;

                if ((Occupancy_now.fTrajectoryOccupancy >
                     (CD_TRKASGN_MIN_TRACK_OVERLAP +
                      TRKASGN_TRACK_OVERLAP_NOW_OFFSET)) ||
                    (Occupancy_now.fObjectOccupancy >
                     (CD_TRKASGN_MIN_OBJECT_OVERLAP +
                      TRKASGN_OBJ_OVERLAP_NOW_OFFSET)))

                {
                    /* predict overlap for stationary objects*/
                    if ((eObjDynProp !=
                         (Envm_t_GenObjDynamicProperty)
                             Envm_GEN_OBJECT_DYN_PROPERTY_STATIONARY) &&
                        (eObjDynProp !=
                         (Envm_t_GenObjDynamicProperty)
                             Envm_GEN_OBJECT_DYN_PROPERTY_STOPPED)) {
                        bTrackAssigned = TRUE;
                    } else {
                        const CPCDObjToTrajRelation_t *sObjEgoTrajRel =
                            CPCDGetObjToTrajRelationEgo(iObjectIndex);
                        fDistCourse =
                            sObjEgoTrajRel->fDistToTraj +
                            (sObjEgoTrajRel->fVelocityToTraj *
                             MIN_FLOAT(currentObjInternal->TTC,
                                       CD_TRKASGN_STRAIGHT_OVRLAP_PRED_TIME));

                        DistWidth_pre = DistWidth_now;
                        DistWidth_pre.fDistance = fDistCourse;

                        EMPCPCalculateOverlap(&DistWidth_pre, &Occupancy_pre);
                        Occupancy_pre_B = Occupancy_pre;

                        if ((Occupancy_pre.fTrajectoryOccupancy >
                             (CD_TRKASGN_MIN_TRACK_OVERLAP +
                              CD_TRKASGN_TRACK_OVERLAP_PRED_OFFSET +
                              TRKASGN_TRACK_OVERLAP_PRED_OFFSET)) ||
                            (Occupancy_pre.fObjectOccupancy >
                             (CD_TRKASGN_MIN_OBJECT_OVERLAP +
                              CD_TRKASGN_OBJ_OVERLAP_PRED_OFFSET +
                              TRKASGN_OBJ_OVERLAP_PRED_OFFSET))) {
                            bTrackAssigned = TRUE;
                        } else {
                            bTrackAssigned = FALSE;
                        }
                    }
                } else {
                    bTrackAssigned = FALSE;
                }
            } else {
                /* Object quality not sufficient */
                Occupancy_now.fTrajectoryOccupancy = 0;
                Occupancy_now.fObjectOccupancy = 0;
                Occupancy_pre.fTrajectoryOccupancy = 0;
                Occupancy_pre.fObjectOccupancy = 0;
                bTrackAssigned = FALSE;
            }
        }
    } else {
        /* Deleted object */
        Occupancy_now.fTrajectoryOccupancy = 0;
        Occupancy_now.fObjectOccupancy = 0;
        Occupancy_pre.fTrajectoryOccupancy = 0;
        Occupancy_pre.fObjectOccupancy = 0;
        bTrackAssigned = FALSE;
    }
    bTrackAssigned_flag = bTrackAssigned;
    /* if track assigned update shift register */
    if (bTrackAssigned != FALSE) {
        currentObjInternal->TrackAssigned += (uint8)128;
    }
}

/* ************************************************************************* */
/*   Copyright Tuerme                     */
/* ************************************************************************* */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */