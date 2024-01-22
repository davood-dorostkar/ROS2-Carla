
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

SET_MEMSEC_CONST(SITimeThreshInLaneFromDist)
static const GDBLFunction_t SITimeThreshInLaneFromDist = {

    SI_MINTIME_MOVE_OBJ_IN_LANE_MIN, SI_MINTIME_MOVE_OBJ_IN_LANE_MAX,

    ((SI_MINTIME_MOVE_OBJ_IN_LANE_MAX - SI_MINTIME_MOVE_OBJ_IN_LANE_MIN) /
     (SI_DISTX_MOVE_OBJ_IN_LANE_MAX - SI_DISTX_MOVE_OBJ_IN_LANE_MIN)),

    SI_MINTIME_MOVE_OBJ_IN_LANE_MIN -
        (((SI_MINTIME_MOVE_OBJ_IN_LANE_MAX - SI_MINTIME_MOVE_OBJ_IN_LANE_MIN) /
          (SI_DISTX_MOVE_OBJ_IN_LANE_MAX - SI_DISTX_MOVE_OBJ_IN_LANE_MIN)) *
         SI_DISTX_MOVE_OBJ_IN_LANE_MIN)};

#define SI_IN2OUTLANE_MAX_TRANSITIONTIME \
    (42u) /*!< 42 cycles, that means about three seconds */

/*! Parameters to determine when a stationary object is not selected as OOI
    since it is an oncoming and stopped object */
#define SI_STATOBJ_ONCOMINGCOUNTER_MIN (30u)
#define SI_STATOBJ_ONCOMING_XDIST_HYSTERESIS (5.f)
#define SI_PAR_STOPPED_ONCOMING_INLANE_DISTX_MAX \
    (2.f * SI_PAR_ONCOMMING_KEEP_RELEV_DISTX_MAX)

static const SIFindObjInAreaArgs_t SIOvertakingCheckArgsHighVego =

    {(SI_HIGH_VEGO_OBJ_SUPPRESS_DELTAX_LOWER),
     (SI_HIGH_VEGO_OBJ_SUPPRESS_DELTAX_UPPER),
     (SI_HIGH_VEGO_OBJ_SUPPRESS_DELTAY_LOWER),
     (SI_HIGH_VEGO_OBJ_SUPPRESS_DELTAY_UPPER),
     (SI_HIGH_VEGO_OBJ_SUPPRESS_DELTAV_LOWER),
     (SI_HIGH_VEGO_OBJ_SUPPRESS_DELTAV_UPPER)};

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  FUNCTION
*****************************************************************************/

static eAssociatedLane_t SIGetAssociateLane(const ObjNumber_t ObjId);
static eAssociatedLane_t SIGetObjSelectionLane(
    const ObjNumber_t ObjId, const eAssociatedLane_t eBaseAssocLane);
static boolean SICheckStateFlowOutlaneToInlane(
    const ObjNumber_t iObj, const CPTrajOccupancy_t* pOccupancy);
static boolean SICheckInlaneOccupancies(const ObjNumber_t iObj,
                                        const CPTrajOccupancy_t* pOccupancy);
static float32 SIGetObjOccPickupThreshold(const ObjNumber_t iObj);
static float32 SIGetCorrectedOccupancy(const ObjNumber_t iObj,
                                       const float32 occupancy,
                                       const float32 obj_occ_stddev);
static float32 SIGetLaneOccPickupThreshold(const ObjNumber_t iObj);
static boolean SICheckLaneOccPickupValue(const ObjNumber_t iObj,
                                         const CPTrajOccupancy_t* pOccupancy);
static fTime_t SIGetInLaneTimeThreshold(const ObjNumber_t iObj);
static fTime_t SIGetSituationVrelInLaneTimeThreshold(const ObjNumber_t iObj,
                                                     fTime_t fTimeThreshInLane);
static fTime_t SIGetHighEgoVelInLaneTimeThreshold(const ObjNumber_t iObj,
                                                  fTime_t fTimeThreshInLane);
static fTime_t SICalcInlaneTimeStatVehicle(fVelocity_t const fVelocity,
                                           const ObjNumber_t iObj);
static fTime_t SICalcInlaneTimeStatNonVehicle(fVelocity_t const fVelocity,
                                              const ObjNumber_t iObj);
static boolean SICheckInlaneTimer(const ObjNumber_t iObj);
static void SIResetInlaneTimer(const ObjNumber_t iObj);
static fDistance_t SIGetInLaneDistanceThreshold(const ObjNumber_t iObj);
static boolean SICheckInlaneDistance(const ObjNumber_t iObj);
static void SIResetInlaneDistance(const ObjNumber_t iObj);
static boolean SICheckStateFlowInlaneToOutlane(
    const ObjNumber_t iObj, const CPTrajOccupancy_t* pOccupancy);
static boolean SICheckOutlaneOccupancies(const ObjNumber_t iObj,
                                         const CPTrajOccupancy_t* pOccupancy);
static float32 SIGetObjOccDropThreshold(const ObjNumber_t iObj);
static boolean SICheckObjOccDropValue(const ObjNumber_t iObj,
                                      const CPTrajOccupancy_t* pOccupancy);
static float32 SIGetLaneOccDropThreshold(const ObjNumber_t iObj);
static boolean SICheckLaneOccDropValue(const ObjNumber_t iObj,
                                       const CPTrajOccupancy_t* pOccupancy);
static boolean SICheckObjInlaneAllowed(const ObjNumber_t iObj);

/*************************************************************************************************************************
  Functionname:    SILaneAssociation */
void SILaneAssociation(void) {
    CPDistanceWidth_t* pDistWidth;
    static CPDistanceWidth_t DistWidthArray[Envm_N_OBJECTS];
    CPTrajOccupancy_t Occupancy;
    ObjNumber_t iObj;

    /* Calculate the trace brackets */
    SI_Calculate_AVLC_Corridor(DistWidthArray);

    for (iObj = (ObjNumber_t)(Envm_N_OBJECTS - 1); iObj >= 0; iObj--) {
        eAssociatedLane_t eObjAssocLane; /*!< Lane associated with object (on
                                            external interface) */
        eAssociatedLane_t e_ObjFuncLane; /*!< Functional Lane associated with
                                            object for object selection (on
                                            external interface) */
        /* calc BORDERS*/
        /* SI determines lane width and borders and calls CPCalculateOverlap
         * using this borders */
        pDistWidth = &DistWidthArray[iObj];

        /* Standard overlap and occupancy computation */
        CPCalculateOverlap(pDistWidth, &Occupancy);

        GET_VLC_OBJ(iObj).VLCCustomObjectProperties.Occupancy.fObjectOccupancy =
            Occupancy.fObjectOccupancy;
        GET_VLC_OBJ(iObj)
            .VLCCustomObjectProperties.Occupancy.fObjectOccupancyVar =
            Occupancy.fObjectOccupancyVar;
        GET_VLC_OBJ(iObj).VLCCustomObjectProperties.Occupancy.fOverlap =
            Occupancy.fOverlap;
        GET_VLC_OBJ(iObj).VLCCustomObjectProperties.Occupancy.fOverlapVar =
            Occupancy.fOverlapVar;
        GET_VLC_OBJ(iObj)
            .VLCCustomObjectProperties.Occupancy.fTrajectoryOccupancy =
            Occupancy.fTrajectoryOccupancy;
        GET_VLC_OBJ(iObj)
            .VLCCustomObjectProperties.Occupancy.fTrajectoryOccupancyVar =
            Occupancy.fTrajectoryOccupancyVar;

        if (!OBJ_IS_DELETED(iObj)) {
            if (OBJ_GET_SI(iObj).ObjLaneAccStatus.In2OutlaneTransition > 0u) {
                OBJ_GET_SI(iObj).ObjLaneAccStatus.In2OutlaneTransition =
                    OBJ_GET_SI(iObj).ObjLaneAccStatus.In2OutlaneTransition - 1u;
            } else {
                OBJ_GET_SI(iObj).ObjLaneAccStatus.In2OutlaneTransition = 0u;
            }

            switch (OBJ_GET_SI(iObj).ObjLaneAccStatus.SIInlaneState) {
                case OBJ_STATE_OUTLANE:
                    if (SICheckStateFlowOutlaneToInlane(iObj, &Occupancy) ==
                        TRUE) {
                        OBJ_GET_SI(iObj).ObjLaneAccStatus.SIInlaneState =
                            OBJ_STATE_INLANE;
                        OBJ_GET_SI(iObj).ObjLaneAccStatus.SIActLaneState =
                            OBJ_STATE_INLANE;
                    }
                    break;

                case OBJ_STATE_INLANE:
                    if (SICheckStateFlowInlaneToOutlane(iObj, &Occupancy) ==
                        TRUE) {
                        OBJ_GET_SI(iObj).ObjLaneAccStatus.SIInlaneState =
                            OBJ_STATE_OUTLANE;
                        OBJ_GET_SI(iObj).ObjLaneAccStatus.SIActLaneState =
                            OBJ_STATE_OUTLANE;

                        /* object actually changes from inlane to outlane -> set
                         * timer */
                        OBJ_GET_SI(iObj).ObjLaneAccStatus.In2OutlaneTransition =
                            SI_IN2OUTLANE_MAX_TRANSITIONTIME;
                    }
                    break;

                default:
                    break;
            }

            /* Get associated lane of object (without any quality
             * considerations) */
            eObjAssocLane = SIGetAssociateLane(iObj);

            /* Calculate associated lane used for OOI selection (taking quality
             * into account) */
            e_ObjFuncLane = SIGetObjSelectionLane(iObj, eObjAssocLane);
        } else {
            /* Deleted object : reset information */
            OBJ_GET_SI(iObj).ObjLaneAccStatus.fCorridorRelevantTime = 0.f;
            OBJ_GET_SI(iObj).ObjLaneAccStatus.SIInlaneState = OBJ_STATE_OUTLANE;
            OBJ_GET_SI(iObj).ObjLaneAccStatus.In2OutlaneTransition = 0u;

            /* Default init deleted objects lane to unknown */
            eObjAssocLane = ASSOC_LANE_UNKNOWN;

            /* Default init functional lane to unknown */
            e_ObjFuncLane = ASSOC_LANE_UNKNOWN;
        }

        /* Assign a lane for the external interface (without taking object
         * quality into account) */
        OBJ_GET_ASSOCIATED_LANE(iObj) = eObjAssocLane;

        /* Write the lane used for object selection (taking object quality into
         * account) to the external interface*/
        OBJ_GET_FUNC_LANE(iObj) = e_ObjFuncLane;
    }
}

/*************************************************************************************************************************
  Functionname:    SIGetAssociateLane */
static eAssociatedLane_t SIGetAssociateLane(const ObjNumber_t ObjId) {
    eAssociatedLane_t object_lane;
    float32 DistanceToTraj;
    float32 DistanceToTrajVar;

    /* Get distance to trajectory to decide if object is on left or right side
     * of lane */
    SITrajGetObjToRefDistance(ObjId, &DistanceToTraj, &DistanceToTrajVar);

    /* check if object is associate in lane */
    if (OBJ_GET_SI(ObjId).ObjLaneAccStatus.SIInlaneState == OBJ_STATE_INLANE) {
        object_lane = ASSOC_LANE_EGO;
    }
    /* check if object is on right side of lane */
    else if (DistanceToTraj < 0.0f) {
        object_lane = ASSOC_LANE_RIGHT;
        if ((OBJ_DYNAMIC_PROPERTY(ObjId) != CR_OBJECT_PROPERTY_STATIONARY) ||
            (OBJ_IS_MOVING_TO_STATIONARY(ObjId))) {
            if (DistanceToTraj <
                -(SI_SCALE_LANE_WIDTH_LANE_ASSOC * STRASSENBREITE)) {
                object_lane = ASSOC_LANE_FAR_RIGHT;
            }
        } else {
            if (DistanceToTraj < -(SI_SCALE_LANE_WIDTH_LANE_ASSOC *
                                   SI_STATIONARY_STRASSENBREITE)) {
                object_lane = ASSOC_LANE_FAR_RIGHT;
            }
        }
    }
    /* object is on left side */
    else {
        object_lane = ASSOC_LANE_LEFT;
        if ((OBJ_DYNAMIC_PROPERTY(ObjId) != CR_OBJECT_PROPERTY_STATIONARY) ||
            (OBJ_IS_MOVING_TO_STATIONARY(ObjId))) {
            if (DistanceToTraj >
                +(SI_SCALE_LANE_WIDTH_LANE_ASSOC * STRASSENBREITE)) {
                object_lane = ASSOC_LANE_FAR_LEFT;
            }
        } else {
            if (DistanceToTraj > +(SI_SCALE_LANE_WIDTH_LANE_ASSOC *
                                   SI_STATIONARY_STRASSENBREITE)) {
                object_lane = ASSOC_LANE_FAR_LEFT;
            }
        }
    }
    return (object_lane);
}

/*************************************************************************************************************************
  Functionname:    SIGetObjSelectionLane */
static eAssociatedLane_t SIGetObjSelectionLane(
    const ObjNumber_t ObjId, const eAssociatedLane_t eBaseAssocLane) {
    eAssociatedLane_t eRetLane = eBaseAssocLane;
    /* Check for sufficient ACC-quality and timegap-criterion. Objects, which do
       not pass
       this preselection, may not be assigned to a lane and will be assigned to
       ASSOC_LANE_UNKNOWN */
    const boolean b_ObjLaneQuality = SI_b_CheckObjLaneQuality(ObjId);
    /* Check, if the base associated lane (without quality considerations) is
       valid regarding the OOI-selection
       on adjacent lanes. Moving and stationary objects with invalid lane
       assignment or oncoming objects beyond
       the road border will be set to ASSOC_LANE_UNKNOWN and thus not be
       considered in OOI-selection. */
    const boolean b_ObjAdjLaneValidity =
        SI_b_CheckObjAdjLaneValidity(ObjId, eBaseAssocLane);
    /* Oncoming objects may only be set on host-lane in case of the special
       'relevant object rolling
       back' situation. If the extra function 'SI_b_CheckObjOncRollBack' returns
       FALSE, then the
       object may not be assigned to ego-lane and will be assigned to
       ASSOC_LANE_RIGHT or ASSOC_LANE_LEFT */
    const boolean b_ObjOncRollBack =
        SI_b_CheckObjOncRollBack(ObjId, eBaseAssocLane);

    /* If the configuration dependent extra function 'SICheckObjInlaneAllowed'
    returns
    FALSE, then that means the object is not allowed to be set ego-lane. */
    const boolean bInlaneAllowed = SICheckObjInlaneAllowed(ObjId);

    /* If object is not allowed to be inlane, but base assignment set it in-lane
    then change the default association for selection */
    if (((!bInlaneAllowed) && (eRetLane == ASSOC_LANE_EGO)) ||
        ((!b_ObjOncRollBack) && (eRetLane == ASSOC_LANE_EGO))) {
        /* Set object right/left depending on it's distance to trajectory */
        float32 DistanceToTraj;
        float32 DistanceToTrajVar;

        /* Get distance to trajectory to decide if object is on left or right
         * side  of lane */
        SITrajGetObjToRefDistance(ObjId, &DistanceToTraj, &DistanceToTrajVar);

        if (DistanceToTraj < 0.f) {
            eRetLane = ASSOC_LANE_RIGHT;
        } else {
            eRetLane = ASSOC_LANE_LEFT;
        }
    }
    /* Set lane to unknown, if the object does not pass preselection (including
       ACC-quality
       and timegap-criterion) OR if the adjacent lane is not valid regarding
       OOI-selection */
    if ((!b_ObjLaneQuality) || (!b_ObjAdjLaneValidity)) {
        eRetLane = ASSOC_LANE_UNKNOWN;
    }
    return eRetLane;
}

/*************************************************************************************************************************
  Functionname:    SICheckStateFlowOutlaneToInlane */
static boolean SICheckStateFlowOutlaneToInlane(
    const ObjNumber_t iObj, const CPTrajOccupancy_t* pOccupancy) {
    boolean bRet;
    boolean OccValue, CustomValue;
    boolean InlaneTimeValue, InlaneDistValue;
    boolean InlaneCamValue;

    OccValue = SICheckInlaneOccupancies(iObj, pOccupancy);
    CustomValue = SICheckCustomInlaneCriteria(iObj, pOccupancy);

    OBJ_GET_SI(iObj).Bool.InLOccValue = OccValue;
    OBJ_GET_SI(iObj).Bool.InLCustomValue = CustomValue;

    if (OccValue) {
        InlaneTimeValue = SICheckInlaneTimer(iObj);
        InlaneDistValue = SICheckInlaneDistance(iObj);
        InlaneCamValue = FALSE;
        OBJ_GET_SI(iObj).ObjLaneAccStatus.SIActLaneState = OBJ_STATE_INLANE;
    } else {
        InlaneTimeValue = FALSE;
        InlaneDistValue = FALSE;
        SIResetInlaneTimer(iObj);
        SIResetInlaneDistance(iObj);
        InlaneCamValue = FALSE;
        OBJ_GET_SI(iObj).ObjLaneAccStatus.SIActLaneState = OBJ_STATE_OUTLANE;
    }

    OBJ_GET_SI(iObj).Bool.InLTimeValue = InlaneTimeValue;

    /* State flow: object is inlane of occupancies satisfied together with at
    least one of in-lane timer
    in-lane distance or camera in-lane, or the custom in-lane criteria met
    (usually predicted pickup) */
    if ((OccValue && (InlaneTimeValue || InlaneDistValue || InlaneCamValue)) ||
        CustomValue) {
        bRet = TRUE;
    } else {
        bRet = FALSE;
    }

    if (OBJ_CR_SENSORSPECIFIC(iObj).ucMeasuredSources == CR_MEAS_SEN_NONE) {
        bRet = FALSE;
    }

    return bRet;
}

/*************************************************************************************************************************
  Functionname:    SICheckInlaneOccupancies */
static boolean SICheckInlaneOccupancies(const ObjNumber_t iObj,
                                        const CPTrajOccupancy_t* pOccupancy) {
    boolean bRet;
    boolean ObjOccValue, LaneOccValue;

    ObjOccValue = SICheckObjOccPickupValue(iObj, pOccupancy);
    LaneOccValue = SICheckLaneOccPickupValue(iObj, pOccupancy);

    OBJ_GET_SI(iObj).Bool.InLObjOccValue = ObjOccValue;
    OBJ_GET_SI(iObj).Bool.InLLaneOccValue = LaneOccValue;

    if (ObjOccValue || LaneOccValue) {
        bRet = TRUE;
    } else {
        bRet = FALSE;
    }
    return bRet;
}

/*************************************************************************************************************************
  Functionname:    SIGetObjOccPickupThreshold */
static float32 SIGetObjOccPickupThreshold(const ObjNumber_t iObj) {
    float32 PickupThresholdObj;

    /* Select thresholds due to dynamic property */
    if (((OBJ_DYNAMIC_PROPERTY(iObj) == CR_OBJECT_PROPERTY_STATIONARY) &&
         (!OBJ_IS_MOVING_TO_STATIONARY(iObj))) ||
        (OBJ_DYNAMIC_PROPERTY(iObj) == CR_OBJECT_PROPERTY_ONCOMING)) {
        boolean bUseConservativeThresh = FALSE;

        if (EGO_YAW_RATE_QUALITY_RAW <= SI_VDY_YAWRATE_QUAL_SENSOR_UNLEARNED) {
            bUseConservativeThresh = TRUE;
        }

        if (OBJ_GET_SI(iObj).ObjLaneAccStatus.SIActLaneState ==
            OBJ_STATE_INLANE) {
            if (bUseConservativeThresh) {
                PickupThresholdObj = ObjectOccupancyDropThreshStatConservative;
            } else {
                PickupThresholdObj = ObjectOccupancyDropThreshStat;
            }
        } else {
            if (bUseConservativeThresh) {
                PickupThresholdObj =
                    ObjectOccupancyPickupThreshStatConservative;
            } else {
                PickupThresholdObj = ObjectOccupancyPickupThreshStat;
            }
        }
    } else {
        /* Apply higher object occupancy threshold for out-lane objects ONLY
        when -
        1. Longitudinal distance is at least 40m
        2. Object is not stopped
        */
        if ((OBJ_GET_SI(iObj).ObjLaneAccStatus.SIInlaneState ==
             OBJ_STATE_OUTLANE) &&
            (OBJ_LONG_DISPLACEMENT(iObj) >
             SI_OBJ_OVLC_PICKUP_THRESH_OUTLANE_DISTX_MIN) &&
            (!OBJ_IS_MOVING_TO_STATIONARY(iObj))) {
            PickupThresholdObj = ObjectOccupancyPickupThreshOutlane;
        } else {
            PickupThresholdObj = ObjectOccupancyPickupThreshInlane;
        }
    }

    return (PickupThresholdObj);
}

/*************************************************************************************************************************
  Functionname:    SIGetCorrectedOccupancy */
static float32 SIGetCorrectedOccupancy(const ObjNumber_t iObj,
                                       const float32 occupancy,
                                       const float32 obj_occ_stddev) {
    float32 occ;
    /* Objects at high angles usually have very high standard deviations -
    leading to almost
    no pickups. Artificially decrease standard deviations influence in this case
    */
    const float32 obj_abs_angle = fABS(OBJ_ANGLE(iObj));

    /*! Remark: GDBLFunction_t structure with negative slope */
    static const GDBLFunction_t AngleStdDevFun = {
        SI_STDDEV_OBJ_OVLC_FAC_MAX, SI_STDDEV_OBJ_OVLC_FAC_MIN,
        ((SI_STDDEV_OBJ_OVLC_FAC_MAX - SI_STDDEV_OBJ_OVLC_FAC_MIN) /
         (SI_STDDEV_OBJ_OVLC_ANGLE_MAX - SI_STDDEV_OBJ_OVLC_ANGLE_MIN)),
        SI_STDDEV_OBJ_OVLC_FAC_MIN -
            (((SI_STDDEV_OBJ_OVLC_FAC_MAX - SI_STDDEV_OBJ_OVLC_FAC_MIN) /
              (SI_STDDEV_OBJ_OVLC_ANGLE_MAX - SI_STDDEV_OBJ_OVLC_ANGLE_MIN)) *
             SI_STDDEV_OBJ_OVLC_ANGLE_MIN)};
    float32 stddev_fac;
    if ((obj_abs_angle > SI_STDDEV_OBJ_OVLC_ANGLE_MAX)) {
        /* For high angles linearly interpolate
           This compensation for standard deviation is carried out only outside
           the orientations based occupancy region,
           since within this region no compensation is required.*/
        stddev_fac = dGDBmathLineareFunktion(&AngleStdDevFun, obj_abs_angle);
    } else {
        stddev_fac = 1.f;
    }
    occ = (occupancy - (stddev_fac * obj_occ_stddev));

    return occ;
}

/*************************************************************************************************************************
  Functionname:    SICheckObjOccPickupValue */
boolean SICheckObjOccPickupValue(const ObjNumber_t iObj,
                                 const CPTrajOccupancy_t* pOccupancy) {
    boolean result = FALSE;

    const float32 fObjOccThresh = SIGetObjOccPickupThreshold(iObj);
    const float32 fOccupancy = pOccupancy->fObjectOccupancy;
    // const float32 fOccupancy = SIGetCorrectedOccupancy(
    //     iObj, pOccupancy->fObjectOccupancy,
    //     SQRT_(pOccupancy->fObjectOccupancyVar)); /*!< Remark:
    //                                                 fObjectOccupancyVar >=
    //                                                 0*/

    if (fOccupancy > fObjOccThresh) {
        result = TRUE;
    }
    return (result);
}

/*************************************************************************************************************************
  Functionname:    SIGetLaneOccPickupThreshold */
static float32 SIGetLaneOccPickupThreshold(const ObjNumber_t iObj) {
    float32 PickupThresholdLane;

    /* Select thresholds due to dynamic property */
    if (((OBJ_DYNAMIC_PROPERTY(iObj) == CR_OBJECT_PROPERTY_STATIONARY) &&
         (!OBJ_IS_MOVING_TO_STATIONARY(iObj))) ||
        (OBJ_DYNAMIC_PROPERTY(iObj) == CR_OBJECT_PROPERTY_ONCOMING)) {
        boolean bUseConservativeThresh = FALSE;

        if (EGO_YAW_RATE_QUALITY_RAW <= SI_VDY_YAWRATE_QUAL_SENSOR_UNLEARNED) {
            bUseConservativeThresh = TRUE;
        }

        if (OBJ_GET_SI(iObj).ObjLaneAccStatus.SIActLaneState ==
            OBJ_STATE_INLANE) {
            if (bUseConservativeThresh) {
                PickupThresholdLane = LaneOccupancyDropThreshStatConservative;
            } else {
                PickupThresholdLane = LaneOccupancyDropThreshStat;
            }
        } else {
            if (bUseConservativeThresh) {
                PickupThresholdLane = LaneOccupancyPickupThreshStatConservative;
            } else {
                PickupThresholdLane = LaneOccupancyPickupThreshStat;
            }
        }
    } else {
        PickupThresholdLane = LaneOccupancyPickupThresh;
    }

    return (PickupThresholdLane);
}

/*************************************************************************************************************************
  Functionname:    SICheckLaneOccPickupValue */
static boolean SICheckLaneOccPickupValue(const ObjNumber_t iObj,
                                         const CPTrajOccupancy_t* pOccupancy) {
    boolean result = FALSE;

    const float32 fLaneOccThresh = SIGetLaneOccPickupThreshold(iObj);
    const float32 fOccupancy = pOccupancy->fTrajectoryOccupancy;
    // const float32 fOccupancy = SIGetCorrectedOccupancy(
    //     iObj, pOccupancy->fTrajectoryOccupancy,
    //     SQRT_(pOccupancy
    //               ->fTrajectoryOccupancyVar)); /*!< Remark:
    //                                               fTrajectoryOccupancyVar >=
    //                                               0
    //                                               */

    if (fOccupancy > fLaneOccThresh) {
        result = TRUE;
    }
    return (result);
}

/*************************************************************************************************************************
  Functionname:    SIGetSituationVrelInLaneTimeThreshold */
static fTime_t SIGetSituationVrelInLaneTimeThreshold(
    const ObjNumber_t iObj, fTime_t fTimeThreshInLane) {
    if ((OBJ_LONG_VREL(iObj) > SI_SIT_VREL_INLANE_TIME_INC_MIN_VREL) &&
        (OBJ_LONG_DISPLACEMENT(iObj) >
         SI_SIT_VREL_INLANE_TIME_INC_MIN_EGO_DISTX) &&
        (EGO_SPEED_X_OBJ_SYNC > SI_SIT_VREL_INLANE_TIME_INC_MIN_EGO_VEL) &&
        (OBJ_GET_SI(iObj).Bool.SelectedByPathDecision == FALSE) &&
        (OBJ_GET_SI(iObj).ObjLaneAccStatus.SIInlaneState != OBJ_STATE_INLANE)) {
        if (fABS(EGO_CURVE_OBJ_SYNC) > SI_SIT_VREL_INLANE_TIME_INC_MAX_CURVE) {
            if (OBJ_CLASSIFICATION(iObj) == CR_OBJCLASS_TRUCK) {
                fTimeThreshInLane = MAX_FLOAT(
                    fTimeThreshInLane + SI_SIT_VREL_INLANE_TIME_INC_MAX_INCR,
                    SI_SIT_VREL_INLANE_TIME_INC_MAX_TIME_TRUCKS);
            } else {
                fTimeThreshInLane = MAX_FLOAT(
                    fTimeThreshInLane + SI_SIT_VREL_INLANE_TIME_INC_MED_INCR,
                    SI_MINTIME_MOVE_OBJ_IN_LANE_MAX);
            }
        } else {
            if (fTimeThreshInLane < SI_MINTIME_MOVE_OBJ_IN_LANE_MAX) {
                if (OBJ_LONG_VREL(iObj) >
                    SI_SIT_VREL_INLANE_TIME_INC_MAX_VREL) {
                    fTimeThreshInLane =
                        MAX_FLOAT(fTimeThreshInLane +
                                      SI_SIT_VREL_INLANE_TIME_INC_MED_INCR,
                                  SI_MINTIME_MOVE_OBJ_IN_LANE_MAX);
                } else {
                    fTimeThreshInLane = MAX_FLOAT(
                        fTimeThreshInLane +
                            SI_SIT_VREL_INLANE_TIME_INC_MIN_INCR,
                        SI_SIT_VREL_INLANE_TIME_INC_MAX_TIME_STRAIGHT_CAR);
                }
            }
        }
    } else {
        /* nothing to do here */
    }

    return fTimeThreshInLane;
}

/*************************************************************************************************************************
  Functionname:    SIGetHighEgoVelInLaneTimeThreshold */
static fTime_t SIGetHighEgoVelInLaneTimeThreshold(const ObjNumber_t iObj,
                                                  fTime_t fTimeThreshInLane) {
    boolean b_ObjIsOvertaking = TRUE;

    ObjNumber_t
        a_OvertakenObjID[SI_FIND_MOV_OBJ_IN_AREA_NUM_MAX]; /* Array of objects
                                                              possibly overtaken
                                                              by iObj */
    const sint32 s_NumberLanesLeft = FIP_s_GetLMLeftNumLane();
    const sint32 s_NumberLanesRight = FIP_s_GetLMRightNumLane();

    /* Check velocity and distance thresholds for object suppression at high
     * velocities */
    if (((EGO_SPEED_X_OBJ_SYNC) > (SI_HIGH_VEGO_INLANE_TIME_VEGO_MIN)) &&
        ((EGO_SPEED_X_OBJ_SYNC + OBJ_LONG_VREL(iObj)) >
         SI_HIGH_VEGO_INLANE_TIME_VOBJ_MIN) &&
        (OBJ_LONG_DISPLACEMENT(iObj) > SI_HIGH_VEGO_INLANE_TIME_XDIST_MIN) &&
        (OBJ_LONG_VREL(iObj) < SI_HIGH_VEGO_INLANE_TIME_VREL_MAX) &&
        (OBJ_LONG_AREL(iObj) > SI_HIGH_DIST_OBJ_SUPPRESS_AREL_MIN)) {
        /* Check lane matrix and road border specific conditions. Ego has to be
         * most likely on left lane. */
        if ((s_NumberLanesRight != (sint8)0) &&
            ((s_NumberLanesLeft == (sint8)0))) {
            /* Loop over all objects to check if truck is overtaking */
            SIFindObjInArea(a_OvertakenObjID, iObj,
                            &SIOvertakingCheckArgsHighVego);

            /* Check if possible overtaken object has been found */
            if (a_OvertakenObjID[0] == (ObjNumber_t)-1) {
                /* No possibly overtaken object has been found */
                b_ObjIsOvertaking = FALSE;
            }
        }
    }

    /* If object is most likely not overtaking, increase situative in-lane-timer
     * to avoid drop-ins. */
    if (b_ObjIsOvertaking != TRUE) {
        if ((OBJ_CLASSIFICATION(iObj) == CR_OBJCLASS_TRUCK) ||
            (fABS(EGO_CURVE_RAW) > SI_HIGH_VEGO_INLANE_TIME_STRONG_CURVE)) {
            /* Very high in-lane time for trucks or in strong curves */
            fTimeThreshInLane =
                MAX_FLOAT(SI_HIGH_VEGO_INLANE_TIME_TRUCK, fTimeThreshInLane);
        } else if (OBJ_ATTRIBUTES(iObj).eObjectOcclusion ==
                   Envm_GEN_OBJECT_OCCL_FULL) {
            /* High in-lane time for fully occluded objects */
            fTimeThreshInLane =
                MAX_FLOAT(SI_HIGH_VEGO_INLANE_TIME_OCCLUDED, fTimeThreshInLane);
        } else if (OBJ_CLASSIFICATION(iObj) == CR_OBJCLASS_CAR) {
            /* Increased in-lane time for cars */
            fTimeThreshInLane =
                MAX_FLOAT(SI_HIGH_VEGO_INLANE_TIME_CAR, fTimeThreshInLane);
        } else {
            /* fTimeThreshInLane = fTimeThreshInLane */
        }
    }

    return fTimeThreshInLane;
}

/*************************************************************************************************************************
  Functionname:    SIGetInLaneTimeThreshold */
static fTime_t SIGetInLaneTimeThreshold(const ObjNumber_t iObj) {
    fTime_t fTimeThreshInLane;
    fVelocity_t fVeigen;

    fVeigen = EGO_SPEED_X_OBJ_SYNC;
    fVeigen = fABS(fVeigen);

    if ((OBJ_DYNAMIC_PROPERTY(iObj) != CR_OBJECT_PROPERTY_STATIONARY) ||
        (OBJ_IS_MOVING_TO_STATIONARY(iObj))) {
        fTimeThreshInLane = dGDBmathLineareFunktion(
            &SITimeThreshInLaneFromDist, OBJ_LONG_DISPLACEMENT(iObj));

        fTimeThreshInLane =
            SIGetHighEgoVelInLaneTimeThreshold(iObj, fTimeThreshInLane);
        fTimeThreshInLane =
            SIGetSituationVrelInLaneTimeThreshold(iObj, fTimeThreshInLane);

    } else /* Stationary Objects or vehicles */
    {
        if ((OBJ_CLASSIFICATION(iObj) == CR_OBJCLASS_CAR) ||
            (OBJ_CLASSIFICATION(iObj) == CR_OBJCLASS_TRUCK) ||
            (OBJ_CLASSIFICATION(iObj) == CR_OBJCLASS_MOTORCYCLE)) {
            fTimeThreshInLane = SICalcInlaneTimeStatVehicle(fVeigen, iObj);
        } else {
            fTimeThreshInLane = SICalcInlaneTimeStatNonVehicle(fVeigen, iObj);
        }

        /* Increase the Time in Lane as long as the sensor is unlearned */
        if ((EGO_YAW_RATE_QUALITY_RAW <=
             SI_VDY_YAWRATE_QUAL_SENSOR_UNLEARNED) &&
            (OBJ_LONG_DISPLACEMENT(iObj) > SI_SAFE_DIST_UNLEARNED)) {
            fTimeThreshInLane =
                fTimeThreshInLane * SI_UNLEARNED_SENSOR_INLANE_TIME_RISE_FACTOR;
        }
    }

    return (fTimeThreshInLane);
}

/*************************************************************************************************************************
  Functionname:    SICalcInlaneTimeStatVehicle */
static fTime_t SICalcInlaneTimeStatVehicle(fVelocity_t const fVelocity,
                                           const ObjNumber_t iObj) {
    fTime_t fTimeThreshold;
    fDistance_t fRequiredDistanceInLane;

    _PARAM_UNUSED(iObj);

    if ((fVelocity < SI_STATVEH_INLANE_NOSPEED) ||
        ((EGO_MOTION_STATE_RAW == VED_LONG_MOT_STATE_STDST) &&
         (IS_SIGNAL_STATUS_OK(EGO_MOTION_STATE_STATE)))) {
        fTimeThreshold = SI_STATVEH_INLANE_TIME_NOSPEED;
    } else if (fVelocity < SI_STATVEH_INLANE_PARKSPEED) {
        fTimeThreshold = SI_STATVEH_INLANE_TIME_PARKSPEED;
    } else if (fVelocity < SI_STATVEH_INLANE_LOWSPEED) {
        fTimeThreshold = SI_STATVEH_INLANE_DISTMIN / fVelocity;
    } else if (fVelocity <= SI_STATVEH_INLANE_HIGHSPEED) {
        fRequiredDistanceInLane =
            MUL_ADD_FLOAT(SI_STATVEH_INLANE_DISTSLOPE, fVelocity,
                          SI_STATVEH_INLANE_DISTOFFSET);
        fTimeThreshold = fRequiredDistanceInLane / fVelocity;
    } else {
        fTimeThreshold = SI_STATVEH_INLANE_DISTMAX / fVelocity;
    }

    return fTimeThreshold;
}

/*************************************************************************************************************************
  Functionname:    SICalcInlaneTimeStatNonVehicle */
static fTime_t SICalcInlaneTimeStatNonVehicle(fVelocity_t const fVelocity,
                                              const ObjNumber_t iObj) {
    fTime_t fTimeThreshold;
    fDistance_t fRequiredDistanceInLane;

    _PARAM_UNUSED(iObj);

    if ((fVelocity < SI_STATOBJ_INLANE_NOSPEED) ||
        ((EGO_MOTION_STATE_RAW == VED_LONG_MOT_STATE_STDST) &&
         (IS_SIGNAL_STATUS_OK(EGO_MOTION_STATE_STATE)))) {
        fTimeThreshold = SI_STATOBJ_INLANE_TIME_NOSPEED;
    } else if (fVelocity < SI_STATOBJ_INLANE_PARKSPEED) {
        fTimeThreshold = SI_STATOBJ_INLANE_TIME_PARKSPEED;
    } else if (fVelocity < SI_STATOBJ_INLANE_LOWSPEED) {
        fTimeThreshold = SI_STATOBJ_INLANE_DISTMIN / fVelocity;
    } else if (fVelocity <= SI_STATOBJ_INLANE_HIGHSPEED) {
        fRequiredDistanceInLane =
            MUL_ADD_FLOAT(SI_STATOBJ_INLANE_DISTSLOPE, fVelocity,
                          SI_STATOBJ_INLANE_DISTOFFSET);
        fTimeThreshold = fRequiredDistanceInLane / fVelocity;
    } else {
        fTimeThreshold = SI_STATOBJ_INLANE_DISTMAX / fVelocity;
    }

    return (fTimeThreshold);
}

/*************************************************************************************************************************
  Functionname:    SICheckInlaneTimer */
static boolean SICheckInlaneTimer(const ObjNumber_t iObj) {
    boolean result = FALSE;

    const fTime_t fTimeThres = SIGetInLaneTimeThreshold(iObj);
    if (OBJ_GET_SI(iObj).ObjLaneAccStatus.fCorridorRelevantTime > fTimeThres) {
        if ((OBJ_DYNAMIC_PROPERTY(iObj) != CR_OBJECT_PROPERTY_STATIONARY) ||
            (OBJ_IS_MOVING_TO_STATIONARY(iObj))) {
            OBJ_GET_SI(iObj).ObjLaneAccStatus.fCorridorRelevantTime =
                SI_MAX_CORR_REL_TIME_NONSTAT;
        }
        result = TRUE;
    } else {
        OBJ_GET_SI(iObj).ObjLaneAccStatus.fCorridorRelevantTime +=
            SI_CYCLE_TIME;
    }
    return (result);
}

/*************************************************************************************************************************
  Functionname:    SIResetInlaneTimer */
static void SIResetInlaneTimer(const ObjNumber_t iObj) {
    if ((OBJ_DYNAMIC_PROPERTY(iObj) == CR_OBJECT_PROPERTY_STATIONARY) &&
        (!OBJ_IS_MOVING_TO_STATIONARY(iObj))) {
        OBJ_GET_SI(iObj).ObjLaneAccStatus.fCorridorRelevantTime =
            OBJ_GET_SI(iObj).ObjLaneAccStatus.fCorridorRelevantTime -
            (SI_SCALE_RESET_INLANE_TIMER_STAT * SI_CYCLE_TIME);
        OBJ_GET_SI(iObj).ObjLaneAccStatus.fCorridorRelevantTime = MAX_FLOAT(
            OBJ_GET_SI(iObj).ObjLaneAccStatus.fCorridorRelevantTime, 0.f);
    } else {
        OBJ_GET_SI(iObj).ObjLaneAccStatus.fCorridorRelevantTime = 0.f;
    }
}

/*************************************************************************************************************************
  Functionname:    SIGetInLaneDistanceThreshold */
static fDistance_t SIGetInLaneDistanceThreshold(const ObjNumber_t iObj) {
    fDistance_t fDistThreshInLane;
    fVelocity_t fVeigen;

    fVeigen = EGO_SPEED_X_OBJ_SYNC;
    fVeigen = fABS(fVeigen);

    if ((OBJ_DYNAMIC_PROPERTY(iObj) != CR_OBJECT_PROPERTY_STATIONARY) ||
        (OBJ_IS_MOVING_TO_STATIONARY(iObj))) {
        /* only use the time in lane for moving and stopped objects ->
         * deactivate the distance in lane */
        fDistThreshInLane = (SI_INLANE_DIST_MAXVALUE + C_F32_DELTA);
    } else /* Stationary Objects or vehicles */
    {
        if ((OBJ_CLASSIFICATION(iObj) == CR_OBJCLASS_CAR) ||
            (OBJ_CLASSIFICATION(iObj) == CR_OBJCLASS_TRUCK) ||
            (OBJ_CLASSIFICATION(iObj) == CR_OBJCLASS_MOTORCYCLE)) {
            if (fVeigen < SI_STATVEH_INLANE_PARKSPEED) {
                fDistThreshInLane =
                    MAX_FLOAT(SI_STATVEH_INLANE_DISTMIN,
                              (fVeigen * SI_STATVEH_INLANE_TIME_PARKSPEED));
            } else if (fVeigen < SI_STATVEH_INLANE_LOWSPEED) {
                fDistThreshInLane = SI_STATVEH_INLANE_DISTMIN;
            } else if (fVeigen <= SI_STATVEH_INLANE_HIGHSPEED) {
                /* only use the time in lane -> deactivate the distance in lane
                 */
                fDistThreshInLane = (SI_INLANE_DIST_MAXVALUE + C_F32_DELTA);
            } else {
                fDistThreshInLane = SI_STATVEH_INLANE_DISTMAX;
            }
        } else {
            if (fVeigen < SI_STATOBJ_INLANE_PARKSPEED) {
                fDistThreshInLane =
                    MAX_FLOAT(SI_STATOBJ_INLANE_DISTMIN,
                              (fVeigen * SI_STATOBJ_INLANE_TIME_PARKSPEED));
            } else if (fVeigen < SI_STATOBJ_INLANE_LOWSPEED) {
                fDistThreshInLane = SI_STATOBJ_INLANE_DISTMIN;
            } else if (fVeigen <= SI_STATOBJ_INLANE_HIGHSPEED) {
                /* only use the time in lane -> deactivate the distance in lane
                 */
                fDistThreshInLane = (SI_INLANE_DIST_MAXVALUE + C_F32_DELTA);
            } else {
                fDistThreshInLane = SI_STATOBJ_INLANE_DISTMAX;
            }
        }
    }

    return (fDistThreshInLane);
}

/*************************************************************************************************************************
  Functionname:    SICheckInlaneDistance */
static boolean SICheckInlaneDistance(const ObjNumber_t iObj) {
    boolean bInlaneDist;
    const fAccel_t fEgoAcceleration = EGO_ACCEL_X_OBJ_SYNC;
    const fTime_t fLastCycleTime = SI_CYCLE_TIME;
    const fDistance_t fDistThresh = SIGetInLaneDistanceThreshold(iObj);

    OBJ_GET_SI(iObj).ObjLaneAccStatus.fCorridorRelevantDist +=
        ((EGO_SPEED_X_OBJ_SYNC * fLastCycleTime) +
         (0.5f * fEgoAcceleration * fLastCycleTime * fLastCycleTime));

    OBJ_GET_SI(iObj).ObjLaneAccStatus.fCorridorRelevantDist =
        MIN_FLOAT(SI_INLANE_DIST_MAXVALUE,
                  OBJ_GET_SI(iObj).ObjLaneAccStatus.fCorridorRelevantDist);

    if (OBJ_GET_SI(iObj).ObjLaneAccStatus.fCorridorRelevantDist > fDistThresh) {
        bInlaneDist = TRUE;
    } else {
        bInlaneDist = FALSE;
    }
    return bInlaneDist;
}

/*************************************************************************************************************************
  Functionname:    SIResetInlaneDistance */
static void SIResetInlaneDistance(const ObjNumber_t iObj) {
    /* Only delete the distance in lane when the time in lane is 0 */
    if (OBJ_GET_SI(iObj).ObjLaneAccStatus.fCorridorRelevantTime < C_F32_DELTA) {
        OBJ_GET_SI(iObj).ObjLaneAccStatus.fCorridorRelevantDist = 0.0f;
    }
}

/*************************************************************************************************************************
  Functionname:    SICheckStateFlowInlaneToOutlane */
static boolean SICheckStateFlowInlaneToOutlane(
    const ObjNumber_t iObj, const CPTrajOccupancy_t* pOccupancy) {
    boolean OutOccValue, OutCustomValue;
    boolean InOccValue, InCustomValue;
    boolean bResult;

    OutOccValue = SICheckOutlaneOccupancies(iObj, pOccupancy);
    OutCustomValue = SICheckCustomOutlaneCriteria(iObj, pOccupancy);

    OBJ_GET_SI(iObj).Bool.OutLOccValue = OutOccValue;
    OBJ_GET_SI(iObj).Bool.OutLCustomValue = OutCustomValue;

    /* Update timer and distance information if out-lane occupancies set */
    if (OutOccValue) {
        SIResetInlaneTimer(iObj);
        SIResetInlaneDistance(iObj);
    }

    if (OutOccValue || OutCustomValue) {
        /* At least one of the out-lane criteria satisfied : object may be
        making transition
        from in-lane to out-lane state. Next we need to check in-lane criteria,
        since if those
        aren't met, then the transition is OK */
        InOccValue = SICheckInlaneOccupancies(iObj, pOccupancy);
        InCustomValue = SICheckCustomInlaneCriteria(iObj, pOccupancy);

        OBJ_GET_SI(iObj).Bool.InLOccValue = InOccValue;
        OBJ_GET_SI(iObj).Bool.InLCustomValue = InCustomValue;

        if (InOccValue || InCustomValue) {
            /* Since in-lane criteria satisfied as well, these overrule the
             * out-lane criteria */
            bResult = FALSE;
        } else {
            /* In-lane criteria not satisfied : object really making transition
             * from in-lane to out-lane */
            bResult = TRUE;
        }
    } else {
        /* Neither out-lane criteria satisfied : return FALSE (no state
         * transition) */
        bResult = FALSE;
    }

    return (bResult);
}

/*************************************************************************************************************************
  Functionname:    SICheckOutlaneOccupancies */
static boolean SICheckOutlaneOccupancies(const ObjNumber_t iObj,
                                         const CPTrajOccupancy_t* pOccupancy) {
    boolean ObjOccValue, LaneOccValue;
    boolean result = FALSE;

    ObjOccValue = SICheckObjOccDropValue(iObj, pOccupancy);
    LaneOccValue = SICheckLaneOccDropValue(iObj, pOccupancy);

    OBJ_GET_SI(iObj).Bool.OutLObjOccValue = ObjOccValue;
    OBJ_GET_SI(iObj).Bool.OutLLaneOccValue = LaneOccValue;

    if (ObjOccValue && LaneOccValue) {
        result = TRUE;
    }

    return (result);
}

/*************************************************************************************************************************
  Functionname:    SIGetObjOccDropThreshold */
static float32 SIGetObjOccDropThreshold(const ObjNumber_t iObj) {
    float32 DropThresholdObj;

    /* Select thresholds due to dynamic property */
    if (((OBJ_DYNAMIC_PROPERTY(iObj) == CR_OBJECT_PROPERTY_STATIONARY) &&
         (!OBJ_IS_MOVING_TO_STATIONARY(iObj))) ||
        (OBJ_DYNAMIC_PROPERTY(iObj) == CR_OBJECT_PROPERTY_ONCOMING)) {
        DropThresholdObj = ObjectOccupancyDropThreshStat;
    } else {
        if (OBJ_GET_SI(iObj).ObjLaneAccStatus.fCorridorRelevantTime <
            SI_MAX_REL_TIMER_DROP_OVLC_THRES) {
            DropThresholdObj =
                ((ObjectOccupancyDropThresh - SI_SUM_NONSTAT_DROP_OVLC_THRES) *
                 OBJ_GET_SI(iObj).ObjLaneAccStatus.fCorridorRelevantTime) +
                SI_SUM_NONSTAT_DROP_OVLC_THRES;
            DropThresholdObj =
                MIN_FLOAT(ObjectOccupancyDropThresh, DropThresholdObj);
            OBJ_GET_SI(iObj).ObjLaneAccStatus.fCorridorRelevantTime +=
                SI_CYCLE_TIME;
        } else {
            DropThresholdObj = ObjectOccupancyDropThresh;
        }
    }

    return (DropThresholdObj);
}

/*************************************************************************************************************************
  Functionname:    SICheckObjOccDropValue */
static boolean SICheckObjOccDropValue(const ObjNumber_t iObj,
                                      const CPTrajOccupancy_t* pOccupancy) {
    boolean result = FALSE;

    const float32 fOverlapTreshold = ObjectOccupancyDropMaxOverlapVariance;
    const float32 fObjOccDropThresh = SIGetObjOccDropThreshold(iObj);

    if (pOccupancy->fObjectOccupancy < fObjOccDropThresh) {
        if (pOccupancy->fOverlapVar < fOverlapTreshold) {
            result = TRUE;
        } else {
            if (pOccupancy->fObjectOccupancy <=
                ObjectOccupancyDropThreshInsecureOverlap) {
                result = TRUE;
            }
        }
    }
    return (result);
}

/*************************************************************************************************************************
  Functionname:    SIGetLaneOccDropThreshold */
static float32 SIGetLaneOccDropThreshold(const ObjNumber_t iObj) {
    float32 DropThresholdLane;

    /* Select thresholds due to dynamic property */
    if (((OBJ_DYNAMIC_PROPERTY(iObj) == CR_OBJECT_PROPERTY_STATIONARY) &&
         (!OBJ_IS_MOVING_TO_STATIONARY(iObj))) ||
        (OBJ_DYNAMIC_PROPERTY(iObj) == CR_OBJECT_PROPERTY_ONCOMING)) {
        DropThresholdLane = LaneOccupancyDropThreshStat;
    } else {
        DropThresholdLane = LaneOccupancyDropThresh;
    }

    return (DropThresholdLane);
}

/*************************************************************************************************************************
  Functionname:    SICheckLaneOccDropValue */
static boolean SICheckLaneOccDropValue(const ObjNumber_t iObj,
                                       const CPTrajOccupancy_t* pOccupancy) {
    boolean result = FALSE;

    const float32 fLaneOccDropThresh = SIGetLaneOccDropThreshold(iObj);

    if (pOccupancy->fTrajectoryOccupancy < fLaneOccDropThresh) {
        result = TRUE;
    }
    return (result);
}

/*************************************************************************************************************************
  Functionname:    SICheckObjInlaneAllowed */
static boolean SICheckObjInlaneAllowed(const ObjNumber_t iObj) {
    boolean bRet = TRUE;

    /* Extra check currently done for stationary objects:
      1. These have to have an ACC object quality more than
        FUN_PRESEL_AVLC_MIN_INLANE_OBJ_QUAL to be selectable
        for ego-lane.
      2. Stopped objects that have not been seen moving may only become
        ego lane if they are either below a given distance or were only
        seen oncoming for a short period of time */
    if (OBJ_DYNAMIC_PROPERTY(iObj) == CR_OBJECT_PROPERTY_STATIONARY) {
        if (OBJ_IS_MOVING_TO_STATIONARY(iObj)) {
            if ((OBJ_GET_SI(iObj).Bool.Moving == 0u) &&
                (OBJ_GET_SI(iObj).Bool.Oncoming != 0u)) {
                /* First determine maximum X distance up to which object can
                become in-lane.
                Set default maximum distance for oncoming stopped object inlane.
                */
                float32 fMaxDistX = SI_PAR_STOPPED_ONCOMING_INLANE_DISTX_MAX;

                /* If stationary object was seen oncoming for
                SI_STATOBJ_ONCOMINGCOUNTER_MIN cycles and
                is over a given distance (with hysteresis), then do not allow
                the object to become in-lane */
                if (OBJ_GET_SI(iObj).StatObjWasOncoming.uiOncomingCounter >
                    SI_STATOBJ_ONCOMINGCOUNTER_MIN) {
                    if (OBJ_GET_SI(iObj).Bool.StatObjWasOncoming) {
                        fMaxDistX = (SI_PAR_ONCOMMING_KEEP_RELEV_DISTX_MAX -
                                     SI_STATOBJ_ONCOMING_XDIST_HYSTERESIS);
                    } else {
                        fMaxDistX = SI_PAR_ONCOMMING_KEEP_RELEV_DISTX_MAX;
                    }
                }
                /* If stopped object only seen as oncoming is over given
                distance then do
                not allow it to go into ego-lane */
                if (OBJ_LONG_DISPLACEMENT(iObj) > fMaxDistX) {
                    OBJ_GET_SI(iObj).Bool.StatObjWasOncoming = (boolean)TRUE;
                    bRet = FALSE;
                } else {
                    OBJ_GET_SI(iObj).Bool.StatObjWasOncoming = (boolean)FALSE;
                }
            }
        } else {
            /* Stationary objects need to be of at least
            FUN_PRESEL_AVLC_MIN_INLANE_OBJ_QUAL
            quality to be selectable for ego-lane. Note that Objects of less
            than
            FUN_PRESEL_AVLC_MIN_INLANE_OBJ_QUAL quality are also handeled in
            function
            SI_b_CheckObjLaneQuality */
            if ((OBJ_GET_AVLC_FUN_PRESEL_QUALITY(iObj) <=
                 FUN_PRESEL_AVLC_MIN_OBJ_QUAL) &&
                (OBJ_DYNAMIC_SUB_PROPERTY(iObj) !=
                 CR_OBJECT_SUBPROP_CROSSING)) {
                bRet = FALSE;
            }
        }
    }

    OBJ_GET_SI(iObj).Bool.InLQualityValue = bRet;

    return bRet;
}

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */