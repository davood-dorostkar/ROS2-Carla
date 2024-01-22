
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "stddef.h"
#include "TM_Global_Types.h"
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
#define SI_PASSIVE_OBJOFFSET (999.9f)

/* defines for detecting an unlikely cutin situation */
#define CUTIN_RADIUS_BLINDSPOTAREA (3.0f)
#define CUTIN_MIN_TTC (1.6f)
#define CUTIN_MIN_TG (0.16f)
#define SI_CUTIN_MAX_TTC (999.f)
#define SI_CUTIN_MAX_TG (999.f)
#define SI_CUTIN_INIT_BLINDSPOTOFFSET (99.0f)
#define SI_CUTIN_MAX_LONG_DIST_BLINDSPOT (5.f)

/* defines for sppressing LongPot in case no lateral velocity observed */
/* for prevention of toggling, do not suppress potential if in last cycle
 * potential was already higher than x % */
#define SI_CUTIN_MINPOT_HYTSTERESE (50u)
/* minimal lateral velocity for calculating LongPotential */
#define SI_CUTIN_MIN_LAT_VEL (0.25f)

#define SI_CUTIN_NUM_OF_POINTS_FOR_NO_CUTIN (6)
#define SI_CUTIN_OBJECT_ORIENTATION_THRESHOLD (0.0175f)

/* restrictions on truck objects with not plausible high lateral dynamics */
#define TRUCK_MIN_NOT_PLAUSIBLE_VELTOTRAJ (1.25f)
#define TRUCK_MAX_NOT_PLAUSIBLE_VELTOTRAJ (1.5f)

/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
SET_MEMSEC_VAR(SICutInObjData)

/*! @vaddr: SI_CUTINOBJ_DATA_MEAS_VADDR @cycleid: VLC_ENV */
SICutInObjData_t SICutInObjData[Envm_N_OBJECTS];
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"

SET_MEMSEC_CONST(SICutInObjData_MeasInfo)
static const MEASInfo_t SICutInObjData_MeasInfo =

    {
        SI_CUTINOBJ_DATA_MEAS_VADDR, /* .VirtualAddress */
        sizeof(SICutInObjData),      /* .Length */
        SI_MEAS_FUNC_ID,             /* .FuncID */
        SI_MEAS_FUNC_CHAN_ID         /* .FuncChannelID */
};

/*****************************************************************************
  FUNCTION
*****************************************************************************/
static uint8 SICalcCutInFromLateralKinematics(ObjNumber_t ObjId);

static boolean SICheckCutInPreconditions(const ObjNumber_t ObjId);
static float32 SIGetApplicablePredTime(const ObjNumber_t ObjId,
                                       const float32 RelVelTrajVar,
                                       const float32 DistToTraj,
                                       const float32 RelVelToTraj);
static float32 SICalcPredPosNeighborObj(ObjNumber_t ObjId);

static float32 SICutInCalcTTC(const ObjNumber_t ObjId);
static float32 SICutInCalcTG(const ObjNumber_t ObjId);
static boolean SICutInCheckLatPos(const ObjNumber_t ObjId);

/*************************************************************************************************************************
  Functionname:    SICheckCutInPreconditions */
static boolean SICheckCutInPreconditions(const ObjNumber_t ObjId) {
    const Envm_t_GenObjKinEnvmatics* const pKin = &OBJ_KINEMATIC(ObjId);
    const float32 ego_speed = EGO_SPEED_X_OBJ_SYNC;
    boolean retval;

    /* Only have a prediction time for moving objects & those that are ahead
    of us */
    if ((OBJ_DYNAMIC_PROPERTY(ObjId) == CR_OBJECT_PROPERTY_MOVING) &&
        (pKin->fDistX > 0.0f)) {
        /* If object's current X position won't be reached by the ego vehicle
        within the next 'SiMaxDistPredictionTime' time, then do not calculate
        cut-in (as would be too unsure anyway) */
        if (pKin->fDistX < (ego_speed * SiMaxDistPredictionTime)) {
            /* Object's current position could be reached by ego within time
             * limit */
            retval = TRUE;
        } else {
            /* Object is simply too far to predict anything reliably */
            retval = FALSE;
        }
    } else {
        /* Object not moving or already behind us: it makes no sense to
        calculate a cut-in potential in this case */
        retval = FALSE;
    }
    return retval;
}

/*************************************************************************************************************************
  Functionname:    SIGetApplicablePredTime */
static float32 SIGetApplicablePredTime(const ObjNumber_t ObjId,
                                       const float32 RelVelTrajVar,
                                       const float32 DistToTraj,
                                       const float32 RelVelToTraj) {
    const Envm_t_GenObjKinEnvmatics* const pKin = &OBJ_KINEMATIC(ObjId);
    const float32 ego_speed = EGO_SPEED_X_OBJ_SYNC;
    float32 pred_time;
    float32 temp;

    if (pKin->fDistX >= 0.f) {
        /* Set default (maximal) prediction time */
        pred_time = SI_MAX_CUT_IN_PRED_TIME;

        /* If ego speed is useable as divisor use it to clamp prediction time
        into the time frame needed to reach the point where the object is now */
        if (ego_speed > C_F32_DELTA) {
            temp = pKin->fDistX / ego_speed;
            /* Use time scaling factor */
            temp *= SI_PRED_TIME_TEST_FACTOR;
            /* Take smaller of two values */
            pred_time = MIN_FLOAT(temp, pred_time);
        }

        /* in case of not plausible high lateral movement towards course of
         * trucks, limit */
        /* prediction time down to zero. Trucks are unlikely to have a "real"
         * high lateral velocity, */
        /* these are often caused my merging with mirror or ghost objects. */
        /* Downside: high dynamic ego lane changes behind trucks, high dynamic
         * cutins of pickups/vans (rarely trucks) */
        if (OBJ_CLASSIFICATION(ObjId) == CR_OBJCLASS_TRUCK) {
            if (DistToTraj <= 0.f) {
                if (RelVelToTraj > TRUCK_MIN_NOT_PLAUSIBLE_VELTOTRAJ) {
                    temp = (TRUCK_MAX_NOT_PLAUSIBLE_VELTOTRAJ - RelVelToTraj) /
                           (TRUCK_MAX_NOT_PLAUSIBLE_VELTOTRAJ -
                            TRUCK_MIN_NOT_PLAUSIBLE_VELTOTRAJ);
                    temp = MINMAX_FLOAT(0.0F, 1.0F, temp);
                    pred_time *= temp;
                }
            } else {
                if (RelVelToTraj < (-TRUCK_MIN_NOT_PLAUSIBLE_VELTOTRAJ)) {
                    temp = (TRUCK_MAX_NOT_PLAUSIBLE_VELTOTRAJ -
                            fABS(RelVelToTraj)) /
                           (TRUCK_MAX_NOT_PLAUSIBLE_VELTOTRAJ -
                            TRUCK_MIN_NOT_PLAUSIBLE_VELTOTRAJ);
                    temp = MINMAX_FLOAT(0.0F, 1.0F, temp);
                    pred_time *= temp;
                }
            }
        }

        /* Use the relative velocity to trajectory's variance to limit
         * prediction time */
        if (RelVelTrajVar < SI_REL_VEL_TRAJ_VAR_MAX) {
            /* Linear function */
            temp = (SI_REL_VEL_TRAJ_VAR_MAX - RelVelTrajVar) /
                   (SI_REL_VEL_TRAJ_VAR_MAX);
            pred_time = MIN_FLOAT(temp, pred_time);
            /* truck prediction time shall not be limited to 0.25 */
            if (OBJ_CLASSIFICATION(ObjId) != CR_OBJCLASS_TRUCK) {
                pred_time = MAX_FLOAT(pred_time, SI_MIN_CUT_IN_PRED_TIME);
            }
        } else {
            /* truck prediction time shall not be limited to 0.25 */
            if (OBJ_CLASSIFICATION(ObjId) != CR_OBJCLASS_TRUCK) {
                /* Variance of speed to course too large, we don't really know
                 * the speed */
                pred_time = SI_MIN_CUT_IN_PRED_TIME;
            } else {
                /*! Remark: For trucks the value shall NOT exceed
                 * SI_MIN_CUT_IN_PRED_TIME */
                pred_time = MIN_FLOAT(SI_MIN_CUT_IN_PRED_TIME, pred_time);
            }
        }
    } else {
        pred_time = 0.f;
    }

    return pred_time;
}

/*************************************************************************************************************************
  Functionname:    SICalcPredPosNeighborObj */
static float32 SICalcPredPosNeighborObj(ObjNumber_t ObjId) {
    float32 pred_time;
    float32 dist_to_traj;
    float32 dist_to_traj_var;
    float32 rel_vel_to_traj;
    float32 rel_vel_to_traj_var;
    float32 corr_dist_to_traj;
    float32 corr_rel_vel_to_traj;
    float32 fn_val;

    /* Check if cut-in probability pre-conditions are satisifed */
    if (SICheckCutInPreconditions(ObjId)) {
        /* Get actual distance to course and the gradient (Vrel to course) */
        SITrajGetObjToRefDistance(ObjId, &dist_to_traj, &dist_to_traj_var);
        SITrajGetObjToRefDistanceGradient(ObjId, &rel_vel_to_traj,
                                          &rel_vel_to_traj_var);

        /* Calculate variance corrected distance / velocity to trajectory */
        if (dist_to_traj >= 0.f) {
            corr_dist_to_traj =
                dist_to_traj +
                SQRT_(dist_to_traj_var); /*!< Remark: dist_to_traj_var >= 0 */
            corr_rel_vel_to_traj =
                rel_vel_to_traj +
                ((SI_VARIANCE_SCALE_FACTOR)*SQRT_(
                    rel_vel_to_traj_var)); /*!< Remark: rel_vel_to_traj_var >= 0
                                            */
        } else {
            corr_dist_to_traj =
                dist_to_traj -
                SQRT_(dist_to_traj_var); /*!< Remark: dist_to_traj_var >= 0 */
            corr_rel_vel_to_traj =
                rel_vel_to_traj +
                ((-SI_VARIANCE_SCALE_FACTOR) *
                 SQRT_(rel_vel_to_traj_var)); /*!< Remark: rel_vel_to_traj_var
                                                 >= 0 */
        }

        /* If the variance correction value is too large (i.e.: the sign of
        corr_rel_vel_to_traj and rel_vel_to_traj is different, then set
        corrected relative velocity to zero (basically no
        effect, as we are not sure enough about the relative speed to the
        trajectory */
        if ((corr_rel_vel_to_traj * rel_vel_to_traj) < 0.F) {
            corr_rel_vel_to_traj = 0.F;
        }

        /* Calculate prediction time */
        pred_time =
            SIGetApplicablePredTime(ObjId, rel_vel_to_traj_var,
                                    corr_dist_to_traj, corr_rel_vel_to_traj);
        /* obj didn't just changed lanes from ego to neighbour */

        /* Calculate approximated function value */
        fn_val = corr_dist_to_traj + (corr_rel_vel_to_traj * pred_time);

        /* Would the object cross our trajectory? If yes, set it's predicted
         * course distance to zero */
        if ((fn_val * corr_dist_to_traj) < 0.F) {
            fn_val = 0.F;
        } else {
            fn_val = fABS(fn_val);
        }

    } else {
        /* No prediction done: use passive object offset */
        fn_val = SI_PASSIVE_OBJOFFSET;
    }
    return fn_val;
}

/*************************************************************************************************************************
  Functionname:    SICalcCutInFromLateralKinematics */
static uint8 SICalcCutInFromLateralKinematics(ObjNumber_t ObjId) {
    const float32 ego_speed = EGO_SPEED_X_OBJ_SYNC;
    const Envm_t_GenObjKinEnvmatics* const pKin = &OBJ_KINEMATIC(ObjId);
    float32 min_traj_dist;
    float32 max_traj_dist;
    float32 fn_val;
    float32 cut_in_pot;

    /* Calculate the predicted lateral offset of the object */
    fn_val = SICalcPredPosNeighborObj(ObjId);

    /* On transition from far range to near range sometimes vehicles are
    pulled in. */
    if ((pKin->fDistX > SI_NEAR_FAR_LIMIT_MIN_DIST) &&
        (pKin->fDistX < SI_NEAR_FAR_LIMIT_MAX_DIST) &&
        OBJ_GET_FOV_OVERLAP_FAR(ObjId)) {
        fn_val +=
            SI_REFLECTION_SHIFT_WIDTH_FACTOR * SIGetObjWidthForCorridor(ObjId);
    }

    /* Store the predicted distance to course */
    OBJ_GET_SI(ObjId).fPredictedLatDispl = fn_val;

    /* Minimal necessary trajectory distance for cut-in calculation */
    min_traj_dist =
        ((SI_FAHRZEUGBREITE + SIGetObjWidthForCorridor(ObjId)) * 0.5f) +
        SI_CUT_IN_MAX_POT_OFFSET;

    /* Push trucks out of our track */
    if (OBJ_CLASSIFICATION(ObjId) == CR_OBJCLASS_TRUCK) {
        min_traj_dist -= SI_CUT_IN_MAX_POT_OFFSET;
    }

    /* Maximal necessary trajectory distance for cut-in calculation */
    max_traj_dist =
        (min_traj_dist + (SI_CUT_IN_MIN_POT_OFFSET - SI_CUT_IN_MAX_POT_OFFSET));

    /* Since the farther away the object is, the more unsure we are about our
    course, modulate the min and max ramp positions with the object distance */

    if (ego_speed > C_F32_DELTA) {
        const float32 pos_reach_time = (pKin->fDistX / ego_speed);
        float32 scale_factor =
            SI_MIN_CUT_IN_POT_SCALE_FACTOR +
            ((1.F - ((pos_reach_time - SI_DIST_RAMP_FULL_PRED_TIME) /
                     (SiMaxDistPredictionTime - SI_DIST_RAMP_FULL_PRED_TIME))) *
             (SI_MAX_CUT_IN_POT_SCALE_FACTOR - SI_MIN_CUT_IN_POT_SCALE_FACTOR));
        scale_factor =
            MINMAX_FLOAT(SI_MIN_CUT_IN_POT_SCALE_FACTOR,
                         SI_MAX_CUT_IN_POT_SCALE_FACTOR, scale_factor);
        min_traj_dist *= scale_factor;
        max_traj_dist *= scale_factor;
    }

    /* When driving very curvy roads, our course tends to change very rapidly,
    so depending on curvature modify ramps */
    if (fABS(SICourseData.fCurve) > C_F32_DELTA) {
        const float32 curve_radius = (1.f / fABS(SICourseData.fCurve));
        if (curve_radius < SI_CUT_IN_CURV_RADIUS_MAX) {
            float32 scale_factor;
            if (curve_radius > SI_CUT_IN_CURV_RADIUS_MIN) {
                scale_factor =
                    SI_CUT_IN_CURV_SCALE_FACTOR_MIN +
                    ((1.f - SI_CUT_IN_CURV_SCALE_FACTOR_MIN) *
                     ((curve_radius - SI_CUT_IN_CURV_RADIUS_MIN) /
                      (SI_CUT_IN_CURV_RADIUS_MAX - SI_CUT_IN_CURV_RADIUS_MIN)));
            } else {
                scale_factor = SI_CUT_IN_CURV_SCALE_FACTOR_MIN;
            }
            /* Modify potential high/low markers according to scaling factor */
            min_traj_dist *= scale_factor;
            max_traj_dist *= scale_factor;
        }
    }

    /* Check the crossing point for trucks. As trucks have often a wrong high
       cut-in potential, the crossing distance is used to further scale the
       ramps */
    if (OBJ_CLASSIFICATION(ObjId) == CR_OBJCLASS_TRUCK) {
        float32 scale_factor;
        float32 fCrossDist;
        float32 fRelCrossDist;
        fCrossDist = SIGetCrossingDistTrace(ObjId);
        fRelCrossDist = fCrossDist - OBJ_LONG_DISPLACEMENT(ObjId);
        /* A crossing distance of zero means, that the trajectory of the object
           is not crossing our own trajectory. However, the trace polynomial is
           not very accurate, especially for objects at closer distances
           (numSamples <= 6). Thus, check the objects orientation to decide. If
           the object is moving with at least 1.0 deg towards us.*/
        if (fCrossDist <= BML_f_Delta) {
            const TraceID_t iObjTrace = OBJ_GET_STATIC_TRACE_ID(ObjId);
            if ((iObjTrace != FIP_u_TRACE_INVALID_ID) &&
                (FIP_STATIC_TRACE_GET_NO_OF_POINTS(iObjTrace) >=
                 SI_CUTIN_NUM_OF_POINTS_FOR_NO_CUTIN)) {
                /* Trace is reliable enough. Assume no cutin */
                fRelCrossDist = SI_CUT_IN_CROSS_DIST_MAX;
            } else {
                const float32 fAbsOrientation = fABS(OBJ_ORIENTATION(ObjId));
                /* Trace is unreliable, check object orientation to decide */
                if (fAbsOrientation > SI_CUTIN_OBJECT_ORIENTATION_THRESHOLD) {
                    if (((pKin->fDistY < 0.f) &&
                         (OBJ_ORIENTATION(ObjId) > 0.f)) ||
                        ((pKin->fDistY > 0.f) &&
                         (OBJ_ORIENTATION(ObjId) < 0.f))) {
                        /* Object is headed towards us */
                        fRelCrossDist =
                            SI_CUT_IN_CROSS_DIST_MAX -
                            ((SI_CUT_IN_CROSS_DIST_MAX -
                              SI_CUT_IN_CROSS_DIST_MIN) *
                             ((fAbsOrientation -
                               SI_CUTIN_OBJECT_ORIENTATION_THRESHOLD) /
                              (0.05f - SI_CUTIN_OBJECT_ORIENTATION_THRESHOLD)));
                    } else {
                        fRelCrossDist = SI_CUT_IN_CROSS_DIST_MAX;
                    }
                } else {
                    fRelCrossDist = SI_CUT_IN_CROSS_DIST_MAX;
                }
            }
        }

        scale_factor =
            SI_MAX_CUT_IN_POT_SCALE_FACTOR -
            ((1.0f - SI_CUT_IN_CROSS_SCALE_MIN) *
             ((fRelCrossDist - SI_CUT_IN_CROSS_DIST_MIN) /
              (SI_CUT_IN_CROSS_DIST_MAX - SI_CUT_IN_CROSS_DIST_MIN)));

        /* Set lower limit for scaling factor but do not restrict an upper limit
         * if we are sure */
        if ((scale_factor < SI_CUT_IN_CROSS_SCALE_MIN) &&
            (fCrossDist > BML_f_Delta)) {
            scale_factor = SI_CUT_IN_CROSS_SCALE_MIN;
        } else {
            scale_factor =
                MINMAX_FLOAT(SI_CUT_IN_CROSS_SCALE_MIN,
                             SI_MAX_CUT_IN_POT_SCALE_FACTOR, scale_factor);
        }
        /* Modify potential high/low markers according to scaling factor */
        min_traj_dist *= scale_factor;
        max_traj_dist *= scale_factor;
    }

    /* Now fn_val can be linearly ramped into a cut-in probability: the closer
    the value is to zero the higher the probability */
    if (fn_val > min_traj_dist) {
        if (fn_val < max_traj_dist) {
            /* Predicted position within min-max band, calculate linear ramp
             * based potential */
            cut_in_pot = 1.0f - ((fn_val - min_traj_dist) /
                                 (max_traj_dist - min_traj_dist));
        } else {
            /* Predicted position outside of minimum potential band around
             * course */
            cut_in_pot = 0.0f;
        }
    } else {
        /* Predicted position within maximum potential band around course */
        cut_in_pot = 1.0f;
    }

    /* Convert the [0..1] ranged internal cut-in into percent */
    cut_in_pot *= 100.f;

    return (uint8)cut_in_pot;
}

/*************************************************************************************************************************
  Functionname:    SIGetCrossingDistTrace */
float32 SIGetCrossingDistTrace(const ObjNumber_t ObjID) {
    float32 fDistCrossing = 0.0f;
    float32 fAuxCross1, fAuxCross2, fAuxP, fAuxQ;
    CPTracePolyL2_t pTracePoly;
    const TraceID_t iObjTrace = OBJ_GET_STATIC_TRACE_ID(ObjID);

    if (iObjTrace == FIP_u_TRACE_INVALID_ID) {
        /* No valid trace found, set crossing distance to zero.
           This results in a scaling factor of one. */
        fDistCrossing = 0.0f;
    } else {
        /* Store polynomial trace parameters of trace iObjTrace*/
        CPGetTracePoly(&pTracePoly, iObjTrace);

        if ((pTracePoly.isValid != FALSE) &&
            (fABS(pTracePoly.fC2 - (0.5f * SICourseData.fCurve)) >
             BML_f_AlmostZero)) {
            /* Calculate the crossing point between the two curves */
            fAuxP = pTracePoly.fC1 /
                    (pTracePoly.fC2 - (0.5f * SICourseData.fCurve));
            fAuxQ = pTracePoly.fC0 /
                    (pTracePoly.fC2 - (0.5f * SICourseData.fCurve));

            if (SQR(fAuxP / 2.0f) > fAuxQ) {
                fAuxCross1 = -(fAuxP / 2.0f) + SQRT_(SQR(fAuxP / 2.0f) - fAuxQ);
                fAuxCross2 = -(fAuxP / 2.0f) - SQRT_(SQR(fAuxP / 2.0f) - fAuxQ);

                /* Use the larger crossing point as result */
                fDistCrossing =
                    (fAuxCross1 > fAuxCross2) ? fAuxCross1 : fAuxCross2;
            } else {
                /* The two polynomials are not crossing.  */
                fDistCrossing = 0.0f;
            }
        } else {
            /* Trace is invalid */
            fDistCrossing = 0.0f;
        }
    }

    return fDistCrossing;
}

/*************************************************************************************************************************
  Functionname:    SICalcCutInNeighborObj */
uint8 SICalcCutInNeighborObj(const ObjNumber_t ObjId) {
    uint8 uiPotential_LateralKinematics;
    float32 ttc_ego_obj;
    float32 tg_ego_obj;
    boolean plausible_lateral_position;
    uint8 uiPotential_fused;

    /* get values for detecting a situation which is very unlikely for a cutin
     */
    /* (compare ego kinematics to object kinematics) */
    plausible_lateral_position = SICutInCheckLatPos(ObjId);
    ttc_ego_obj = SICutInCalcTTC(ObjId);
    tg_ego_obj = SICutInCalcTG(ObjId);

    /* if the kinematic relation of ego vehicle and object are unlikely for a
     * cutin */
    /* do not calculate potential */
    /* (this mainly avoids false positives in short distances) */
    /* no potential for trucks in tunnel */
    /* no potential for objects with In2OutlaneTransition > 0 */
    /* either the case after lane change or after object position has been
     * behind roadestimation */
    if ((plausible_lateral_position == FALSE) ||
        ((OBJ_LONG_VREL(ObjId) < -C_F32_DELTA) &&
         ((ttc_ego_obj < CUTIN_MIN_TTC) || (tg_ego_obj < CUTIN_MIN_TG))) ||
        ((OBJ_CLASSIFICATION(ObjId) == CR_OBJCLASS_TRUCK) &&
         (TUNNEL_PROBABILITY > 0.5f)) ||
        (OBJ_GET_SI(ObjId).ObjLaneAccStatus.In2OutlaneTransition > 0u)) {
        uiPotential_LateralKinematics = 0u;
    } else {
        uiPotential_LateralKinematics = SICalcCutInFromLateralKinematics(ObjId);
    }

    /* Additive Fusion */
    uiPotential_fused = uiPotential_LateralKinematics;
    SICutInObjData[ObjId].PotentialLateralKinematics =
        uiPotential_LateralKinematics;

    return uiPotential_fused;
}

/*************************************************************************************************************************
  Functionname:    SICutInObjectDataInit */
void SICutInObjectDataInit(void) {
    ObjNumber_t ObjID;

    /* reset data for all objects */
    for (ObjID = 0; ObjID < Envm_N_OBJECTS; ObjID++) {
        SICutInObjData[ObjID].PotentialLateralKinematics = 0u;
        SICutInObjData[ObjID].PotentialMultiObjectAnalyse = 0u;
        SICutInObjData[ObjID].dummy01 = 0u;
        SICutInObjData[ObjID].dummy02 = 0u;
    }
}

/*************************************************************************************************************************
  Functionname:    SICutInCalcTTC */
static float32 SICutInCalcTTC(const ObjNumber_t ObjId) {
    float32 ttc_ego_obj;

    if (OBJ_LONG_VREL(ObjId) < -C_F32_DELTA) {
        ttc_ego_obj = OBJ_LONG_DISPLACEMENT(ObjId) / -OBJ_LONG_VREL(ObjId);
    } else {
        ttc_ego_obj = SI_CUTIN_MAX_TTC;
    }
    return ttc_ego_obj;
}

/*************************************************************************************************************************
  Functionname:    SICutInCalcTG */
static float32 SICutInCalcTG(const ObjNumber_t ObjId) {
    float32 tg_ego_obj;

    if (EGO_SPEED_X_OBJ_SYNC > C_F32_DELTA) {
        tg_ego_obj = OBJ_LONG_DISPLACEMENT(ObjId) / EGO_SPEED_X_OBJ_SYNC;
    } else {
        tg_ego_obj = SI_CUTIN_MAX_TG;
    }
    return tg_ego_obj;
}

/*************************************************************************************************************************
  Functionname:    SICutInCheckLatPos */
static boolean SICutInCheckLatPos(const ObjNumber_t ObjId) {
    boolean plausible_lateral_position = TRUE;
    float32 blindspotoffset = SI_CUTIN_INIT_BLINDSPOTOFFSET;
    float32 fOBJ_LONG_DISPLACEMENT_ObjId;
    float32 fabs_OBJ_LONG_DISPLACEMENT_ObjId;

    if (OBJ_LONG_DISPLACEMENT(ObjId) < SI_CUTIN_MAX_LONG_DIST_BLINDSPOT) {
        blindspotoffset =
            SQR(OBJ_LONG_DISPLACEMENT(ObjId)) *
            (0.5f / CUTIN_RADIUS_BLINDSPOTAREA); /* curve with radius 3m */
        fOBJ_LONG_DISPLACEMENT_ObjId = OBJ_LONG_DISPLACEMENT(ObjId);
        fabs_OBJ_LONG_DISPLACEMENT_ObjId = fABS(OBJ_LAT_DISPLACEMENT(ObjId));
        if ((fOBJ_LONG_DISPLACEMENT_ObjId < 0.0f) ||
            (fabs_OBJ_LONG_DISPLACEMENT_ObjId > blindspotoffset)) {
            plausible_lateral_position = FALSE;
        }
    }
    return plausible_lateral_position;
}

/*************************************************************************************************************************
  Functionname:    SICutInOjectDataFreeze */

void SICutInObjectDataFreeze(void) {
    //(void)VLC_FREEZE_DATA(&SICutInObjData_MeasInfo, &SICutInObjData[0],
    //&SIMeasCallback);
}

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */