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
//#include "vlc_par.h"
#include "vlc_init.h"
/*****************************************************************************
  MACROS
*****************************************************************************/

#define SI_CURVEOBS_NB_MEANVALUES ((uint8)5)

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*! Data type used for storing range reduction histogram information. Note:
this type is used in simulation for initialization purposes! When changing,
also update the adapter for handling the new layout! */
typedef struct {
    float32 DrivenDist;
    float32 SumCurveAbs;
    sint32 NumberSamples;
    float32 MeanCurveHist[SI_CURVEOBS_NB_MEANVALUES];
    float32 RangeFactor;

} SI_CurveObserv_RangeRed_t;

typedef enum SI_OcclusionByTraceSide_t {
    SI_OCCL_BY_TRACE_LEFT,
    SI_OCCL_BY_TRACE_RIGHT
} t_SI_OcclusionByTraceSide;

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

/* parameter for curve observer based on mean curve history */
#define SI_CURVEOBS_SAMPLEDISTANCE (200.0f)

#define SI_RANGERED_TRESH_CURVY_LONGTIME (0.0006f)
#define SI_RANGERED_TRESH_CURVY_SHORTTIME (SI_RANGERED_TRESH_CURVY_LONGTIME)

#define SI_RANGERED_MINSPEED (60.0f / C_KMH_MS)
#define SI_RANGERED_ON (1.0f)
#define SI_RANGERED_OFF (0.0f)
#define SI_RANGERED_FILTERFAST (5.0f)
#define SI_RANGERED_FILTERSLOW (250.0f)

#define SI_MAX_DIST_OOI_TO_TRACE_OBJ (75.f)
#define SI_MIN_DIST_OOI_TO_TRACE (-1.f)
#define SI_MIN_DIST_OOI_TO_TRACE_AVLC_TRAJ (-2.f)
#define SI_MAX_DIST_OOI_TO_TRACE (4.f)
#define SI_MIN_DIST_OOI_TO_AVLC_TRAJ (-1.f)

SET_MEMSEC_CONST(SI_CurveObs_MeasInfo)
static const MEASInfo_t SI_CurveObs_MeasInfo =

    {
        SI_RANGE_FACTOR_MEAS_VADDR,        /* .VirtualAddress */
        sizeof(SI_CurveObserv_RangeRed_t), /* .Length */
        SI_MEAS_FUNC_ID,                   /* .FuncID */
        SI_MEAS_FUNC_CHAN_ID               /* .FuncChannelID */
};

/*! Parameter to determine when a stationary object is not selected as OOI
    since it is an oncoming and stopped object */
#define SI_STATOBJ_ONCOMINGCOUNTER_MAX (100u)

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
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/*! Range reduction factor @min:0 @max:1 */
SET_MEMSEC_VAR(fRangeFac)
float32 fRangeFac = 0.0F;
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"

#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*! The pick up range for moving objects (with range factor correction) */
SET_MEMSEC_VAR(fSIMovingObjPickUpRange)
static fDistance_t fSIMovingObjPickUpRange;
/*! The base pick up range for moving objects (without range factor correction)
 */
SET_MEMSEC_VAR(fSIMovingObjBasePickUpRange)
static fDistance_t fSIMovingObjBasePickUpRange;

SET_MEMSEC_VAR(SI_CurveObs)
/*! @VADDR: SI_RANGE_FACTOR_MEAS_VADDR @CYCLEID: VLC_ENV */
static SI_CurveObserv_RangeRed_t SI_CurveObs;
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  FUNCTION
*****************************************************************************/

static void SIResetCurveObs(void);
static void SIUpdateCurveObsRangeFactor(void);
static void SIFreezeCurveObs(void);
static float32 SI_f_EgoSpeedPickupDistMoving(float32 f_EgoSpeed);
static boolean SICheckACCTimeGap(ObjNumber_t ObjNr);
static boolean SICheckACCTimeGap_propertytype(ObjNumber_t ObjNr);

static void SI_v_DetectOccludedByTrace(void);
static void SI_v_DetectOccludedByTraceSigleSide(
    const ObjNumber_t t_NextLongOOI, t_SI_OcclusionByTraceSide t_OcclSide);

/*************************************************************************************************************************
  Functionname:    SIResetCurveObs */
static void SIResetCurveObs(void) {
    uint8 i;

    SI_CurveObs.DrivenDist = 0.0f;
    SI_CurveObs.NumberSamples = 0;
    SI_CurveObs.SumCurveAbs = 0.0f;
    for (i = 0u; i < SI_CURVEOBS_NB_MEANVALUES; i++) {
        SI_CurveObs.MeanCurveHist[i] = 0.0f;
    }
    SI_CurveObs.RangeFactor = 0.0f;

    /* Reset range factor (global) */
    fRangeFac = 0.0f;
}

/*************************************************************************************************************************
  Functionname:    SIInitAccFunPreselection */
void SIInitAccFunPreselection(void) { SIResetCurveObs(); }

/*************************************************************************************************************************
  Functionname:    SIUpdateCurveObsRangeFactor */
static void SIUpdateCurveObsRangeFactor(void) {
    /**********************************************************************************/
    /* complex curve observer and range reduction logic */
    /**********************************************************************************/
    float32 CurveAbs = fABS(EGO_CURVE_OBJ_SYNC);
    float32 MeanCurve;
    FIP_t_FusedRoadType RoadType;

    float32 OldRangeRedFactor, NewRangeRedFactor;
    float32 FilterFactor;
    uint8 i;

    /*! sampling of curve only if vehicle is moving faster than
     * SI_RANGERED_MINSPEED */
    if (EGO_SPEED_X_OBJ_SYNC > SI_RANGERED_MINSPEED) {
        /* update driven distance */
        SI_CurveObs.DrivenDist += EGO_SPEED_X_OBJ_SYNC * SI_CYCLE_TIME;

        /* update sum of curve samples */
        SI_CurveObs.SumCurveAbs += CurveAbs;

        /* update number of curve samples */
        SI_CurveObs.NumberSamples++;

        /*  if driven distance > 200m, update mean values and reset */
        if (SI_CurveObs.DrivenDist > SI_CURVEOBS_SAMPLEDISTANCE) {
            if (SI_CurveObs.NumberSamples > 0) {
                MeanCurve = SI_CurveObs.SumCurveAbs /
                            (float32)SI_CurveObs.NumberSamples;

                /* update mean curve values of the last 200, 400, 600, 800,
                 * 1000m */
                /* update the last 1000, 800, 600, 400m */
                for (i = SI_CURVEOBS_NB_MEANVALUES - 1u; i > 0u; i--) {
                    SI_CurveObs.MeanCurveHist[i] =
                        (((float32)i * SI_CurveObs.MeanCurveHist[i - 1u]) +
                         MeanCurve) /
                        ((float32)i + 1.f);
                }
                /* update the last 200m */
                SI_CurveObs.MeanCurveHist[0] = MeanCurve;
            } else {
                /* NumberSamples == 0, but DrivenDist > 200m -> should never
                 * happen */
            }
            /* reset */
            SI_CurveObs.DrivenDist = 0.0f;
            SI_CurveObs.NumberSamples = 0;
            SI_CurveObs.SumCurveAbs = 0.0f;
        }
    } else /*! reset curve observer when velocity lower than
              SI_RANGERED_MINSPEED */
    {
        SIResetCurveObs();
    }

    /*! update RangeReductionFactor */
    OldRangeRedFactor = SI_CurveObs.RangeFactor;

    RoadType = FIP_t_Get_FuseRoadType();
    if (RoadType != FIP_ROAD_TYPE_HIGHWAY) {
        /*! if not on a highway, deactivate range reduction factor */
        NewRangeRedFactor = SI_RANGERED_OFF;
        FilterFactor = SI_RANGERED_FILTERSLOW;
    } else {
        /* ********************************************************** */

        if (SI_CurveObs.MeanCurveHist[SI_CURVEOBS_NB_MEANVALUES - 1u] >
            SI_RANGERED_TRESH_CURVY_LONGTIME) {
            if (SI_CurveObs.MeanCurveHist[0] >
                SI_RANGERED_TRESH_CURVY_SHORTTIME) {
                NewRangeRedFactor = SI_RANGERED_ON;
                FilterFactor = SI_RANGERED_FILTERFAST;
            } else {
                NewRangeRedFactor = SI_RANGERED_OFF;
                FilterFactor = SI_RANGERED_FILTERSLOW;
            }
        } else {
            if (SI_CurveObs.MeanCurveHist[0] >
                SI_RANGERED_TRESH_CURVY_SHORTTIME) {
                NewRangeRedFactor = SI_RANGERED_ON;
                FilterFactor = SI_RANGERED_FILTERSLOW;
            } else {
                NewRangeRedFactor = SI_RANGERED_OFF;
                FilterFactor = SI_RANGERED_FILTERFAST;
            }
        }
    }

    SI_CurveObs.RangeFactor =
        GDB_FILTER(NewRangeRedFactor, OldRangeRedFactor, FilterFactor);
    SI_CurveObs.RangeFactor = (SI_CurveObs.RangeFactor < C_F32_DELTA)
                                  ? 0.0f
                                  : SI_CurveObs.RangeFactor;

    fRangeFac = SI_CurveObs.RangeFactor;
}

/*************************************************************************************************************************
  Functionname:    SIFreezeCurveObs */
static void SIFreezeCurveObs(void) {
    //(void)VLC_FREEZE_DATA(&SI_CurveObs_MeasInfo, (void *)&SI_CurveObs,
    //&SIMeasCallback);
}

/*************************************************************************************************************************
  Functionname:    SIObjectPreselection */
void SIObjectPreselection(void) {
    ObjNumber_t ObjNr;
    boolean TGValue;

    /* Update range reduction factor */
    SIUpdateCurveObsRangeFactor();

    /* freeze data */
    SIFreezeCurveObs();

    /*! Calculate ego-speed dependent pick up range for moving objects without
     * range reduction */
    fSIMovingObjBasePickUpRange =
        SI_f_EgoSpeedPickupDistMoving(EGO_SPEED_X_OBJ_SYNC);
    /*! Calculate ego-speed dependent pick up range for moving objects with
     * range reduction */
    fSIMovingObjPickUpRange = SI_f_EgoSpeedPickupDistMoving(
        EGO_SPEED_X_OBJ_SYNC - (RANGEREDUCTION_VEGO_DELTA * fRangeFac));

    /* Determine if objects are occluded based on a trace analysis */
    SI_v_DetectOccludedByTrace();

    for (ObjNr = (ObjNumber_t)(Envm_N_OBJECTS - 1); ObjNr >= 0; ObjNr--) {
        if (!OBJ_IS_DELETED(ObjNr)) {
            TGValue = SICheckACCTimeGap(ObjNr);
        } else {
            TGValue = FALSE;
        }

        /* Save preselection decisions for debugging and display */
        OBJ_GET_SI(ObjNr).Bool.FctPreselTG = TGValue;

        /* Call customizable function to merge selections */
        SIBasePreselObjList[ObjNr] = SICustMergePreselection(
            ObjNr, OBJ_GET_AVLC_FUN_PRESEL_QUALITY(ObjNr), TGValue);

        /* Update sticky dynamic property bits (sticky, since once an
        object was detected moving/stationary/oncoming these attribute
        bits will remain set). Note: currently only 'moving' bit has
        a functional effect: it allows selection of oncoming objects
        rolling backwards. */

#if (defined(_MSC_VER))
#pragma message(__FILE__ "(" STRING_QUOTE( \
    __LINE__) "): Warning: known difference in SI bit setting behaviour with most likely no real consequences!")
#endif

        switch (OBJ_DYNAMIC_PROPERTY(ObjNr)) {
            case CR_OBJECT_PROPERTY_MOVING:
                OBJ_GET_SI(ObjNr).Bool.Moving = 1u;
                break;
            case CR_OBJECT_PROPERTY_STATIONARY:
                OBJ_GET_SI(ObjNr).Bool.Stationary = 1u;
                break;
            case CR_OBJECT_PROPERTY_ONCOMING:
                OBJ_GET_SI(ObjNr).Bool.Oncoming = 1u;
                if ((OBJ_GET_SI(ObjNr).StatObjWasOncoming.uiOncomingCounter <
                     SI_STATOBJ_ONCOMINGCOUNTER_MAX) &&
                    (OBJ_LONG_DISPLACEMENT(ObjNr) >
                     SI_PAR_ONCOMMING_KEEP_RELEV_DISTX_MAX)) {
                    OBJ_GET_SI(ObjNr).StatObjWasOncoming.uiOncomingCounter++;
                }
                break;
            default:
                /* default = do nothing */
                break;
        }
    }
}

/*************************************************************************************************************************
  Functionname:    SI_f_EgoSpeedPickupDistMoving */
static float32 SI_f_EgoSpeedPickupDistMoving(float32 f_EgoSpeed) {
    float32 f_TmpMaxRelDist;
    uint8 u_NumPointsCurve = 0u;

    /*! Get sample points and number of sample points */
    const GDBVector2_t* p_SamplePoints =
        SI_GetPickupDistSamplePoints(&u_NumPointsCurve);

    /* vehicle velocity depending maximum distance for relevant objects */
    /* pickup range specified by p_SamplePoints */
    f_TmpMaxRelDist = GDB_Math_CalculatePolygonValue(
        u_NumPointsCurve, p_SamplePoints, f_EgoSpeed);

    return f_TmpMaxRelDist;
}

/*************************************************************************************************************************
  Functionname:    SI_v_DetectOccludedByTrace */
static void SI_v_DetectOccludedByTrace(void) {
    ObjNumber_t ObjNr;

    /* Set default values */
    for (ObjNr = (ObjNumber_t)(Envm_N_OBJECTS - 1); ObjNr >= 0; ObjNr--) {
        OBJ_GET_SI(ObjNr).Bool.OccludedByTrace = 0u;
    }

    /* Check for traces which go through the next long OOI on the right side */
    SI_v_DetectOccludedByTraceSigleSide(
        OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_NEXT_LONG_RIGHT_OOI),
        SI_OCCL_BY_TRACE_RIGHT);
    /* Check for traces which go through the next long OOI on the left side */
    SI_v_DetectOccludedByTraceSigleSide(
        OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_NEXT_LONG_LEFT_OOI),
        SI_OCCL_BY_TRACE_LEFT);
}

/*************************************************************************************************************************
  Functionname:    SI_v_DetectOccludedByTraceSigleSide */
static void SI_v_DetectOccludedByTraceSigleSide(
    const ObjNumber_t t_NextLongOOI, t_SI_OcclusionByTraceSide t_OcclSide) {
    uint8 u_IdxTrace, u_IdxTraceSample, u_NumApproxPts;
    float32 f_XDistNextLongOOI, f_YDistNextLongOOI;
    float32 f_YDistToTracePolyNextLongOOI;
    float32 f_DistToTrajNextLongOOI, f_DistToTrajVarNextLongOOI;
    CPTracePolyL2_t t_ApproxPolyTrace;

    /* Find occluded objects only if a next long OOI object is available */
    if (t_NextLongOOI != OBJ_NOT_OOI) {
        /* Get the x- and y-position of the next long OOI */
        f_XDistNextLongOOI = OBJ_LONG_DISPLACEMENT(t_NextLongOOI);
        f_YDistNextLongOOI = OBJ_LAT_DISPLACEMENT(t_NextLongOOI);

        /* Check if there exists a trace which goes "through" the next long OOI
          -> the object of this trace is marked as "occluded by trace" */
        for (u_IdxTrace = 0u; u_IdxTrace < FIP_STATIC_TRACE_NO_OF_TRACES;
             u_IdxTrace++) {
            /* If the object of the trace is valid and if the distance to the
             * next long OOI is lower than a threshold */
            if ((FIP_STATIC_TRACE_GET_VLC_ID(u_IdxTrace) <
                 TRACE_VALID_NO_OBJ_ID) &&
                (FIP_STATIC_TRACE_GET_VLC_ID(u_IdxTrace) != t_NextLongOOI) &&
                (OBJ_LONG_DISPLACEMENT(
                     FIP_STATIC_TRACE_GET_VLC_ID(u_IdxTrace)) -
                     (OBJ_LONG_DISPLACEMENT(t_NextLongOOI)) <
                 SI_MAX_DIST_OOI_TO_TRACE_OBJ)) {
                /* Check if there is a x-sample lower and higher than the
                 * x-position of the next long OOI */
                // changed this by guotao 20190418. Bug found when
                // FIP_STATIC_TRACE_GET_NO_OF_POINTS(u_IdxTrace) == 0
                for (u_IdxTraceSample = 0u;
                     (u_IdxTraceSample <
                      (uint8)FIP_STATIC_TRACE_GET_NO_OF_POINTS(u_IdxTrace) -
                          1u) &&
                     (uint8)FIP_STATIC_TRACE_GET_NO_OF_POINTS(u_IdxTrace) > 0;
                     u_IdxTraceSample++) {
                    if ((FIP_STATIC_TRACE_GET_X(u_IdxTrace)[u_IdxTraceSample] >
                         OBJ_LONG_DISPLACEMENT(t_NextLongOOI)) &&
                        (FIP_STATIC_TRACE_GET_X(
                             u_IdxTrace)[u_IdxTraceSample + 1u] <=
                         OBJ_LONG_DISPLACEMENT(t_NextLongOOI))) {
                        /* Find the number of samples with positive X coordinate
                         */
                        u_NumApproxPts =
                            (uint8)FIP_STATIC_TRACE_GET_NO_OF_POINTS(
                                u_IdxTrace);
                        while ((u_NumApproxPts > 1u) &&
                               (FIP_STATIC_TRACE_GET_X(
                                    u_IdxTrace)[u_NumApproxPts - 2u] <= 0.f)) {
                            u_NumApproxPts--;
                        }

                        if (u_NumApproxPts >= 3u) {
                            /* Interpolate a polynomial through the trace
                             * samples */
                            CPCalcPointApproxPolyL2(
                                &t_ApproxPolyTrace,
                                FIP_STATIC_TRACE_GET_X(u_IdxTrace),
                                FIP_STATIC_TRACE_GET_Y(u_IdxTrace),
                                u_NumApproxPts);

                            /* Get the distance of the next long OOI to the ACC
                             * trajectory */
                            SITrajGetObjToRefDistance(
                                t_NextLongOOI, &f_DistToTrajNextLongOOI,
                                &f_DistToTrajVarNextLongOOI);

                            /* Get the distance of the next long OOI to the
                             * trace polynomial */
                            f_YDistToTracePolyNextLongOOI =
                                t_ApproxPolyTrace.fC0 +
                                t_ApproxPolyTrace.fC1 * f_XDistNextLongOOI +
                                t_ApproxPolyTrace.fC2 * f_XDistNextLongOOI *
                                    f_XDistNextLongOOI;

                            /* If the distance of the trace to the next long OOI
                              is within limits, set the "occluded by trace"-bit
                              (if the distance of the next long OOI to the ACC
                              trajectory is high, the threshold is different) */
                            if (t_OcclSide == SI_OCCL_BY_TRACE_RIGHT) {
                                if (((f_YDistNextLongOOI -
                                          f_YDistToTracePolyNextLongOOI >
                                      SI_MIN_DIST_OOI_TO_TRACE) ||
                                     ((f_YDistNextLongOOI -
                                           f_YDistToTracePolyNextLongOOI >
                                       SI_MIN_DIST_OOI_TO_TRACE_AVLC_TRAJ) &&
                                      (f_DistToTrajNextLongOOI -
                                           f_YDistToTracePolyNextLongOOI >
                                       SI_MIN_DIST_OOI_TO_AVLC_TRAJ))) &&
                                    (f_YDistNextLongOOI -
                                         f_YDistToTracePolyNextLongOOI <
                                     SI_MAX_DIST_OOI_TO_TRACE)) {
                                    OBJ_GET_SI(
                                        FIP_STATIC_TRACE_GET_VLC_ID(u_IdxTrace))
                                        .Bool.OccludedByTrace = 1u;
                                    break;
                                }
                            } else if (t_OcclSide == SI_OCCL_BY_TRACE_LEFT)

                            {
                                if (((f_YDistNextLongOOI -
                                          f_YDistToTracePolyNextLongOOI <
                                      -SI_MIN_DIST_OOI_TO_TRACE) ||

                                     ((f_YDistNextLongOOI -
                                           f_YDistToTracePolyNextLongOOI <
                                       -SI_MIN_DIST_OOI_TO_TRACE_AVLC_TRAJ) &&
                                      (f_DistToTrajNextLongOOI -
                                           f_YDistToTracePolyNextLongOOI <
                                       -SI_MIN_DIST_OOI_TO_AVLC_TRAJ))) &&
                                    (f_YDistNextLongOOI -
                                         f_YDistToTracePolyNextLongOOI >
                                     -SI_MAX_DIST_OOI_TO_TRACE)) {
                                    OBJ_GET_SI(
                                        FIP_STATIC_TRACE_GET_VLC_ID(u_IdxTrace))
                                        .Bool.OccludedByTrace = 1u;
                                    break;
                                    /* */
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

/*************************************************************************************************************************
  Functionname:    SIGetMovingObjPickupRange */
fDistance_t SIGetMovingObjPickupRange(void) { return fSIMovingObjPickUpRange; }

/*************************************************************************************************************************
  Functionname:    SIGetMovingObjBasePickupRange */
fDistance_t SIGetMovingObjBasePickupRange(void) {
    return fSIMovingObjBasePickUpRange;
}

/*************************************************************************************************************************
  Functionname:    SICheckACCTimeGap_propertytype */
static boolean SICheckACCTimeGap_propertytype(ObjNumber_t ObjNr) {
    return ((OBJ_DYNAMIC_PROPERTY(ObjNr) == CR_OBJECT_PROPERTY_MOVING) ||
            ((OBJ_DYNAMIC_PROPERTY(ObjNr) == CR_OBJECT_PROPERTY_STATIONARY) &&
             (OBJ_IS_MOVING_TO_STATIONARY(ObjNr))));
}

static void SIObjDistCheck(const float32 DistMax,
                           ObjNumber_t ObjNr,
                           boolean* p_bRet) {
    if ((OBJ_LONG_DISPLACEMENT(ObjNr) > 0.0f) &&
        (OBJ_LONG_DISPLACEMENT(ObjNr) < DistMax)) {
        *p_bRet = (TRUE);
    } else {
        *p_bRet = (FALSE);
    }
}

/*************************************************************************************************************************
  Functionname:    SICheckACCTimeGap */
static boolean SICheckACCTimeGap(ObjNumber_t ObjNr) {
    boolean bRet;
    float32 DistMax;
    float32 ftemp;
    float32 fMaxDistOncomming;
    float32 fRadius;
    float32 fCurve;
    float32 fRangeFacLocal;
    float32 fSIMovingObjPickUpRangeLocal;
    float32 fOOIDepedentRangeAddend;
    float32 fTmpVeigen;
    ObjNumber_t
        a_OvertakenObjID[SI_FIND_MOV_OBJ_IN_AREA_NUM_MAX]; /* Array of objects
                                                              possibly overtaken
                                                              by ObjNr */

    const sint32 s_NumberLanesLeft = FIP_s_GetLMLeftNumLane();
    const sint32 s_NumberLanesRight = FIP_s_GetLMRightNumLane();

    /*write rangefac to local variable*/
    fRangeFacLocal = fRangeFac;

    /*! Enable range reduction if object is occluded.
        Consider occlusion only if the object can become the OOI-0 based in the
       longitudinal distance in order to reduce the influence on the OOI-1).*/
    if ((((OBJ_ATTRIBUTES(ObjNr).eObjectOcclusion >=
           Envm_GEN_OBJECT_OCCL_PARTLY) &&
          (OBJ_LONG_DISPLACEMENT(ObjNr) < SI_OCCLUSION_MAX_DIST)) ||
         (OBJ_GET_SI(ObjNr).Bool.OccludedByTrace == 1u)) &&
        (OBJ_GET_RELEVANT_OBJ_NR != ObjNr) &&
        ((OBJ_GET_RELEVANT_OBJ_NR == OBJ_INDEX_NO_OBJECT) ||
         ((OBJ_GET_RELEVANT_OBJ_NR != OBJ_INDEX_NO_OBJECT) &&
          (OBJ_LONG_DISPLACEMENT(OBJ_GET_RELEVANT_OBJ_NR) +
               OT_GET_OBJ_LENGTH(OBJ_GET_RELEVANT_OBJ_NR) >
           OBJ_LONG_DISPLACEMENT(ObjNr))))) {
        fRangeFacLocal = 1.0f;
    }

    /*Reduce acc-pickup-range immediately, if ego-vehicle driving in a curvy
     * tunnel*/
    if ((TUNNEL_PROBABILITY > SI_TUNNEL_PROB_THRES) &&
        (OBJ_GET_RELEVANT_OBJ_NR != ObjNr) &&
        (fABS(EGO_CURVE_OBJ_SYNC) > RANGEREDUCTION_CURVE_THRESH)) {
        fRangeFacLocal = 1.0f;
    }

    CPTrajectoryData_t* pTrajectoryData; /*!< Pointer on the estimated
                                      trajectory fusion properties */
    /* Get trajectory fusion properties */
    pTrajectoryData = SIGetTrajectoryData();
    if ((OBJ_GET_SI(ObjNr).Bool.AlreadyOOI == FALSE) &&
        (OBJ_DYNAMIC_PROPERTY(ObjNr) == CR_OBJECT_PROPERTY_MOVING) &&
        ((OBJ_LONG_VREL(ObjNr) > (SI_VDY_ONLY_VREL_REDUC_MIN_VREL)) &&
         ((pTrajectoryData->State.FusionRoadstimation == FALSE) ||
          ((pTrajectoryData->State.FusionRoadstimation != FALSE)) ||
          (OBJ_ATTRIBUTES(ObjNr).eObjectOcclusion >=
           Envm_GEN_OBJECT_OCCL_FULL)))) {
        fRangeFacLocal = 1.0f;
    }

    /* Suppression of trucks at high distances when ego is on the left lane */
    /* This feature is useful for german autobahn and above 140 kp/h. It assumes
     * german traffic direction for these velocities. */
    /* When a truck is detected and there is no other object which could be
     * overtaken by it, */
    /* set the pick-up distance to a minimum */
    if ((OBJ_GET_SI(ObjNr).Bool.AlreadyOOI == (ubit16_t)0) &&
        (OBJ_CLASSIFICATION(ObjNr) == CR_OBJCLASS_TRUCK) &&
        (OBJ_DYNAMIC_PROPERTY(ObjNr) == CR_OBJECT_PROPERTY_MOVING) &&
        ((EGO_SPEED_X_OBJ_SYNC) > (SI_HIGH_DIST_TRUCK_SUPPRESS_VEGO_MIN)) &&
        (OBJ_LONG_DISPLACEMENT(ObjNr) >
         SI_HIGH_DIST_TRUCK_SUPPRESS_XDIST_MIN) &&
        (OBJ_LONG_VREL(ObjNr) < SI_HIGH_DIST_TRUCK_SUPPRESS_VREL_MAX) &&
        ((EGO_SPEED_X_OBJ_SYNC + OBJ_LONG_VREL(ObjNr)) >
         SI_HIGH_DIST_TRUCK_SUPPRESS_VOBJ_MIN) &&
        (OBJ_LONG_AREL(ObjNr) > SI_HIGH_DIST_OBJ_SUPPRESS_AREL_MIN)) {
        /* Check lane matrix and road border specific conditions. Ego has to be
         * most likely on left lane. */
        if ((s_NumberLanesRight != 0) && ((s_NumberLanesLeft == 0))) {
            /* Loop over all objects to check if truck is overtaking */
            SIFindObjInArea(a_OvertakenObjID, ObjNr,
                            &SIOvertakingCheckArgsHighVego);

            /* Check if possible overtaken object has been found */
            if (a_OvertakenObjID[0] == (ObjNumber_t)-1) {
                /* Truck is most likely not overtaking. Set pick-up distance to
                 * minimum to avoid drop-ins. */
                fRangeFacLocal = 1.0F;
            }
        }
    }

    if (!OBJ_IS_DELETED(ObjNr)) {
        /* this part for CR_OBJECT_PROPERTY_STATIONARY (subcategory STOPPED
         * moved to moving) */
        if ((OBJ_DYNAMIC_PROPERTY(ObjNr) == CR_OBJECT_PROPERTY_STATIONARY) &&
            (!OBJ_IS_MOVING_TO_STATIONARY(ObjNr))) {
            DistMax = (SI_ABSTAND_MAX_NORELOBJ);

            /* Set Hysteresis for dropping OOIs */
            /* @todo only relevant when driving backwards or when object is
             * moving slowly (e.G. pedestrians). remove fRangeFac dependency */
            if (OBJ_GET_SI(ObjNr).Bool.AlreadyOOI == (ubit16_t)1) {
                fOOIDepedentRangeAddend =
                    MAX_FLOAT(SI_MAX_DIST_HYSTERESIS_MIN,
                              SI_MAX_DIST_HYSTERESIS * (1.0f - fRangeFacLocal));
                DistMax += fOOIDepedentRangeAddend;
            }

            DistMax = MIN_FLOAT(
                DistMax, (EGO_SPEED_X_OBJ_SYNC * OBJ_REGELZEITABSTAND_MAX));
            DistMax = MAX_FLOAT(DistMax, STABI_ABSTAND_REGEL_MIN_STATIONARY);

            /* reduce maximum range for stationarys, if sensor is unlearned
             * (steering angle offset) */
            if (EGO_YAW_RATE_QUALITY_RAW <=
                SI_VDY_YAWRATE_QUAL_SENSOR_UNLEARNED) {
                DistMax = MIN_FLOAT(DistMax, SI_ABSTAND_MAX_UNLEARNED_SENSOR);
            }
        }
        /* this part for CR_OBJECT_PROPERTY_MOVING and subcategory STOPPED */
        else if (SICheckACCTimeGap_propertytype(ObjNr) == TRUE) {
            /*! Calculate ego-speed dependent pick up range for moving objects
             * with range reduction */
            fTmpVeigen = EGO_SPEED_X_OBJ_SYNC -
                         (RANGEREDUCTION_VEGO_DELTA * fRangeFacLocal);
            fSIMovingObjPickUpRangeLocal =
                SI_f_EgoSpeedPickupDistMoving(fTmpVeigen);

            DistMax = MINMAX_FLOAT(DISTANCE_MIN, DISTANCE_MAX,
                                   fSIMovingObjPickUpRangeLocal);

            /* Set Hysteresis for dropping OOIs */
            if (OBJ_GET_SI(ObjNr).Bool.AlreadyOOI == (ubit16_t)1) {
                {
                    /* hysteresis of OOI fOOIDepedentRangeAddend */
                    fOOIDepedentRangeAddend = MAX_FLOAT(
                        SI_MAX_DIST_HYSTERESIS_MIN,
                        SI_MAX_DIST_HYSTERESIS_MOV * (1.0f - fRangeFacLocal));
                }
                DistMax += fOOIDepedentRangeAddend;
            }

            DistMax = MAX_FLOAT(STABI_ABSTAND_REGEL_MIN, DistMax);
        }
        /* this part for CR_OBJECT_PROPERTY_ONCOMING */
        else if (OBJ_DYNAMIC_PROPERTY(ObjNr) == CR_OBJECT_PROPERTY_ONCOMING) {
            /* reduce range for oncoming objects MIN(2sec control
             * time,f(curve))) */
            /* Get Curvature */
            fCurve = EGO_CURVE_OBJ_SYNC;
            if (fABS(fCurve) > C_F32_DELTA) {
                fRadius = fABS(1.0f / fCurve);
            } else {
                fRadius = 1.0f / C_F32_DELTA;
            }

            ftemp = MIN_FLOAT(
                (fABS(OBJ_LONG_VREL(ObjNr)) * (REGELZEITABSTAND_ONCOMING)),
                MAXDISTONCOMING);
            fMaxDistOncomming =
                REGELZEITABSTAND_ONCOMING * SQRT_(AYMAX * fRadius);
            DistMax = MIN_FLOAT(ftemp, fMaxDistOncomming);

#if (defined(_MSC_VER))
#pragma COMPILEMSG( \
    "MEGABUG: Already present in ARS300/ARS310 and co. If the target vehicle starts rolling back very slowly, it can get very-very small ranges! Temporary workaround to keep object!")
#endif
            DistMax = MAX_FLOAT(SI_PAR_ONCOMMING_KEEP_RELEV_DISTX_MAX, DistMax);
        } else {
            /* this branch should not be reached with current definition of
             * Envm_t_CR_DynamicProperty */
            DistMax = (0.0f);
        }
    } else {
        /* Deleted object branch */
        DistMax = (0.0f);
    }

    /*! Limit DistMax independent of the object's dynamic property to the
     * maximal sensor range RW_VLC_MAX */
    DistMax = MIN_FLOAT(DistMax, RW_VLC_MAX);

    /* Objects which are above the maximum longitudinal distance shall not be
       selected as OOI as the ACC trajectory, object position and object
       classification are unreliable in high distances. Furthermore object
       selection with a high time-gap value is not so important since there is
       enough reaction time. */
    SIObjDistCheck(DistMax, ObjNr, &bRet);
    return bRet;
}

/*************************************************************************************************************************
  Functionname:    SIFindObjInArea */
void SIFindObjInArea(ObjNumber_t p_ObjInArea[],
                     const ObjNumber_t ObjA,
                     SIFindObjInAreaArgs_t const* const AreaArgs) {
    ObjNumber_t ObjBSorted; /* Index in range sorted VLC list */
    ObjNumber_t ObjB;       /* Index of object B in normal VLC list       */
    float32 f_DeltaDistX;   /* Longitudinal displacement of object B relative to
                               object A  */
    float32 f_DeltaDistY; /* Lateral displacement of object B relative to object
                             A       */
    float32 f_DeltaVel;   /* Velocity of object B relative to object A   */
    uint8 u_ResultArrayIndex; /* index of current position of object in result
                                 array         */

    /* Initialize result array with default values */
    for (u_ResultArrayIndex = 0;
         (u_ResultArrayIndex < SI_FIND_MOV_OBJ_IN_AREA_NUM_MAX);
         u_ResultArrayIndex++) {
        p_ObjInArea[u_ResultArrayIndex] = (ObjNumber_t)-1;
    }

    /* Initialize result array index value with zero */
    u_ResultArrayIndex = (uint8)0;

    /* Loop over range sorted VLC list so that the result of this function is
     * independant of  */
    /* object position in normal VLC list. */
    /* In Envm_OBJ_INDEX_DISTX_SORTED[i] "empty" objects have index -1. */
    /* They are at the end of the list. That is why we loop over
     * OBJ_NUMBER_OF_OBJ_USED       */
    for (ObjBSorted = 0; (ObjBSorted < OBJ_NUMBER_OF_OBJ_USED) &&
                         (u_ResultArrayIndex < SI_FIND_MOV_OBJ_IN_AREA_NUM_MAX);
         ObjBSorted++) {
        /* Get index in normal VLC list */
        ObjB = Envm_OBJ_INDEX_DISTX_SORTED[ObjBSorted];

        /* Check if candidate is not identical to to object handed to
         * function,if it is moving  */
        /* and has acc quality */
        if ((ObjB != ObjA) &&
            (OBJ_DYNAMIC_PROPERTY(ObjB) == CR_OBJECT_PROPERTY_MOVING) &&
            (OBJ_GET_AVLC_FUN_PRESEL_QUALITY(ObjB) >=
             FUN_PRESEL_AVLC_MIN_OBJ_QUAL)) {
            /* Compute position and velocity delta of object B relative to
             * object A                 */
            f_DeltaDistX =
                OBJ_LONG_DISPLACEMENT(ObjB) - OBJ_LONG_DISPLACEMENT(ObjA);
            f_DeltaDistY =
                OBJ_LAT_DISPLACEMENT(ObjB) - OBJ_LAT_DISPLACEMENT(ObjA);
            f_DeltaVel = OBJ_LONG_VREL(ObjB) - OBJ_LONG_VREL(ObjA);

            /* Check if position and velocity delta of object B relative to
             * object A               */
            /* are whithin boundaries handed to the function */
            if ((f_DeltaDistX > (AreaArgs->DeltaDistXLower)) &&
                (f_DeltaDistX < (AreaArgs->DeltaDistXUpper)) &&
                (f_DeltaDistY > (AreaArgs->DeltaDistYLower)) &&
                (f_DeltaDistY < (AreaArgs->DeltaDistYUpper)) &&
                (f_DeltaVel > (AreaArgs->DeltaVelLower)) &&
                (f_DeltaVel < (AreaArgs->DeltaVelUpper))) {
                /* Store object ID in result array for further computation */
                p_ObjInArea[u_ResultArrayIndex] = ObjB;
                u_ResultArrayIndex++;
            }
        }
    }
}

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */