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
#include "fip_ext.h"
/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/* Pickup and drop thresholds for predicted lane association */
#define SI_PRED_INLANE_DIST_FAST (-0.275f)
#define SI_PRED_INLANE_DIST_SMALL \
    (0.666f * (0.60f + (SI_TOLERABLE_CUT_IN_ERROR / 2.0f)))
#define SI_PRED_INLANE_DIST (0.60f + (SI_TOLERABLE_CUT_IN_ERROR / 2.0f))
#define SI_PRED_OUTLANE_DIST (2.30f - (SI_TOLERABLE_CUT_OUT_ERROR))
#define SI_PRED_OUTLANE_DIST_MIN (SI_PRED_OUTLANE_DIST - 0.25f)
#define SI_MIN_PEX_FOR_PRED_PICKUP 96u /*0.96f*/
#define SI_PRED_INLANE_DIST_SMALL_ROADS 0.10f
#define SI_PRED_INLANE_DIST_CITY_ROADS 0.10f
#define SI_PRED_OUTLANE_DIST_SMALL_ROADS (2.10f - (SI_TOLERABLE_CUT_OUT_ERROR))

#define SI_EXPANDED_DROP_OVLC_ANGLE 7.0f

/* variance factor calculation constants for in lane decision */
/* The variance factor is used to respect the variance  of the object occupancy
for necessary minimum
occupancy for predicted in lane decision. The variance factor is a function of
distance. This means that
at bigger distances the OO varince is more respected than at lower distances. */
#define SI_PRED_VARIANCE_FACTOR_MIN 0.25f
#define SI_PRED_VARIANCE_FACTOR_MAX 0.50f
#define SI_DIST_MIN_VARIANCE_FACTOR 40.0f
#define SI_DIST_MAX_VARIANCE_FACTOR 100.0f
#define SI_PRED_MIN_CURVE_VARIANCE_TRUCK (0.5f / 2000.0f)
#define SI_PRED_MIN_PRED_TIME_INLANE (0.1f)

/* The variance factor is also dependend on Ego speed. At high speed approaches
it is necessary to reduce variance
factor faster to get cutting in objects relevant at earlier time. For this the
real distance of the object to calculate
the variance factor is modified by an offset, that is ego speed dependend. So at
higher ego speeds the variance factor
decreases faster than at lower speeds */
#define SI_DIST_MIN_VARIANCE_FACTOR_OFFSET 0.0f
#define SI_DIST_MAX_VARIANCE_FACTOR_OFFSET 60.0f
#define SI_SPEED_DIST_MIN_VARIANCE_FACTOR_OFFSET (120.0f / C_KMH_MS)
#define SI_SPEED_DIST_MAX_VARIANCE_FACTOR_OFFSET (200.0f / C_KMH_MS)

/* constants for calculation of allowed standard deviation of predicted distance
 * for in lane decision */
#define SI_SDEV_AT_MIN_DIST_SDEV_CALC (0.325f * FAHRZEUGBREITE)
#define SI_SDEV_AT_MAX_DIST_SDEV_CALC (0.2f * FAHRZEUGBREITE)
#define SI_MAX_DIST_SDEV_CALC 100.0f
#define SI_MIN_DIST_SDEV_CALC 60.0f

/* maximum allowed standard deviation of predicted distance for cut out decision
 */
#define SI_SDEV_MAX_FOR_CUT_OUT (0.325f * FAHRZEUGBREITE)

/* constants for radius dependend increase of minimum overlap value for
 * predicted pickup */
/* as trucks in curves can falsly get relevant by predicted pickup minimum
  overlap is increased
  radius dependend for all objects classified as trucks. */

#define SI_CURVE_MIN_PICKUP_OVERLAP_PRED (3600.f / (0.5f))
#define SI_CURVE_MAX_PICKUP_OVERLAP_PRED (1200.f / (0.5f))

/* Object Occupancy borders to reduce cut out line for slow cutting out objects
 */
#define SI_MAX_OO_REDUCING_DROP_TRSHLD (0.64f)
#define SI_MIN_OO_REDUCING_DROP_TRSHLD (0.44f)

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/
SET_MEMSEC_CONST(SIPredLaneAssVarianceFactor)
static const GDBLFunction_t SIPredLaneAssVarianceFactor = {

    SI_PRED_VARIANCE_FACTOR_MIN, SI_PRED_VARIANCE_FACTOR_MAX,
    ((SI_PRED_VARIANCE_FACTOR_MAX - SI_PRED_VARIANCE_FACTOR_MIN) /
     (SI_DIST_MAX_VARIANCE_FACTOR - SI_DIST_MIN_VARIANCE_FACTOR)),
    SI_PRED_VARIANCE_FACTOR_MIN -
        (((SI_PRED_VARIANCE_FACTOR_MAX - SI_PRED_VARIANCE_FACTOR_MIN) /
          (SI_DIST_MAX_VARIANCE_FACTOR - SI_DIST_MIN_VARIANCE_FACTOR)) *
         SI_DIST_MIN_VARIANCE_FACTOR)};

SET_MEMSEC_CONST(SIPredLaneAssVarianceFactorDistOffset)
static const GDBLFunction_t SIPredLaneAssVarianceFactorDistOffset = {

    SI_DIST_MIN_VARIANCE_FACTOR_OFFSET, SI_DIST_MAX_VARIANCE_FACTOR_OFFSET,
    ((SI_DIST_MAX_VARIANCE_FACTOR_OFFSET - SI_DIST_MIN_VARIANCE_FACTOR_OFFSET) /
     (SI_SPEED_DIST_MAX_VARIANCE_FACTOR_OFFSET -
      SI_SPEED_DIST_MIN_VARIANCE_FACTOR_OFFSET)),
    SI_DIST_MIN_VARIANCE_FACTOR_OFFSET -
        (((SI_DIST_MAX_VARIANCE_FACTOR_OFFSET -
           SI_DIST_MIN_VARIANCE_FACTOR_OFFSET) /
          (SI_SPEED_DIST_MAX_VARIANCE_FACTOR_OFFSET -
           SI_SPEED_DIST_MIN_VARIANCE_FACTOR_OFFSET)) *
         SI_SPEED_DIST_MIN_VARIANCE_FACTOR_OFFSET)};

SET_MEMSEC_CONST(SIPredLaneAssMaxLatDistSdev)
static const GDBLFunction_t SIPredLaneAssMaxLatDistSdev = {

    SI_SDEV_AT_MIN_DIST_SDEV_CALC, SI_SDEV_AT_MAX_DIST_SDEV_CALC,
    ((SI_SDEV_AT_MAX_DIST_SDEV_CALC - SI_SDEV_AT_MIN_DIST_SDEV_CALC) /
     (SI_MAX_DIST_SDEV_CALC - SI_MIN_DIST_SDEV_CALC)),
    SI_SDEV_AT_MIN_DIST_SDEV_CALC -
        (((SI_SDEV_AT_MAX_DIST_SDEV_CALC - SI_SDEV_AT_MIN_DIST_SDEV_CALC) /
          (SI_MAX_DIST_SDEV_CALC - SI_MIN_DIST_SDEV_CALC)) *
         SI_MIN_DIST_SDEV_CALC)};

SET_MEMSEC_CONST(SIPredLaneAssMinOverlapToCurve)
static const GDBLFunction_t SIPredLaneAssMinOverlapToCurve = {

    SI_TRUCK_OBJ_OVLC_PRED_PICKUP_THRESH, SI_MAX_OBJ_OVLC_PRED_PICKUP_THRESH,
    ((SI_MAX_OBJ_OVLC_PRED_PICKUP_THRESH -
      SI_TRUCK_OBJ_OVLC_PRED_PICKUP_THRESH) /
     (SI_CURVE_MAX_PICKUP_OVERLAP_PRED - SI_CURVE_MIN_PICKUP_OVERLAP_PRED)),
    SI_TRUCK_OBJ_OVLC_PRED_PICKUP_THRESH -
        (((SI_MAX_OBJ_OVLC_PRED_PICKUP_THRESH -
           SI_TRUCK_OBJ_OVLC_PRED_PICKUP_THRESH) /
          (SI_CURVE_MAX_PICKUP_OVERLAP_PRED -
           SI_CURVE_MIN_PICKUP_OVERLAP_PRED)) *
         SI_CURVE_MIN_PICKUP_OVERLAP_PRED)};

SET_MEMSEC_CONST(SIPickupBorder_Predicted)
static const GDBLFunction_t SIPickupBorder_Predicted = {

    SI_PRED_INLANE_DIST_SMALL, /* Ausgangswert A1 */
    SI_PRED_INLANE_DIST,       /* Ausgangswert A2 */
    (SI_PRED_INLANE_DIST - SI_PRED_INLANE_DIST_SMALL) /
        (SI_MAX_SPEED_HIGWAYLANEWIDTH - SI_MIN_SPEED_HIGWAYLANEWIDTH),
    /* Achsabschnitt der Anpassungsgerade:   A1 - (A2-A1)/(E2-E1) * E1 */
    SI_PRED_INLANE_DIST_SMALL -
        (((SI_PRED_INLANE_DIST - SI_PRED_INLANE_DIST_SMALL) /
          (SI_MAX_SPEED_HIGWAYLANEWIDTH - SI_MIN_SPEED_HIGWAYLANEWIDTH)) *
         SI_MIN_SPEED_HIGWAYLANEWIDTH)};

SET_MEMSEC_CONST(SIDropBorderToOOccupancy)
static const GDBLFunction_t SIDropBorderToOOccupancy = {

    SI_PRED_OUTLANE_DIST_MIN, SI_PRED_OUTLANE_DIST,
    ((SI_PRED_OUTLANE_DIST - SI_PRED_OUTLANE_DIST_MIN) /
     (SI_MAX_OO_REDUCING_DROP_TRSHLD - SI_MIN_OO_REDUCING_DROP_TRSHLD)),
    SI_PRED_OUTLANE_DIST_MIN -
        (((SI_PRED_OUTLANE_DIST - SI_PRED_OUTLANE_DIST_MIN) /
          (SI_MAX_OO_REDUCING_DROP_TRSHLD - SI_MIN_OO_REDUCING_DROP_TRSHLD)) *
         SI_MIN_OO_REDUCING_DROP_TRSHLD)};

SET_MEMSEC_CONST(SIDropBorderToAngle)
static const GDBLFunction_t SIDropBorderToAngle = {

    SI_PRED_OUTLANE_DROP_BORDER_ANGLE_MOD_MIN, /* Output value 1 (for good
                                                  angles don't modify drop
                                                  border) */
    SI_PRED_OUTLANE_DROP_BORDER_ANGLE_MOD_MAX, /* Output value 2 (for big angles
                                                  modify drop border) */
    ((SI_PRED_OUTLANE_DROP_BORDER_ANGLE_MOD_MAX -
      SI_PRED_OUTLANE_DROP_BORDER_ANGLE_MOD_MIN) /
     (SI_PRED_OUTLANE_DROP_BORDER_ANGLE_MAX -
      SI_PRED_OUTLANE_DROP_BORDER_ANGLE_MIN)),
    SI_PRED_OUTLANE_DROP_BORDER_ANGLE_MOD_MIN -
        (((SI_PRED_OUTLANE_DROP_BORDER_ANGLE_MOD_MAX -
           SI_PRED_OUTLANE_DROP_BORDER_ANGLE_MOD_MIN) /
          (SI_PRED_OUTLANE_DROP_BORDER_ANGLE_MAX -
           SI_PRED_OUTLANE_DROP_BORDER_ANGLE_MIN)) *
         SI_PRED_OUTLANE_DROP_BORDER_ANGLE_MIN)};

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  FUNCTION
*****************************************************************************/

static fTime_t SIGetApplInlanePredTime(ObjNumber_t ObjId);

/*************************************************************************************************************************
  Functionname:    SIGetApplInlanePredTime */
static fTime_t SIGetApplInlanePredTime(ObjNumber_t ObjId) {
    fTime_t pred_time;

    if ((OBJ_GET_RELEVANT_OBJ_NR == OBJ_INDEX_NO_OBJECT)) {
        /* In free-drive mode take lower prediction time to avoid drop-ins */
        pred_time = SI_MAX_LANE_ASSOC_PRED_TIME_LOW;
    } else {
        /* In follow-mode take higher default prediction time for better ego
         * lane change performance */
        pred_time = SI_MAX_LANE_ASSOC_PRED_TIME;
    }

    /* Keep MS-VC happy about using argument */
    ObjId = ObjId;
    return pred_time;
}

/*************************************************************************************************************************
  Functionname:    SICheckPredictedInlaneCriteria */
boolean SICheckPredictedInlaneCriteria(ObjNumber_t iObj,
                                       const CPTrajOccupancy_t* pOccupancy) {
    boolean RetVal;
    float32 prediction_time;
    SIPredictedDistance_t predicted_distance;
    fVelocity_t Veigen;
    float32 PredictedPickupDropBorder;
    float32 fVarianceFactor;
    float32 fVarianceFactorDistOffset;
    float32 VarianceDistance;
    float32 MaxPdistSDeviation;
    float32 Curve;
    float32 MinimumOverlap;
    float32 fObjOccStdDev;

    Veigen = EGO_SPEED_X_OBJ_SYNC;

    /************************ Calculate border for minimum overlap
     *******************************************
     Minimum overlap is calculated radius dependent for trucks, as trucks
     falsely got relevant in curves
      due to predicted lane association */
    /*! @todo : change this algo to general valid data for decision to restrict
       predicted lane association, when
        available by tracking, in order not to give to much disadvantage to
       trucks in curves in general       */
    /**********************************************************************************************************/
    Curve = fABS((0.5f) * EGO_CURVE_OBJ_SYNC);
    if (OBJ_CLASSIFICATION(iObj) == CR_OBJCLASS_TRUCK) {
        /* object is a truck */
        if (Curve > (1.0f / SI_CURVE_MIN_PICKUP_OVERLAP_PRED)) {
            /* curve is narrow enough to increase minimum overlap value to
             * restrict predicted lane association */
            MinimumOverlap = dGDBmathLineareFunktion(
                &SIPredLaneAssMinOverlapToCurve, 1.0f / Curve);
        } else {
            /* curve is not narrow, don't restrict predicted lane association */
            MinimumOverlap = SI_TRUCK_OBJ_OVLC_PRED_PICKUP_THRESH;
        }
    } else {
        /* Object is not a truck, don't restrict predicted lane association */
        MinimumOverlap = SI_OBJ_OVLC_PRED_PICKUP_THRESH;
    }
    /************************ End of calculate border for minimum overlap
     * *************************************/

    /* cut in case */
    prediction_time = SIGetApplInlanePredTime(iObj);

    fVarianceFactorDistOffset =
        dGDBmathLineareFunktion(&SIPredLaneAssVarianceFactorDistOffset, Veigen);
    VarianceDistance = OBJ_LONG_DISPLACEMENT(iObj) - fVarianceFactorDistOffset;
    if (VarianceDistance > 0.0f) {
        fVarianceFactor = dGDBmathLineareFunktion(&SIPredLaneAssVarianceFactor,
                                                  VarianceDistance);
    } else {
        fVarianceFactor = SI_PRED_VARIANCE_FACTOR_MIN;
    }

    /* For trucks in curves smaller 2000m distance variance shall be fully
       respected by setting variance
       factor to 1.0f, if object is hidden by other objects !!!*/
    if ((OBJ_PD_NEAR_OBJ_IN_BEAM(iObj) == FALSE) &&
        (Curve > SI_PRED_MIN_CURVE_VARIANCE_TRUCK) &&
        (OBJ_CLASSIFICATION(iObj) == CR_OBJCLASS_TRUCK)) {
        fVarianceFactor = 1.0f;
    }

    MaxPdistSDeviation = dGDBmathLineareFunktion(&SIPredLaneAssMaxLatDistSdev,
                                                 OBJ_LONG_DISPLACEMENT(iObj));
    /* @todo: delete again if OT or RSP have better angle prediction for hidden
     * objects */
    if (prediction_time >= SI_PRED_MIN_PRED_TIME_INLANE) {
        SICalcPredDisplToCourseStandard(prediction_time,
                                        OBJ_CLASSIFICATION(iObj), iObj,
                                        &predicted_distance);
        predicted_distance.pdist =
            predicted_distance.pdist +
            SQRT_(
                predicted_distance.pdist_var); /*!< Remark:
                                                  predicted_distance.pdist_var
                                                  >= 0 */
        PredictedPickupDropBorder =
            dGDBmathLineareFunktion(&SIPickupBorder_Predicted, Veigen);
        /* Calculate object occupancy standard deviation for condition below */
        fObjOccStdDev = SQRT_(
            pOccupancy
                ->fObjectOccupancyVar); /*!< Remark: fObjectOccupancyVar >= 0 */

        /**************** Following conditions have to be fullfilled for
         * predicted lane association: ****************/
        /*  - predicted distance must be closer than distance border (incl.
           variance)
            - object occupancy has to be at a minimum value (incl. variance )
           for trucks depending on curve value
              (see above)
            - Probability of existence must be very high, so that only best
           quality objects are predicted
            - Variance must be as small, that predicted object position can be
           placed within half standard
              lane width
            - object must be one of the neighbor objects on left or right side
           */
        /************************************************************************************************************/
        if ((predicted_distance.pdist < PredictedPickupDropBorder) &&
            (((pOccupancy->fObjectOccupancy -
               (fVarianceFactor * fObjOccStdDev)) > MinimumOverlap) ||
             ((predicted_distance.pdist < SI_PRED_INLANE_DIST_FAST) &&
              (pOccupancy->fObjectOccupancy > 0.0f))) &&
            (OBJ_PROBABILITY_OF_EXIST(iObj) > (SI_MIN_PEX_FOR_PRED_PICKUP)) &&
            (predicted_distance.pdist_var_fullpred < SQR(MaxPdistSDeviation)) &&
            (OBJ_GET_SI(iObj).Bool.AlreadyOOI)) {
            /* set corridor relevant time to max., to get object immediately in
             * lane */
            RetVal = TRUE;
        } else {
            RetVal = FALSE;
        }
    } else {
        /* do not do predicted cut in absolute near range area, where objects
         * reach sensor range border */
        RetVal = FALSE;
    }
    return RetVal;
}

/*************************************************************************************************************************
  Functionname:    SICheckPredictedOutlaneCriteria */
boolean SICheckPredictedOutlaneCriteria(ObjNumber_t iObj,
                                        const CPTrajOccupancy_t* pOccupancy) {
    boolean bRet;
    float32 prediction_time;
    SIPredictedDistance_t predicted_distance;
    float32 PredictedPickupDropBorder;
    /* cut out case */
    prediction_time = SILimitPredictionTimeDist(iObj);
    SICalcPredDisplToCutOut(prediction_time, OBJ_CLASSIFICATION(iObj), iObj,
                            &predicted_distance);
    predicted_distance.pdist =
        predicted_distance.pdist -
        SQRT_(predicted_distance
                  .pdist_var); /*!< Remark: predicted_distance.pdist_var >= 0 */

    /* Adapt drop border to Object Occupancy for earlier release of slow cut out
     * objects */
    PredictedPickupDropBorder = dGDBmathLineareFunktion(
        &SIDropBorderToOOccupancy, pOccupancy->fObjectOccupancy);

    /* Modify drop border for objects that are on beam border */
    PredictedPickupDropBorder -=
        dGDBmathLineareFunktion(&SIDropBorderToAngle, fABS(OBJ_ANGLE(iObj)));

    if ((predicted_distance.pdist > PredictedPickupDropBorder) &&
        (predicted_distance.pdist_var_fullpred <
         SQR(SI_SDEV_MAX_FOR_CUT_OUT))) {
        /* Branch depending on object angle: requirements for dropping object
        are different
        in main range (]-SI_EXPANDED_DROP_OVLC_ANGLE ..
        SI_EXPANDED_DROP_OVLC_ANGLE[) and
        outside of it */
        if (fABS(OBJ_ANGLE(iObj)) < SI_EXPANDED_DROP_OVLC_ANGLE) {
            /* In this angle range ]-SI_EXPANDED_DROP_OVLC_ANGLE ..
            SI_EXPANDED_DROP_OVLC_ANGLE[ verify
            that occupancies for pick-up are not satisfied */
            if (SICheckObjOccPickupValue(iObj, pOccupancy) == FALSE) {
                bRet = TRUE;
            } else {
                bRet = FALSE;
            }
        } else {
            /* In this angle range, object occupancy dropping below given
             * threshold is sufficient */
            if (pOccupancy->fObjectOccupancy <
                SI_LOW_NEAR_DIST_OBJ_OVLC_PRED_DROP_THRESH) {
                bRet = TRUE;
            } else {
                bRet = FALSE;
            }
        }
    } else {
        bRet = (FALSE);
    }
    return bRet;
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */
